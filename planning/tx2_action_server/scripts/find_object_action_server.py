#! /usr/bin/env python3
import rospy
import tf
import math
import time
import os
import actionlib
import numpy as np
from std_msgs.msg import Float32MultiArray, Float64MultiArray, MultiArrayDimension, Bool, String
from nav_msgs.msg import Odometry, Path
from communication_msgs.msg import FindObjectGoal, FindObjectAction, FindObjectFeedback, FindObjectResult
from control_mobile_robot.msg import ControlAction, ControlGoal, ControlFeedback, ControlResult
from geometry_msgs.msg import PoseStamped, Twist
from husky_tidy_bot_cv.msg import Objects, Objects3d, Categories
from visualization_msgs.msg import Marker
import yaml
current_pose = []
path = []
DEFAULT_RATE = 10


class FindObject:
    def __init__(self):
        # Setup feedback and result
        rospy.init_node('find_object_action_server')#, log_level=rospy.DEBUG)
        self.goal = FindObjectGoal()
        self.feedback = FindObjectFeedback()
        self.result = FindObjectResult()
        
        # Init ROS params
        global_frame = rospy.get_param('~global_frame')
        rate = rospy.get_param('~rate', DEFAULT_RATE)
        """
        self.object_category_mapping = {
            0: 'cube',
            1: 'bin box basket container',
            2: 'table',
            3: 'chair',
            4: 'person',
            5: 'nightstand',
            6: 'drawer',
            7: 'handle',
            8: 'sink',
            9: 'cucumber',
            10: 'potato',
            11: 'chili pepper',
            12: 'bell pepper',
            13: 'corn',
            14: 'eggplant',
            15: 'carrot',
            16: 'garlic',
            17: 'tomato',
            18: 'toy_cat',
            19: 'plates',
            20: 'mug',
            21: 'cup',
            22: 'crocs',
            23: 'slippers',
            24: 'hat with earflaps',
            25: 't-shirt',
            26: 'hoodie',
            27: 'towel',
            28: 'socks',
            29: 'hat',
            30: 'chips',
            31: 'snickers',
            32: 'bottle',
            33: 'cans',
            34: 'plastic bags',
            35: 'charging'
        }
        """
        self.object_category_mapping = None
        #rospy.loginfo("Object categories: %s", ', '.join(self.object_category_mapping.values()))

        # Setup publishers and subscribers
        self.control_status_subscriber = rospy.Subscriber('control_status', String, self.control_status_callback)
        self.odom_subscriber = rospy.Subscriber('/cartographer/tracked_global_odometry', Odometry, self.odom_callback)
        self.tracked_objects_subscriber = rospy.Subscriber('/tracked_objects_3d', Objects3d, self.tracking_callback)
        self.segmentation_labels_subscriber = rospy.Subscriber('/segmentation_labels', Categories, self.segmentation_labels_callback)
        self.task_publisher = rospy.Publisher('/task', Float32MultiArray, queue_size=1)
        self.object_marker_pub = rospy.Publisher('/object_marker', Marker,  latch=True, queue_size=100)
        self.cmd_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        
        # Init variables
        self.global_frame = global_frame
        self.rate = rospy.Rate(rate)
        self.goal_reached = False
        self.object_x = None
        self.object_y = None
        self.object_theta = None
        self.odom_x = None
        self.odom_y = None
        self.target_x = None
        self.target_y = None
        self.target_theta = None
        self.object_type = None
        self.found = False
        self.object_detected = False
        self.preempted = False
        self.start_time = 0
        
        # Setup controller client
        self.controller_client = actionlib.SimpleActionClient('control_motion', ControlAction)
        rospy.loginfo('Waiting for controller action server...')
        self.controller_client.wait_for_server()
        rospy.loginfo('Done!')
        
        # Setup action server
        self.tf_listener = tf.TransformListener(cache_time=rospy.Duration(100.0))
        self.server = actionlib.SimpleActionServer('find_object', FindObjectAction, self.execute, False )
        self.server.start()
        rospy.loginfo('Server started')
      
        
    def normalize(self, angle):
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        return angle
    
    
    def odom_callback(self, msg):
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        orientation = msg.pose.pose.orientation
        _, __, self.odom_theta = tf.transformations.euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        
        
    def segmentation_labels_callback(self, msg):
        self.object_category_mapping = {}
        rospy.logdebug("Object categories mapping:")
        for i in range(len(msg.classes_ids)):
            self.object_category_mapping[msg.classes_ids[i]] = msg.labels[i]
            rospy.logdebug("%d: %s", msg.classes_ids[i], msg.labels[i])
        
    
    def segmentation_callback(self, msg):
        if self.object_category_mapping is None:
            rospy.logwarn('NO OBJECT CATEGORY MAPPING RECEIVED!')
            return
        rospy.logdebug("Msg classes ids: %s", ', '.join([str(x) for x in msg.classes_ids]))
        for i in range(len(msg.classes_ids)):
            if self.object_category_mapping.get(msg.classes_ids[i], '') == self.object_type:
                #print('Object mask:', msg.boxes[i])
                self.object_detected = True


    def control_status_callback(self, msg):
        if self.target_x is None:
            return
        #print(msg.data)
        dst_robot_to_goal = np.sqrt((self.odom_x - self.target_x) ** 2 + (self.odom_y - self.target_y) ** 2)
        if msg.data == 'reached' and dst_robot_to_goal < 0.5:# and abs(angle_diff) < 0.5:# and rospy.Time.now().to_sec() - self.start_time > 10:
            self.goal_reached = True
    
    
    def wait_until_come_or_found(self):
        start_motion_goal = ControlGoal()
        start_motion_goal.goal = 'StartMotion'
        self.controller_client.send_goal(start_motion_goal)
        rospy.loginfo('Send command to controller')
        if self.preempted:
            rospy.logwarn('Server is preempted, skipping')
            return
        start_time = time.time()
        self.goal_reached = False
        while not rospy.is_shutdown():
            current_x_new, current_y_new = self.odom_x, self.odom_y
            if current_x_new is None:
                continue
            if self.server.is_preempt_requested() and not self.preempted:
                rospy.logwarn('SERVER PREEMPTED')
                self.server.set_preempted()
                self.controller_client.stop_tracking_goal()
                self.controller_client.cancel_goal()
                self.preempted = True
                break
            self.feedback.x = current_x_new
            self.feedback.y = current_y_new
            self.server.publish_feedback(self.feedback)
            self.publish_pathplanning_task(current_x_new, current_y_new, self.target_x, self.target_y, self.target_theta)
            goal_reached = self.controller_client.wait_for_result(rospy.Duration(0.05))
            if goal_reached:
                rospy.loginfo('Interm goal reached!')
                break
            #if self.object_detected:
            #    if self.wait_for_tracking():
            #        break
            if self.found:
                rospy.loginfo('Object found!')
                break
        return self.found


    def publish_pathplanning_task(self, start_x, start_y, goal_x, goal_y, goal_theta):
        task = Float32MultiArray()
        task.layout.data_offset = 0
        task.layout.dim.append(MultiArrayDimension())
        task.layout.dim[0].label = "width"
        if goal_theta is None:
            task.layout.dim[0].size  = 4
            task.layout.dim[0].stride  = 4
            task.data = [start_x, start_y, goal_x, goal_y]
        else:
            task.layout.dim[0].size  = 5
            task.layout.dim[0].stride  = 5
            task.data = [start_x, start_y, goal_x, goal_y, goal_theta]
        self.task_publisher.publish(task)
        
        
    def tracking_callback(self, msg):
        if self.object_category_mapping is None:
            rospy.logwarn('NO OBJECT CATEGORY MAPPING RECEIVED!')
            return
        for i in range(msg.num):
            rospy.logdebug('%s is seen', self.object_category_mapping.get(msg.classes_ids[i], ''))
            if self.object_category_mapping.get(msg.classes_ids[i], '') == self.object_type and not self.found:
                rospy.loginfo('Object of type {} is tracked!'.format(self.object_type))
                self.object_x = msg.positions[i].x
                self.object_y = msg.positions[i].y
                self.object_theta = 0
                self.found = True
                break
            
            
    def wait_for_tracking(self):
        start_time = rospy.Time.now().to_sec()
        rospy.loginfo('Object is detected by segmentation, waiting for tracking...')
        self.publish_pathplanning_task(self.odom_x, self.odom_y, self.odom_x, self.odom_y, None)
        while not self.found:
            rospy.sleep(0.05)
            if rospy.Time.now().to_sec() - start_time > 3:
                self.object_detected = False
                break
        rospy.loginfo('Done!')
        return self.object_detected
    
    
    def rotate_360(self):
        if self.preempted:
            rospy.logwarn('Server is preempted, skipping')
            return
        start_theta = self.odom_theta
        final_theta = self.normalize(start_theta - 0.5)
        left_rotation_msg = Twist()
        left_rotation_msg.linear.x = 0
        left_rotation_msg.angular.z = 0.25
        while not self.found and abs(self.normalize(self.odom_theta - final_theta)) > 0.1 and not rospy.is_shutdown():
            if self.server.is_preempt_requested() and not self.preempted:
                rospy.logwarn('SERVER PREEMPTED')
                self.server.set_preempted()
                self.preempted = True
                break
            #if self.object_detected:
            #    if self.wait_for_tracking():
            #        break
            self.cmd_vel_publisher.publish(left_rotation_msg)            
    
    
    def explore_polygon(self, object_type):
        room1_loc1 = [43.8, -6.2]
        room1_loc2 = [42.6, -4.1]
        room2_loc1 = [39.7, -6.0]
        room1_borders = [41.4, -7.3, 45.0, -3.0]
        room2_borders = [38.8, -7.3, 41.4, -3.0]
        while self.odom_x is None:
            rospy.logwarn('NO ODOMETRY!!!')
            rospy.sleep(0.05)
        if self.odom_x > room1_borders[0] and self.odom_x < room1_borders[2] and self.odom_y > room1_borders[1] and self.odom_y < room1_borders[3]:
            rospy.loginfo('Explore room 1, location 1')
            self.target_x, self.target_y = room1_loc1
            self.publish_pathplanning_task(self.odom_x, self.odom_y, self.target_x, self.target_y, None)
            self.wait_until_come_or_found()
            if self.found:
                return True
            self.rotate_360()
            rospy.loginfo('Explore room 1, location 2')
            if not self.found:
                self.target_x, self.target_y = room1_loc2
                self.target_theta = 0
                self.publish_pathplanning_task(self.odom_x, self.odom_y, self.target_x, self.target_y, None)
                self.wait_until_come_or_found()
                if self.found:
                    return True
                self.rotate_360()
            rospy.loginfo('Explore room 2, location 1')
            if not self.found:
                self.target_x, self.target_y = room2_loc1
                self.target_theta = 0
                self.publish_pathplanning_task(self.odom_x, self.odom_y, self.target_x, self.target_y, None)
                self.wait_until_come_or_found()
                if self.found:
                    return True
                self.rotate_360()
        else:
            rospy.loginfo('Explore room 2, location 1')
            self.target_x, self.target_y = room2_loc1
            self.target_theta = 0
            self.publish_pathplanning_task(self.odom_x, self.odom_y, self.target_x, self.target_y, None)
            self.wait_until_come_or_found()
            if self.found:
                return True
            self.rotate_360()
            rospy.loginfo('Explore room 1, location 2')
            if not self.found:
                self.target_x, self.target_y = room1_loc2
                self.target_theta = 0
                self.publish_pathplanning_task(self.odom_x, self.odom_y, self.target_x, self.target_y, None)
                self.wait_until_come_or_found()
                if self.found:
                    return True
                self.rotate_360()
            rospy.loginfo('Explore room 1, location 1')
            if not self.found:
                self.target_x, self.target_y = room1_loc1
                self.target_theta = 0
                self.publish_pathplanning_task(self.odom_x, self.odom_y, self.target_x, self.target_y, None)
                self.wait_until_come_or_found()
                if self.found:
                    return True
                self.rotate_360()
        return self.found
                

    def execute(self, goal):
        print('============================================================\n\n==========================================================================')
        rospy.loginfo('Task received')
        self.goal_reached = False
        self.target_x = None
        self.target_y = None
        self.target_theta = None
        self.object_x = None
        self.object_y = None
        self.found = False
        self.object_detected = False
        self.preempted = False
        self.start_time = rospy.Time.now().to_sec()
        rospy.loginfo('Goal object: %s', goal.object)
        rospy.loginfo('Exploration started')
        self.object_type = goal.object
        self.explore_polygon(goal.object)
        rospy.loginfo('Exploration finished')
        if self.preempted:
            #rospy.logwarn('Server is preempted, kill MPC and return')
            #os.system('rosnode kill mpc_planner')
            return
        success = self.found
        if success:
            #rospy.sleep(6.0)
            self.result.message = 'OK'
            self.result.code = 0
            self.server.set_succeeded(self.result)
        else:
            self.result.message = 'Could not find object of type {}'.format(goal.object)
            self.result.code = 1
            self.server.set_aborted(self.result)


if __name__ == '__main__':
    server = FindObject()
    rospy.spin()
