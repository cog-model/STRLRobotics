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
from communication_msgs.msg import MoveToPointWithOrientationAction, MoveToPointWithOrientationGoal, \
                                   MoveToPointWithOrientationFeedback, MoveToPointWithOrientationResult, \
                                   MoveToAction, MoveToGoal, MoveToFeedback, MoveToResult
from control_mobile_robot.msg import ControlAction, ControlGoal, ControlFeedback, ControlResult
from geometry_msgs.msg import PoseStamped, Twist
from husky_tidy_bot_cv.msg import Objects, Objects3d, Categories
from visualization_msgs.msg import Marker
import yaml
current_pose = []
path = []
DEFAULT_RATE = 10
DEFAULT_TIMEOUT = 30
DEFAULT_N_FAILS = 5


class MoveRobot:
    def __init__(self):
        # Setup feedback and result
        rospy.init_node('tx2_action_server')#, log_level=rospy.DEBUG)
        self.goal = MoveToGoal()
        self.feedback = MoveToFeedback()
        self.result = MoveToResult()
        
        # Init ROS params
        global_frame = rospy.get_param('~global_frame')
        rate = rospy.get_param('~rate', DEFAULT_RATE)
        timeout = rospy.get_param('~timeout', DEFAULT_TIMEOUT)
        
        # Load location list and object category mapping
        locations_file = '/home/administrator/pointnav_ws/src/tx2_action_server/scripts/objects.yaml'
        with open(locations_file, 'r') as fd:
            self.locations = yaml.safe_load(fd)
        rospy.loginfo('Extracted locations from file %s', '/home/administrator/pointnav_ws/src/tx2_action_server/scripts/objects.yaml')
        for location, coord in self.locations.items():
            if len(coord) == 4:
                x, y, _, theta = coord
            else:
                x, y, theta = coord
            rospy.loginfo("%s: %f %f %f", location, x, y, theta)
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
        self.odom_subscriber = rospy.Subscriber('/cartographer/tracked_global_odometry', Odometry, self.odom_callback)
        self.path_subscriber = rospy.Subscriber('/path', Path, self.path_callback)
        #self.segmentation_subscriber = rospy.Subscriber('/segmentation', Objects, self.segmentation_callback)
        self.tracked_objects_subscriber = rospy.Subscriber('/tracked_objects_3d', Objects3d, self.tracking_callback)
        self.segmentation_labels_subscriber = rospy.Subscriber('/segmentation_labels', Categories, self.segmentation_labels_callback)
        #self.object_pose_subscriber = rospy.Subscriber('object_pose', PoseStamped, self.object_pose_callback)
        self.task_publisher = rospy.Publisher('/task', Float32MultiArray, queue_size=1)
        
        # Init variables
        self.global_frame = global_frame
        self.rate = rospy.Rate(rate)
        self.timeout = timeout
        self.goal_reached = False
        self.object_x = None
        self.object_y = None
        self.object_theta = None
        self.odom_x = None
        self.odom_y = None
        self.target_x = None
        self.target_y = None
        self.target_theta = None
        self.path = None
        self.object_type = None
        self.found = False
        self.object_detected = False
        self.tracking_refreshed = False
        self.preempted = False
        self.start_time = 0
        
        # Setup controller client
        self.controller_client = actionlib.SimpleActionClient('control_motion', ControlAction)
        rospy.loginfo('Waiting for controller action server...')
        self.controller_client.wait_for_server()
        rospy.loginfo('Done!')
        
        # Setup action server
        self.tf_listener = tf.TransformListener(cache_time=rospy.Duration(100.0))
        self.server = actionlib.SimpleActionServer('move_to_location', MoveToAction, self.execute, False )
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
     
        
    def path_callback(self, msg):
        if len(msg.poses) == 0:
            rospy.logwarn('Received empty path!')
            self.result.code = 2
            self.result.message = 'No path to object {} with coords ({}, {})'.format(self.object_type, self.object_x, self.object_y)
            return
        self.path = [[pose.pose.position.x, pose.pose.position.y] for pose in msg.poses]
        self.path = [[self.odom_x, self.odom_y]] + self.path
        
    
    def segmentation_callback(self, msg):
        if self.object_category_mapping is None:
            rospy.logwarn('NO OBJECT CATEGORY MAPPING RECEIVED!')
            return
        rospy.logdebug("Msg classes ids: %s", ', '.join([str(x) for x in msg.classes_ids]))
        for i in range(len(msg.classes_ids)):
            if self.object_category_mapping.get(msg.classes_ids[i], '') == self.object_type:
                #print('Object mask:', msg.boxes[i])
                self.object_detected = True


    def wait_until_come(self):
        start_time = time.time()
        n_path_fails = 0
        succeeded = False
        self.goal_reached = False
        self.result.code = 2
        self.result.message = 'Timeout reached'
        while time.time() - start_time < self.timeout and not rospy.is_shutdown():
            current_x_new, current_y_new = self.odom_x, self.odom_y
            if current_x_new is None:
                continue
            if self.server.is_preempt_requested() and not self.preempted:
                rospy.logwarn('SERVER PREEMPTED')
                self.server.set_preempted()
                self.controller_client.cancel_goal()
                self.preempted = True
                break
            self.feedback.x = current_x_new
            self.feedback.y = current_y_new
            self.server.publish_feedback(self.feedback)
            self.publish_pathplanning_task(current_x_new, current_y_new, self.target_x, self.target_y, self.target_theta)
            goal_reached = self.controller_client.wait_for_result(rospy.Duration(0.05))
            print('Goal reached:', goal_reached)
            # if we reach the goal, finish with success
            if goal_reached:
                rospy.loginfo('Goal reached!')
                self.result.message = 'OK'
                self.result.code = 0
                succeeded = True
                break
        if time.time() - start_time > self.timeout and not succeeded:
            rospy.logwarn('Goal timed out!')
        return succeeded


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
        if not self.tracking_refreshed:
            rospy.loginfo('Tracking output:')
        for i in range(msg.num):
            if not self.tracking_refreshed:
                rospy.loginfo('%s with coords (%f, %f)', self.object_category_mapping.get(msg.classes_ids[i]), 
                            msg.positions[i].x, msg.positions[i].y)
            #print(self.object_category_mapping.get(msg.classes_ids[i], ''), self.object_type)
            if self.object_category_mapping.get(msg.classes_ids[i], '') == self.object_type and not self.found:
                rospy.loginfo('Object is tracked!')
                self.object_x = msg.positions[i].x
                self.object_y = msg.positions[i].y
                self.object_theta = 0
                self.found = True
                break
        self.tracking_refreshed = True
            
            
    def wait_for_path(self):
        print('Object x:', self.target_x)
        print('Object y:', self.target_y)
        print('Path:', self.path)
        while self.path is None or abs(self.path[-1][0] - self.target_x) > 0.25 or abs(self.path[-1][1] - self.target_y) > 0.25:
            #self.publish_pathplanning_task(self.odom_x, self.odom_y, self.object_x, self.object_y, None)
            print('Object x:', self.target_x)
            print('Object y:', self.target_y)
            print('Path:', self.path)
            rospy.sleep(0.05)
            
            
    def wait_for_tracking(self):
        start_time = rospy.Time.now().to_sec()
        while not self.tracking_refreshed:
            rospy.sleep(0.05)
            if rospy.Time.now().to_sec() - start_time > 5:
                break
            
            
    def get_point_on_path(self, path, object_reach_distance):
        i = len(path) - 1
        length = 0
        while i > 0:
            segment = np.sqrt((path[i][0] - path[i - 1][0]) ** 2 + (path[i][1] - path[i - 1][1]) ** 2)
            if length + segment >= object_reach_distance:
                alpha = (object_reach_distance - length) / segment
                x = alpha * path[i - 1][0] + (1 - alpha) * path[i][0]
                y = alpha * path[i - 1][1] + (1 - alpha) * path[i][1]
                theta = np.arctan2(path[i][1] - path[i - 1][1], path[i][0] - path[i - 1][0])
                return x, y, theta
            i -= 1
            length += segment
        return self.odom_x, self.odom_y, np.arctan2(path[1][1] - self.odom_y, path[1][0] - self.odom_x)
                

    def execute(self, goal):
        print('============================================================\n\n==========================================================================')
        rospy.loginfo('Task received')
        self.goal_reached = False
        self.path = None
        self.target_x = None
        self.target_y = None
        self.target_theta = None
        self.object_x = None
        self.object_y = None
        self.found = False
        self.object_detected = False
        self.preempted = False
        self.start_time = rospy.Time.now().to_sec()
        start_motion_goal = ControlGoal()
        start_motion_goal.goal = 'StartMotion'
        rospy.loginfo('Goal object: %s', goal.object)
        rospy.loginfo('Goal location: %s', goal.location)
        self.object_type = goal.object
        if goal.location in self.locations:
            self.target_x, self.target_y, _, self.target_theta = self.locations[goal.location]
            #rospy.loginfo('Move to location {} ({}, {})'.format(goal.location, self.target_x, self.target_y))
            #self.controller_client.send_goal(start_motion_goal)
            #rospy.loginfo('Send command to controller')
            #success = self.wait_until_come()
        else:
            rospy.loginfo('Waiting for tracking...')
            self.tracking_refreshed = False
            self.wait_for_tracking()
            rospy.loginfo('Done!')
            if self.found:
                while self.odom_x is None:
                    rospy.logwarn('NO ODOMETRY!!!')
                    rospy.sleep(0.05)
                rospy.loginfo('Object x and y: %f %f', self.object_x, self.object_y)
                self.publish_pathplanning_task(self.odom_x, self.odom_y, self.object_x, self.object_y, 0)
                rospy.loginfo('Waiting for path to the object...')
                self.target_x = self.object_x
                self.target_y = self.object_y
                self.wait_for_path()
                rospy.loginfo('Done!')
                object_reach_distance = 0.8
                self.target_x, self.target_y, self.target_theta = self.get_point_on_path(self.path, object_reach_distance)
            else:
                rospy.logwarn('OBJECT NOT FOUND!')
                self.result.message = 'Object of type {} is not tracked yet'.format(goal.object)
                self.result.code = 1
                self.server.set_aborted(self.result)
                return
        rospy.loginfo('Moving to target pose: %f %f %f', self.target_x, self.target_y, self.target_theta)
        self.publish_pathplanning_task(self.odom_x, self.odom_y, self.target_x, self.target_y, self.target_theta)
        rospy.loginfo('Waiting for path...')
        self.wait_for_path()
        rospy.loginfo('Done!')
        rospy.sleep(2.0)
        self.controller_client.send_goal(start_motion_goal)
        rospy.loginfo('Send command to controller')
        success = self.wait_until_come()
        rospy.loginfo("Reaching success: %d", success)
        if self.preempted:
            #rospy.logwarn('Server is preempted,  kill MPC and return')
            #os.system('rosnode kill mpc_planner')
            return
        if success:
            #rospy.sleep(6.0)
            self.server.set_succeeded(self.result)
        else:
            self.server.set_aborted(self.result)

if __name__ == '__main__':
    server = MoveRobot()
    rospy.spin()
