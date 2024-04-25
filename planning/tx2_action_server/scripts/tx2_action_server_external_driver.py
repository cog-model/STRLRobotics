#! /usr/bin/env python3

import roslib
import rospy
import actionlib
import tf
import math
import time

from geometry_msgs.msg import PoseStamped, Point
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionGoal
from communication_msgs.msg import MoveToPointAction, MoveToPointActionGoal, MoveToPointActionFeedback, MoveToPointActionResult
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, String, Bool
from copy import copy

current_pose = []
path = []

DEFAULT_TOLERANCE = 1
DEFAULT_RATE = 10
DEFAULT_TIMEOUT = 30
DEFAULT_N_FAILS = 5


class MoveRobot:
    def __init__(self):
        
        rospy.init_node('tx2_action_server')
        tolerance = rospy.get_param('~tolerance', DEFAULT_TOLERANCE)
        rate = rospy.get_param('~rate', DEFAULT_RATE)
        timeout = rospy.get_param('~timeout', DEFAULT_TIMEOUT)
        max_path_fails = rospy.get_param('~max_path_fails', DEFAULT_N_FAILS)
        print('TOLERANCE IS', tolerance)
        self.server = actionlib.SimpleActionServer('move_to_point', MoveToPointAction, self.execute, False )
        print('SERVER CREATED')
        self.server.start()
        print('SERVER STARTED')
        self.goal_publisher = rospy.Publisher('/exploration_goal', PoseStamped, queue_size=5)
        self.result_publisher = rospy.Publisher('/goal_nav_result', String, queue_size=5)
        self.task_publisher = rospy.Publisher('/task', Float32MultiArray, queue_size=5)
        self.path_subscriber = rospy.Subscriber('/path', Path, self.path_callback)
        self.goal_marker_subscriber = rospy.Subscriber('/explore/objectgoal_is_found', Bool, self.goal_marker_callback)
        self.tolerance = tolerance
        self.rate = rospy.Rate(rate)
        self.timeout = timeout
        self.max_path_fails = max_path_fails
        self.current_pose = None
        self.goal_detected = False
        self.path = []
        self.path_received = False
        self.tf_listener = tf.TransformListener()

    def path_callback(self, msg):
        self.path_received = True
        self.path = [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses]

    def goal_marker_callback(self, msg):
        self.goal_detected = msg.data

    def get_robot_pose(self):
        try:
            pos, quat = self.tf_listener.lookupTransform(
                                'local_map_lidar', 'base_link',
                                self.tf_listener.getLatestCommonTime('local_map_lidar',
                                'base_link'))
        except:
            print('NO TRANSFORM FROM ODOM TO BASE LINK!!!!')
            return None, None
        current_x, current_y, _ = pos
        return current_x, current_y

    def wait_until_come(self, target_x, target_y):
        start_time = time.time()
        self.path_received = False
        n_path_fails = 0
        succeeded = False
        goal_detected = self.goal_detected
        while time.time() - start_time < self.timeout:
            if self.goal_detected and not goal_detected:
                print('Dropping current goal: semantic goal is appeared!')
                break
            goal_detected = self.goal_detected
            current_x_new, current_y_new = self.get_robot_pose()
            target_pose = PoseStamped()
            target_pose.header.stamp = rospy.Time.now()
            target_pose.header.frame_id = 'local_map_lidar'
            target_pose.pose.position.x = self.goal.x
            target_pose.pose.position.y = self.goal.y
            self.goal_publisher.publish(target_pose)
            self.publish_pathplanning_task(current_x_new, current_y_new, target_x, target_y)
            dst_to_goal = math.sqrt((current_x_new -  target_x) ** 2 + (current_y_new - target_y) ** 2)
            # if we reach the goal, finish with success
            if dst_to_goal < self.tolerance:
                print('Goal reached!')
                succeeded = True
                break
            self.rate.sleep()
            # if path to goal not found, finish without success
            if self.path_received and len(self.path) == 0:
                n_path_fails += 1
                self.path_received = False
            if n_path_fails >= self.max_path_fails:
                print('Goal unavailable: path not found!')
                break
        if time.time() - start_time > self.timeout and not succeeded:
            print('Goal timed out!')
            succeeded = True
        return succeeded

    def publish_pathplanning_task(self, start_x, start_y, goal_x, goal_y):
        task = Float32MultiArray()
        task.layout.data_offset = 0
        task.layout.dim.append(MultiArrayDimension())
        task.layout.dim[0].label = "width"
        task.layout.dim[0].size  = 4
        task.layout.dim[0].stride  = 4
        #print(start_x, start_y, goal_x, goal_y)
        task.data = [start_x, start_y, goal_x, goal_y]
        self.task_publisher.publish(task)

    def execute( self, goal ):
        self.goal = goal
        #pose = goal.target_pose
        target_x, target_y = goal.x, goal.y
        print( "Recieved goal: {}, {}".format(target_x, target_y))
        target_pose = PoseStamped()
        target_pose.header.stamp = rospy.Time.now()
        target_pose.header.frame_id = 'local_map_lidar'
        target_pose.pose.position.x = target_x
        target_pose.pose.position.y = target_y
        self.goal_publisher.publish(target_pose)
        goal_reached = self.wait_until_come(target_x, target_y)
        result_msg = String()
        if goal_reached:
            print('SET SUCCEEDED')
            self.server.set_succeeded()
            result_msg.data = "SUCCEEDED"
        else:
            print('SET ABORTED')
            self.server.set_aborted()
            result_msg.data = "ABORTED"
        self.result_publisher.publish(result_msg)

if __name__ == '__main__':
    server = MoveRobot()
    rospy.spin()
