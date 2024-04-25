#! /usr/bin/env python3

import roslib
import rospy
import actionlib
import tf
import math
import time

from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
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
        self.task_publisher = rospy.Publisher('/task', Float32MultiArray, queue_size=5)
        self.tolerance = tolerance
        self.rate = rospy.Rate(rate)
        self.timeout = timeout
        self.max_path_fails = max_path_fails
        self.current_pose = None
        self.path = []
        self.goal_position = None
        self.path_received = False
        self.tf_listener = tf.TransformListener()
        self.goal_subscriber = rospy.Subscriber('move_base_simple/goal', PoseStamped, self.goal_callback)
        self.odom_subscriber = rospy.Subscriber('/cartographer/tracked_global_odometry', Odometry, self.odom_callback)
        self.path_subscriber = rospy.Subscriber('path', Path, self.path_callback)

    def path_callback(self, msg):
        self.path_received = True
        self.path = msg.poses

    def get_robot_pose(self):
        if self.current_pose is None:
            print('NO ODOMETRY!!!')
            return None, None
        _, __, angle = tf.transformations.euler_from_quaternion(quat)
        current_x, current_y = self.current_pose.position.x, self.current_pose.position.y
        current_x_new = current_x * math.cos(-angle) + current_y * math.sin(-angle)
        current_y_new = -current_x * math.sin(-angle) + current_y * math.cos(-angle)
        return current_x_new, current_y_new

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose
        current_x_new, current_y_new = self.current_pose.position.x, self.current_pose.position.y
        if self.goal_position is None:
            print('No goal received')
            return
        target_x, target_y = self.goal_position
        target_theta = self.goal_orientation
        self.publish_pathplanning_task(current_x_new, current_y_new, target_x, target_y, target_theta)

    def publish_pathplanning_task(self, start_x, start_y, goal_x, goal_y, goal_theta):
        task = Float32MultiArray()
        task.layout.data_offset = 0;
        task.layout.dim.append(MultiArrayDimension())
        task.layout.dim[0].label = "width"
        task.layout.dim[0].size  = 5
        task.layout.dim[0].stride  = 5
        task.data = [start_x, start_y, goal_x, goal_y, goal_theta]
        self.task_publisher.publish(task)

    def goal_callback( self, msg ):
        target_x, target_y = msg.pose.position.x, msg.pose.position.y
        _, __, target_theta = tf.transformations.euler_from_quaternion([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
        self.goal_position = (target_x, target_y)
        print('TARGET THETA:', target_theta)
        self.goal_orientation = target_theta
        print( "Recieved goal: {}, {}".format(target_x, target_y))

if __name__ == '__main__':
    server = MoveRobot()
    rospy.spin()
