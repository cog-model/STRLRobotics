#! /usr/bin/env python3
import rospy
import tf
import math
import time
import actionlib
import numpy as np
from std_msgs.msg import Float32MultiArray, Float64MultiArray, MultiArrayDimension, Bool, String
from communication_msgs.msg import MoveToPointWithOrientationAction, MoveToPointWithOrientationActionGoal, MoveToPointWithOrientationActionFeedback, MoveToPointWithOrientationActionResult
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
        self.goal = MoveToPointWithOrientationActionGoal()
        self.feedback = MoveToPointWithOrientationActionFeedback()
        self.feedback_relative = MoveToPointWithOrientationActionFeedback()
        self.result = MoveToPointWithOrientationActionResult()
        self.result_relative = MoveToPointWithOrientationActionResult()
        global_frame = rospy.get_param('~global_frame')
        tolerance = rospy.get_param('~tolerance', DEFAULT_TOLERANCE)
        rate = rospy.get_param('~rate', DEFAULT_RATE)
        timeout = rospy.get_param('~timeout', DEFAULT_TIMEOUT)
        max_path_fails = rospy.get_param('~max_path_fails', DEFAULT_N_FAILS)
        print('TOLERANCE IS', tolerance)
        self.control_status_subscriber = rospy.Subscriber('control_status', String, self.control_status_callback)
        self.relative_motion_status_subscriber = rospy.Subscriber('relative_motion_status', String, self.relative_motion_status_callback)
        self.door_subscriber = rospy.Subscriber('door_coord', Float32MultiArray, self.door_callback)
        self.task_publisher = rospy.Publisher('/task', Float32MultiArray, queue_size=1)
        self.relative_task_publisher = rospy.Publisher('/relative_motion', Float64MultiArray, queue_size=1)
        self.global_frame = global_frame
        self.tolerance = tolerance
        self.rate = rospy.Rate(rate)
        self.timeout = timeout
        self.goal_reached = False
        self.door = None
        self.max_path_fails = max_path_fails
        self.tf_listener = tf.TransformListener()
        self.server = actionlib.SimpleActionServer('move_to_point_with_orientation', MoveToPointWithOrientationAction, self.execute, False )
        self.server_relative = actionlib.SimpleActionServer('relative_move', MoveToPointWithOrientationAction, self.execute_relative, False )
        self.server.start()
        self.server_relative.start()
        
    def normalize(self, angle):
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        return angle

    def get_robot_pose(self):
        try:
            pos, quat = self.tf_listener.lookupTransform(
                                self.global_frame, 'base_link',
                                self.tf_listener.getLatestCommonTime(self.global_frame,
                                'base_link'))
        except:
            print('NO TRANSFORM FROM ODOM TO BASE LINK!!!!')
            return None, None
        current_x, current_y, _ = pos
        return current_x, current_y
    
    def get_robot_orientation(self):
        try:
            pos, quat = self.tf_listener.lookupTransform(
                                self.global_frame, 'base_link',
                                self.tf_listener.getLatestCommonTime(self.global_frame,
                                'base_link'))
        except:
            print('NO TRANSFORM FROM ODOM TO BASE LINK!!!!')
            return None, None
        _, __, theta = tf.transformations.euler_from_quaternion(quat)
        return theta

    def control_status_callback(self, msg):
        if msg.data == 'reached':
            self.goal_reached = True
            
    def relative_motion_status_callback(self, msg):
        if msg.data == 'done':
            self.relative_goal_reached = True

    def door_callback(self, msg):
        self.door = msg.data

    def wait_until_come(self):
        start_time = time.time()
        n_path_fails = 0
        succeeded = False
        while time.time() - start_time < self.timeout and not rospy.is_shutdown():
            current_x_new, current_y_new = self.get_robot_pose()
            if current_x_new is None:
                continue
            if self.server.is_preempt_requested():
                print('SERVER PREEMPTED')
                break
            self.feedback.feedback.x = current_x_new
            self.feedback.feedback.y = current_y_new
            self.server.publish_feedback(self.feedback.feedback)
            self.publish_pathplanning_task(current_x_new, current_y_new, self.target_x, self.target_y, self.target_theta)
            #dst_to_goal = math.sqrt((current_x_new -  self.target_x) ** 2 + (current_y_new - self.target_y) ** 2)
            # if we reach the goal, finish with success
            #if dst_to_goal < self.tolerance:
            if self.goal_reached:
                print('Goal reached!')
                succeeded = True
                break
            self.rate.sleep()
        if time.time() - start_time > self.timeout and not succeeded:
            print('Goal timed out!')
        return succeeded
    
    def wait_until_come_relative(self, x, y, rel_theta):
        start_time = time.time()
        n_path_fails = 0
        succeeded = False
        current_robot_theta = self.get_robot_orientation()
        d_theta = np.arctan2(y, x)
        if abs(d_theta) > np.pi / 2:
            d_theta = self.normalize(d_theta + np.pi)
            x *= -1
            y *= -1
        theta1 = current_robot_theta + d_theta
        dx = np.sqrt(x ** 2 + y ** 2)
        theta2 = current_robot_theta + rel_theta
        self.publish_relative_motion_task((x < 0), theta1, dx, theta2)
        while time.time() - start_time < self.timeout and not rospy.is_shutdown():
            current_x_new, current_y_new = self.get_robot_pose()
            if current_x_new is None:
                continue
            if self.server.is_preempt_requested():
                print('SERVER PREEMPTED')
                break
            self.feedback_relative.feedback.x = current_x_new
            self.feedback_relative.feedback.y = current_y_new
            self.server_relative.publish_feedback(self.feedback_relative.feedback)
            #dst_to_goal = math.sqrt((current_x_new -  self.target_x) ** 2 + (current_y_new - self.target_y) ** 2)
            # if we reach the goal, finish with success
            #if dst_to_goal < self.tolerance:
            if self.relative_goal_reached:
                print('Goal reached!')
                succeeded = True
                break
            self.rate.sleep()
        if time.time() - start_time > self.timeout and not succeeded:
            print('Goal timed out!')
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
        
    def publish_relative_motion_task(self, backward_motion, theta1, dx, theta2):
        task = Float64MultiArray()
        task.layout.data_offset = 0
        task.layout.dim.append(MultiArrayDimension())
        task.layout.dim[0].label = "width"
        task.layout.dim[0].size  = 4
        task.layout.dim[0].stride  = 4
        task.data = [float(backward_motion), theta1, dx, theta2]
        self.relative_task_publisher.publish(task)

    def execute(self, goal):
        print('Execute')
        self.goal_reached = False
        self.door = None
        self.target_x = goal.x
        self.target_y = goal.y
        if goal.use_orientation:
            self.target_theta = goal.theta
        else:
            self.target_theta = None
        success = self.wait_until_come()
        if self.door is None:
            self.result.result.type = 'finished'
        else:
            self.result.result.type = 'door_closed'
            self.result.result.x1 = self.door[0]
            self.result.result.y1 = self.door[1]
            self.result.result.x2 = self.door[2]
            self.result.result.y2 = self.door[3]
        if success:
            self.server.set_succeeded(self.result.result)
        else:
            self.server.set_preempted()
            
    def execute_relative(self, goal):
        print('Execute relative')
        self.relative_goal_reached = False
        success = self.wait_until_come_relative(goal.x, goal.y, goal.theta)
        self.result_relative.result.type = 'finished'
        if success:
            self.server_relative.set_succeeded(self.result_relative.result)
        else:
            self.server_relative.set_preempted()


if __name__ == '__main__':
    server = MoveRobot()
    rospy.spin()
