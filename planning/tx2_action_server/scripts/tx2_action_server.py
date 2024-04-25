#! /usr/bin/env python

import roslib
import rospy
import actionlib
import tf
import math

import turtlebot_mover as tm

from geometry_msgs.msg import PoseStamped, Point
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionGoal
from nav_msgs.msg import Path
from copy import copy

global trajectory
trajectory = []

def callback(data):
    trajectory.append(data.poses[-1].pose)

def get_euler_angles(pose):
        orientation = pose.orientation
        current_euler_angles = tf.transformations.euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        cur_x, cur_y, cur_z = current_euler_angles
        return cur_x, cur_y, cur_z

class MoveRobot:
    def __init__(self):

        self.tm_mover = tm.TurtlebotMover(name='mobile_base', ground_z=3.0, max_angular_speed=0.2, trajectory_type='line')
        self.server = actionlib.SimpleActionServer( 'move_base', MoveBaseAction, self.execute, False )
        self.server.start()
        initial_state = self.tm_mover.state
        self.start_x, self.start_y = initial_state.pose.position.x, initial_state.pose.position.y
        orientation = initial_state.pose.orientation
        orientation_euler = tf.transformations.euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        x_angle, y_angle, z_angle = orientation_euler
        self.start_angle = z_angle
        self.scale = 1.
        self.goal_publisher = rospy.Publisher('/exploration_goal', PoseStamped, queue_size=5)
        self.Subscriber = rospy.Subscriber('mapPath', Path, callback)

    def transform(self, x, y):
        new_x = x * math.cos(self.start_angle) - y * math.sin(self.start_angle) + self.start_x
        new_y = x * math.sin(self.start_angle) + y * math.cos(self.start_angle) + self.start_y
        return new_x, new_y

    def execute( self, goal ):

        current_x = self.tm_mover.state.pose.position.x
        current_y = self.tm_mover.state.pose.position.y
        if len(trajectory) == 0:
            print('I CANNOT FIND MAPPATH TOPIC!!!')
            current_localization_x = current_x
            current_localization_y = current_y
        else:
            current_localization_x = trajectory[-1].position.x
            current_localization_y = trajectory[-1].position.y
        pose = goal.target_pose.pose
        local_target_x, local_target_y = pose.position.x, pose.position.y
        exploration_goal = (local_target_x, local_target_y)
        global_target_x, global_target_y = self.transform(local_target_x, local_target_y)
        global_x_estimated, global_y_estimated = self.transform(current_localization_x, current_localization_y)
        global_x_corrected = (global_target_x - global_x_estimated) * self.scale + current_x
        global_y_corrected = (global_target_y - global_y_estimated) * self.scale + current_y
        print('Current true position:', current_x, current_y)
        print('Current estimated position:', global_x_estimated, global_y_estimated)
        print('Target position:', global_target_x, global_target_y)
        print('Corrected target position:', global_x_corrected, global_y_corrected)
        print('Error:', math.sqrt((current_x - global_x_estimated) ** 2 + (current_y - global_y_estimated) ** 2))
        print( "Recieved goal" )
        print('In local coordinate system: ({}, {})'.format(local_target_x, local_target_y))
        print('In global coordinate system: ({}, {})'.format(global_x_corrected, global_y_corrected))
        self.goal_publisher.publish(goal.target_pose)
        self.tm_mover.move_to( global_x_corrected, global_y_corrected, exploration_goal)
        self.server.set_succeeded()

if __name__ == '__main__':
    #rospy.init_node('tx2_action_server')
    server = MoveRobot()
    rospy.spin()
