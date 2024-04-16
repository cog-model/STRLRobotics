#!/usr/bin/python
import rospy
import actionlib
import roslib; roslib.load_manifest('ur_driver')
from robotiq_2f_gripper_msgs.msg import CommandRobotiqGripperFeedback, CommandRobotiqGripperResult, CommandRobotiqGripperAction, CommandRobotiqGripperGoal
from robotiq_2f_gripper_control.robotiq_2f_gripper_driver import Robotiq2FingerGripperDriver as Robotiq
from sensor_msgs.msg import Joy
from control_msgs.msg import *
from trajectory_msgs.msg import *
from sensor_msgs.msg import JointState
from math import pi
import numpy as np
import time


class Gripper(object):
    def __init__(self):
    	# params
    	self.pos_max = 0.085
    	self.pos_min = 0.0
    	self.progress = 0.5

    	self.pos = self.pos_max # open
    	self.speed = 0.10       # 5 sm/sec
        self.force = 0          # 25 Nytons (minimal force)
        self.last_time = None
        self.delta_t = None
        self.alpha = 0.2
        # client
        self.robotiq_client = actionlib.SimpleActionClient('command_robotiq_action', CommandRobotiqGripperAction)
    	self.robotiq_client.wait_for_server()
    	rospy.loginfo("Connected to the gripper server")
    
    def move(self, direction): 
    	# 1 - open; -1 - close
        if self.last_time is None:
            self.last_time = time.time()
            return

        if self.delta_t is None:
            self.delta_t = time.time() - self.last_time
        else:
            self.delta_t = (1 - self.alpha) * self.delta_t + self.alpha * (time.time() - self.last_time)
        self.last_time = time.time()

        speed = self.speed * self.progress
        change_pos = direction * self.speed * self.delta_t 
        self.pos = np.clip(self.pos + change_pos, self.pos_min, self.pos_max)
        Robotiq.goto(self.robotiq_client, pos=self.pos, speed=speed, force=self.force, block=False)


class Manipulator(object):
    def __init__(self):
        # params
        self.JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        parameters = rospy.get_param(None)
        index = str(parameters).find('prefix')
        if (index > 0):
            prefix = str(parameters)[index+len("prefix': '"):(index+len("prefix': '")+str(parameters)[index+len("prefix': '"):-1].find("'"))]
            for i, name in enumerate(self.JOINT_NAMES):
                self.JOINT_NAMES[i] = prefix + name

        self.position = None # [pi/2, 0, -pi/2, -pi/2, -pi/2, 0]
        self.velocities = [0, 0, 0, 0, 0, 0]
        self.angle_speed = pi / 2 / 10  # 90 degree per 5 seconds (max)
        self.progress = 0.5

        self.last_time = None
        self.delta_t = None
        self.alpha = 0.2
        # client
        self.client = actionlib.SimpleActionClient('follow_joint_trajectory', FollowJointTrajectoryAction)
        self.client.wait_for_server()
        rospy.loginfo("Connected to the manipulator server")

    def move(self, base_dir, shoulder_dir, elbow_dir, wrist1_dir, wrist2_dir, wrist3_dir):
        if self.last_time is None:
            self.last_time = time.time()
            return

        if self.delta_t is None:
            self.delta_t = time.time() - self.last_time
        else:
            self.delta_t = (1 - self.alpha) * self.delta_t + self.alpha * (time.time() - self.last_time)
        self.last_time = time.time()

        g = FollowJointTrajectoryGoal()
        g.trajectory = JointTrajectory()
        g.trajectory.joint_names = self.JOINT_NAMES
        
        if self.position is None:
            joint_states = None
            while (joint_states is None) or (joint_states.name != self.JOINT_NAMES):
                joint_states = rospy.wait_for_message("joint_states", JointState)
            self.position = joint_states.position

        velocities = 6 * [self.angle_speed * self.progress]
        change_position = self.angle_speed * self.delta_t * np.array([base_dir, shoulder_dir, elbow_dir, wrist1_dir, wrist2_dir, wrist3_dir])
        new_position = list(np.array(self.position) + change_position)
        print new_position
        g.trajectory.points = [
            JointTrajectoryPoint(positions=self.position, velocities=[0]*6, time_from_start=rospy.Duration(0)),
            JointTrajectoryPoint(positions=new_position, velocities=[0]*6, time_from_start=rospy.Duration(0.5))
        ]
        self.position = new_position
        self.client.send_goal(g)
        self.client.wait_for_result()
        #time.sleep(0.1)


gripper = None
manipulator = None

def joystick_cb(data):
    global gripper, manipulator
    # gripper
    direction = data.axes[7]    # 1 - open; -1 - close
    gripper.move(direction)
    # manipulator
    #base_dir     = data.axes[3]
    #shoulder_dir = data.axes[4]
    #elbow_dir    = data.buttons[2] - data.buttons[0]
    #wrist1_dir   = data.axes[6]
    #wrist2_dir   = data.buttons[3] - data.buttons[1]
    #wrist3_dir   = (data.axes[2] - data.axes[5]) / 2
    # print base_dir, shoulder_dir, elbow_dir, wrist1_dir, wrist2_dir, wrist3_dir
    #manipulator.move(base_dir, shoulder_dir, elbow_dir, wrist1_dir, wrist2_dir, wrist3_dir)


if __name__ == '__main__':
    rospy.init_node("gripper_controller")
    gripper = Gripper()
    # manipulator = Manipulator()
    rospy.Subscriber("/joy_teleop/joy", Joy, joystick_cb, queue_size=1)
    rospy.spin()
