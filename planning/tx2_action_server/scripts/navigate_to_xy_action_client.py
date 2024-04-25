#! /usr/bin/env python3
import rospy
import actionlib
from communication_msgs.msg import MoveToPointWithOrientationAction, MoveToPointWithOrientationActionGoal

if __name__ == '__main__':
	rospy.init_node('navigate_to_xy_action_client')
	x = rospy.get_param('~x', 0)
	y = rospy.get_param('~y', 0)
	theta = rospy.get_param('~theta', 0)
	client = actionlib.SimpleActionClient('relative_move', MoveToPointWithOrientationAction)
	client.wait_for_server()
	goal = MoveToPointWithOrientationActionGoal()
	goal.goal.x = x
	goal.goal.y = y
	client.send_goal(goal.goal)
	client.wait_for_result()
	print('Result:', client.get_result())
