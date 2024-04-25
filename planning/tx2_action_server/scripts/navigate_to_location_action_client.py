#! /usr/bin/env python3
import rospy
import actionlib
from communication_msgs.msg import MoveToAction, MoveToActionGoal

if __name__ == '__main__':
    rospy.init_node('navigate_to_xy_action_client')
    obj = rospy.get_param('~object', '')
    location = rospy.get_param('~location', 'box')
    client = actionlib.SimpleActionClient('move_to_location', MoveToAction)
    client.wait_for_server()
    goal = MoveToActionGoal()
    goal.goal.object = obj
    goal.goal.location = location
    client.send_goal(goal.goal)
    client.wait_for_result()
    print('Result:', client.get_result())
