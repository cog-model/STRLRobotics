import rospy
try:
    import actionlib
    from robotiq_2f_gripper_msgs.msg import (CommandRobotiqGripperFeedback, #type: ignore
                                             CommandRobotiqGripperResult, 
                                             CommandRobotiqGripperAction, 
                                             CommandRobotiqGripperGoal) 
    from robotiq_2f_gripper_control.robotiq_2f_gripper_driver import Robotiq2FingerGripperDriver as Robotiq #type: ignore
except ImportError:
    rospy.logwarn("robotiq import error")

class HuskyGripper:

    def __init__(self) -> None:
        try:
            self._robotiq_client = actionlib.SimpleActionClient(
                'command_robotiq_action', CommandRobotiqGripperAction)
            rospy.loginfo("GRIPPER CONNECTED")
        except:
            self._robotiq_client = None
    # gripper
    def CloseGripper(self, dist = 0.0):
        if self._robotiq_client is not None:
            self._robotiq_client.wait_for_server()
            Robotiq.goto(self._robotiq_client, pos=dist,
                         speed=1, force=1, block=True)

    def OpenGripper(self):
        if self._robotiq_client is not None:
            self._robotiq_client.wait_for_server()
            Robotiq.goto(self._robotiq_client, pos=1,
                         speed=0.1, force=1, block=True)