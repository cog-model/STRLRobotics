#!/usr/bin/python3
import rospy
rospy.init_node('test_node', log_level=rospy.FATAL)
import numpy as np
import unittest
from unittest.mock import MagicMock, create_autospec, patch
import rtde_control 
import rtde_receive 
rtde_c_mock = create_autospec(rtde_control)
rtde_r_mock = create_autospec(rtde_receive)
rtde_r_mock.RTDEReceiveInterface.getActualTCPPose.side_effect = lambda : [0]*6
with patch.dict('sys.modules', {'rtde_control' : rtde_c_mock, 'rtde_receive' : rtde_r_mock}):
    from skill_bot.robot.husky_ur import HuskyUr
    from skill_bot.robot.husky import Robot
    from skill_bot.robot.husky_base import HuskyBase
    from skill_bot.robot.husky_gripper import HuskyGripper

class UrRobotTestCase(unittest.TestCase):
    def setUp(self) -> None:
        self.robot = HuskyUr('MOCK')
        self.robot._rtde_c : MagicMock = self.robot._rtde_c
        self.robot._rtde_r : MagicMock = self.robot._rtde_r
        
        self.robot._rtde_r.getActualTCPPose.side_effect = lambda : [0]*6
        # self.robot._rtde_r.getActualQ.side_effect = [[0]*6]
        
    def test_mock(self):
        from collections.abc import Iterable
        self.assertIsInstance(self.robot.GetActualQ(), MagicMock)
        self.assertIsInstance(self.robot.GetActualQd(), MagicMock)
        self.assertIsInstance(self.robot.getAsyncOperationProgress(), MagicMock)
        self.assertIsInstance(self.robot.GetActualTCPForce(), MagicMock)
        self.assertIsInstance(self.robot.GetActualTCPSpeed(), MagicMock)
        self.assertIsInstance(self.robot.getAsyncOperationProgress(), MagicMock)
        self.assertIsInstance(self.robot.GetActualTCPPose(), Iterable)
        self.assertIsInstance(self.robot.GetActualRotMatrix(), Iterable)
        
        self.assertIsNone(self.robot.Fold())
        self.assertIsNotNone(self.robot.MoveJ([0]*6))
        self.assertIsNotNone(self.robot.MoveL([0]*6))
        self.assertIsNotNone(self.robot.ActivateTeachMode())
        self.assertIsNotNone(self.robot.DeactivateTeachMode())
        self.assertIsNone(self.robot.ForceMode([0]*6, [0]*6, [0]*6, 0, [0]*6))
        
    def test_logic(self):
        rot = self.robot.Basis2Rotvec(np.eye(3))
        cond = all(rot - np.zeros(3) < 10**-3) 
        self.assertTrue(cond)



class HuskuWheelTest(unittest.TestCase):
    def setUp(self) -> None:
        self.robot = HuskyBase()
    def test_api(self):
        self.robot.GetBasePosition()
        self.robot.GetBaseTwist()
        self.robot.MoveBaseX(0, 0.)
        self.robot.RotateBaseZ(0, 0)
        
    def test_callback(self):
        from nav_msgs.msg import Odometry
        test_odom_msg = Odometry()
        self.robot.OdomCallback(test_odom_msg)
        self.assertIs(self.robot._actual_base_twist, test_odom_msg.twist.twist)
        
        from geometry_msgs.msg import PoseStamped
        test_ps = PoseStamped()
        self.robot.PoseCallback(test_ps)
        self.assertIs(test_ps.pose, self.robot._actual_base_pos)
    
    
class HuskyGripperTest(unittest.TestCase):
    def setUp(self) -> None:
        self.robot = HuskyGripper()
    def test_api(self):
        self.robot.CloseGripper()
        self.robot.OpenGripper()
        # self.assertIsNone(gripper._robotiq_client)
        
        
class FullRobotTest(UrRobotTestCase, HuskuWheelTest, HuskyGripperTest):
    def setUp(self) -> None:
        self.robot = Robot('MOCK')
        self.robot._rtde_r.getActualTCPPose.side_effect = lambda : [0]*6
    
    def test_logic(self):
        self.robot.LookAt([1,0,0])
        test_rotvec = self.robot.Basis2Rotvec([
            [ 0,-1, 0],
            [ 0, 0,-1],
            [ 1, 0, 0]
        ])
        called_rotvec = self.robot._rtde_c.moveJ_IK.call_args[0][0][3:]
        self.assertTrue(np.all(test_rotvec - called_rotvec < 10e-5))
        self.robot.Fold()
        called_fold = np.array(self.robot._rtde_c.moveJ.call_args[0][0])
        self.assertTrue(np.all(called_fold - self.robot.FOLDED_JOINTS < 10e-5)) 
                
    
if __name__ == '__main__':
    unittest.main(verbosity=2)