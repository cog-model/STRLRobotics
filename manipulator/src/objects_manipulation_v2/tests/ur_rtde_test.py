#!/usr/bin/python3
import sys
import numpy as np
import unittest
from unittest.mock import Mock, MagicMock, create_autospec, patch
import rtde_control 
import rtde_receive 
rtde_c_mock = create_autospec(rtde_control)
rtde_r_mock = create_autospec(rtde_receive)
with patch.dict('sys.modules', {'rtde_control' : rtde_c_mock, 'rtde_receive' : rtde_r_mock}):
    from skill_bot.robot.husky_ur import HuskyUr

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
        
        
     
        
if __name__ == '__main__':
    unittest.main()