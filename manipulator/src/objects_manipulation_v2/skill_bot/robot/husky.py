#!/usr/bin/python3

#std
from scipy.spatial.transform import Rotation
import numpy as np
import math
from collections import deque
import copy

# ros imports
import rospy
import tf
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

# custom
from skill_bot.utils.utils_ import ParamProvider
from skill_bot.robot.husky_gripper import HuskyGripper
from skill_bot.robot.husky_ur import HuskyUr
from skill_bot.robot.husky_base import HuskyBase
from  objects_manipulation_v2.msg import ManipulatorState

class PositionHystrory:
    def __init__(self, pose_getter, maxlen=4000) -> None:
        self._eef_pose_getter = pose_getter
        self.hystory = deque(maxlen=maxlen)
        self._time = 'time'
        self._pose = 'pose'
        self.timer = rospy.Timer(rospy.Duration(0,1000), self.callback)



    def callback(self, arg):
        if rospy.is_shutdown(): return
        self.hystory.append({self._time: rospy.Time.now(),
                             self._pose: self._eef_pose_getter()})
        manipulator_pose : np.ndarray = np.array(self._eef_pose_getter())
        if manipulator_pose.shape[0] == 0: return
        position = manipulator_pose[:3].tolist()
        rotation = Rotation.from_rotvec(manipulator_pose[3:]).as_quat().tolist()
        br = tf.TransformBroadcaster()
        try:
            actual_time = rospy.Time.now()
            br.sendTransform(position, rotation, actual_time, 'ur_gripper', 'ur_arm_base')
        except rospy.exceptions.ROSException as e:
            rospy.logwarn(f'ERROR PUBLISH TO TF-TREE: {e}')

        rospy.loginfo_once('UR5 PUBLIHED TO TF-TREE')



    def FindPoseByMinTimeDiff(self, time):
        actual = self.hystory.copy()
        min_time_diff = abs(time.data.to_nsec() -
                          actual[0][self._time].to_nsec())
        i_minimum = 0

        for i in range(len(actual)):
            cur_diff = abs(
                actual[i][self._time].to_nsec() - time.data.to_nsec())
            if min_time_diff >= cur_diff:
                min_time_diff = cur_diff
                i_minimum = i
        return actual[i_minimum][self._pose]



    def GetHystory(self):
        return copy.copy(self.hystory)


class Robot(HuskyGripper, HuskyUr, HuskyBase):
    def __init__(self, UR_IP) -> None:

        self.pose_locked = False

        HuskyBase.__init__(self)
        HuskyGripper.__init__(self)
        HuskyUr.__init__(self, UR_IP)


        self._eef_hystory = PositionHystrory(self.GetActualTCPPose)

        self._joint_state_publisher = rospy.Publisher('/arm/1/joint_states', JointState, queue_size=1)
        self._manipulator_state_publisher = rospy.Publisher('/state/arm/0/arm_state', ManipulatorState, queue_size=1)
        self._publlish_joint_state_timer = rospy.Timer(rospy.Duration(0, 1000), self.cb_joint_state_publisher)


    def cb_joint_state_publisher(self, fuck):
        if rospy.is_shutdown(): return
        msg = JointState()
        msg.header = Header(frame_id = 'ur_arm_base', stamp = rospy.Time(0))
        msg.position = self.GetActualQ()
        msg.velocity = self.GetActualQd()
        msg.effort  = self.GetActualTCPForce()
        try:
            self._joint_state_publisher.publish(msg)
        except rospy.exceptions.ROSException as e:
            rospy.logwarn(f'Error at publish joint state, ros says:\n\t\t\t\t{e}')
        self.cb_publish_manipulator_state()


    def OdomCallback(self, odom):
        HuskyBase.OdomCallback(self, odom)
        if self.pose_locked:
            self.CorrectPositionByTwist()



    def cb_publish_manipulator_state(self):
        if rospy.is_shutdown(): return
        msg = ManipulatorState()
        msg.q_target = self._rtde_r.getTargetQ()
        msg.qd_target = self._rtde_r.getTargetQd()
        msg.i_target = self._rtde_r.getTargetCurrent()
        msg.m_target = self._rtde_r.getTargetMoment()
        # msg.tau_target = self._rtde_c.getJointTorques()
        msg.tool_vector_target = self._rtde_r.getTargetTCPPose()
        msg.q_actual = self._rtde_r.getActualQ()
        msg.qd_actual = self._rtde_r.getActualQd()
        msg.i_actual = self._rtde_r.getActualCurrent()
        msg.tau_actual = self._rtde_r.getTargetMoment()
        msg.tcp_force = self._rtde_r.getActualTCPForce()
        msg.tool_vector_actual = self._rtde_r.getActualTCPPose()
        msg.tcp_speed = self._rtde_r.getActualTCPSpeed()
        msg.motor_temperatures = self._rtde_r.getJointTemperatures()
        msg.joint_modes =  [float(item) for item in self._rtde_r.getJointMode()]
        msg.controller_timer = self._rtde_r.getActualExecutionTime()
        msg.qdd_target = self._rtde_r.getTargetQdd()
        msg.qdd_actual = []
        msg.tool_acc_values = self._rtde_r.getActualToolAccelerometer()
        msg.robot_mode = self._rtde_r.getRobotMode()
        msg.digital_input_bits = float(self._rtde_r.getActualDigitalInputBits())
        msg.test_value = 0.0
        try:
            self._manipulator_state_publisher.publish(msg)
        except rospy.exceptions.ROSException as e:
            rospy.logwarn(f'Error with publishing manipulator state\n\t\t\t\t{e}')



    def LockPose(self):
        rospy.loginfo("POSE LOCKED")
        self.pose_locked = True



    def PoseUnlock(self):
        rospy.loginfo("POSE UNLOCKED")
        self.pose_locked = False


    def CorrectPositionByTwist(self):
        twist = copy.copy(self._actual_base_twist)
        pose = self.GetActualTCPPose()

        cmd = [twist.linear.y - np.linalg.norm(pose[:2])*twist.angular.z*math.cos(math.atan2(pose[1] - 0.389, pose[0]) + math.pi/2), #type: ignore
               twist.linear.x - np.linalg.norm(pose[:2])*twist.angular.z*math.sin( math.atan2(pose[1] - 0.389, pose[0]) + math.pi/2), #type: ignore
            -twist.linear.z, #type: ignore
            twist.angular.y, #type: ignore
            twist.angular.x, #type: ignore
            -twist.angular.z] #type: ignore
        self.SpeedL(cmd, 2)



    def ManualPose(self):
        self.ActivateTeachMode()
        print(self.GetActualTCPPose())
        input("set init pose and press enter")
        self.DeactivateTeachMode()



    def LookAt(self, point, vel = 0.25, acc = 1.2, asyncro = False):
        point = np.array(point, dtype=np.float32)
        tcpXYZ = np.array( self.GetActualTCPPose()[0:3])
        forward = point - tcpXYZ
        forward /= np.linalg.norm(forward)
        right = np.cross(forward, [0, 0, 1])
        up = np.cross(forward, right)
        rot = self.Basis2Rotvec([right, up, forward])
        self.ChangeOrientation(rot, vel, acc, asyncro)



    def SoftDetachFromObj(self):
        self.ForceMode(self.GetActualTCPPose(), [*[1]*3, *[0]*3], [0]*6, 2, [*[0.5]*3, *[math.pi/2]*3])
        self.OpenGripper()
        self.ForceModeStop()


