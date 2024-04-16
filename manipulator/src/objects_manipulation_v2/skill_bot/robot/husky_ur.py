#std (pip)
import math
import time
import numpy as np
from rtde_control import RTDEControlInterface
from rtde_receive import RTDEReceiveInterface
from scipy.spatial.transform import Rotation

#ros
import rospy

class HuskyUr:

    def __init__(self, UR_IP):
        try:
            self.__init_rtde(UR_IP)
            rospy.loginfo("UR5 CONNECTED ON IP {}".format(UR_IP))
        except RuntimeError as e:
            rospy.logfatal(e)
            exit()
        self.FOLDED_JOINTS = [1.6017078161239624, -2.8689330259906214, 2.7661876678466797, -2.154529396687643, -1.5819104353534144, -0.0010188261615198257]
        self.INITIAL_JOINTS = [1.5547375679016113, -2.3851588408099573, 1.8522262573242188, -2.5852845350848597, -1.5306795279132288, -0.0008385817157190445]
        self.REACH_RADIUS = 2
        self.FM_SAFE_RADIUS_REAR = 0.5
        self.FM_SAFE_RADIUS_FAR = 1

    def __init_rtde(self, UR_IP):
        flags = RTDEControlInterface.FLAG_VERBOSE | RTDEControlInterface.FLAG_USE_EXT_UR_CAP
        self._rtde_c = RTDEControlInterface(UR_IP, flags)
        self._rtde_r = RTDEReceiveInterface(UR_IP)

    # rtde library part
    def ForceModeStop(self):
        self._rtde_c.forceModeStop()


    def _IsNaNorInf(self, array):
        for q in array:
            if math.isnan(q) or math.isinf(q):
                return True
        return False

    def GetActualQ(self):
        return self._rtde_r.getActualQ()

    def GetActualTCPPose(self):
        return self._rtde_r.getActualTCPPose()

    def GetActualTCPSpeed(self):
        return self._rtde_r.getActualTCPSpeed()

    def GetActualQd(self):
        return self._rtde_r.getActualQd()

    def GetActualTCPForce(self):
        return self._rtde_r.getActualTCPForce()

    def MoveJ(self, pose, vel=1.05, acc=1.4, asyncro=False):
        if self._IsNaNorInf(pose):
            rospy.logwarn("robot controller got inf or nan")
            return False
        return self._rtde_c.moveJ(pose, vel, acc, asyncro)

    def MoveL(self, pose, vel=0.25, acc=1.2, asyncro=False):
        if self._IsNaNorInf(pose):
            rospy.logwarn("robot controller got inf or nan")
            return False
        return self._rtde_c.moveL(pose, vel, acc, asyncro)

    def SpeedJ(self, speed, acc=0.5, time=0.0):
        if self._IsNaNorInf(speed):
            rospy.logwarn("robot controller got inf or nan")
            return False
        return self._rtde_c.speedJ(speed, acc, time)

    def SpeedL(self, speed, acc=0.25, time=0.0):
        if self._IsNaNorInf(speed):
            rospy.logwarn("robot controller got inf or nan")
            return False
        return self._rtde_c.speedL(speed, acc, time)

    def SpeedStop(self, acc=10.0):
        self._rtde_c.speedStop(acc)

    def ActivateTeachMode(self):
        return self._rtde_c.teachMode()

    def DeactivateTeachMode(self):
        return self._rtde_c.endTeachMode()

    def ForceMode(self, targetFrame, selector, wrench, mode, limits):
        self._rtde_c.forceMode(targetFrame, selector, wrench, mode, limits)

    def PushUntilForce(self, selector, targetForce, limits, forceStep=[10]*6, timeOut=-1):
        startTime = time.time()
        prevPose = np.array(self.GetActualTCPPose())
        checkTime = time.time()
        while not rospy.is_shutdown():
            self.ForceMode(self.GetActualTCPPose(),
                           selector, targetForce, 2, limits)
            if timeOut > 0 and time.time() - startTime > timeOut:
                break
            rospy.sleep(0.1)
            if time.time() - checkTime > 2:
                checkTime = time.time()
                if all(np.array(self.GetActualTCPPose()) - prevPose < 0.03):
                    break
                prevPose = self.GetActualTCPPose()
        self._rtde_c.forceModeStop()

    def AlignYZtoXYPlane(self, vel=0.25, acc=1.2, asyncro=False):
        actual = self.GetActualTCPPose()
        rot = Rotation.from_rotvec(actual[3:]).as_matrix()
        x = rot @ np.eye(3)[0]
        y = rot @ np.eye(3)[1]
        z = rot @ np.eye(3)[2]
        newRot = Rotation.from_matrix([
            [0,    y[0]/math.sqrt(y[0]**2 + y[1]**2), z[0]/math.sqrt(z[0]**2 + z[1]**2)],
            [0,    y[1]/math.sqrt(y[0]**2 + y[1]**2), z[1]/math.sqrt(z[0]**2 + z[1]**2)],
            [x[2]/math.fabs(x[2]), 0,    0]
        ]).as_rotvec()
        cmd = np.concatenate((actual[0:3], newRot))
        self.MoveL(cmd, vel, acc, asyncro)

    def AlignXZtoXYPlane(self, vel=0.25, acc=1.2, asyncro=False):
        actual = self.GetActualTCPPose()
        rot = Rotation.from_rotvec(actual[3:]).as_matrix()
        x = rot @ [1, 0, 0]
        y = rot @ [0, 1, 0]
        z = rot @ [0, 0, 1]
        if y[2] < 0:
            newRot = Rotation.from_matrix([
                [x[0]/math.sqrt(x[0]**2 + x[1]**2), 0, z[0]/math.sqrt(z[0]**2 + z[1]**2)],
                [x[1]/math.sqrt(x[0]**2 + x[1]**2), 0, z[1]/math.sqrt(z[0]**2 + z[1]**2)],
                [0, y[2]/math.fabs(y[2]), 0]
            ]).as_rotvec()
        else:
            newRot = Rotation.from_matrix([
                [-x[0]/math.sqrt(x[0]**2 + x[1]**2), 0, z[0]/math.sqrt(z[0]**2 + z[1]**2)],
                [-x[1]/math.sqrt(x[0]**2 + x[1]**2), 0, z[1]/math.sqrt(z[0]**2 + z[1]**2)],
                [0, -y[2]/math.fabs(y[2]), 0]
            ]).as_rotvec()

        cmd = np.concatenate((actual[0:3], newRot))
        self.MoveL(cmd, vel, acc, asyncro)

    def IfAlignedYZtoXYPlane(self, eps=0.1):
        actual = self.GetActualTCPPose()
        rot = Rotation.from_rotvec(actual[3:]).as_matrix()
        
        x = rot @ [1, 0, 0]
        y = rot @ [0, 1, 0]
        z = rot @ [0, 0, 1]
        return (math.fabs(y[2]) < eps and math.fabs(z[2]) < eps)

    def Basis2Rotvec(self, basis):
        basis = np.array(basis)
        return Rotation.from_matrix(basis.T).as_rotvec()

    

    def ForceModeWithSingularityAwoid(self, force, selector, limits, R1=0.48, R2=0.9, fmax=350):
        def Force(x, R1, R2, fmax):
            n = 8*fmax/((R1-R2)**3)
            b = -(R1+R2)/2.0
            return n*((x+b)**3)

        def ToTCP(pose, vec):
            v = np.array(vec)
            rot = Rotation.from_rotvec(pose[3:]).as_matrix()
            return np.linalg.inv(rot) @ v

        npF = np.array(force)
        actual = self.GetActualTCPPose()
        normal = np.linalg.norm(actual[:3])
        f = Force(math.sqrt(actual[0]**2 + actual[1]**2), R1, R2, fmax)
        force = [actual[0]*f/normal, actual[1]*f/normal, 0]

        forceInTCP = ToTCP(actual, force)
        cmd = np.array(list(forceInTCP) + [0]*3) + npF
        self._rtde_c.forceMode(actual, selector, cmd, 2, limits)

    
    
    def GetDistToTCPFromBaseXY(self):
        curTCP = np.array(self.GetActualTCPPose()[0:2])
        return np.linalg.norm(curTCP)



    def OrientationByForceMode(self, estimationBasis, epsilon=0.1):
        estimationBasis = np.array(estimationBasis)
        newRot = Rotation.from_matrix(
            [np.transpose(estimationBasis)]
        ).as_rotvec()

        while (not rospy.is_shutdown()) and not (np.abs(np.array(self.GetActualTCPPose()[3:]) - newRot) < epsilon).all():
            diff = -np.array(self.GetActualTCPPose())[3:] + newRot
            f = [0, 0, 0] + list(10*diff[0])
            rospy.loginfo(diff[0])
            self.ForceMode(self.GetActualTCPPose(), [*[0]*3, *[1]*3], f, 2, [*[2]*3, *[math.pi]*3])
            time.sleep(0.1)

    def ChangeOrientation(self, rotvec, vel = 0.25, acc = 1.2, asyncro = False):
        actual = self.GetActualTCPPose()
        self._rtde_c.moveJ_IK([*actual[:3], *rotvec], vel, acc, asyncro)

    
    def MoveL_point_rot(self, pose, orient, vel=0.25, acc=1.2, IK = False, asyncro=False):
        cmd = list(pose) + list(orient)
        if not IK:
            self.MoveL(cmd, vel, acc, asyncro)
        else:
            if self._IsNaNorInf(cmd):
                rospy.logwarn("robot controller got inf or nan")
                return False
            self._rtde_c.moveJ_IK(cmd, vel, acc, asyncro)

    def GetActualRotMatrix(self):
        return Rotation.from_rotvec(self.GetActualTCPPose()[3:]).as_matrix()

    def Fold(self, vel = 0.25, acc = 1.2, asynchro = False):
        self.MoveJ(self.FOLDED_JOINTS, vel, acc, asynchro)
        
    def getAsyncOperationProgress(self):
        '''
        <0: async finished or not started
        0: async started
        >0: in progress
        '''
        return self._rtde_c.getAsyncOperationProgress()

