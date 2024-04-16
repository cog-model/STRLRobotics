#std
import copy
import numpy as np

#ros
import rospy
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

#custom
from skill_bot.utils.utils_ import ParamProvider

class HuskyBase:
    def __init__(self) -> None:
        self._vel_ropic_n = ParamProvider.vel_topic
        self._vel_topic_pub = rospy.Publisher(
            self._vel_ropic_n, 
            Twist, 
            queue_size=1)

        self._odom_topic_n = ParamProvider.odom_topic
        self._odom_sub = rospy.Subscriber(
            self._odom_topic_n, Odometry, 
            callback=self.OdomCallback, 
            queue_size=1)
        self._pose_topic_n = "/current_pose"
        self._pose_topic_sub = rospy.Subscriber(self._pose_topic_n, PoseStamped, callback=self.PoseCallback, queue_size=10)
        self._actual_base_pos = Pose()
        self._actual_base_twist = None
        self._target_twist = Twist()

    def PoseCallback(self, pose : PoseStamped):
        self._actual_base_pos  = pose.pose

    def GetBaseTwist(self):
        return copy.copy(self._actual_base_twist)

    def GetBasePosition(self) -> Pose:
        return copy.copy(self._actual_base_pos)

    def OdomCallback(self, odom : Odometry):
        #self._actual_base_pos = odom.pose.pose
        self._actual_base_twist = odom.twist.twist

    def SetBaseSpeed(self, cmd):
        t = TwistStamped()
        t.twist.linear.x = cmd[0]
        t.twist.linear.y = cmd[1]
        t.twist.linear.z = cmd[2]
        t.twist.angular.x = cmd[3]
        t.twist.angular.y = cmd[4]
        t.twist.angular.z = cmd[5]
        t.header.stamp = rospy.get_rostime()
        self._vel_topic_pub.publish(t)

    def BaseSpeedCallBack(self, args):
        '''
        not callback, publisher
        '''
        t = copy.copy(self._target_twist)
        self._vel_topic_pub.publish(t)

    def MoveBaseX(self, meters, speed, checkRate_ms=50):
        self._target_twist.linear.x = speed
        timer = rospy.Timer(rospy.Duration(0, 
            checkRate_ms * 10**7), self.BaseSpeedCallBack)

        startPosition = np.array([self.GetBasePosition().position.x,
                                  self.GetBasePosition().position.y,
                                  self.GetBasePosition().position.z])

        actualPosition = np.array([self.GetBasePosition().position.x,
                                   self.GetBasePosition().position.y,
                                   self.GetBasePosition().position.z])

        while (not rospy.is_shutdown()) and np.linalg.norm(startPosition - actualPosition) < meters:
            print( np.linalg.norm(startPosition - actualPosition)<meters)
            rospy.sleep(checkRate_ms / 1000)
            actualPosition = np.array([self.GetBasePosition().position.x,
                                       self.GetBasePosition().position.y,
                                       self.GetBasePosition().position.z])

        timer.shutdown()

    def RotateBaseZ(self, angle, speed, checkRate_ms=500):
        pass