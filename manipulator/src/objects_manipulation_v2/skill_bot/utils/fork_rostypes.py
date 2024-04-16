import numpy as np
import rospy
from geometry_msgs.msg import Point, Vector3Stamped, Vector3, PointStamped, Quaternion
class m_Point(Point):
    def as_np(self):
        return np.array([self.x, self.y, self.z])
    def from_np(self, p):
        self.x, self.y, self.z = p
        return self

class m_PointStamped(PointStamped):
    def as_np(self):
        return m_Point(self.point.x, self.point.y, self.point.x).as_np()
    def from_np(self, p, frame_id, stamp ):
        self.point = m_Point().from_np(p)
        self.header.stamp = stamp
        self.header.frame_id = frame_id
        return self

class m_Vector3(Vector3):
    def as_np(self):
        return np.array([self.x, self.y, self.z])
    def from_np(self, v):
        self.x, self.y, self.z = v
        return self

class m_Vector3Stamped(Vector3Stamped):
    def as_np(self):
        return np.array([self.vector.x, self.vector.y, self.vector.z])
    def from_np(self, v, frame_id, stamp):
        self.vector = m_Vector3().from_np(v)
        self.header.stamp = stamp
        self.header.frame_id = frame_id
        return self
