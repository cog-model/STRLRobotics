#! /usr/bin/env python3
#std
import math 
import random
import numpy as np

#ros
import rospy 
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

class Arrow(Marker):
    def __init__(self, *args, **kwds):
        super().__init__(*args, **kwds)
        self.type = Marker.ARROW
        self.scale.x = 0.05
        self.scale.y = 0.1
        self.scale.z = 0.2
        self.header.frame_id = "map"
        self.color.a = 1
        self.color.r = 1 
        self.pub = rospy.Publisher("/arrow", Marker, queue_size=10)
    def SetPose(self, point1, point2):
        self.points.append(Point(*point1)) # type: ignore
        self.points.append(Point(*point2)) # type: ignore
    def Pub(self):
        self.pub.publish(self)

class Cube(Marker):
    def __init__(self, *args, **kwds):
        super().__init__(*args, **kwds)
        self.type = Marker.CUBE
        self.scale.x = 1
        self.scale.y = 1
        self.scale.z = 1
        self.header.frame_id = "map"
        self.color.a = 0.5
        self.color.g = 1
        self.pub = rospy.Publisher(f"/cube/{self.header.seq}", Marker, queue_size=10)
    def SetQart(self, u, angle):
        self.pose.orientation.w = math.cos(angle/2.0)
        self.pose.orientation.x = math.sin(angle/2.0)*u[0]
        self.pose.orientation.y = math.sin(angle/2.0)*u[1]
        self.pose.orientation.z = math.sin(angle/2.0)*u[2]
    def Pub(self):
        self.pub.publish(self)

rospy.init_node("test")
pub_array = rospy.Publisher("/cube/points", MarkerArray, queue_size=10)
u = np.array([random.random(), random.random(), random.random()])
u /= np.linalg.norm(u)
cube1 = Cube()
cube2 = Cube()
cube1.SetQart(u, 0.0)
arrow = Arrow()
arrow.SetPose([0,0,0], u)
angle = 0
while not rospy.is_shutdown():
    cube1.Pub()
    arrow.Pub()
    cube1.SetQart(u, angle)
    cube2.Pub()
    angle += math.pi/100
    rospy.sleep(0.1)