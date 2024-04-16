#! /usr/bin/env python3
#std (pip)
import numpy as np
import cv2

#ros imports
import rospy
from message_filters import Subscriber as SyncSub
from message_filters import TimeSynchronizer as TimeSync
from geometry_msgs.msg import Point32
from sensor_msgs.msg import PointCloud, Image, CameraInfo, ChannelFloat32


class Realsense2PointCloud:
    def __init__(self, frame = "rs_camera", topic = "/realsense/point_cloud") -> None:
        self.semaphore = False
        self.pub = rospy.Publisher(topic, PointCloud, queue_size=5, tcp_nodelay=True)
        self.synchronizer = TimeSync(
            [
                SyncSub('/realsense_gripper/color/image_raw', Image),
                SyncSub('/realsense_gripper/aligned_depth_to_color/image_raw', Image),
                SyncSub('/realsense_gripper/color/camera_info', CameraInfo),
            ],
            queue_size=2,
        )
        self.x_arr = None
        self.y_arr = None
        self.pc = PointCloud()
        self.pc.header.frame_id = frame
        self.pc.points = None
        self.cv_img  = None 
        self.cv_depth = None
        self.synchronizer.registerCallback(self.CameraCallback)
        self.projected_x = None
        self.projected_y = None
        self.v_func = np.vectorize(lambda x_real, y_real, d_real : Point32(x_real, y_real, d_real))
        self.cv_pc = None

    def CameraCallback(self, color : Image, depth : Image, cam_info : CameraInfo):
        if self.semaphore:
            return
        self.semaphore = True
        cv_img = np.frombuffer(color.data, dtype=np.uint8).reshape(color.height, color.width, -1)
        cv_depth =  np.frombuffer(depth.data, dtype=np.uint16).reshape(depth.height, depth.width) / 1000.0
        
        if self.x_arr is None or self.y_arr is None:
            fx, _, cx, _, fy, cy, *_ = cam_info.K
            fx = 1.0/fx
            fy = 1.0/fy
            self.x_arr = np.arange(color.width)
            self.y_arr = np.arange(color.height)
            self.y_arr = (self.y_arr - cy)*fy
            self.x_arr = (self.x_arr - cx)*fx
            #cv_img =  cv_img / 256
            #self.pc.channels = [ChannelFloat32('r', cv_img[:,:,0].flatten() ),ChannelFloat32('g', cv_img[:,:,1].flatten() ), ChannelFloat32('b', cv_img[:,:,2].flatten() )]
        
        projected_x = cv_depth*self.x_arr
        projected_y = (cv_depth.T*self.y_arr).T

        if self.pc.points is None:
            self.pc.points = [0]*projected_x.size
        self.cv_pc = np.stack([projected_x, projected_y, cv_depth], axis=2).reshape(-1, 3)
        self.pc.points = self.v_func(projected_x.flatten(), projected_y.flatten(), cv_depth.flatten())
        #cv_img = cv_img.reshape(cv_img.shape[0]*cv_img.shape[1], cv_img.shape[2]) /256
        
        #self.pc.channels[0].values = cv_img[:,0]
        #self.pc.channels[1].values = cv_img[:,1]
        #self.pc.channels[2].values = cv_img[:,2]
        self.pub.publish(self.pc)
        self.semaphore  = False
        #rospy.loginfo_once("PUBLISHED")   
        
        

if __name__ == '__main__':
    rospy.init_node("realsense_to_pc")
    r = Realsense2PointCloud()
    rospy.spin()