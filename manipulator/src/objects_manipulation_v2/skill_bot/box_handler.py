#! /usr/bin/env python3
from point_cloud import Realsense2PointCloud
from aruco_localization_v2.msg import aruco_array_msg, aruco_msg
import rospy
import skill_bot.primitives.entity as entity
import numpy as np
from visualization_msgs.msg import Marker
import copy
from sensor_msgs.msg import PointCloud
from skill_bot.utils.utils_ import ParamProvider
class BoxHandler(Realsense2PointCloud):
    def __init__(self):
        super().__init__(frame="rs_camera")
        self.aruco_sub = rospy.Subscriber(ParamProvider.aruco_tn, aruco_array_msg, self.aruco_callback, queue_size = 10)
        self.box : entity.Box = None #type: ignore
        self.table : entity.Table = None #type: ignore
        self.actualId = None
        self.box_ids = {i for i in range(15)}
        self.boxes = dict()
        self.box_pub = rospy.Publisher("/box/pc", PointCloud, queue_size=10)
        
    def process_aruco_msg(self, ar_msg : aruco_msg):
        '''
        aruco 
                 y   
            0---->1
            |.....|
          x v.....|
            3---->2
        '''
        
        p32_as_np = lambda p: np.array([p.x, p.y, p.z])
        points_as_np = [p32_as_np(v) for v in ar_msg.points] #type: ignore
        x_vec = points_as_np[3] - points_as_np[0]
        x_vec /= np.linalg.norm(x_vec)
        y_vec = points_as_np[1] - points_as_np[0]
        y_vec /= np.linalg.norm(y_vec)
        z_vec = np.cross(x_vec, y_vec)
        z_vec /= np.linalg.norm(z_vec)
        box  =entity.Box(
            position = np.mean(points_as_np, axis=0),
            basis = np.array( [x_vec, y_vec, z_vec]),
            frame="rs_camera",
            sizes=(0, 0, 0), 
            plane=entity.Plane().FromPoints(*points_as_np[:3])
        )
        # rospy.loginfo("%s ", box.position)
        br = tf.TransformBroadcaster() #type: ignore
        from scipy.spatial.transform import Rotation
        br.sendTransform(
                box.position,
                list(Rotation.from_matrix(box.basis).as_quat()),
                rospy.Time.now(),
                f"aruco_{ar_msg.arucoId}",
                "rs_camera",
            )
        return box
    
    def aruco_callback(self, ar_ar_msg = None):
        buf = dict()
        for ar_msg in ar_ar_msg.data: #type: ignore
            if not  ar_msg.arucoId in self.box_ids:
                continue
            box = self.process_aruco_msg(ar_msg)
            box =box.TransformToFrame(rospy.get_rostime(), "ur_arm_base", True)
            rospy.loginfo(box.position)
            buf[ar_msg.arucoId] = copy.copy(box)
        self.boxes = copy.copy(buf)
        return True

    def get_aruco(self):
        t = rospy.Time.now()
        ar_msg = rospy.wait_for_message(ParamProvider.aruco_tn, aruco_msg)
        if not  ar_msg.arucoId in self.box_ids: #type: ignore
            return None
        self.actualId = ar_msg.arucoId #type: ignore
        self.box = self.process_aruco_msg(ar_msg) #type: ignore
        self.box = self.box.TransformToFrame(t, "ur_arm_base", True)
        rospy.loginfo(self.box.position)
        return copy.copy(self.box)
    
import tf
def TransformVec(vec, from_frame, to_frame,  tf_listner, time):
    from geometry_msgs.msg import Vector3Stamped, Vector3
    from std_msgs.msg import Header
    
    vec_s = Vector3Stamped(
        Header(
            stamp = time,
            frame_id = from_frame
        ),
        Vector3(*vec)
    )
    new_vec : Vector3Stamped =  tf_listner.transformVector3(to_frame, vec_s)
    return np.array([new_vec.vector.x, new_vec.vector.y, new_vec.vector.z])
    
def TransformPoint(point, from_frame, to_frame, tf_listner, time):
    from geometry_msgs.msg import PointStamped, Point
    from std_msgs.msg import Header
    point_s = PointStamped(
        Header(
            stamp = time,
            frame_id = from_frame
        ),
        Point(*point)
    )
    new_point : PointStamped=  tf_listner.transformPoint(to_frame, point_s)
    return np.array([new_point.point.x, new_point.point.y, new_point.point.z])
if __name__ == "__main__": 
    rospy.init_node('test_box_pc', log_level=rospy.DEBUG)
    tf_listner = tf.TransformListener() #type: ignore
    from skill_bot.robot.husky import Robot
    r = Robot("192.168.131.40")
    def Cancel():
        r.ActivateTeachMode()
        r.OpenGripper()
    rospy.on_shutdown(Cancel)
    r.OpenGripper()
    bh = BoxHandler()
    r.ActivateTeachMode()
    r.DeactivateTeachMode()
    #rospy.spin()
    arucos_in_view = dict()
    while len(bh.boxes) ==0 and not rospy.is_shutdown():
        rospy.sleep(0.1)
    # for i in range(10):
    #     box = bh.get_aruco()
    #     arucos_in_view[bh.actualId] = box
    #     if len(arucos_in_view) > 2:
    #         break 
    base_id = list(bh.boxes)[0]
    base_box = bh.boxes[base_id]
    start_p = list(r.GetActualQ())
    h = 0.06
    for box in bh.boxes:
        rospy.loginfo("box")
        rospy.loginfo(bh.boxes[box].position)
    cp_box = copy.copy(bh.boxes)
    
    for key in cp_box:
        # if key == base_id: # comment for test box
        #     continue
        cmd1 = cp_box[key].position  + 0.05 * cp_box[key].top_plane.normal
        cmd2 = cp_box[key].position
        cmd1 = list(cmd1) +  list(r.RotvecFromBasis([-cp_box[key].basis[0], cp_box[key].basis[1], -cp_box[key].basis[2]]))
        cmd2 = list(cmd2) +  list(r.RotvecFromBasis([-cp_box[key].basis[0], cp_box[key].basis[1], -cp_box[key].basis[2]]))
        r.MoveL         (list(cmd1), 0.5, 0.5)
        r.MoveL         (list(cmd2), 0.5, 0.5)
        rospy.logdebug(cmd1)
        rospy.logdebug(cp_box[key].position)
        rospy.logdebug(r.GetActualTCPPose())
        r.CloseGripper  ()
        r.MoveJ(start_p, 0.5, 0.5)
    # # r.MoveJ([1.602, -2.869, 2.683, -2.869, -1.584, -0.001])
    #     # #r.CloseGripper  ()
    #     # input("next")
    #     # r.MoveL         (list(cmd1), 0.5, 0.5)
    #     cmd3 = base_box.position + h * base_box.top_plane.normal
    #     cmd3 = list(cmd3) +  list(r.RotvecFromBasis([-base_box.basis[0], base_box.basis[1], -base_box.basis[2]]))
    #     h+=0.02
    #     # r.MoveJ(start_p, 0.5, 0.5)
    #     # #r.MoveJ([-0.5200894514666956, -1.336278263722555, 0.785484790802002, -1.2737353483783167, -1.4993074576007288, 0.06180414929986], 0.5, 0.5)
    #     r.MoveL         (list(cmd3), 0.5, 0.3)
    #     r.OpenGripper()
    #     r.MoveJ(start_p, 0.5, 0.5)
    # t = rospy.get_rostime()
    # cmd1 = bh.box.position + 0.05 * bh.box.top_plane.normal
    # cmd2 = bh.box.position
    # cmd3 = bh.box.position + 0.2 * bh.box.basis[0] +  0.05 * bh.box.top_plane.normal
    # print(bh.box.basis)
    # cmd1 = list(cmd1) +  list(r.RotvecFromBasis([-bh.box.basis[0], bh.box.basis[1], -bh.box.basis[2]]))
    # cmd2 = list(cmd2) +  list(r.RotvecFromBasis([-bh.box.basis[0], bh.box.basis[1], -bh.box.basis[2]]))
    # cmd3 = list(cmd3) +  list(r.RotvecFromBasis([-bh.box.basis[0], bh.box.basis[1], -bh.box.basis[2]]))
    # start_p = list(r.GetActualQ())
    # print(cmd1)
    # if rospy.is_shutdown():
    #     exit()
    # r.MoveL         (list(cmd1), 0.1, 0.1)
    # r.MoveL         (list(cmd2), 0.1, 0.1)
    # r.CloseGripper  ()
    # r.MoveJ         (start_p, 0.2, 0.2)
    # a = r.GetActualQ()
    # import math
    # a[0] -= math.pi
    # r.MoveJ(a, 0.2, 0.4)
    # bh.get_aruco()
    # cmd4  = list( bh.box.position + 0.3 * bh.box.top_plane.normal) + list(r.GetActualTCPPose()[3:])
    # r.MoveL         (cmd4, 0.1, 0.1)
    # r.OpenGripper   ()
    # r.MoveJ         (start_p, 0.1, 0.1)