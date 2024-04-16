#std (pip)
from multipledispatch import dispatch
from typing import List
import numpy as np
from numpy_ros import to_numpy
from scipy.spatial.transform import Rotation


#ros imports
from geometry_msgs.msg import PoseStamped, Quaternion
import rospy
import tf

#custom
from aruco_localization_v2.msg import aruco_msg, aruco_array_msg
from skill_bot.primitives.entity import Box, EESource, BinBox, Table, Entity
from skill_bot.primitives.categories import  CATEGORIES




class EntityFabric(object):
    '''
    
    '''
    def __init__(self) -> None:
        pass
    
    
    @dispatch(object)
    def create_box(self, msg): # type: ignore
        raise NotImplementedError(f'box creation not implemented for object type {msg.__class__}')
    
    
    @dispatch(aruco_msg)
    def create_box(self, msg) -> List[Box]: # type: ignore
        points_as_np = [to_numpy(v) for v in msg.points]
        x_vec = points_as_np[3] - points_as_np[0]
        x_vec /= np.linalg.norm(x_vec)
        y_vec = points_as_np[1] - points_as_np[0]
        y_vec /= np.linalg.norm(y_vec)
        z_vec = np.cross(x_vec, y_vec)
        z_vec /= np.linalg.norm(z_vec)
        box = Box(
            position=np.mean(points_as_np, axis=0),
            basis=np.array([x_vec, y_vec, z_vec]),
            frame=msg.header.frame_id,
            sizes=(0, 0, 0),
        )
        if any([np.isnan(to_numpy(point)).any() for point in msg.points]): #type: ignore
                return [box]
        br = tf.TransformBroadcaster() # type: ignore
        br.sendTransform(
            box.position,
            Rotation.from_matrix(box.basis).as_quat().tolist(),
            rospy.Time.now(),
            f"aruco_{msg.arucoId}",
            msg.header.frame_id,
        )
        return [box]
    
    
    @dispatch(aruco_array_msg)  
    def create_box(self, msg) -> List[Box]: # type: ignore
        rt = list()
        for mark in msg.data:
            rt += self.create_box(mark)
            
        return rt
    
    
    @dispatch(PoseStamped)
    def create_box(self, msg) -> List[Box]:
        trans_comp = to_numpy(msg.pose.position)
        ori = msg.pose.orientation
        rot_comp = Rotation.from_quat([ori.x, ori.y,  ori.z, ori.w]).as_matrix()
        box = Box(
            position=trans_comp,
            basis= rot_comp.T,
            frame=msg.header.frame_id,
            sizes=(0, 0, 0),
        )
        return [box]
    
    @dispatch(PoseStamped, int, int)
    def create_box(self, msg, class_id, track_id = -1) -> List[Entity]:
        trans_comp = to_numpy(msg.pose.position)
        ori = msg.pose.orientation
        rot_comp = Rotation.from_quat([ori.x, ori.y,  ori.z, ori.w]).as_matrix()
        object_class = CATEGORIES.id2Class(class_id)
        entt =  object_class(
            position=trans_comp,
            basis= rot_comp.T,
            frame=msg.header.frame_id,
            sizes=(0,0,0),
        ) # type: Entity | Box | BinBox
        entt.track_id = track_id
        entt.entity_type_id = class_id
        try:
            entt.type_label = CATEGORIES.id2label[class_id]
        except:
            rospy.logwarn_throttle(10, f'ID {class_id} NF IN {CATEGORIES._id2l}')
            return [None]
        return [entt]
    
    @dispatch(PoseStamped, int, int, int)
    def create_box(self, msg, class_id, track_id = -1, track_2d_id = -1):
        entt : Entity = self.create_box(msg, class_id, track_id)[0]
        entt.track_2d_id = track_2d_id
        return [entt]

    

default_fabric = EntityFabric()