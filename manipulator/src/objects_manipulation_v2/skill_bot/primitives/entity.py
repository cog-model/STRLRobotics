#std
import copy
import numpy as np
from scipy.spatial.transform import Rotation
import enum
import numpy_ros 

#ros imports
import tf
import rospy
from geometry_msgs.msg import Quaternion
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA, Header

#custom
from skill_bot.utils.fork_rostypes import m_Point, m_PointStamped, m_Vector3, m_Vector3Stamped


TF_LISTNER = tf.TransformListener() #type: ignore

class EEntityType(enum.Enum):
    UNKNOWN = enum.auto()
    CUBE = enum.auto()
    CONTAINER = enum.auto()
    

class EESource(enum.Enum):
    UNKNOWN = enum.auto()
    ARUCO = enum.auto()
    DETECTRON = enum.auto()
    TRACKING = enum.auto()


class Entity:
    def __init__(self, 
                 position = np.zeros(3), 
                 basis = np.eye(3), 
                 sizes = (0, 0, 0), 
                 frame = "", 
                 type_id = -1, 
                 track_id = -1,
                 type_label = 'unknown',
                 track_2d_id = -1) -> None:
        self.position = np.array(position)
        self.frame = frame
        self.basis = np.array(basis)
        self.sizes = sizes
        self.entity_type = EEntityType.UNKNOWN
        self.source_type = EESource.UNKNOWN
        self.operate_point = np.zeros(3)
        self.track_id = type_id
        self.entity_type_id = track_id
        self.type_label = type_label
        self.track_2d_id = track_2d_id
        self._rostime_stamp = rospy.Time.now()

    def TransformToFrame(self, time : rospy.Time, frame : str, inplace = False): # type: (...) -> Entity
        assert self.frame != '', 'Frame can not be empy. Fill frame field'
        time = rospy.Time(0)
        TF_LISTNER.waitForTransform(self.frame, frame, time, rospy.Duration(4))
        new_basis_vec3 = [TF_LISTNER.transformVector3(frame, m_Vector3Stamped().from_np(v, self.frame, time)) for v in self.basis]
        new_pos_Point = TF_LISTNER.transformPoint(frame, m_PointStamped().from_np(self.position, self.frame, time))
        target = self if inplace else copy.deepcopy(self)
        target.basis = np.array([numpy_ros.to_numpy(v) for v in new_basis_vec3 ])
        target.position = numpy_ros.to_numpy(new_pos_Point)
        target.frame = frame
        return target


    def MakeMarker(self) -> Marker:
        m = Marker()
        m.action = Marker.ADD
        m.pose.position = m_Point().from_np(self.position)
        m.pose.orientation = Quaternion(*Rotation.from_matrix(self.basis).as_quat())
        m.header = Header(stamp = rospy.Time.now(), frame_id = self.frame)
        return m
    
    
    def rel2absolute(self, relative_position) -> np.ndarray:
        relative_position = np.array(relative_position)
        return self.basis.T @ relative_position + self.position
    
    def grasp_pose(self):
        return self.rel2absolute(self.operate_point)
    
    def correct_basis(self, aware_y = False):
        print('-----')
        corrected_basis = copy.copy(self.basis)
        min_angle = 0.1
        for select in ['x', 'y', 'z']:
            if aware_y:
                theta_vars = [0, np.pi] if select in ['x', 'z'] else [0, np.pi/2, np.pi, 3*np.pi/2]
            else:
                theta_vars = [0, np.pi/2, np.pi, 3*np.pi/2]
            for theta in theta_vars:
                m = Rotation.from_euler(select, theta).as_matrix()
                tf_basis_mat = self.basis.T @ m
                z_axis = tf_basis_mat @ [0,0,1]
                y_axis = tf_basis_mat @ [0,1,0]
                dot_y  = np.dot([0, 1, 0], y_axis)
                dot_z = np.dot([0,0,1], z_axis)
                print(np.round(tf_basis_mat, 2))
                print()
                if dot_y > 0 and  dot_z > 0 and np.abs( np.arccos(dot_z)) < min_angle:
                    rospy.logdebug(f'CORRECTION BASIS, dist: {dot_z}, angle {np.arccos(dot_z)}, min_angle {min_angle}')
                    min_angle = np.arccos(dot_z)
                    corrected_basis = tf_basis_mat.T.copy()
        print('-----')
        self.basis = corrected_basis
    
    
    def __str__(self) -> str:
        return f'{self.type_label}({self.entity_type_id}) {self.track_id} {self.frame} {self.position}'
    
    

class Box(Entity):
    def __init__(self, position=np.zeros(3), basis=np.eye(3), sizes=(0, 0, 0), frame="", color = np.random.random(3)) -> None:
        super().__init__(position, basis, sizes, frame)
        self.entity_type = EEntityType.CUBE
        self.color = color

    def TransformToFrame(self, time : rospy.Time, frame : str, inplace = False): # type: (...) -> Box
        rs : Box = super().TransformToFrame(time, frame, inplace = inplace) # type: ignore
        return rs
    
    def Makemarker(self) -> Marker:
        m = Entity.MakeMarker(self)
        m.type = Marker.CUBE
        m.color = ColorRGBA(*self.color, 1)
        return m
    
    
   
class Container(Entity):
    def __init__(self, position=np.zeros(3), basis=np.eye(3), sizes=(0, 0, 0), frame="") -> None:
        super().__init__(position, basis, sizes, frame)
        self.entity_type = EEntityType.CONTAINER
       
class BinBox(Container):
    def __init__(self, position=np.zeros(3), basis=np.eye(3), sizes=(0, 0, 0), frame="") -> None:
        super().__init__(position, basis, sizes, frame)
        self.entity_type = EEntityType.CONTAINER

class Table(Entity):
    def __init__(self, position=np.zeros(3), basis=np.eye(3), sizes=(0, 0, 0), frame="") -> None:
        super().__init__(position, basis, sizes, frame)
        self.entity_type = EEntityType.UNKNOWN