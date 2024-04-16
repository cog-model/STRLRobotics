#std (pip)
import copy
from typing import List, Dict, Any, Callable
import yaml
import enum
import numpy as np
from dataclasses import dataclass, field
from sklearn.linear_model import Ridge

#ros imports
import rospy
import rospkg
from geometry_msgs.msg import PoseStamped, Pose, Quaternion
from sensor_msgs.msg import Image, CameraInfo
import sensor_msgs.point_cloud2


if __name__ == '__main__':
    rospy.init_node('object_operator', log_level=rospy.DEBUG)

#custom imports
from aruco_localization_v2.msg import aruco_array_msg, aruco_msg
from skill_bot.planner.grasp_drawer import PoseDrawer 
from skill_bot.utils.utils_ import ParamProvider
import skill_bot.primitives.entity as entity 
from skill_bot.primitives.entity_fabric import default_fabric
from husky_tidy_bot_cv.msg import Objects3d
from husky_tidy_bot_cv.srv import GetObjectPose, GetObjectPointCloud, GetObjectPointCloudResponse


class By(enum.Enum):
    ID = enum.auto()
    COLOR = enum.auto()
    TYPE = enum.auto()
    DISTANCE = enum.auto()
    TIME = enum.auto()
    SOURCE = enum.auto()

class Rule(enum.Enum):
    L = enum.auto()
    H = enum.auto()
    E = enum.auto()
    LQ = enum.auto() # less or eq
    HQ = enum.auto() # higher or eq
    LH = enum.auto() # less or higher
    NLH = enum.auto() # not less or higher
    IN = enum.auto() # in set
    check = {
        By.ID : [L, H, E, LQ, HQ, LH, NLH, IN],
        By.COLOR : [E, IN],
        By.TYPE  : [E, IN],
        By.DISTANCE : [L, H, E, LQ, HQ, LH, NLH], 
        By.SOURCE : [E, IN]
    }
    


    
@dataclass
class Ctx:
    _boxes : Dict[Any, entity.Box] = field(default_factory=dict)
    locations   : dict = field(default_factory=dict) 
    last_update : rospy.Time = field(default_factory=rospy.Time.now)
    @property
    def boxes(self) -> Dict[Any, entity.Box]:
        self.last_update = rospy.Time.now()
        return self._boxes
    
    @property
    def entities(self) -> Dict[Any, entity.Box]:
        return copy.deepcopy(self._boxes)
    
    def select(self, by : By, rule: Rule, arg): # type: (By, Rule, Any)->Ctx
        if by not in Rule.check.value.keys():
            raise NotImplementedError(f'rules for select by {by} not implemented')
        if rule.value not in Rule.check.value[by]:
            raise TypeError(f'rule {rule} can not be used with select by {by}, supported {Rule.check.value[by]}')
        if not by in self._lookup_select.keys(): 
            raise NotImplementedError(f'select by {by} not implemented')
        return self._lookup_select[by](self, rule, arg)

    def _lambdagen(self, rule):
        if rule == Rule.E:
            return lambda a, b : a == b
        if rule == Rule.H:
            return lambda a, b : a > b
        if rule == Rule.L:
            return lambda a, b : a < b
        if rule == Rule.HQ:
            return lambda a, b : a >= b
        if rule == Rule.LQ:
            return lambda a, b : a <= b
        if rule == Rule.LH:
            return lambda a, b : a < b[0] or a > b[1]
        if rule == Rule.NLH:
            return lambda a, b : a > b[0] and a < b[1]
        if rule == Rule.IN: 
            return lambda a, b : a in b 
    
    def _by_id(self, rule : Rule, arg : Any): #type: (...) -> Ctx
        rs = Ctx()
        tester = self._lambdagen(rule)
        cp_ent = copy.deepcopy(self._boxes)
        for id_, val in cp_ent.items() : 
            if tester(val.entity_type_id, arg):
                rs.boxes.update({id_ : copy.copy(val)})
        return rs
        
    def _by_type(self, rule : Rule, arg : Any): # type: (...) -> Ctx
        rs = Ctx()
        tester = self._lambdagen(rule)
        cp_ent = copy.deepcopy(self._boxes)
        for id_, val in cp_ent.items():
            if tester(val.entity_type, arg):
                rs.boxes.update({id_ : copy.copy(val)})
        return rs
    
    def _by_source(self, rule : Rule, arg : Any): # type: (...) -> Ctx
        rs = Ctx()
        tester = self._lambdagen(rule)
        cp_ent = copy.deepcopy(self._boxes)
        for id_, val in cp_ent.items():
            if tester(val.source_type, arg):
                rs.boxes.update({id_ : copy.copy(val)})
                
        return rs
    
    def _by_dist(self, rule : Rule, arg : Any): # type: (...) -> Ctx
        rs = Ctx()
        tester = self._lambdagen(rule)
        cp_ent = copy.deepcopy(self._boxes)
        for id_, val in cp_ent.items():
            val_tf = val.TransformToFrame(rospy.Time.now(), arg[0], True)
            if tester(np.linalg.norm(val.position), arg[1]):
                rs.boxes.update({id_ : copy.copy(val_tf)})
        
        return rs
        
    
    _lookup_select : Dict[By, Callable] = field( default_factory= lambda  : {
        By.ID : Ctx._by_id,
        By.TYPE : Ctx._by_type,
        By.DISTANCE : Ctx._by_dist,
        
    })
    
    def __str__(self) -> str:
        rs = f'{self.last_update}\n'
        for k, v in copy.copy(self.boxes).items():
            rs += str(v) + '\n'
        return rs




class CtxManager():
    def __init__(self):
        rospack = rospkg.RosPack()
        self.pose_drawer = PoseDrawer(
            f'{rospack.get_path("meshes")}/2F85_Opened_20190924.stl')
        self.aruco_sub = rospy.Subscriber(ParamProvider.aruco_tn, aruco_array_msg, self.aruco_callback, queue_size = 1)
        self.yolo_sub = rospy.Subscriber(ParamProvider.nn_topic_name, PoseStamped, self.cb_pose_stamped, queue_size = 1)
        self.tracker_sub = rospy.Subscriber(ParamProvider.tracker_topic_name, Objects3d, callback=self.cb_tracker, queue_size=1)
        
        self.get_object_pose_srv = rospy.ServiceProxy('/object_pose_estimation/get_object_pose', GetObjectPose)
        self.get_object_point_cloud_srv = rospy.ServiceProxy('/object_point_cloud_extraction/get_object_point_cloud', GetObjectPointCloud)
        
        self._dynamic_ctx = Ctx()
        self.locations = dict()
        objects_yaml = ParamProvider.scene_config_path
        with open(objects_yaml, 'r') as fd:
            self.locations = yaml.safe_load(fd) 
            rospy.loginfo(f'LOAD SCENE CONFIG FROM:\n\t{objects_yaml}\n\t{self.locations}')
        self._dynamic_ctx.locations = self.locations



    @property
    def dynamic_ctx(self) -> Ctx:
        '''
        Obtain dynamic updating contex.
        Can be used for estimate objects poses.
        '''
        return copy.copy(self._dynamic_ctx)



    def _pose_drawer(self, box: entity.Box):
        b: np.ndarray = np.array(box.basis)
        p: np.ndarray = np.array(box.position)

        pose = np.hstack([b.T, [[p[0]], [p[1]], [p[2]]]])
        pose = np.vstack([pose, [0, 0, 0, 1]])
        img_msg: Image = rospy.wait_for_message(ParamProvider.rs_camera_tn, Image) # type: ignore
        cam_info : CameraInfo = rospy.wait_for_message(ParamProvider.rs_camera_info_tn, CameraInfo) # type: ignore
        cam = np.frombuffer(img_msg.data, dtype = np.uint8)
        cam = cam.reshape(img_msg.height, img_msg.width, -1)
        self.pose_drawer.show(cam, pose, cam_info)



    def aruco_callback(self, ar_ar_msg : aruco_array_msg) -> Dict[int, entity.Entity]:
        rospy.loginfo_once('ARUCO RECEIVED')
        buf = dict()
        boxes = default_fabric.create_box(ar_ar_msg)
        for ar_msg, box in zip(ar_ar_msg.data, boxes): #type: ignore
            ar_msg : aruco_msg = ar_msg            
            box : entity.Box = box
            try:
                box = box.TransformToFrame(rospy.Time.now(), 'local_map_lidar', True)
            except:
                rospy.logwarn('ERROR IN SAVING RESULTS IN local_map_lidar frame', True)
                box = box.TransformToFrame(rospy.Time.now(), 'ur_arm_base')
            box.entity_type = entity.EEntityType.CONTAINER
            buf[ar_msg.arucoId] = box
        self._dynamic_ctx.boxes.update(buf)
        return buf



    def cb_pose_stamped(self, box_pose : PoseStamped, id = None, track_id = -1, track_2d_id = -1) -> Dict[int, entity.Entity]:
        rospy.logdebug_once('POSE RECIEVED')
        box = default_fabric.create_box(box_pose, id, track_id, track_2d_id)[0]
        box : entity.Entity = box
        if box is None:
            return None
        try:
            box = box.TransformToFrame(rospy.Time.now(), 'local_map_lidar', True)
        except:
            rospy.logwarn('ERROR IN SAVING RESULTS IN local_map_lidar frame')
            box : entity.Box = box.TransformToFrame(rospy.Time.now(), 'ur_arm_base', True)
        rt = {track_id: box}
        
        self._dynamic_ctx.boxes.update(rt)
        return rt
    
    
    
    def cb_tracker(self, msg : Objects3d) -> Dict[int, entity.Entity]:
        '''
        Get and save data from tracker, reuse old pose stamped callback.
        return dict with [track_id: Entity]
        '''
        msg.tracking_2d_ids
        rs_dict = dict()
        for class_id, track_2d_id, track_id, position in zip(msg.classes_ids, msg.tracking_ids, msg.tracking_2d_ids, msg.positions):
            ps = PoseStamped(msg.header, Pose(position, Quaternion(0,0,0,1)))
            rs = self.cb_pose_stamped(ps, class_id, track_id, track_2d_id)
            if rs is not None:
                rs_dict.update(rs)
    
        return rs_dict
            
    def numpy_object_pc(self, track_id, try_count = 3) -> np.ndarray:
        for i in range(try_count):
            self.get_object_pose_srv.wait_for_service(rospy.Duration(4))
            srv_resp : GetObjectPointCloudResponse = self.get_object_point_cloud_srv(track_id)
            if srv_resp.return_code == 1: 
                rospy.logwarn(f'{self.get_object_point_cloud_srv.resolved_name} return fail, attempt {i}, trackID {track_id}')
                if i == try_count-1: return None
                else: continue
            break
        pc_ros = srv_resp.object_point_cloud.point_cloud
        #--convert to numpy
        pc_np = np.zeros((pc_ros.height, 3))
        for i, point in enumerate(sensor_msgs.point_cloud2.read_points(pc_ros, skip_nans=True)):
            pc_np[i] = point
        return pc_np
        
            
    def get_object_pc(self, track_id, try_count = 3) -> entity.Entity: #type: (int, int) -> entity.Entity | None
        '''
        calling object point cloud masker and calculate main diag 
        Main diag estimation based on Ridge function
        '''
        #--catch valid response from service, try 3 times by default. 
        for i in range(try_count):
            self.get_object_pose_srv.wait_for_service(rospy.Duration(4))
            srv_resp : GetObjectPointCloudResponse = self.get_object_point_cloud_srv(track_id)
            if srv_resp.return_code == 1: 
                rospy.logwarn(f'{self.get_object_point_cloud_srv.resolved_name} return fail, attempt {i}, trackID {track_id}')
                if i == try_count-1: return None
                else: continue
            break
        #--
        pc_ros = srv_resp.object_point_cloud.point_cloud
        #--convert to numpy
        pc_np = np.zeros((pc_ros.height, 3))
        for i, point in enumerate(sensor_msgs.point_cloud2.read_points(pc_ros, skip_nans=True)):
            pc_np[i] = point
        ##--
        
        #--predict main line
        reg = Ridge().fit(pc_np[:, 0].reshape((-1, 1)), pc_np[:, [1,2]])
        yz_a, yz_b = reg.predict([[ pc_np.max(0)[0] ]]), reg.predict([[ pc_np.min(0)[0] ]])
        a = np.array([pc_np.max(0)[0], yz_a[0,0], yz_a[0, 1]])
        b = np.array([pc_np.min(0)[0], yz_b[0,0], yz_b[0,1]])
        #--
        
        #--construct basis
        c = np.array([reg.coef_[0,0], reg.coef_[1,0], -1])
        yv = a - b
        yv /= np.linalg.norm(yv)
        zv = -c
        xv = np.cross(yv, zv)
        c = pc_np.mean(0)
        #--
        return entity.Entity(c, [xv, yv, zv], frame=pc_ros.header.frame_id)
        
        
        
    def update(self) -> Ctx:
        '''
        Syncronically builds actual context.
        Wait for msgs from all data topics with data for some secconds.
        returns actual context
        '''
        log_wait_err = lambda e: rospy.logwarn(f'may be time limit or smth, thx ros for fucked up exeptions throw, ros says:\n\t\t\t {e}')
        wait_msg = lambda name, topic_type : rospy.wait_for_message(name, topic_type, rospy.Duration(4))
        res = dict()
        def async_wait_msg(name, topic_type, buf : list, i):
            try:
                buf[i] = wait_msg(name, topic_type)
            except rospy.exceptions.ROSException as e:
                log_wait_err(e)
                buf[i] = None
        buf = [None]*3
        aruco_timer = rospy.Timer(rospy.Duration(0,1), lambda x : async_wait_msg(ParamProvider.aruco_tn, aruco_array_msg, buf, 0), True)
        nn_timer = rospy.Timer(rospy.Duration(0,1), lambda x : async_wait_msg(ParamProvider.nn_topic_name, PoseStamped, buf, 1), True)
        tracker_timer =  rospy.Timer(rospy.Duration(0,1), lambda x : async_wait_msg(ParamProvider.tracker_topic_name, Objects3d, buf, 2), True)
        aruco_timer.join()
        nn_timer.join()
        tracker_timer.join()
        detected_aruco : aruco_array_msg = buf[0]
        detected_nn : PoseStamped = buf[1]
        tracked : Objects3d = buf[2] if detected_nn is None else None
        
                
        if detected_aruco is not None:
            aruco_detection = self.aruco_callback(detected_aruco)
            print(aruco_detection)
            res.update(aruco_detection)
        if detected_nn is not None:
            nn_detection = self.cb_pose_stamped(detected_nn)
            res.update(nn_detection)
        if tracked is not None:
            tracked_detection = self.cb_tracker(tracked)
            res.update(tracked_detection)
            
        return Ctx(res, locations=self.locations)
    
    
    
if __name__ == '__main__':
    ctx_manager = CtxManager()
    ctx_manager.update()