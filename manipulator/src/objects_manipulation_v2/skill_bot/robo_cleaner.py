#! /usr/bin/python3

#std (pip)  
import time
import numpy as np
import threading


#ros imports
import tf
import rospy
from actionlib import SimpleActionServer
from actionlib import exceptions
from geometry_msgs.msg import PoseStamped


if __name__ == '__main__':
    rospy.init_node('object_operator', log_level=rospy.DEBUG)
    

#custom imports
from communication_msgs.msg import *
from skill_bot.robot.husky import Robot 
import skill_bot.primitives.entity as entity 
from skill_bot.planner.ctx_manager import CtxManager, By, Rule
from skill_bot.utils.utils_ import ParamProvider
from skill_bot.primitives.categories import CATEGORIES


class Consts:
    '''
    - J_* - joints position
    - L_* - decatr position
    - M_* - multiplicator
    '''
    M_L_SPD_SCALE             = 0.5
    M_J_SPD_SCALE             = 0.5
    J_WATCH_ON_FLOOR            = [ 1.384, -1.934,  1.677, -1.374, -1.546,  0.004]
    J_WATCH_ON_FLOOR_2          = [ 1.595, -2.145,  2.487, -2.297, -1.627,  0.023]
    J_WATCH_ON_CONTAINER        = [ 1.479, -2.147,  2.152, -2.127, -1.585,  0.000]
    J_RIGHT_ROBOT_CORNER        = [ 0.160, -0.541,  1.842, -2.905, -1.523, -0.000]
    J_WATCH_RIGHT_ROBOT_CORNER  = [ 0.160, -0.541,  1.842, -2.905, -1.523, -0.000]



class Node():
    def __init__(self) -> None:
        self.tf_listener = tf.TransformListener() #type: ignore
        self.robot = Robot(ParamProvider.ur_ip)
        rospy.on_shutdown(self.cancel)
        self.robot.ActivateTeachMode()
        self.bh = CtxManager()
        self.pick_act = SimpleActionServer('pick_up_object', PickupObjectAction, self.cb_pickup, False)
        self.place_act = SimpleActionServer('put_object', PutObjectAction, self.cb_put_object, False)
        self.pick_act.register_preempt_callback(self._cancel_action)
        self.place_act.register_preempt_callback(self._cancel_action)
        self.pick_act.start()
        self.place_act.start()
        self.action_lock = threading.Lock()
        
        


    
    def _wait_manipulator(self, action: SimpleActionServer) -> bool:
        '''
        Wait while async operation is executing:
        async progress >=0 - async execution in progress
        async progress >0 - manipulator ready
        
        return False if ros shutdown else True
        (idk нужна ли такая проверка)  
        '''
        while not rospy.is_shutdown() and self.robot.getAsyncOperationProgress() >= 0:
            rospy.sleep(rospy.Duration(0, 200))
            
            if action and action.is_preempt_requested():
                rospy.loginfo('PREEMPT REQUESTED FOR ACTION')
                rospy.logdebug('RAISING EXCEPTION')
                raise exceptions.ActionException('GOAL PREEMPTED')
            
        if rospy.is_shutdown():
            raise rospy.exceptions.ROSException('KILLED BY USER')
            
        return True


        
    def _cancel_action(self):
        '''
        Callback for preempted action.
        Returns robot to folded position.
        '''
        rospy.loginfo('CANCEL ACTION REQUEST')
        with self.action_lock:
            self.robot.OpenGripper()
            self.robot.MoveJ(self.robot.INITIAL_JOINTS, 0.3, 0.1, True)
            self._wait_manipulator(None)
            self.robot.Fold(0.3, 0.1)
            self._wait_manipulator(None)
            self.robot.ActivateTeachMode()
        rospy.loginfo('CANCEL ACTION REQUEST COMPLETED')     
        
        
        
    def _robot_action(cb_function):
        '''
        Decorator for all actions.
        Folding robot and activating teach mode.
        '''
        def robot_action_wrapper(self, msg):
            rospy.loginfo(f'STARTING ACTION {cb_function.__name__}')
            with self.action_lock:
                self.robot.DeactivateTeachMode()
                try:
                    rs = cb_function(self, msg)
                except exceptions.ActionException as e:
                    rospy.loginfo(f'ACTION COMPLETED WITH: {e}')
                    return
                except rospy.exceptions.ROSException as e:
                    rospy.logwarn(f'ACTION EXITED WITH: {e}')
                    return
                self.robot.Fold(0.3, 0.1, True)
                self._wait_manipulator(None)
                self.robot.ActivateTeachMode()
            rospy.loginfo(f'ACTION COMPLETED:\n{rs}')
        return robot_action_wrapper
        

    @_robot_action
    def cb_pickup(self, msg : PickupObjectGoal, WATCH_POSE = Consts.J_WATCH_ON_FLOOR_2):
        '''
        Callback pickup cube from floor.
        [New features worked at least once]
        '''
        is_async = True
        this_action = self.pick_act
        def manipulator_control(func, pose, speed = 0.2, acc = 0.1):
            if pose is not None:
                func(pose, speed*Consts.M_J_SPD_SCALE, acc*Consts.M_J_SPD_SCALE, is_async)
            else:
                func(speed*Consts.M_J_SPD_SCALE, acc*Consts.M_J_SPD_SCALE, is_async)
            if is_async:
                self._wait_manipulator(this_action)
        #-- prepare manipulator
        self.pick_act.publish_feedback(PickupObjectActionFeedback('', '',''))
        self.robot.OpenGripper()
        manipulator_control(self.robot.MoveJ, Consts.J_WATCH_ON_FLOOR, 0.2, 0.1)
        #-- 
        
        class_id = CATEGORIES.label2id[msg.object]
        rospy.loginfo(f'PICK UP REQEST {msg.object} with id {class_id}')
        
        
        by_class_id = (By.ID, Rule.E, class_id)
        by_dist = (By.DISTANCE, Rule.L, ['ur_arm_base', 1])
        entities_dynamic = dict()
        rospy.loginfo('WAIT OBJECTS')
        t_limit = 10
        t_start = time.time()
        while len(entities_dynamic) == 0 and not rospy.is_shutdown() and time.time() - t_start < t_limit:
            entities_dynamic = self.bh.dynamic_ctx.select(*by_class_id).select(*by_dist).boxes
            rospy.loginfo(entities_dynamic)
            rospy.sleep(1)
        rospy.loginfo('WAIT COMPLETED')
        target = None
        for target_id, target in entities_dynamic.items():
            target = target.TransformToFrame(rospy.Time.now(), 'ur_arm_base', True)
            rospy.loginfo(f'LOOK AT {target}')
            manipulator_control(self.robot.LookAt, target.position, 0.3, 0.1)
            after_lookat = self.bh.update().select(*by_class_id).select(*by_dist).boxes
            
            if len(after_lookat) > 0:
                for k, v in after_lookat.items():
                    if v.entity_type_id == target.entity_type_id and v.track_2d_id != -1:
                        rospy.logdebug(f's {v} {k} {target.track_id}')
                        target.track_id = k
                        target.track_2d_id = v.track_2d_id
     
            if entity.EEntityType.CUBE == target.entity_type:        
                target_6dof = self.bh.get_object_pose_srv(target.track_2d_id)
            
                rospy.loginfo(f'TARGET 6DOF POSE {target_6dof}')
                if target_6dof.return_code == 0:
                    target_ps = PoseStamped(target_6dof.object_pose.header, target_6dof.object_pose.pose)
                    target_entt = self.bh.cb_pose_stamped(target_ps, target.entity_type_id,target_id)
                    _, target = target_entt.popitem()
                    break
            else:
                rospy.loginfo('MAKE ENTITY FROM PC')
                target =  self.bh.get_object_pc(target.track_id)
                if target is not None:
                    break
                
        if target is None:
            rospy.loginfo('NOTHING FOUND')
            manipulator_control(self.robot.MoveJ, self.robot.INITIAL_JOINTS, 0.3, 0.1)
            manipulator_control(self.robot.Fold, None, 0.3, 0.1)
            rs = PickupObjectActionResult('', '', '')
            self.pick_act.set_aborted(rs)
            return rs
        
        target = target.TransformToFrame(rospy.Time.now(), 'ur_arm_base', True)
        if entity.EEntityType.CUBE == target.entity_type:  
            target.correct_basis()
        else:
            target.correct_basis(aware_y=True)
        ori = self.robot.Basis2Rotvec([comp*i for comp, i in zip(target.basis, [-1, 1, -1])]).tolist()
        plan = [
            lambda : manipulator_control(self.robot.MoveL, target.rel2absolute([0, 0, 0.07]).tolist() + ori , 0.5, 0.1),
            lambda : manipulator_control(self.robot.MoveL,  target.rel2absolute([0, 0, 0.0]).tolist() + ori, 0.5, 0.1),
            lambda : self.robot.CloseGripper(0.0),
            lambda : rospy.sleep(1),
            lambda : manipulator_control(self.robot.Fold, None, 0.2, 0.1)
        ]
        for cmd in plan: cmd()
        
        rs = PickupObjectActionResult('', '', '')
        self.pick_act.set_succeeded(rs)
        return rs


    @_robot_action
    def cb_put_object(self, msg :PutObjectGoal):
        is_async = True
        this_action = self.place_act
        def manipulator_control(func, pose, speed = 0.2, acc = 0.1):
            if pose is not None:
                func(pose, speed*Consts.M_J_SPD_SCALE, acc*Consts.M_J_SPD_SCALE, is_async)
            else:
                func(speed*Consts.M_J_SPD_SCALE, acc*Consts.M_J_SPD_SCALE, is_async)
            if is_async:
                self._wait_manipulator(this_action)
        fb = PutObjectActionFeedback()
        self.place_act.publish_feedback(fb)
        target = entity.Container(self.bh.dynamic_ctx.locations[f'{msg.location}_position'], np.eye(3), frame = 'local_map_lidar')
        target = target.TransformToFrame(rospy.Time.now(), 'ur_arm_base', True)
        target.basis = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]])
        ori = self.robot.Basis2Rotvec([comp*i for comp, i in zip(target.basis, [-1, 1, -1])]).tolist()
        
        plan = [
            lambda : manipulator_control(self.robot.MoveJ, self.robot.INITIAL_JOINTS, 0.3, 0.2), 
            lambda : manipulator_control(self.robot.MoveL, target.rel2absolute([0,0,0]).tolist() + ori, 0.5, 0.1), 
            lambda : self.robot.OpenGripper(), 
            lambda : manipulator_control(self.robot.Fold, None, 0.3, 0.1)
        ]
        for cmd  in plan: cmd()
        
       
        rs = PutObjectActionResult('', '', '')
        self.place_act.set_succeeded(rs)
        return rs


    @_robot_action
    def cb_open_drawer(self, msg):
        self.robot.DeactivateTeachMode()
        self.robot.OpenGripper()
        cp_drawer = self.bh.update().boxes
        _, drawer = cp_drawer.popitem()
        handle_position = drawer.rel2absolute([0.03, 0.13, 0.025])
        ori = self.robot.Basis2Rotvec([-drawer.basis[0], drawer.basis[1], -drawer.basis[2]])
        target = [*handle_position, *ori]
        rospy.logdebug(f'handle = {target}')
        self.robot.MoveJ(self.robot.INITIAL_JOINTS, 0.5, 0.5)
        self.robot.MoveL(target, 0.1, 0.1)
        self.robot.CloseGripper()
        open_position = handle_position + drawer.rel2absolute([0, 0, 0.2])
        while np.linalg.norm((open_position - self.robot.GetActualTCPPose()[:3])) < 0.2 and not rospy.is_shutdown():
            force = [0, 0, -70, 0, 0, 0]
            selector = [0, 0, 1, 0, 0, 0]
            wrench =  [1]*6
            self.robot.ForceMode(target, selector, force, 2, wrench)
        self.robot.ForceModeStop()
        self.robot.OpenGripper()
        self.robot.Fold(0.3, 0.1)
        
        
    @_robot_action
    def place_box_near(self, msg):
        self.robot.MoveJ(Consts.J_RIGHT_ROBOT_CORNER, 0.2, 0.1)
        self.robot.OpenGripper()
        self.robot.Fold(0.3, 0.1)


    @_robot_action
    def place_in_drawer(self, msg):
        self.place_box_near(None)
        self.cb_open_drawer(None)
        self.cb_pickup(None, Consts.J_WATCH_RIGHT_ROBOT_CORNER)

        rospy.logwarn('place')
        self.robot.DeactivateTeachMode()
        cp_box = self.bh.update().boxes
        self.robot.MoveJ(self.robot.INITIAL_JOINTS)
        _, target = cp_box.popitem()
        target : entity.Box = target
        on_top_drawer = target.rel2absolute([-0.25, 0.13, -0.05]).tolist()
        ori = self.robot.Basis2Rotvec([-target.basis[0], target.basis[1], -target.basis[2]]).tolist()
        self.robot.MoveL(on_top_drawer+ori, 0.3, 0.1)
        self.robot.OpenGripper()
        self.robot.MoveJ(self.robot.INITIAL_JOINTS, 0.3, 0.1)

        self.robot.Fold()
        rs = PutObjectActionResult('', '', '')
        self.place_act.set_succeeded(rs)



    def cancel(self):
        self.robot.ActivateTeachMode()
        self.robot.OpenGripper()



if __name__ == "__main__":
    n = Node()
    rospy.spin()