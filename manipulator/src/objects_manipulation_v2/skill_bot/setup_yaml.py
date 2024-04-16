#!/usr/bin/python3
import yaml
import numpy as np
from scipy.spatial.transform import Rotation

import rospy
from std_msgs.msg import String

if __name__ == '__main__':
    rospy.init_node('yaml_updater')

from skill_bot.robo_cleaner import Node 
from skill_bot.utils.utils_ import ParamProvider


class YamlSetupNode(Node):
    def __init__(self) -> None:
        super().__init__()
        self.save_sub = rospy.Subscriber('~save', String, self.cb_save)
        self.robot.ActivateTeachMode()
        
        
        
    def cb_save(self, msg : String):
        rospy.loginfo('REQ RECIEVED')
        ctx = self.bh.update()
        name = msg.data
        key, box = ctx.boxes.popitem()
        global_box = box.TransformToFrame(rospy.Time.now(), 'local_map_lidar')
        
        with open(ParamProvider.scene_config_path, 'r') as fd:
            cur_file = yaml.safe_load(fd)
        cur_file.update({f'{name}_position': global_box.position.tolist()})
        rospy.loginfo(f'UPDATE YAML WITH [{name}_position: {global_box.position}]')
        pos, rot_q = self.tf_listener.lookupTransform('local_map_lidar',  'base_link', rospy.Time(0))# позиция работает ориентация не верная или нет
        arrow = Rotation.from_quat(rot_q).as_matrix()[:2, :2] @ [1, 0]
        theta  = np.arctan2(arrow[1],  arrow[0]) 
        cur_file.update({name: [*pos[:2], 0, float(theta)]})
        rospy.loginfo(f'UPDATE YAML WITH [{name}: {[*pos[:2], 0, float(theta)]}]') 
        
        with open(ParamProvider.scene_config_path, 'w')as fd:
            yaml.dump(cur_file, fd)
            
            
        



if __name__ == "__main__":
    rospy.loginfo('UTILITY FOR UPDATING SCENE CONFIG FILE')
    rospy.loginfo(f'file: {ParamProvider.scene_config_path}')
    rospy.loginfo('START LOADING..')
    n = YamlSetupNode()
    rospy.loginfo('LOAD COMPLETED')
    rospy.loginfo('Point the camera at the object and')
    rospy.loginfo(f'send name in topic: {n.save_sub.resolved_name}')
    rospy.spin()