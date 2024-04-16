#! /usr/bin/python3
import os
import numpy as np
import yaml


import rospy
if __name__ == '__main__':
    rospy.init_node('pc_recorder', log_level=rospy.INFO)

from skill_bot.robot.husky import Robot 
from skill_bot.utils.utils_ import ParamProvider
from skill_bot.planner.ctx_manager import CtxManager, By, Rule
from husky_tidy_bot_cv.msg import Objects
from skill_bot.primitives.categories import CATEGORIES

class RecorderNode:
    def __init__(self) -> None:
        self.robot = Robot(ParamProvider.ur_ip)
        self.bh = CtxManager()
        self.labels = []
        self.point_clouds = []
        self.grasp_poses = []
        self.base_path = 'default'
        rospy.on_shutdown(self.on_shutdown)
        
    def save(self, base_path):
        if not os.path.exists(base_path):
            os.mkdir(base_path)
        labels = np.array(self.labels)
        objects_pc = np.array(self.point_clouds)
        grasp_poses = np.array(self.grasp_poses)
        np.save(os.path.join(base_path, 'labels.npy'), labels)
        np.save(os.path.join(base_path, 'ply.npy'), objects_pc)
        np.save(os.path.join(base_path, 'grasp.npy'), grasp_poses)
        uniq_label  = np.unique(labels)
        statistic = {'labels' : uniq_label}
        for l in labels:
            statistic.update({str(l) : int(labels[labels == l].shape[0])})
        with open(os.path.join(base_path, 'stat.yaml'), 'w') as fd:
            yaml.dump(statistic, fd)
    
    def load(self, base_path):
        self.labels = np.load(os.path.join(base_path, 'labels.npy'))
        self.point_clouds = np.load(os.path.join(base_path, 'ply.npy'))
        self.grasp_poses = np.load(os.path.join(base_path, 'grasp.npy'))
        with open(os.path.join(base_path, 'stat.yaml'), 'r') as fd:
            stats = yaml.load(fd)
        return stats
    
    
    def recorder_pipeline(self):
        self.robot.ActivateTeachMode()
        while not rospy.is_shutdown():
            print('Create or load?')
            print('l - load')
            print('n - new')
            prmpt = input('[N/l]> ')
            prmpt = prmpt.lower()
            if prmpt == '' or prmpt == 'n':
                load = False
                break
            elif prmpt == 'l':
                load = False
                break
        print('Enter path')
        base_path = input('> ')
        self.base_path = base_path
        if load: 
            stats = self.load(base_path)
            print('loaded')
            for i, (k,v) in enumerate(stats.items()):
                if k == 'labels' : continue
                print(f"{stats['labels'][i]} ({v})")
                
        while not rospy.is_shutdown():
            input('direct camera to object and press Enter > ')
            segmented_objects : Objects = rospy.wait_for_message('/tracking', Objects, rospy.Duration(4))
            selected_indx = 0
            print(segmented_objects.tracking_ids, segmented_objects.classes_ids)
            if len(segmented_objects.tracking_ids) > 1:
                print('multiple objects, select one:')
                for i, (track_id, class_id) in enumerate(zip(segmented_objects.tracking_ids, segmented_objects.classes_ids)):
                    print(f'[{i}] - {CATEGORIES.id2label[class_id]}')
                selected_indx = int(input('> '))
            elif len(segmented_objects.tracking_ids) == 0:
                continue
            target_id = segmented_objects.tracking_ids[selected_indx]
            self.point_clouds.append(self.bh.numpy_object_pc(target_id, try_count = 3))
            self.labels.append(CATEGORIES.id2label[segmented_objects.classes_ids[selected_indx]])
            input('move to grasp pose and press Enter > ')
            self.grasp_poses.append(self.robot.GetActualTCPPose())
            
            
        # dsegmented_objects : Objects = rospy.wait_for_message('/segmentation', Objects)
        
        
    def on_shutdown(self):
        self.save(self.base_path)
        
    
if __name__ == '__main__':
    node = RecorderNode()
    node.recorder_pipeline()
    

