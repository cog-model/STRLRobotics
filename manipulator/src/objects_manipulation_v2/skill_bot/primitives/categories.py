from skill_bot.primitives.entity import *
from typing import Dict, AnyStr
import copy
from husky_tidy_bot_cv.msg import Categories
ROSBAG_CATEGORIES = [
        {'id': 0,  'class': Box     , 'name': 'cube',                       'supercategory': ''},
        {'id': 1,  'class': BinBox  , 'name': 'bin box basket container',   'supercategory': ''},
        {'id': 2,  'class': Table   , 'name': 'table',                      'supercategory': ''},
        {'id': 3,  'class': Entity  , 'name': 'chair',                      'supercategory': ''},
        {'id': 4,  'class': Entity  , 'name': 'person',                     'supercategory': ''},
        {'id': 5,  'class': Entity  , 'name': 'nightstand',                 'supercategory': ''},
        {'id': 6,  'class': Entity  , 'name': 'drawer',                     'supercategory': ''},
        {'id': 7,  'class': Entity  , 'name': 'handle',                     'supercategory': ''},
        {'id': 8,  'class': Entity  , 'name': 'sink',                       'supercategory': ''},
        {'id': 9,  'class': Entity  , 'name': 'cucumber',                   'supercategory': ''},
        {'id': 10, 'class': Entity  , 'name': 'potato',                     'supercategory': ''},
        {'id': 11, 'class': Entity  , 'name': 'chili pepper',               'supercategory': ''},
        {'id': 12, 'class': Entity  , 'name': 'bell pepper',                'supercategory': ''},
        {'id': 13, 'class': Entity  , 'name': 'corn',                       'supercategory': ''},
        {'id': 14, 'class': Entity  , 'name': 'eggplant',                   'supercategory': ''},
        {'id': 15, 'class': Entity  , 'name': 'carrot',                     'supercategory': ''},
        {'id': 16, 'class': Entity  , 'name': 'garlic', 'supercategory': ''},
        {'id': 17, 'class': Entity  , 'name': 'tomato', 'supercategory': ''},
        {'id': 18, 'class': Entity  , 'name': 'toy cat', 'supercategory': ''},
        {'id': 19, 'class': Entity  , 'name': 'plates', 'supercategory': ''},
        {'id': 20, 'class': Entity  , 'name': 'mug', 'supercategory': ''},
        {'id': 21, 'class': Entity  , 'name': 'cup', 'supercategory': ''},
        {'id': 22, 'class': Entity  , 'name': 'crocs', 'supercategory': ''},
        {'id': 23, 'class': Entity  , 'name': 'slippers', 'supercategory': ''},
        {'id': 24, 'class': Entity  , 'name': 'hat with earflaps', 'supercategory': ''},
        {'id': 25, 'class': Entity  , 'name': 't-shirt', 'supercategory': ''},
        {'id': 26, 'class': Entity  , 'name': 'hoodie', 'supercategory': ''},
        {'id': 27, 'class': Entity  , 'name': 'towel', 'supercategory': ''},
        {'id': 28, 'class': Entity  , 'name': 'socks', 'supercategory': ''},
        {'id': 29, 'class': Entity  , 'name': 'hat', 'supercategory': ''},
        {'id': 30, 'class': Entity  , 'name': 'chips', 'supercategory': ''},
        {'id': 31, 'class': Entity  , 'name': 'snickers', 'supercategory': ''},
        {'id': 32, 'class': Entity  , 'name': 'bottle', 'supercategory': ''},
        {'id': 33, 'class': Entity  , 'name': 'cans', 'supercategory': ''},
        {'id': 34, 'class': Entity  , 'name': 'plastic bags', 'supercategory': ''},
        {'id': 35, 'class': Entity  , 'name': 'charging', 'supercategory': ''},
        {'id': -1, 'class': Entity  , 'name': 'unknown'}
    ]
class CategoriesClass:
    def __init__(self) -> None:
        self._sub = rospy.Subscriber('/segmentation_labels', Categories, self.cb_categories, queue_size=1)
        self._id2l = { info['id'] : info['name'] for info in ROSBAG_CATEGORIES}
        self._l2id = {info['name'] : info['id'] for info in ROSBAG_CATEGORIES}
        self._spec_class = {
            0 : Box,
            1 : BinBox
        }
    
    
    @property
    def id2label(self) -> Dict[int, AnyStr]:
        return copy.copy(self._id2l)
    
    
    @property
    def label2id(self) -> Dict[AnyStr, int]:
        return copy.copy(self._l2id)
    
    def label2Class(self, label):
        if self._spec_class.get(self.label2id[label]):
            return self._spec_class.get(self.label2id[label])
        else:
            return Entity
    
    
    
    def id2Class(self, class_id):
        if self._spec_class.get(class_id):
            return self._spec_class.get(class_id)
        else:
            return Entity
    
    
    
    def cb_categories(self, msg : Categories):
        rospy.loginfo_once(f'CATEGORY UPDATE\n{msg}')
        self._id2l = { class_id : label for label, class_id in zip(msg.labels, msg.classes_ids) }
        self._l2id = { label: class_id for label, class_id in zip(msg.labels, msg.classes_ids) }
            
CATEGORIES = CategoriesClass()