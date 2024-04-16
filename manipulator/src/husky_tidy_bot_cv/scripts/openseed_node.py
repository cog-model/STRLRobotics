import argparse
import rospy
import rostopic
from sensor_msgs.msg import Image
from husky_tidy_bot_cv.msg import Objects
from cv_bridge import CvBridge
from conversions import to_objects_msg, from_clasess_msg
import numpy as np
import sys
from openseed_model import OpenSeeD_wrapper
from kas_utils.time_measurer import TimeMeasurer
from kas_utils.visualization import draw_objects
from kas_utils.masks import get_masks_rois, get_masks_in_rois

from fixed_cats import FIXED_CATEGORIES

def build_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument('-vis', '--enable-visualization', action='store_true')
    return parser


class OpenSeeD_node(OpenSeeD_wrapper):
    def __init__(self, categories, model_file, weights_file, image_topic, out_segmentation_topic, labels_topic=None, 
            out_visualization_topic=None, min_score_seen=0.7, min_score_unseen=0.1, palette=((0, 0, 255),)):

        super().__init__(model_file, weights_file, min_score_seen=min_score_seen, min_score_unseen=min_score_unseen)
        self.categories = categories 
        self.categories_with_features = categories
        self.categories_with_features_ids = [i for i in range(len(self.categories))]
        self.image_topic = image_topic
        self.out_segmentation_topic = out_segmentation_topic
        self.out_visualization_topic = out_visualization_topic
        self.labels_topic = labels_topic
        self.palette = (
            (0, 0, 255), (180, 120, 120),
            (6, 230, 230), (80, 50, 50), (4, 200, 3), (120, 120, 80),
            (140, 140, 140), (204, 5, 255), (230, 230, 230), (4, 250, 7),
            (224, 5, 255), (235, 255, 7), (150, 5, 61), (120, 120, 70),
            (8, 255, 51), (255, 6, 82), (143, 255, 140), (204, 255, 4),
            (255, 51, 7), (204, 70, 3), (0, 102, 200), (61, 230, 250),
            (255, 6, 51), (11, 102, 255), (255, 7, 71), (255, 9, 224),
            (9, 7, 230), (220, 220, 220), (255, 9, 92), (112, 9, 255),
            (8, 255, 214), (7, 255, 224), (255, 184, 6), (10, 255, 71),
            (255, 41, 10), (7, 255, 255), (224, 255, 8), (102, 8, 255),
            (255, 61, 6), (255, 194, 7), (255, 122, 8), (0, 255, 20),
            (255, 8, 41), (255, 5, 153), (6, 51, 255), (235, 12, 255),
            (160, 150, 20), (0, 163, 255), (140, 140, 140), (250, 10, 15),
            (20, 255, 0), (31, 255, 0), (255, 31, 0), (255, 224, 0),
            (153, 255, 0), (255, 71, 0), (0, 235, 255),
            (0, 173, 255), (31, 0, 255), (11, 200, 200), (255, 82, 0),
            (0, 255, 245), (0, 61, 255), (0, 255, 112), (0, 255, 133),
            (255, 0, 0), (255, 163, 0), (255, 102, 0), (194, 255, 0),
            (0, 143, 255), (51, 255, 0), (0, 82, 255), (0, 255, 41),
            (0, 255, 173), (10, 0, 255), (173, 255, 0),
            (0, 255, 153), (255, 92, 0), (255, 0, 255), (255, 0, 245),
            (255, 0, 102), (255, 173, 0), (255, 0, 20), (255, 184, 184),
            (0, 31, 255), (0, 255, 61), (0, 71, 255), (255, 0, 204),
            (0, 255, 194), (0, 255, 82), (0, 10, 255), (0, 112, 255),
            (51, 0, 255), (0, 194, 255), (0, 122, 255), (0, 255, 163),
            (255, 153, 0), (0, 255, 10), (255, 112, 0), (143, 255, 0),
            (82, 0, 255), (163, 255, 0), (255, 235, 0), (8, 184, 170),
            (133, 0, 255), (0, 255, 92), (184, 0, 255), (255, 0, 31),
            (0, 184, 255), (0, 214, 255), (255, 0, 112), (92, 255, 0),
            (0, 224, 255), (112, 224, 255), (70, 184, 160), (163, 0, 255),
            (153, 0, 255), (71, 255, 0), (255, 0, 163), (255, 204, 0),
            (255, 0, 143), (0, 255, 235), (133, 255, 0), (255, 0, 235),
            (245, 0, 255), (255, 0, 122), (255, 245, 0), (10, 190, 212),
            (214, 255, 0), (0, 204, 255), (20, 0, 255), (255, 255, 0),
            (0, 153, 255), (0, 41, 255), (0, 255, 204), (41, 0, 255),
            (41, 255, 0), (173, 0, 255), (0, 245, 255), (71, 0, 255),
            (122, 0, 255), (0, 255, 184), (0, 92, 255), (184, 255, 0),
            (0, 133, 255), (255, 214, 0), (25, 194, 194), (102, 255, 0),
            (92, 0, 255)
        )

        self.segmentation_pub = rospy.Publisher(
            self.out_segmentation_topic, Objects, queue_size=10)

        if self.out_visualization_topic:
            self.visualization_pub = rospy.Publisher(
                self.out_visualization_topic, Image, queue_size=10)
        else:
            self.visualization_pub = None

        self.bridge = CvBridge()

        self.from_ros_tm = TimeMeasurer("  from ros")
        self.segm_tm = TimeMeasurer("  segment")
        self.to_ros_tm = TimeMeasurer("  to ros")
        self.vis_tm = TimeMeasurer("  visualization")
        self.total_tm = TimeMeasurer("total")

    def start(self):
        image_topic_type, _, _ = rostopic.get_topic_class(self.image_topic)
        self.image_sub = rospy.Subscriber(
            self.image_topic, image_topic_type, self.callback,
            queue_size=1, buff_size=2 ** 24)

        cats_topic_type, _, _ = rostopic.get_topic_class(self.labels_topic)
        self.cats_sub = rospy.Subscriber(
            self.labels_topic, cats_topic_type, self.callback_cats,
            queue_size=1, buff_size=2 ** 24)

    def callback(self, image_msg):
        with self.total_tm:
            with self.from_ros_tm:
                if image_msg._type == "sensor_msgs/Image":
                    image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
                elif image_msg._type == "sensor_msgs/CompressedImage":
                    image = self.bridge.compressed_imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
                else:
                    raise RuntimeError("Unkown message type")

            with self.segm_tm:
                scores, classes_ids, boxes, masks = \
                    self.segment(image, self.categories,
                                 self.categories_with_features,
                                 self.categories_with_features_ids,
                                 reject_categories=['background'])

            with self.to_ros_tm:
                rois = get_masks_rois(masks)
                masks_in_rois = get_masks_in_rois(masks, rois)
                height, width = image.shape[:2]
                segmentation_objects_msg = to_objects_msg(
                    image_msg.header, scores, classes_ids, np.empty((0,)), boxes,
                    masks_in_rois, rois, width, height)

            self.segmentation_pub.publish(segmentation_objects_msg)

            if self.visualization_pub is not None:
                with self.vis_tm:
                    vis = image.copy()
                    draw_objects(vis, scores, classes_ids, boxes=boxes, masks=masks,
                        draw_scores=True, draw_ids=True, draw_masks=True, draw_boxes=True, palette=self.palette, color_by_object_id=True)
                vis_msg = self.bridge.cv2_to_imgmsg(vis, encoding='bgr8')
                vis_msg.header = image_msg.header
                self.visualization_pub.publish(vis_msg)

    def callback_cats(self, cats_msg):
        self.categories, self.categories_with_features, \
            self.categories_with_features_ids = from_clasess_msg(cats_msg)

        labels_dict = {
            cat_id: cat_name
            for (cat_id, cat_name) in zip(self.categories_with_features_ids, self.categories_with_features)
        }
        rospy.loginfo(f'Changed OpenSeeD categories to {labels_dict}')



if __name__ == "__main__":
    parser = build_parser()
    args, unknown_args = parser.parse_known_args()
    for i in range(len(unknown_args)-1, -1, -1):
        if unknown_args[i].startswith('__name:=') or unknown_args[i].startswith('__log:='):
            del unknown_args[i]
    if len(unknown_args) > 0:
        raise RuntimeError("Unknown args: {}".format(unknown_args))

    rospy.init_node("segmentation")
    if args.enable_visualization:
        out_visualization_topic = "/segmentation_vis"
    else:
        out_visualization_topic = None
    
    categories = [cat["name"] for cat in FIXED_CATEGORIES]
    
    segmentation_node = OpenSeeD_node(categories,
        "/home/administrator/zemskova_ts/husky_tidy_bot_cv_ws/src/openseed_src/openseed_swint_lang_rosbag.yaml",
        "/home/administrator/zemskova_ts/husky_tidy_bot_cv_ws/src/openseed_src/model_final.pth",
#        "/home/wingrune/cv/OpenSeeD/configs/openseed/openseed_swint_lang_rosbag.yaml",
#        "/hdd/wingrune/openseed/output_finetune/model_0003599.pth",
        "/realsense_gripper/color/image_raw/compressed", "/segmentation", labels_topic="/segmentation_labels",
        out_visualization_topic=out_visualization_topic, min_score_seen=0.5, min_score_unseen=0.1)
    segmentation_node.start()

    print("Spinning...")
    rospy.spin()

    print()
    del segmentation_node
