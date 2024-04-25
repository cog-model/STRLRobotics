import argparse
import rospy
import rostopic
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CompressedImage
from husky_tidy_bot_cv.msg import Objects
from vild import VILD_CLIP
from conversions import to_objects_msg
from kas_utils.visualization import draw_objects
import cv2
import numpy as np
from kas_utils.time_measurer import TimeMeasurer


def build_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument('-cats', '--categories', type=str, nargs='+', required=True)
    parser.add_argument('-vis', '--enable-visualization', action='store_true')
    return parser


class VILD_CLIP_node(VILD_CLIP):
    def __init__(self, categories, image_topic, out_segmentation_topic,
            out_visualization_topic=None, min_score=0.7, palette=((0, 0, 255),)):
        super().__init__("vild", min_score=min_score)

        self.categories = categories
        self.image_topic = image_topic
        self.out_segmentation_topic = out_segmentation_topic
        self.out_visualization_topic = out_visualization_topic
        self.palette = palette

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
                cv2.imwrite('tmp.jpg', image)
                scores, roi_scores, classes_ids, boxes, masks = \
                    self.segment('tmp.jpg', self.categories + ['background'],
                        reject_categories=['background'])

            with self.to_ros_tm:
                tracking_ids = np.empty((0,), dtype=int)
                objects_msg = to_objects_msg(image_msg.header, scores, classes_ids,
                    tracking_ids, boxes, masks)
            self.segmentation_pub.publish(objects_msg)

            if self.out_visualization_topic is not None:
                with self.vis_tm:
                    vis = image.copy()
                    draw_objects(vis, scores, classes_ids, masks=masks,
                        draw_scores=True, draw_ids=True, draw_masks=True,
                        palette=self.palette)
                vis_msg = self.bridge.cv2_to_imgmsg(vis, encoding='bgr8')
                vis_msg.header = image_msg.header
                self.visualization_pub.publish(vis_msg)


if __name__ == '__main__':
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
    segmentation_node = VILD_CLIP_node(args.categories,
        "/realsense_gripper/color/image_raw/compressed",
        "/segmentation", out_visualization_topic=out_visualization_topic,
        min_score=0.5)
    segmentation_node.start()

    print("Spinning...")
    rospy.spin()

    print()
    del segmentation_node
