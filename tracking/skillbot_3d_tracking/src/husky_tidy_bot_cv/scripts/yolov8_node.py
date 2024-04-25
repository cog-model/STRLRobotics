import argparse
import rospy
import rostopic
from sensor_msgs.msg import Image
from husky_tidy_bot_cv.msg import Objects
from cv_bridge import CvBridge
from conversions import to_objects_msg
import numpy as np
import cv2
from yolov8 import YOLOv8_wrapper
from kas_utils.time_measurer import TimeMeasurer
from kas_utils.visualization import draw_objects
from kas_utils.masks import get_masks_rois, get_masks_in_rois
import time
import os
import os.path as osp


def build_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument('--node-name', type=str, default="segmentation")
    parser.add_argument('--image-topic', type=str)
    parser.add_argument('--out-segmentation-topic', type=str)
    parser.add_argument('--out-visualization-topic', type=str)

    camera = parser.add_mutually_exclusive_group()
    camera.add_argument('-real', '--realsense', action='store_true')
    camera.add_argument('-zed', '--zed', action='store_true')
    parser.add_argument('-comp', '--compressed', action='store_true')

    parser.add_argument('-vis', '--enable-visualization', action='store_true')
    return parser


class YOLOv8_node(YOLOv8_wrapper):
    def __init__(self, model_file, weights_file, image_topic, out_segmentation_topic,
            out_visualization_topic=None, min_score=0.7, palette=((0, 0, 255),)):

        super().__init__(model_file, weights_file, min_score=min_score)

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

        self.input_stamps = list()
        self.input_delays = list()
        self.output_stamps = list()
        self.output_delays = list()

    def start(self):
        image_topic_type, _, _ = rostopic.get_topic_class(self.image_topic)
        self.image_sub = rospy.Subscriber(self.image_topic, image_topic_type, self.callback,
            queue_size=1, buff_size=2 ** 24)

    def callback(self, image_msg):
        with self.total_tm:
            input_delay = rospy.get_rostime() - image_msg.header.stamp
            self.input_stamps.append(image_msg.header.stamp)
            self.input_delays.append(input_delay)

            with self.from_ros_tm:
                if image_msg._type == "sensor_msgs/Image":
                    image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
                elif image_msg._type == "sensor_msgs/CompressedImage":
                    image = self.bridge.compressed_imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
                else:
                    raise RuntimeError("Unkown message type")

            with self.segm_tm:
                scores, classes_ids, boxes, masks = self.segment(image)

            with self.to_ros_tm:
                rois = get_masks_rois(masks)
                masks_in_rois = get_masks_in_rois(masks, rois)
                height, width = image.shape[:2]
                segmentation_objects_msg = to_objects_msg(
                    image_msg.header, scores, classes_ids, np.empty((0,)), boxes,
                    masks_in_rois, rois, width, height)
            output_delay = rospy.get_rostime() - segmentation_objects_msg.header.stamp
            self.output_stamps.append(segmentation_objects_msg.header.stamp)
            self.output_delays.append(output_delay)
            self.segmentation_pub.publish(segmentation_objects_msg)

            if self.visualization_pub is not None:
                with self.vis_tm:
                    vis = image.copy()
                    draw_objects(vis, scores, classes_ids, masks=masks,
                        draw_scores=True, draw_masks=True, palette=self.palette)
                vis_msg = self.bridge.cv2_to_imgmsg(vis, encoding='bgr8')
                vis_msg.header = image_msg.header
                self.visualization_pub.publish(vis_msg)


def complete_args(args):
    assert not (args.realsense and args.zed)

    camera_selected = (args.realsense or args.zed)
    if args.image_topic is None and not camera_selected:
        args.realsense = True
        camera_selected = True

    if args.realsense:
        args.image_topic = "/realsense_gripper/color/image_raw"
    if args.zed:
        args.image_topic = "/zed_node/left/image_rect_color"
    if args.compressed and camera_selected:
        args.image_topic += "/compressed"

    if args.out_segmentation_topic is None:
        args.out_segmentation_topic = "/segmentation"

    if not args.enable_visualization:
        args.out_visualization_topic = None
    if args.enable_visualization and args.out_visualization_topic is None:
        args.out_visualization_topic = "/segmentation_vis"

    assert args.image_topic is not None
    assert args.out_segmentation_topic is not None


if __name__ == "__main__":
    time_str = time.strftime("%Y-%m-%d_%H.%M.%S")

    parser = build_parser()
    args, unknown_args = parser.parse_known_args()
    for i in range(len(unknown_args)-1, -1, -1):
        if unknown_args[i].startswith('__name:=') or unknown_args[i].startswith('__log:='):
            del unknown_args[i]
    if len(unknown_args) > 0:
        raise RuntimeError("Unknown args: {}".format(unknown_args))

    complete_args(args)

    rospy.init_node(args.node_name)
    segmentation_node = YOLOv8_node(
        "/home/administrator/krishtopik/husky_tidy_bot_cv_ws/src/husky_tidy_bot_cv/yolov8n/yolov8n-seg-1class.yaml",
        "/home/administrator/krishtopik/husky_tidy_bot_cv_ws/src/husky_tidy_bot_cv/yolov8n/yolov8n.pt",
        args.image_topic,

        args.out_segmentation_topic,
        out_visualization_topic=args.out_visualization_topic,
        min_score=0.5)
    segmentation_node.start()

    print("Spinning...")
    rospy.spin()

    if len(segmentation_node.output_delays) > 0:
        logs_folder = osp.abspath(osp.expanduser(osp.join(
            osp.dirname(__file__), "logs/yolov8_node")))
        os.makedirs(logs_folder, exist_ok=True)
        with open(osp.join(logs_folder, f"{time_str}_input.txt"), 'w') as f:
            lines = [f"{str(stamp)} {str(delay)}\n" for stamp, delay in \
                zip(segmentation_node.input_stamps, segmentation_node.input_delays)]
            lines[-1] = lines[-1][:-1]
            f.writelines(lines)
        with open(osp.join(logs_folder, f"{time_str}_output.txt"), 'w') as f:
            lines = [f"{str(stamp)} {str(delay)}\n" for stamp, delay in \
                zip(segmentation_node.output_stamps, segmentation_node.output_delays)]
            lines[-1] = lines[-1][:-1]
            f.writelines(lines)

    print()
    del segmentation_node
