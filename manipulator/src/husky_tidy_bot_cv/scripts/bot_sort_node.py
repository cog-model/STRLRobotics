import argparse
import rospy
import rostopic
import message_filters
from sensor_msgs.msg import Image, CompressedImage
from husky_tidy_bot_cv.msg import Objects
from cv_bridge import CvBridge
from conversions import from_objects_msg, to_objects_msg
import numpy as np
import cv2
from bot_sort import BoTSORT_wrapper
from kas_utils.time_measurer import TimeMeasurer
from kas_utils.visualization import draw_objects
from kas_utils.masks import get_full_masks
import time
import os
import os.path as osp


def build_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument('-vis', '--enable-visualization', action='store_true')
    return parser


class BoTSORT_node(BoTSORT_wrapper):
    def __init__(self, segmentation_topic, image_topic, out_tracking_topic,
            out_visualization_topic=None,
            track_high_thresh=0.3, track_low_thresh=0.1,
            new_track_thresh=0.3, track_buffer_size=30, match_thresh=0.8,
            cmc_method="sparseOptFlow", with_reid=False,
            palette=((0, 0, 255), (0, 255, 0), (255, 0, 0), (255, 0, 255), (0, 255, 255))):

        super().__init__(track_high_thresh=track_high_thresh,
            track_low_thresh=track_low_thresh, new_track_thresh=new_track_thresh,
            track_buffer_size=track_buffer_size, match_thresh=match_thresh,
            cmc_method=cmc_method, with_reid=with_reid)

        self.segmentation_topic = segmentation_topic
        self.image_topic = image_topic
        self.out_tracking_topic = out_tracking_topic
        self.out_visualization_topic = out_visualization_topic
        self.palette = palette

        self.tracking_pub = rospy.Publisher(self.out_tracking_topic, Objects, queue_size=10)
        if self.out_visualization_topic:
            self.visualization_pub = rospy.Publisher(self.out_visualization_topic, Image, queue_size=10)
        else:
            self.visualization_pub = None

        self.bridge = CvBridge()

        self.from_ros_tm = TimeMeasurer("  from ros")
        self.track_tm = TimeMeasurer("  track")
        self.to_ros_tm = TimeMeasurer("  to ros")
        self.vis_tm = TimeMeasurer("  visualization")
        self.total_tm = TimeMeasurer("total")

        self.input_stamps = list()
        self.input_delays = list()
        self.output_stamps = list()
        self.output_delays = list()

    def start(self):
        self.segmentation_sub = message_filters.Subscriber(self.segmentation_topic, Objects)
        image_topic_type, _, _ = rostopic.get_topic_class(self.image_topic)
        self.image_sub = message_filters.Subscriber(self.image_topic, image_topic_type)
        self.sync_sub = message_filters.TimeSynchronizer(
            [self.segmentation_sub, self.image_sub], 10)
        self.sync_sub.registerCallback(self.callback)

    def callback(self, segmentation_objects_msg, image_msg):
        with self.total_tm:
            input_delay = rospy.get_rostime() - segmentation_objects_msg.header.stamp
            self.input_stamps.append(segmentation_objects_msg.header.stamp)
            self.input_delays.append(input_delay)

            with self.from_ros_tm:
                scores, classes_ids, _, boxes, masks_in_rois, rois, widths, heights = \
                    from_objects_msg(segmentation_objects_msg)

                if image_msg._type == "sensor_msgs/Image":
                    image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
                elif image_msg._type == "sensor_msgs/CompressedImage":
                    image = self.bridge.compressed_imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
                else:
                    raise RuntimeError("Unkown message type")

            with self.track_tm:
                infos = [(mask_in_roi, roi, width, height) \
                    for mask_in_roi, roi, width, height in \
                        zip(masks_in_rois, rois, widths, heights)]
                infos = np.array(infos, dtype=object)
                tracked_objects = self.track(boxes, scores, classes_ids, image, infos=infos)
                scores, classes_ids, tracking_ids, boxes, infos = \
                    BoTSORT_wrapper.preprocess(tracked_objects)

            with self.to_ros_tm:
                if len(infos) > 0:
                    masks_in_rois, rois, widths, heights = zip(*infos)
                else:
                    masks_in_rois, rois, widths, heights = [tuple()] * 4
                tracked_objects_msg = to_objects_msg(
                    segmentation_objects_msg.header, scores, classes_ids, tracking_ids, boxes,
                    masks_in_rois, rois, widths, heights)
            output_delay = rospy.get_rostime() - tracked_objects_msg.header.stamp
            self.output_stamps.append(tracked_objects_msg.header.stamp)
            self.output_delays.append(output_delay)
            self.tracking_pub.publish(tracked_objects_msg)

            if self.visualization_pub is not None:
                with self.vis_tm:
                    vis = image.copy()
                    masks = get_full_masks(masks_in_rois, rois, widths, heights)
                    customs = list(zip(classes_ids, tracking_ids))
                    draw_objects(vis, scores, tracking_ids, masks=masks, customs=customs,
                        draw_masks=True, format="{s:.02f}, c: {c[0]}, t: {c[1]}",
                        palette=self.palette, color_by_object_id=True)
                vis_msg = self.bridge.cv2_to_imgmsg(vis, encoding='bgr8')
                vis_msg.header = segmentation_objects_msg.header
                self.visualization_pub.publish(vis_msg)


if __name__ == "__main__":
    time_str = time.strftime("%Y-%m-%d_%H.%M.%S")

    parser = build_parser()
    args, unknown_args = parser.parse_known_args()
    for i in range(len(unknown_args)-1, -1, -1):
        if unknown_args[i].startswith('__name:=') or unknown_args[i].startswith('__log:='):
            del unknown_args[i]
    if len(unknown_args) > 0:
        raise RuntimeError("Unknown args: {}".format(unknown_args))

    rospy.init_node("tracking")
    if args.enable_visualization:
        out_visualization_topic = "/tracking_vis"
    else:
        out_visualization_topic = None
    tracking_node = BoTSORT_node("/segmentation",
        "/realsense_gripper/color/image_raw/compressed",
        "/tracking",
        out_visualization_topic=out_visualization_topic)
    tracking_node.start()

    print("Spinning...")
    rospy.spin()

    if len(tracking_node.output_delays) > 0:
        logs_folder = osp.abspath(osp.expanduser(osp.join(
            osp.dirname(__file__), "logs/bot_sort_node")))
        os.makedirs(logs_folder, exist_ok=True)
        with open(osp.join(logs_folder, f"{time_str}_input.txt"), 'w') as f:
            lines = [f"{str(stamp)} {str(delay)}\n" for stamp, delay in \
                zip(tracking_node.input_stamps, tracking_node.input_delays)]
            lines[-1] = lines[-1][:-1]
            f.writelines(lines)
        with open(osp.join(logs_folder, f"{time_str}_output.txt"), 'w') as f:
            lines = [f"{str(stamp)} {str(delay)}\n" for stamp, delay in \
                zip(tracking_node.output_stamps, tracking_node.output_delays)]
            lines[-1] = lines[-1][:-1]
            f.writelines(lines)

    print()
    del tracking_node
