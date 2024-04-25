import rospy
import tf2_ros
import message_filters
from std_msgs.msg import Header
from sensor_msgs.msg import CameraInfo, Image
from visualization_msgs.msg import Marker, MarkerArray
from husky_tidy_bot_cv.msg import Objects, Objects3d
from husky_tidy_bot_cv.srv import ResetRequest, ResetResponse, Reset
from cv_bridge import CvBridge
from conversions import from_objects_msg, to_objects3d_msg
from tracker_3d import Tracker3D
import numpy as np
from kas_utils.time_measurer import TimeMeasurer
import argparse
from ros_numpy.geometry import transform_to_numpy
from threading import Lock
import time
import os
import os.path as osp


ROSBAG_CATEGORIES = [
        {'id': 0, 'name': 'cube', 'supercategory': ''},
        {'id': 1, 'name': 'container', 'supercategory': ''},
        {'id': 2, 'name': 'table', 'supercategory': ''},
        {'id': 3, 'name': 'chair', 'supercategory': ''},
        {'id': 4, 'name': 'person', 'supercategory': ''},
        {'id': 5, 'name': 'nightstand', 'supercategory': ''},
        {'id': 6, 'name': 'drawer', 'supercategory': ''},
        {'id': 7, 'name': 'handle', 'supercategory': ''},
        {'id': 8, 'name': 'sink', 'supercategory': ''},
        {'id': 9, 'name': 'cucumber', 'supercategory': ''},
        {'id': 10, 'name': 'potato', 'supercategory': ''},
        {'id': 11, 'name': 'chili pepper', 'supercategory': ''},
        {'id': 12, 'name': 'bell pepper', 'supercategory': ''},
        {'id': 13, 'name': 'corn', 'supercategory': ''},
        {'id': 14, 'name': 'eggplant', 'supercategory': ''},
        {'id': 15, 'name': 'carrot', 'supercategory': ''},
        {'id': 16, 'name': 'garlic', 'supercategory': ''},
        {'id': 17, 'name': 'tomato', 'supercategory': ''},
        {'id': 18, 'name': 'toy cat', 'supercategory': ''},
        {'id': 19, 'name': 'plates', 'supercategory': ''},
        {'id': 20, 'name': 'mug', 'supercategory': ''},
        {'id': 21, 'name': 'cup', 'supercategory': ''},
        {'id': 22, 'name': 'crocs', 'supercategory': ''},
        {'id': 23, 'name': 'slippers', 'supercategory': ''},
        {'id': 24, 'name': 'hat with earflaps', 'supercategory': ''},
        {'id': 25, 'name': 't-shirt', 'supercategory': ''},
        {'id': 26, 'name': 'hoodie', 'supercategory': ''},
        {'id': 27, 'name': 'towel', 'supercategory': ''},
        {'id': 28, 'name': 'socks', 'supercategory': ''},
        {'id': 29, 'name': 'hat', 'supercategory': ''},
        {'id': 30, 'name': 'chips', 'supercategory': ''},
        {'id': 31, 'name': 'snickers', 'supercategory': ''},
        {'id': 32, 'name': 'bottle', 'supercategory': ''},
        {'id': 33, 'name': 'cans', 'supercategory': ''},
        {'id': 34, 'name': 'plastic bags', 'supercategory': ''},
        {'id': 35, 'name': 'charging', 'supercategory': ''},
    ]


CATEGORIES_TO_TRACK = ['cube', 'container', 'toy cat']


def build_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument('--objects-topics', type=str, nargs='+')
    parser.add_argument('--depth-info-topics', type=str, nargs='+')
    parser.add_argument('--depth-topics', type=str, nargs='+')
    parser.add_argument('--out-tracked-objects-3d-topic', type=str)
    parser.add_argument('--out-visualization-topic', type=str)

    objects = parser.add_mutually_exclusive_group()
    objects.add_argument('-segm', '--segmentation', action='store_true')
    objects.add_argument('-track', '--tracking', action='store_true')
    camera = parser.add_mutually_exclusive_group()
    camera.add_argument('-real', '--realsense', action='store_true')
    camera.add_argument('-zed', '--zed', action='store_true')

    parser.add_argument('-vis', '--enable-visualization', action='store_true')
    return parser


class Tracker3D_node(Tracker3D):
    def __init__(self, objects_topic, depth_info_topic, depth_topic,
            out_tracked_objects_3d_topic, erosion_size,
            out_visualization_topic=None):
        print("Waiting for depth info message...")
        depth_info_msg = rospy.wait_for_message(depth_info_topic, CameraInfo)
        K = np.array(depth_info_msg.K).reshape(3, 3)
        D = np.array(depth_info_msg.D)

        super().__init__(erosion_size, K, D)
        self.objects_topic = objects_topic
        self.depth_topic = depth_topic
        self.out_tracked_objects_3d_topic = out_tracked_objects_3d_topic
        self.out_visualization_topic = out_visualization_topic

        self.map_frame = "local_map_lidar"

        self.tracked_objects_3d_pub = \
            rospy.Publisher(self.out_tracked_objects_3d_topic, Objects3d, queue_size=10)

        if self.out_visualization_topic:
            self.visualization_pub = rospy.Publisher(
                self.out_visualization_topic, MarkerArray, queue_size=10)
        else:
            self.visualization_pub = None

        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

        self.last_stamp = rospy.Time()

        self.mutex = Lock()

        self.bridge = CvBridge()

        self.class_id_to_name = dict()
        for category_info in ROSBAG_CATEGORIES:
            self.class_id_to_name[category_info['id']] = category_info['name']

        self.from_ros_tm = TimeMeasurer("  from ros")
        self.track_3d_tm = TimeMeasurer("  track 3d")
        self.to_ros_tm = TimeMeasurer("  to ros")
        self.vis_tm = TimeMeasurer("  visualization")
        self.total_tm = TimeMeasurer("total")

        self.input_stamps = list()
        self.input_delays = list()
        self.output_stamps = list()
        self.output_delays = list()

    def start(self):
        self.objects_sub = message_filters.Subscriber(self.objects_topic, Objects)
        self.depth_sub = message_filters.Subscriber(self.depth_topic, Image)
        self.sync_sub = message_filters.TimeSynchronizer(
            [self.objects_sub, self.depth_sub], 50)
        self.sync_sub.registerCallback(self.callback)

        self.reset_srv = rospy.Service("~reset", Reset, self.reset)

    def reset(self, req: ResetRequest):
        with self.mutex:
            super().reset()

            resp = ResetResponse()
            resp.process_after_stamp = self.last_stamp
            return resp

    def callback(self, objects_msg, depth_msg):
        with self.total_tm:
            input_delay = rospy.get_rostime() - objects_msg.header.stamp
            self.input_stamps.append(objects_msg.header.stamp)
            self.input_delays.append(input_delay)

            assert objects_msg.header.frame_id == depth_msg.header.frame_id

            with self.from_ros_tm:
                _, classes_ids, tracking_ids, _, masks_in_rois, rois, _, _ = \
                    from_objects_msg(objects_msg)
                depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')

            try:
                transform_msg = self.tf_buffer.lookup_transform(
                    self.map_frame, depth_msg.header.frame_id, depth_msg.header.stamp,
                    timeout=rospy.Duration(0.2))
            except Exception as ex:
                print(ex)
                return
            pose = transform_to_numpy(transform_msg.transform)

            with self.mutex:
                with self.track_3d_tm:
                    if len(CATEGORIES_TO_TRACK) > 0:
                        names = [self.class_id_to_name.get(class_id) for class_id in classes_ids]
                        objects_to_track = np.isin(names, CATEGORIES_TO_TRACK)
                        classes_ids = classes_ids[objects_to_track]
                        if len(tracking_ids) > 0:
                            tracking_ids = tracking_ids[objects_to_track]
                        masks_in_rois = masks_in_rois[objects_to_track]
                        rois = rois[objects_to_track]
                    self.update(pose, depth, classes_ids, tracking_ids, masks_in_rois, rois)
                    self.last_stamp = depth_msg.header.stamp

                with self.to_ros_tm:
                    classes_ids = list()
                    tracking_ids = list()
                    tracking_2d_ids = list()
                    positions = list()
                    for tracked_object_3d in self.tracked_objects:
                        class_id = tracked_object_3d.class_id
                        tracking_id = tracked_object_3d.tracking_id
                        tracking_2d_id = tracked_object_3d.tracking_2d_id
                        position = tracked_object_3d.pose
                        classes_ids.append(class_id)
                        tracking_ids.append(tracking_id)
                        if tracked_object_3d.frame_id == self.frame_id:
                            tracking_2d_ids.append(tracking_2d_id)
                        else:
                            tracking_2d_ids.append(-1)
                        positions.append(position)
                    classes_ids = np.array(classes_ids)
                    tracking_ids = np.array(tracking_ids)
                    tracking_2d_ids = np.array(tracking_2d_ids)
                    positions = np.array(positions)

                    header = Header()
                    header.frame_id = self.map_frame
                    header.stamp = depth_msg.header.stamp
                    tracked_objects_3d_msg = to_objects3d_msg(
                        header, classes_ids, tracking_ids, tracking_2d_ids, positions)
                output_delay = rospy.get_rostime() - tracked_objects_3d_msg.header.stamp
                self.output_stamps.append(tracked_objects_3d_msg.header.stamp)
                self.output_delays.append(output_delay)
                self.tracked_objects_3d_pub.publish(tracked_objects_3d_msg)

                if self.visualization_pub is not None:
                    with self.vis_tm:
                        pose_inv = np.linalg.inv(pose)
                        vis_msg = MarkerArray()
                        delete_all_markers = Marker()
                        delete_all_markers.action = Marker.DELETEALL
                        vis_msg.markers.append(delete_all_markers)
                        for tracked_object_3d in self.tracked_objects:
                            is_visible = tracked_object_3d.is_visible(pose_inv, depth, self.K, self.D)

                            vis_object_3d_msg = Marker()
                            vis_object_3d_msg.header.frame_id = self.map_frame
                            vis_object_3d_msg.header.stamp = depth_msg.header.stamp
                            vis_object_3d_msg.ns = "tracked_objects"
                            vis_object_3d_msg.id = tracked_object_3d.tracking_id
                            vis_object_3d_msg.type = Marker.CUBE
                            vis_object_3d_msg.action = Marker.ADD
                            vis_object_3d_msg.pose.position.x = tracked_object_3d.pose[0]
                            vis_object_3d_msg.pose.position.y = tracked_object_3d.pose[1]
                            vis_object_3d_msg.pose.position.z = tracked_object_3d.pose[2]
                            vis_object_3d_msg.pose.orientation.w = 1.0
                            vis_object_3d_msg.scale.x = 0.13
                            vis_object_3d_msg.scale.y = 0.13
                            vis_object_3d_msg.scale.z = 0.13
                            if is_visible:
                                vis_object_3d_msg.color.g = 1.0
                            else:
                                vis_object_3d_msg.color.r = 1.0
                            vis_object_3d_msg.color.a = 0.5
                            vis_msg.markers.append(vis_object_3d_msg)

                            vis_label_msg = Marker()
                            vis_label_msg.header.frame_id = self.map_frame
                            vis_label_msg.header.stamp = depth_msg.header.stamp
                            vis_label_msg.ns = "labels"
                            vis_label_msg.id = tracked_object_3d.tracking_id
                            vis_label_msg.type = Marker.TEXT_VIEW_FACING
                            vis_label_msg.action = Marker.ADD
                            vis_label_msg.pose.position.x = tracked_object_3d.pose[0]
                            vis_label_msg.pose.position.y = tracked_object_3d.pose[1]
                            vis_label_msg.pose.position.z = tracked_object_3d.pose[2] + 0.25
                            vis_label_msg.scale.x = 0.25
                            vis_label_msg.scale.y = 0.25
                            vis_label_msg.scale.z = 0.25
                            vis_label_msg.color.r = 1.0
                            vis_label_msg.color.g = 1.0
                            vis_label_msg.color.a = 1.0
                            vis_label_msg.text = \
                                f"{self.class_id_to_name.get(tracked_object_3d.class_id, 'unknown')} " \
                                f"({tracked_object_3d.class_id}), " \
                                f"t: {tracked_object_3d.tracking_id}"
                            vis_msg.markers.append(vis_label_msg)

                    self.visualization_pub.publish(vis_msg)


def complete_args(args):
    assert not (args.segmentation and args.tracking)
    assert not (args.realsense and args.zed)

    objects_selected = (args.segmentation or args.tracking)
    if args.objects_topics is None and not objects_selected:
        args.tracking = True
        objects_selected = True

    camera_selected = (args.realsense or args.zed)
    if (args.depth_info_topics is None or args.depth_topics is None) \
            and not camera_selected:
        args.realsense = True
        camera_selected = True

    if args.segmentation:
        args.objects_topics = ["/segmentation"]
    if args.tracking:
        args.objects_topics = ["/tracking"]

    if args.realsense:
        args.depth_info_topics = ["/realsense_gripper/aligned_depth_to_color/camera_info"]
        args.depth_topics = ["/realsense_gripper/aligned_depth_to_color/image_raw"]
    if args.zed:
        args.depth_info_topics = ["/zed_node/depth/camera_info"]
        args.depth_topics = ["/zed_node/depth/depth_registered"]

    if args.out_tracked_objects_3d_topic is None:
        args.out_tracked_objects_3d_topic = "/tracked_objects_3d"

    if not args.enable_visualization:
        args.out_visualization_topic = None
    if args.enable_visualization and args.out_visualization_topic is None:
        args.out_visualization_topic = "/tracked_objects_3d_vis"

    assert len(args.objects_topics)
    assert len(args.depth_info_topics)
    assert len(args.depth_topics)
    assert args.out_tracked_objects_3d_topic is not None


if __name__ == '__main__':
    time_str = time.strftime("%Y-%m-%d_%H.%M.%S")

    parser = build_parser()
    args, unknown_args = parser.parse_known_args()
    for i in range(len(unknown_args)-1, -1, -1):
        if unknown_args[i].startswith('__name:=') or unknown_args[i].startswith('__log:='):
            del unknown_args[i]
    if len(unknown_args) > 0:
        raise RuntimeError("Unknown args: {}".format(unknown_args))

    complete_args(args)

    rospy.init_node("tracker_3d")
    tracker_3d_node = Tracker3D_node(
        args.objects_topics[0],
        args.depth_info_topics[0],
        args.depth_topics[0],

        args.out_tracked_objects_3d_topic,
        out_visualization_topic=args.out_visualization_topic,
        erosion_size=5)
    tracker_3d_node.start()

    print("Spinning...")
    rospy.spin()

    if len(tracker_3d_node.output_delays) > 0:
        logs_folder = osp.abspath(osp.expanduser(osp.join(
            osp.dirname(__file__), "logs/tracker_3d_node")))
        os.makedirs(logs_folder, exist_ok=True)
        with open(osp.join(logs_folder, f"{time_str}_input.txt"), 'w') as f:
            lines = [f"{str(stamp)} {str(delay)}\n" for stamp, delay in \
                zip(tracker_3d_node.input_stamps, tracker_3d_node.input_delays)]
            lines[-1] = lines[-1][:-1]
            f.writelines(lines)
        with open(osp.join(logs_folder, f"{time_str}_output.txt"), 'w') as f:
            lines = [f"{str(stamp)} {str(delay)}\n" for stamp, delay in \
                zip(tracker_3d_node.output_stamps, tracker_3d_node.output_delays)]
            lines[-1] = lines[-1][:-1]
            f.writelines(lines)

    print()
    del tracker_3d_node
