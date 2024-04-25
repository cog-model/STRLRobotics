import argparse
import rospy
import tf2_ros
import message_filters
from cv_bridge import CvBridge
from husky_tidy_bot_cv.msg import Objects, ObjectPointCloud
from husky_tidy_bot_cv.srv import \
    SetObjectIdRequest, SetObjectIdResponse, SetObjectId, \
    GetObjectPointCloudRequest, GetObjectPointCloudResponse, GetObjectPointCloud
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from ros_numpy.geometry import transform_to_numpy
from ros_numpy.point_cloud2 import array_to_pointcloud2
from conversions import from_objects_msg
import numpy as np
from kas_utils.time_measurer import TimeMeasurer
from threading import Lock
from object_point_cloud_extraction import ObjectPointCloudExtraction


def build_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument('--depth-info-topics', type=str, nargs='+')
    parser.add_argument('--depth-topics', type=str, nargs='+')
    parser.add_argument('--objects-topics', type=str, nargs='+')
    parser.add_argument('--out-object-point-cloud-topic', type=str)
    parser.add_argument('--out-visualization-topic', type=str)

    parser.add_argument('--target-frame', type=str, default='base_link')

    camera = parser.add_mutually_exclusive_group()
    camera.add_argument('-real', '--realsense', action='store_true')
    camera.add_argument('-zed', '--zed', action='store_true')

    parser.add_argument('-vis', '--enable-visualization', action='store_true')
    return parser


class ObjectPointCloudExtraction_node(ObjectPointCloudExtraction):
    def __init__(self, depth_info_topic, depth_topic, objects_topic,
            out_object_point_cloud_topic, out_visualization_topic=None,
            target_frame='base_link', erosion_size=0, pool_size=2):
        print("Waiting for depth info message...")
        depth_info_msg = rospy.wait_for_message(depth_info_topic, CameraInfo)
        K = np.array(depth_info_msg.K).reshape(3, 3)
        D = np.array(depth_info_msg.D)

        super().__init__(K, D, erosion_size=erosion_size, pool_size=pool_size)

        self.depth_topic = depth_topic
        self.objects_topic = objects_topic
        self.out_object_point_cloud_topic = out_object_point_cloud_topic
        self.out_visualization_topic = out_visualization_topic
        self.target_frame = target_frame

        self.object_point_cloud_pub = rospy.Publisher(
            self.out_object_point_cloud_topic, ObjectPointCloud, queue_size=50)
        if self.out_visualization_topic:
            self.visualization_pub = rospy.Publisher(
                self.out_visualization_topic, PointCloud2, queue_size=50)
        else:
            self.visualization_pub = None

        self.bridge = CvBridge()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.check_timeout = rospy.Duration(3)
        self.check_rate = 100
        self.max_tries_num = 2

        self.object_id = -1
        self.last_stamp = rospy.Time()

        self.last_depth_msg = None
        self.last_objects_msg = None

        self.mutex = Lock()

        self.from_ros_tm = TimeMeasurer("  from ros")
        self.extract_tm = TimeMeasurer("  extract point cloud")
        self.to_ros_tm = TimeMeasurer("  to ros")
        self.total_tm = TimeMeasurer("total")

    def start(self):
        self.depth_sub = message_filters.Subscriber(self.depth_topic, Image)
        self.objects_sub = message_filters.Subscriber(self.objects_topic, Objects)
        self.sync_sub = message_filters.TimeSynchronizer(
            [self.depth_sub, self.objects_sub], 50)
        self.sync_sub.registerCallback(self.callback)

        self.set_object_id_srv = rospy.Service(
            "~set_object_id", SetObjectId, self.set_object_id)

        self.get_object_point_cloud_srv = rospy.Service(
            "~get_object_point_cloud", GetObjectPointCloud, self.get_object_point_cloud)

    def set_object_id(self, req: SetObjectIdRequest):
        with self.mutex:
            self.object_id = req.object_id

            resp = SetObjectIdResponse()
            resp.process_after_stamp = self.last_stamp
            return resp

    def get_object_point_cloud(self, req: GetObjectPointCloudRequest):
        with self.mutex:
            rospy.loginfo(f"Received request for point cloud extraction "
                f"for object id {req.object_id}")

            rate = rospy.Rate(self.check_rate)
            start_time = rospy.get_rostime()
            timeout_exceeded = False
            while self.last_depth_msg is None:
                self.mutex.release()
                if rospy.get_rostime() - start_time > self.check_timeout:
                    timeout_exceeded = True
                    object_point_cloud_msg = None
                    self.reason = f"Timeout of {self.check_timeout.to_sec()} seconds " \
                        "is exceeded while waiting for depth and objects messages"
                rate.sleep()
                self.mutex.acquire()

            if not timeout_exceeded:
                # strange exception sometimes occurs in extract_point_cloud_ros(),
                # but when this function is called from callback(), all works fine
                retry = True
                tries_counter = 0
                while retry:
                    try:
                        object_point_cloud_msg = self.extract_point_cloud_ros(
                            self.last_depth_msg, self.last_objects_msg, req.object_id)
                    except RuntimeError:
                        tries_counter += 1
                        if tries_counter >= self.max_tries_num:
                            object_point_cloud_msg = None
                            self.reason = "Strange bug"
                            break
                    else:
                        retry = False

                self.last_depth_msg = None
                self.last_objects_msg = None

            resp = GetObjectPointCloudResponse()
            if object_point_cloud_msg is not None:
                resp.return_code = 0
                resp.object_point_cloud = object_point_cloud_msg
                rospy.loginfo(f"Successfully returned point cloud "
                    f"for object id {req.object_id}")
            else:
                resp.return_code = 1
                rospy.loginfo(f"Error occured while trying to extract point cloud "
                    f"for object id {req.object_id}: {self.reason}")
            return resp

    def callback(self, depth_msg, objects_msg):
        with self.mutex:
            self.last_depth_msg = depth_msg
            self.last_objects_msg = objects_msg

            if self.object_id < 0:
                return

            object_point_cloud_msg = self.extract_point_cloud_ros(
                depth_msg, objects_msg, self.object_id)
            self.last_stamp = depth_msg.header.stamp

        if object_point_cloud_msg is not None:
            self.object_point_cloud_pub.publish(object_point_cloud_msg)
            if self.visualization_pub is not None:
                self.visualization_pub.publish(object_point_cloud_msg.point_cloud)

    def extract_point_cloud_ros(self, depth_msg, objects_msg, object_id):
        self.total_tm.start()

        self.reason = None

        assert depth_msg.header.frame_id == objects_msg.header.frame_id
        assert object_id >= 0

        with self.from_ros_tm:
            depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
            _, classes_ids, tracking_ids, _, masks_in_rois, rois, _, _ = \
                from_objects_msg(objects_msg)

        self.extract_tm.start()
        object_point_cloud, object_index = self.extract_point_cloud(depth,
            classes_ids, tracking_ids, masks_in_rois, rois, object_id)
        if object_point_cloud is None:
            return None
        self.extract_tm.stop()

        try:
            tf = self.tf_buffer.lookup_transform(
                self.target_frame, depth_msg.header.frame_id, depth_msg.header.stamp,
                timeout=rospy.Duration(0.1))
        except tf2_ros.ExtrapolationException:
            self.reason = "Lookup transform extrapolation error"
            return None
        tf_mat = transform_to_numpy(tf.transform).astype(np.float32)

        object_point_cloud = \
            np.matmul(tf_mat[:3, :3], object_point_cloud.transpose()).transpose() + \
            tf_mat[:3, 3]
        if not object_point_cloud.flags.c_contiguous:
            object_point_cloud = np.ascontiguousarray(object_point_cloud)

        dtype = [('x', np.float32), ('y', np.float32), ('z', np.float32)]
        object_point_cloud = object_point_cloud.view(dtype)

        with self.to_ros_tm:
            object_point_cloud_msg = ObjectPointCloud()
            object_point_cloud_msg.class_id = classes_ids[object_index]
            if tracking_ids.size > 0:
                object_point_cloud_msg.tracking_id = tracking_ids[object_index]
            else:
                object_point_cloud_msg.tracking_id = -1
            object_point_cloud_msg.point_cloud = array_to_pointcloud2(object_point_cloud,
                stamp=depth_msg.header.stamp, frame_id=self.target_frame)
            object_point_cloud_msg.header = object_point_cloud_msg.point_cloud.header

        self.total_tm.stop()

        return object_point_cloud_msg


def complete_args(args):
    assert not (args.realsense and args.zed)

    camera_selected = (args.realsense or args.zed)
    if (args.depth_info_topics is None or args.depth_topics is None) \
            and not camera_selected:
        args.realsense = True
        camera_selected = True

    if args.realsense:
        args.depth_info_topics = ["/realsense_gripper/aligned_depth_to_color/camera_info"]
        args.depth_topics = ["/realsense_gripper/aligned_depth_to_color/image_raw"]
    if args.zed:
        args.depth_info_topics = ["/zed_node/depth/camera_info"]
        args.depth_topics = ["/zed_node/depth/depth_registered"]

    if args.objects_topics is None:
        args.objects_topics = ["/tracking"]

    if args.out_object_point_cloud_topic is None:
        args.out_object_point_cloud_topic = "/object_point_cloud"

    if not args.enable_visualization:
        args.out_visualization_topic = None
    if args.enable_visualization and args.out_visualization_topic is None:
        args.out_visualization_topic = "/object_point_cloud_vis"

    assert len(args.depth_info_topics)
    assert len(args.depth_topics)
    assert len(args.objects_topics)
    assert args.out_object_point_cloud_topic is not None


if __name__ == '__main__':
    parser = build_parser()
    args, unknown_args = parser.parse_known_args()
    for i in range(len(unknown_args)-1, -1, -1):
        if unknown_args[i].startswith('__name:=') or unknown_args[i].startswith('__log:='):
            del unknown_args[i]
    if len(unknown_args) > 0:
        raise RuntimeError("Unknown args: {}".format(unknown_args))

    complete_args(args)

    rospy.init_node("object_point_cloud_extraction")
    object_pose_estimation_node = ObjectPointCloudExtraction_node(
        args.depth_info_topics[0],
        args.depth_topics[0],
        args.objects_topics[0],

        args.out_object_point_cloud_topic,
        out_visualization_topic=args.out_visualization_topic,
        target_frame=args.target_frame,
        erosion_size=5, pool_size=2)
    object_pose_estimation_node.start()

    print("Spinning...")
    rospy.spin()

    print()
    del object_pose_estimation_node
