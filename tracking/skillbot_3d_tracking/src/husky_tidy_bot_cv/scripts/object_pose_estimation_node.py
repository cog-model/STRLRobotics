import argparse
import rospy
import tf2_ros
from husky_tidy_bot_cv.msg import ObjectPointCloud, ObjectPose
from husky_tidy_bot_cv.srv import \
    GetObjectPoseRequest, GetObjectPoseResponse, GetObjectPose, \
    GetObjectPointCloudRequest, GetObjectPointCloudResponse, GetObjectPointCloud
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped, TransformStamped
from ros_numpy.geometry import numpy_to_pose, transform_to_numpy, numpy_to_transform, pose_to_numpy
from ros_numpy.point_cloud2 import pointcloud2_to_xyz_array, array_to_pointcloud2
from object_pose_estimation import ObjectPoseEstimation, get_box_point_cloud, \
    align_poses, align_poses_90
import numpy as np
from kas_utils.time_measurer import TimeMeasurer
import open3d as o3d
from threading import Lock


def build_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument('--object-point-cloud-topic', type=str, default="/object_point_cloud")
    parser.add_argument('--out-object-pose-topic', type=str, default="/object_pose")

    parser.add_argument('--target-frame', type=str, default='base_link')
    parser.add_argument('-vis', '--enable-visualization', action='store_true')
    return parser


class ObjectPoseEstimation_node:
    class PreviousResults:
        def __init__(self):
            self.class_id = -1
            self.tracking_id = -1
            self.object_pose = None

    def __init__(self, object_point_cloud_topic, out_object_pose_topic,
            out_pose_visualization_topic=None,
            out_gt_pc_visualization_topic=None, out_pc_visualization_topic=None,
            target_frame='base_link', object_frame="object", publish_to_tf=True):
        self.object_point_cloud_topic = object_point_cloud_topic
        self.out_object_pose_topic = out_object_pose_topic
        self.out_pose_visualization_topic = out_pose_visualization_topic
        self.out_gt_pc_visualization_topic = out_gt_pc_visualization_topic
        self.out_pc_visualization_topic = out_pc_visualization_topic
        self.target_frame = target_frame
        self.object_frame = object_frame
        self.publish_to_tf = publish_to_tf

        self.object_pose_pub = rospy.Publisher(
            self.out_object_pose_topic, ObjectPose, queue_size=10)
        
        if self.out_pose_visualization_topic:
            self.pose_visualization_pub = rospy.Publisher(
                self.out_pose_visualization_topic, PoseStamped, queue_size=10)
        else:
            self.pose_visualization_pub = None

        if self.out_gt_pc_visualization_topic:
            self.gt_pc_visualization_pub = rospy.Publisher(
                self.out_gt_pc_visualization_topic, PointCloud2, queue_size=10)
        else:
            self.gt_pc_visualization_pub = None

        if self.out_pc_visualization_topic:
            self.pc_visualization_pub = rospy.Publisher(
                self.out_pc_visualization_topic, PointCloud2, queue_size=10)
        else:
            self.pc_visualization_pub = None

        self.get_object_point_cloud = rospy.ServiceProxy(
            "/object_point_cloud_extraction/get_object_point_cloud", GetObjectPointCloud)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        self.prev_service = ObjectPoseEstimation_node.PreviousResults()
        self.prev_callback = ObjectPoseEstimation_node.PreviousResults()

        self.reason = None

        self.mutex = Lock()

        self.object_pose_estimators = dict()

        # toy box
        self.object_pose_estimators[0] = ObjectPoseEstimation(
            get_box_point_cloud([0.07, 0.07, 0.07], points_per_cm=7),
            voxel_size=0.005,
            max_correspondence_distances=[0.04, 0.029, 0.018, 0.007])

        # container
        self.object_pose_estimators[1] = ObjectPoseEstimation(
            o3d.io.read_point_cloud('white_container_scan_v2/merged_filtered_aligned.pcd'),
            voxel_size=0.03,
            max_correspondence_distances=np.array([0.04, 0.029, 0.018, 0.011]) * 2)

        # toy cat
        #self.object_pose_estimators[18] = ObjectPoseEstimation(
        #    o3d.io.read_point_cloud('toy_cat.pcd'),
        #    voxel_size=0.02,
        #    max_correspondence_distances=np.array([0.04, 0.029, 0.018, 0.011]))

        self.from_ros_tm = TimeMeasurer("  from ros")
        self.estimate_tm = TimeMeasurer("  estimate pose")
        self.to_ros_tm = TimeMeasurer("  to ros")
        self.total_tm = TimeMeasurer("total")

    def start(self):
        self.object_point_cloud_sub = rospy.Subscriber(
            self.object_point_cloud_topic, ObjectPointCloud, self.callback,
            queue_size=1, buff_size=2 ** 24)

        self.get_object_pose_srv = rospy.Service(
            "~get_object_pose", GetObjectPose, self.get_object_pose)

    def get_object_pose(self, req: GetObjectPoseRequest):
        rospy.loginfo(f"Received request for pose estimation "
            f"for object id {req.object_id}")

        rospy.loginfo(f"Request point cloud "
            f"for object id {req.object_id}")
        object_point_cloud_req = GetObjectPointCloudRequest()
        object_point_cloud_req.object_id = req.object_id
        object_point_cloud_resp = self.get_object_point_cloud(object_point_cloud_req)

        with self.mutex:
            if object_point_cloud_resp.return_code == 0:  # success
                object_point_cloud_msg = object_point_cloud_resp.object_point_cloud
                object_pose_msg, _, _ = self.estimate_pose_ros(
                    object_point_cloud_msg, self.prev_service)
            else:
                object_pose_msg = None
                self.reason = f"Could not get point cloud for object id {req.object_id}"

            resp = GetObjectPoseResponse()
            if object_pose_msg is not None:
                resp.return_code = 0  # success
                resp.object_pose = object_pose_msg
                rospy.loginfo(f"Successfully returned pose "
                    f"for object id {req.object_id}")
            else:
                resp.return_code = 1  # error
                rospy.loginfo(f"Error occured while trying to estimate pose "
                    f"for object id {req.object_id}: {self.reason}")
            return resp

    def callback(self, object_point_cloud_msg: ObjectPointCloud):
        with self.mutex:
            object_pose_msg, object_pose_estimator, object_pose_in_camera = \
                self.estimate_pose_ros(object_point_cloud_msg, self.prev_callback)
            if object_pose_estimator is not None and \
                    object_pose_estimator.pc_down is not None:
                extracted_pc_down = o3d.geometry.PointCloud(object_pose_estimator.pc_down)
            else:
                extracted_pc_down = None

        if object_pose_msg is not None:
            self.object_pose_pub.publish(object_pose_msg)

        if self.gt_pc_visualization_pub is not None and \
                object_pose_estimator is not None:
            gt_pc = object_pose_estimator.gt_pc_down
            gt_points = np.asarray(gt_pc.points, dtype=np.float32)
            dtype = [('x', np.float32), ('y', np.float32), ('z', np.float32)]
            gt_points = gt_points.view(dtype)
            gt_point_cloud_msg = array_to_pointcloud2(gt_points,
                stamp=object_point_cloud_msg.header.stamp, frame_id=self.object_frame)
            self.gt_pc_visualization_pub.publish(gt_point_cloud_msg)

        if self.pc_visualization_pub is not None and \
                extracted_pc_down is not None and \
                object_pose_in_camera is not None:
            pc = extracted_pc_down
            pc.transform(np.linalg.inv(object_pose_in_camera))
            points = np.asarray(pc.points, dtype=np.float32)
            dtype = [('x', np.float32), ('y', np.float32), ('z', np.float32)]
            points = points.view(dtype)
            point_cloud_msg = array_to_pointcloud2(points,
                stamp=object_point_cloud_msg.header.stamp, frame_id=self.object_frame)
            self.pc_visualization_pub.publish(point_cloud_msg)

        if self.publish_to_tf and object_pose_msg is not None:
            object_pose_tf = TransformStamped()
            object_pose_tf.header = object_pose_msg.header
            object_pose_tf.child_frame_id = self.object_frame
            object_pose_tf.transform.translation.x = object_pose_msg.pose.position.x
            object_pose_tf.transform.translation.y = object_pose_msg.pose.position.y
            object_pose_tf.transform.translation.z = object_pose_msg.pose.position.z
            object_pose_tf.transform.rotation = object_pose_msg.pose.orientation
            self.tf_broadcaster.sendTransform(object_pose_tf)

    def estimate_pose_ros(self, object_point_cloud_msg: ObjectPointCloud,
            prev: PreviousResults):
        self.total_tm.start()

        self.reason = None

        class_id = object_point_cloud_msg.class_id
        tracking_id = object_point_cloud_msg.tracking_id
        if prev.class_id != class_id or prev.tracking_id != tracking_id:
            prev.class_id = class_id
            prev.tracking_id = tracking_id
            prev.object_pose = None

        assert class_id >= 0
        object_pose_estimator = self.object_pose_estimators.get(class_id)
        if object_pose_estimator is None:
            self.reason = f"Object with class id {class_id} is unknown"
            return [None] * 3

        with self.from_ros_tm:
            point_cloud = pointcloud2_to_xyz_array(object_point_cloud_msg.point_cloud)

        self.estimate_tm.start()
        object_pose = object_pose_estimator.estimate_pose(point_cloud)
        if object_pose is None:
            self.reason = f"Cloud not estimate object pose: {object_pose_estimator.reason}"
            return None, object_pose_estimator, None

        if prev.object_pose is not None:
            if class_id in (0,):
                object_pose = align_poses_90(prev.object_pose, object_pose)
            elif class_id in (1,):
                object_pose = align_poses(prev.object_pose, object_pose)
        # object_pose is returned from this function, so make copy
        prev.object_pose = object_pose.copy()
        self.estimate_tm.stop()

        try:
            tf = self.tf_buffer.lookup_transform(
                self.target_frame, object_point_cloud_msg.header.frame_id,
                object_point_cloud_msg.header.stamp, timeout=rospy.Duration(0.1))
        except tf2_ros.ExtrapolationException:
            self.reason = "Lookup transform extrapolation error"
            return None, object_pose_estimator, None
        tf_mat = transform_to_numpy(tf.transform)
        object_pose_in_target = np.matmul(tf_mat, object_pose)

        with self.to_ros_tm:
            object_pose_msg = ObjectPose()
            object_pose_msg.header.stamp = object_point_cloud_msg.header.stamp
            object_pose_msg.header.frame_id = self.target_frame
            object_pose_msg.class_id = class_id
            object_pose_msg.tracking_id = tracking_id
            object_pose_msg.pose = numpy_to_pose(object_pose_in_target)

        self.total_tm.stop()

        object_pose_in_camera = object_pose
        return object_pose_msg, object_pose_estimator, object_pose_in_camera


if __name__ == '__main__':
    parser = build_parser()
    args, unknown_args = parser.parse_known_args()
    for i in range(len(unknown_args)-1, -1, -1):
        if unknown_args[i].startswith('__name:=') or unknown_args[i].startswith('__log:='):
            del unknown_args[i]
    if len(unknown_args) > 0:
        raise RuntimeError("Unknown args: {}".format(unknown_args))

    rospy.init_node("object_pose_estimation")
    if args.enable_visualization:
        out_pose_visualization_topic = "/object_pose_vis"
        out_gt_pc_visualization_topic = "/object_gt_points_vis"
        out_pc_visualization_topic = "/object_points_vis"
    else:
        out_pose_visualization_topic = None
        out_gt_pc_visualization_topic = None
        out_pc_visualization_topic = None

    object_pose_estimation_node = ObjectPoseEstimation_node(
        args.object_point_cloud_topic,

        args.out_object_pose_topic,
        out_pose_visualization_topic=out_pose_visualization_topic,
        out_gt_pc_visualization_topic=out_gt_pc_visualization_topic,
        out_pc_visualization_topic=out_pc_visualization_topic,
        target_frame=args.target_frame)
    object_pose_estimation_node.start()

    print("Spinning...")
    rospy.spin()

    print()
    del object_pose_estimation_node
