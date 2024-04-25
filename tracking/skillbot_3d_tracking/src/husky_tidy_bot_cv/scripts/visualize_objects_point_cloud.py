import rospy
import message_filters
from sensor_msgs.msg import PointCloud2, CameraInfo, Image
from husky_tidy_bot_cv.msg import Objects
from cv_bridge import CvBridge
import numpy as np
import cv2
from kas_utils.depth_to_point_cloud import DepthToPointCloud
from conversions import from_objects_msg
from ros_numpy.point_cloud2 import array_to_pointcloud2
from numpy.lib.recfunctions import append_fields


def bgr2number(b, g, r):
    a = 255
    number = np.uint32((a << 24) + (r << 16) + (g << 8) + (b << 0))
    return number


def callback(depth_msg: Image, objects_msg: Objects):
    global bridge, erosion_element, fx, fy, cx, cy, depth_to_point_cloud, palette, pub
    depth = bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
    scores, classes_ids, tracking_ids, boxes, masks_in_rois, rois, widths, heights = \
        from_objects_msg(objects_msg)

    all_points = list()
    for i, (mask_in_roi, roi) in enumerate(zip(masks_in_rois, rois)):
        if erosion_element is not None:
            cv2.erode(mask_in_roi, erosion_element,
                borderType=cv2.BORDER_CONSTANT, borderValue=0,
                dst=mask_in_roi)

        depth_in_roi = depth[roi]
        if not depth_in_roi.flags.writeable:
            depth_in_roi = depth_in_roi.copy()
        depth_in_roi[mask_in_roi == 0] = 0

        depth_to_point_cloud.set_camera_intrinsics(
            fx, fy, cx - roi[1].start, cy - roi[0].start)
        points = depth_to_point_cloud.convert(depth_in_roi)
        dtype = [('x', np.float32), ('y', np.float32), ('z', np.float32)]
        points = points.view(dtype)
        colors = np.full((len(points),), bgr2number(*palette[i]), dtype=np.uint32)

        points = append_fields(points, "rgba", colors)
        all_points.append(points)
    if sum((len(points) for points in all_points)) == 0:
        dtype = [('x', np.float32), ('y', np.float32), ('z', np.float32),
            ('rgba', np.uint32)]
        all_points = np.empty((0,), dtype=dtype)
    else:
        all_points = np.hstack(all_points)

    point_cloud_msg = array_to_pointcloud2(all_points,
        stamp=depth_msg.header.stamp, frame_id=depth_msg.header.frame_id)
    pub.publish(point_cloud_msg)


if __name__ == '__main__':
    rospy.init_node("visualize_objects_point_cloud")

    palette = (
        (120, 120, 120), (180, 120, 120),
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
        (153, 255, 0), (0, 0, 255), (255, 71, 0), (0, 235, 255),
        (0, 173, 255), (31, 0, 255), (11, 200, 200), (255, 82, 0),
        (0, 255, 245), (0, 61, 255), (0, 255, 112), (0, 255, 133),
        (255, 0, 0), (255, 163, 0), (255, 102, 0), (194, 255, 0),
        (0, 143, 255), (51, 255, 0), (0, 82, 255), (0, 255, 41),
        (0, 255, 173), (10, 0, 255), (173, 255, 0),
        (0, 255, 153), (255, 92, 0), (255, 0, 255), (255, 0, 245),
        (255, 0, 102), (255, 173, 0), (255, 0, 20), (255, 184, 184),
        (0, 31, 255), (0, 255, 61), (0, 71, 255), (255, 0, 204),
        (0, 255, 194), (0, 255, 82), (0, 10, 255), (0, 112, 255)
        ,(51, 0, 255), (0, 194, 255), (0, 122, 255), (0, 255, 163),
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

    bridge = CvBridge()

    pub = rospy.Publisher("/objects_point_cloud_vis", PointCloud2, queue_size=10)

    print("Waiting for depth info message...")
    depth_info_msg = rospy.wait_for_message(
        "/realsense_gripper/aligned_depth_to_color/camera_info", CameraInfo)
    K = np.array(depth_info_msg.K).reshape(3, 3)
    D = np.array(depth_info_msg.D)
    assert np.all(D == 0)

    fx = K[0, 0]
    fy = K[1, 1]
    cx = K[0, 2]
    cy = K[1, 2]
    depth_to_point_cloud = DepthToPointCloud(0, 0, 0, 0, 2)

    erosion_size = 5
    if erosion_size > 0:
        erosion_element = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,
            (2 * erosion_size + 1, 2 * erosion_size + 1),
            (erosion_size, erosion_size))
    else:
        erosion_element = None

    depth_sub = message_filters.Subscriber(
        "/realsense_gripper/aligned_depth_to_color/image_raw", Image)
    objects_sub = message_filters.Subscriber("/segmentation", Objects)
    sync_sub = message_filters.TimeSynchronizer([depth_sub, objects_sub], queue_size=50)
    sync_sub.registerCallback(callback)

    print("Spinning...")
    rospy.spin()
