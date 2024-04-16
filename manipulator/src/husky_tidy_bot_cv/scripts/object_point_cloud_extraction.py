import numpy as np
import cv2
from kas_utils.depth_to_point_cloud import DepthToPointCloud


class ObjectPointCloudExtraction:
    def __init__(self, K, D, erosion_size=0, pool_size=2):
        assert np.all(D == 0), "Distorted images are not supported"

        self.K = K
        self.D = D
        self.erosion_size = erosion_size
        self.pool_size = pool_size

        if self.erosion_size > 0:
            self.erosion_element = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,
                (2 * self.erosion_size + 1, 2 * self.erosion_size + 1),
                (self.erosion_size, self.erosion_size))

        # camera intrinsics will be set later to correspond to mask roi
        self.depth_to_point_cloud = DepthToPointCloud(0, 0, 0, 0, self.pool_size)

        self.reason = None

    def extract_point_cloud(self, depth, classes_ids, tracking_ids,
            masks_in_rois, rois, object_id):
        self.reason = None

        if len(classes_ids) == 0:
            self.reason = "No detections"
            return None, None
        if tracking_ids.size > 0:
            object_indices = np.where(tracking_ids == object_id)[0]
            assert len(object_indices) <= 1, \
                "Multiple objects with the same tracking id. " \
                "This should not happen."
            if len(object_indices) == 0:
                self.reason = f"Cloud not find object with tracking id {object_id}"
                return None, None
            object_index = object_indices.item()
        else:
            object_indices = np.where(classes_ids == object_id)[0]
            if len(object_indices) > 1:
                self.reason = f"Found more than one object of class id {object_id}"
                return None, None
            if len(object_indices) == 0:
                self.reason = f"Cloud not find object with class id {object_id}"
                return None, None
            object_index = object_indices.item()
        mask_in_roi = masks_in_rois[object_index]
        roi = rois[object_index]

        if self.erosion_size > 0:
            mask_in_roi = cv2.erode(mask_in_roi, self.erosion_element,
                borderType=cv2.BORDER_CONSTANT, borderValue=0)
        depth_in_roi = depth[roi]
        if not depth_in_roi.flags.writeable:
            depth_in_roi = depth_in_roi.copy()
        depth_in_roi[mask_in_roi == 0] = 0

        fx = self.K[0, 0]
        fy = self.K[1, 1]
        cx = self.K[0, 2]
        cy = self.K[1, 2]
        self.depth_to_point_cloud.set_camera_intrinsics(
            fx, fy, cx - roi[1].start, cy - roi[0].start)
        object_point_cloud = self.depth_to_point_cloud.convert(depth_in_roi)
        assert object_point_cloud.dtype == np.float32

        return object_point_cloud, object_index
