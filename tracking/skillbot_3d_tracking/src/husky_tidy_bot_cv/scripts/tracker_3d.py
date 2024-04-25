# from __future__ import annotations

import numpy as np
import cv2
from kas_utils import get_depth_scale
import lap
from typing import List


class TrackedObject:
    next_tracking_id = 0

    def __init__(self, class_id, tracking_2d_id, pose, frame_id):
        self.class_id = class_id
        self.tracking_2d_id = tracking_2d_id  # (-1) - unspecified
        self.pose = pose
        self.frame_id = frame_id

        self.tracking_id = -1
        self.tracklet_len = 1  # increases only when matched with new object
        self.visible_without_updates = 0

    def activate(self):
        assert self.tracking_id == -1
        self.tracking_id = TrackedObject.next_tracking_id
        TrackedObject.next_tracking_id += 1

    def update(self, update_object):
        assert self.tracking_id != -1
        assert self.class_id == update_object.class_id

        k = self.tracklet_len / (self.tracklet_len + 1)
        max_k = 0.8
        k = min(k, max_k)

        self.tracking_2d_id = update_object.tracking_2d_id
        self.pose = k * self.pose + (1 - k) * update_object.pose
        self.frame_id = update_object.frame_id

        self.tracklet_len += 1
        self.visible_without_updates = 0

    def is_visible(self, camera_pose_inv, depth, K, D, margin=50, radius=10):
        pose_in_camera = np.matmul(camera_pose_inv, np.append(self.pose, 1))[:3]
        pose_z = pose_in_camera[2]
        if pose_z <= 0:
            return False

        pose_in_camera = np.expand_dims(pose_in_camera, axis=(0, 1))
        point, _ = cv2.projectPoints(pose_in_camera, np.zeros((3,)), np.zeros((3,)), K, D)
        point = point[0, 0].astype(int)
        u = point[0]
        v = point[1]
        height, width = depth.shape
        if u < margin or v < margin or u >= width - margin or v >= height - margin:
            return False

        min_u = max(u - radius, 0)
        min_v = max(v - radius, 0)
        max_u = min(u + radius + 1, width)
        max_v = min(v + radius + 1, height)
        depth_selected = depth[min_v:max_v, min_u:max_u]

        y, x = np.mgrid[min_v:max_v, min_u:max_u]
        dist_sqr = (y - v) ** 2 + (x - u) ** 2
        mask = dist_sqr <= radius ** 2
        depth_selected = depth_selected[mask]

        depth_selected = depth_selected * get_depth_scale(depth)

        shift = 0.10
        rate = np.count_nonzero(depth_selected + shift > pose_z) / depth_selected.size

        thresh = 0.5
        if rate > thresh:
            return True
        else:
            return False


class Tracker3D:
    def __init__(self, erosion_size, K, D):
        self.erosion_size = erosion_size
        if self.erosion_size > 0:
            self.erosion_element = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,
                (2 * self.erosion_size + 1, 2 * self.erosion_size + 1),
                (self.erosion_size, self.erosion_size))
        self.K = K
        self.D = D

        self.frame_id = -1
        self.tracked_objects: List[TrackedObject] = list()

        assert np.all(self.D == 0), "Distorted images are not supported"

    def reset(self):
        self.frame_id = -1
        self.tracked_objects = list()
        TrackedObject.next_tracking_id = 0

    def update(self, camera_pose, depth, classes_ids, tracking_ids, masks_in_rois, rois):
        self.frame_id += 1

        objects_poses = self._get_objects_poses(depth, masks_in_rois, rois, camera_pose)
        valid = ~np.isnan(objects_poses[:, 0])
        classes_ids = classes_ids[valid]
        if len(tracking_ids) > 0:
            tracking_ids = tracking_ids[valid]
        masks_in_rois = None  # is not used further
        rois = None  # is not used further
        objects_poses = objects_poses[valid]

        if len(tracking_ids) > 0:
            new_objects = [TrackedObject(class_id, tracking_2d_id, pose, self.frame_id)
                for class_id, tracking_2d_id, pose in zip(classes_ids, tracking_ids, objects_poses)]
        else:
            new_objects = [TrackedObject(class_id, -1, pose, self.frame_id)
                for class_id, pose in zip(classes_ids, objects_poses)]

        dists = self._compute_distances_matrix(new_objects)
        self._fuse_class_id(dists, new_objects)
        if len(tracking_ids) > 0:
            self._fuse_tracking_2d_id(dists, new_objects)

        new_to_tracked, tracked_to_new = self._match(dists)

        # update already tracked objects
        camera_pose_inv = np.linalg.inv(camera_pose)
        for j, (i, tracked_object) in enumerate(zip(tracked_to_new, self.tracked_objects)):
            if i != -1:
                tracked_object.update(new_objects[i])
            elif tracked_object.is_visible(camera_pose_inv, depth, self.K, self.D):
                tracked_object.visible_without_updates += 1

        # remove lost objects
        max_lost_frames = 2  # including
        tracked_objects_keep = list()
        for tracked_object in self.tracked_objects:
            if tracked_object.visible_without_updates <= max_lost_frames:
                tracked_objects_keep.append(tracked_object)
        self.tracked_objects = tracked_objects_keep

        # add new objects
        for j, new_object in zip(new_to_tracked, new_objects):
            if j == -1:
                new_object.activate()
                self.tracked_objects.append(new_object)

    def _get_objects_poses(self, depth, masks_in_rois, rois, camera_pose):
        objects_poses_in_camera = self._get_objects_poses_in_camera(
            depth, masks_in_rois, rois)
        R = camera_pose[:3, :3]
        t = camera_pose[:3, 3]
        objects_poses = np.matmul(R, objects_poses_in_camera.T).T + t
        return objects_poses

    def _get_objects_poses_in_camera(self, depth, masks_in_rois, rois):
        fx = self.K[0, 0]
        fy = self.K[1, 1]
        cx = self.K[0, 2]
        cy = self.K[1, 2]
        depth_scale = get_depth_scale(depth)
        object_poses = list()
        for mask_in_roi, roi in zip(masks_in_rois, rois):
            if self.erosion_size > 0:
                mask_in_roi = cv2.erode(mask_in_roi, self.erosion_element,
                    borderType=cv2.BORDER_CONSTANT, borderValue=0)
            z = depth[roi][mask_in_roi != 0] * depth_scale
            valid = (z > 0) & np.isfinite(z)
            if np.count_nonzero(valid) < 15:
                object_pose = np.array([np.nan] * 3)
                object_poses.append(object_pose)
                continue

            v, u = np.where(mask_in_roi)
            v += roi[0].start
            u += roi[1].start

            z = z[valid]
            u = u[valid]
            v = v[valid]
            x = (u - cx) / fx * z
            y = (v - cy) / fy * z
            object_pose = np.array([x.mean(), y.mean(), z.mean()])
            object_poses.append(object_pose)
        if len(object_poses) > 0:
            object_poses = np.array(object_poses)
        else:
            object_poses = np.empty((0, 3))

        return object_poses

    def _compute_distances_matrix(self, new_objects: List[TrackedObject]):
        dists = np.empty((len(new_objects), len(self.tracked_objects)), dtype=float)
        for i, new_object in enumerate(new_objects):
            for j, tracked_object in enumerate(self.tracked_objects):
                dist = np.sum(np.square(new_object.pose - tracked_object.pose))
                dists[i, j] = dist
        return dists
    
    def _fuse_class_id(self, dists, new_objects: List[TrackedObject]):
        for i, new_object in enumerate(new_objects):
            for j, tracked_object in enumerate(self.tracked_objects):
                if new_object.class_id != tracked_object.class_id:
                    dists[i, j] = np.inf

    def _fuse_tracking_2d_id(self, dists, new_objects: List[TrackedObject]):
        for i, new_object in enumerate(new_objects):
            for j, tracked_object in enumerate(self.tracked_objects):
                if new_object.tracking_2d_id == tracked_object.tracking_2d_id and \
                        new_object.tracking_2d_id != -1:
                    dists[i, j] = 0

    def _match(self, dists):
        if dists.size == 0:
            new_to_tracked = np.full((dists.shape[0],), -1, dtype=int)
            tracked_to_new = np.full((dists.shape[1],), -1, dtype=int)
            return new_to_tracked, tracked_to_new

        max_range = 0.25
        new_to_tracked, tracked_to_new = lap.lapjv(dists, cost_limit=(max_range * max_range), extend_cost=True, return_cost=False)
        return new_to_tracked, tracked_to_new
