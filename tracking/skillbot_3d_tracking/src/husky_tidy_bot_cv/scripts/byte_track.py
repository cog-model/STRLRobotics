import cv2
import numpy as np
from yolox.tracker.byte_tracker import BYTETracker
from kas_utils.visualization import draw_objects


class BYTETracker_wrapper:
    def __init__(self, track_thresh=0.1, track_buffer=30, match_thresh=0.9, frame_rate=30):
        self.track_thresh = track_thresh
        self.track_buffer = track_buffer
        self.match_thresh = match_thresh
        self.frame_rate = frame_rate

        self.trackers = dict()

    def track(self, boxes, scores, classes_ids, masks=None):
        tracked_objects = list()
        unique_classes_ids = np.unique(list(classes_ids) + list(self.trackers.keys()))
        for class_id in unique_classes_ids:
            if class_id not in self.trackers:
                self.trackers[class_id] = BYTETracker(track_thresh=self.track_thresh,
                    track_buffer=self.track_buffer, match_thresh=self.match_thresh,
                    mot20=False, frame_rate=self.frame_rate)
            tracker = self.trackers[class_id]

            selected = (classes_ids == class_id)
            selected_boxes = boxes[selected]
            selected_scores = scores[selected]
            if masks is not None:
                selected_masks = masks[selected]
            else:
                selected_masks = None
            selected_boxes_scores = \
                np.hstack((selected_boxes, selected_scores[..., np.newaxis]))
            tracked_objects_single_class = tracker.update(selected_boxes_scores,
                (1, 1), (1, 1), masks=selected_masks)
            tracked_objects_single_class = \
                zip([class_id] * len(tracked_objects_single_class), tracked_objects_single_class)
            tracked_objects.extend(tracked_objects_single_class)
        return tracked_objects

    @staticmethod
    def draw_tracked_objects(image, tracked_objects, draw_boxes=False,
            palette=((0, 0, 255), (0, 255, 0), (255, 0, 0), (255, 0, 255), (0, 255, 255))):
        scores = list()
        tracking_ids = list()
        masks = list()
        for class_id, tracked_object in tracked_objects:
            scores.append(1)
            tracking_ids.append(tracked_object.track_id)
            masks.append(tracked_object.mask)
        scores = np.array(scores)
        tracking_ids = np.array(tracking_ids)
        masks = np.array(masks)
        draw_objects(image, scores, tracking_ids, masks=masks,
            draw_ids=True, draw_masks=True,
            palette=palette, color_by_object_id=True)
