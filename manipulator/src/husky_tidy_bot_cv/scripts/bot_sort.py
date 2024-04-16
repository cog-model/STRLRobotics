import cv2
import numpy as np
from tracker.bot_sort import BoTSORT
from kas_utils.visualization import draw_objects


class BoTSORT_wrapper(BoTSORT):
    def __init__(self, track_high_thresh=0.3, track_low_thresh=0.1,
            new_track_thresh=0.3, track_buffer_size=30, match_thresh=0.8,
            cmc_method="sparseOptFlow", with_reid=False):

        super().__init__(track_high_thresh=track_high_thresh,
            track_low_thresh=track_low_thresh, new_track_thresh=new_track_thresh,
            track_buffer_size=track_buffer_size, match_thresh=match_thresh,
            cmc_method=cmc_method, with_reid=with_reid, mot20=False)

    def track(self, boxes, scores, classes_ids, image, infos=None):
        boxes_scores = \
            np.hstack((boxes, scores[..., np.newaxis], classes_ids[..., np.newaxis]))
        tracked_objects = self.update(boxes_scores, image, infos=infos)
        return tracked_objects

    @staticmethod
    def preprocess(tracked_objects):
        if len(tracked_objects) > 0:
            scores, classes_ids, tracking_ids, boxes, infos = \
                zip(*[(to.score, to.class_id, to.track_id, to.tlbr, to.info)
                    for to in tracked_objects])

            scores = np.array(scores, dtype=float)
            classes_ids = np.array(classes_ids, dtype=int)
            tracking_ids = np.array(tracking_ids, dtype=int)
            boxes = np.array(boxes, dtype=int)
        else:
            scores = np.empty((0,), dtype=float)
            classes_ids = np.empty((0,), dtype=int)
            tracking_ids = np.empty((0,), dtype=int)
            boxes = np.empty((0, 4), dtype=int)
            infos = list()

        return scores, classes_ids, tracking_ids, boxes, infos
