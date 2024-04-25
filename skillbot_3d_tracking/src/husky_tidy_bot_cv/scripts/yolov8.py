import torch
from ultralytics import YOLO
from ultralytics.yolo.utils.ops import preprocess_results
from kas_utils.visualization import draw_objects
import numpy as np
import cv2


class YOLOv8_wrapper(YOLO):
    def __init__(self, model_file, weights_file, min_score=0.7):
        super().__init__(model_file)

        self.model_file = model_file
        self.weights_file = weights_file
        self.min_score = min_score

        weights = torch.load(self.weights_file)['model']
        self.model.load(weights)
        self.warmup()

    def segment(self, image):
        if isinstance(image, str):
            image = cv2.imread(image)
        results = self(image, save=False, show=False, verbose=False,
            conf=self.min_score, imgsz=1280)
        height, width = image.shape[:2]
        scores, classes_ids, boxes, masks = preprocess_results(results, (height, width))
        return scores, classes_ids, boxes, masks
