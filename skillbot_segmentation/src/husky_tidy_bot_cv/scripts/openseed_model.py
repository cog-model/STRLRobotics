import torch
import cv2
import albumentations as A
from albumentations.pytorch import ToTensorV2
import sys 
import numpy as np
from PIL import Image
from torchvision.ops.boxes import nms
from detectron2.config import get_cfg, LazyConfig
from detectron2.checkpoint import DetectionCheckpointer
import open_clip

from torch.nn import functional as F
from fixed_cats import FIXED_CATEGORIES
# add fpreviouser with "openseed" directory to path
#pth = "/home/wingrune/catkin_ws/src/openseed_src"
pth = "/home/administrator/zemskova_ts/husky_tidy_bot_cv_ws/src/openseed_src"
sys.path.insert(0, pth)

from openseed import build_model
from openseed.BaseModel import BaseModel

class OpenSeeD_wrapper:
    def __init__(self, model_file, weights_file, min_score_seen=0.7, min_score_unseen=0.1, similarity_threshold=0.25, nms_threshold=0.5):
        cfg = get_cfg()
        cfg = LazyConfig.load(model_file)
        self.model = BaseModel(cfg, build_model(cfg))
        self.model_file = model_file
        self.weights_file = weights_file
        self.min_score_seen = min_score_seen
        self.min_score_unseen = min_score_unseen
        self.similarity_threshold = similarity_threshold
        DetectionCheckpointer(self.model, save_dir=cfg.OUTPUT_DIR).resume_or_load(
            self.weights_file, resume=False
        )
        self.model = self.model.eval().cuda()
        self.model_clip, _, self.preprocess = open_clip.create_model_and_transforms('ViT-B-32', pretrained='openai')
        self.tokenizer = open_clip.get_tokenizer('ViT-B-32')
        self.model_clip.cuda()
        self.transforms = A.Compose([A.Resize(256, 480, interpolation=cv2.INTER_CUBIC), ToTensorV2()])
        self.previous_categories = ['background']
        self.set_categories = ['background']
        self.previous_ids = [0]
        
        self.openseed_id_to_cat = {
            i: cat
            for i, cat in enumerate(self.previous_categories)
        }
        self.cats_for_clip = []
        self.cats_with_features_to_cat_id = {
            'background': 0
        }
        self.cat_id_to_cats = {
            0: 'background'
        }
        self.cats_to_cat_id = {
            'background': 0
        }
        self.seen_classes = [cat["name"] for cat in FIXED_CATEGORIES]
        self.text = self.tokenizer(self.cats_for_clip).cuda()
        with torch.no_grad(), torch.autocast(enabled=False, device_type='cuda', dtype=torch.float16):
            self.text_features = self.model_clip.encode_text(self.text)
            self.text_features /= self.text_features.norm(dim=-1, keepdim=True)

        self.model.model.thing_dataset_id_to_contiguous_id = {x:x for x in range(len(self.previous_categories))}
        with torch.no_grad(), torch.autocast(enabled=False, device_type='cuda', dtype=torch.float16):
            self.model.model.sem_seg_head.predictor.lang_encoder.get_text_embeddings(self.previous_categories, is_eval=True)
        self.model.model.sem_seg_head.num_classes = len(self.previous_categories)
        self.nms_threshold = nms_threshold
        self._warmup()

    def _warmup(self):
        image = np.zeros((25, 25, 3), dtype=np.uint8)
        cv2.imwrite('tmp.jpg', image)
        self.segment('tmp.jpg', self.previous_categories, self.previous_categories, [0], reject_categories=tuple())

    def segment(self, image_ori, categories, categories_with_features, categories_with_features_ids, reject_categories=tuple()):
        assert len(categories) == len(categories_with_features), "Number of categories with features is not equal to number of categories without features"
        assert len(categories_with_features) == len(categories_with_features_ids), "Number of classes ids is not equal to number of classes names"

        if len(categories) == 0:
            return [], [], [], []

        if isinstance(image_ori, str):
            image_ori = cv2.imread(image_ori)

        image_ori = cv2.cvtColor(image_ori, cv2.COLOR_BGR2RGB)
        if categories_with_features == self.previous_categories and categories_with_features_ids == self.previous_ids:
            pass
        else:
            self.set_categories = list(categories)
            self.openseed_id_to_cat = {
                i: cat
                for i, cat in enumerate(self.set_categories)
            }
            self.cats_for_clip = [
                cat_with_feature
                for i, cat_with_feature in enumerate(categories_with_features)
                if cat_with_feature != categories[i] and cat_with_feature not in self.seen_classes
            ]

            self.cats_with_features_to_cat_id = {
                cat: i
                for cat, i in zip(categories_with_features, categories_with_features_ids)
            }
            self.cat_id_to_cats = {
                i: cat
                for i, cat in zip(categories_with_features_ids, categories)
            }

            self.cats_to_cat_id = {
                cat: i
                for i, cat in zip(categories_with_features_ids, categories)
            }

            self.model.model.thing_dataset_id_to_contiguous_id = {x:x for x in range(len(self.set_categories))}
            with torch.no_grad(), torch.autocast(enabled=False, device_type='cuda', dtype=torch.float16):
                self.model.model.sem_seg_head.predictor.lang_encoder.get_text_embeddings(self.set_categories, is_eval=True)
            self.model.model.sem_seg_head.num_classes = len(self.set_categories)
            self.previous_categories = categories_with_features
            self.previous_ids = categories_with_features_ids
            self.text = self.tokenizer([f"a photo of a {cat_clip}" for cat_clip in self.cats_for_clip]).cuda()
            with torch.no_grad(), torch.autocast(enabled=False, device_type='cuda', dtype=torch.float16):
                self.text_features = self.model_clip.encode_text(self.text)
                self.text_features /= self.text_features.norm(dim=-1, keepdim=True)

        height, width, _ = image_ori.shape
        images = self.transforms(image=image_ori)['image']
        #print(images.shape)

        batch_inputs = [{'image': images, 'height': images.shape[1], 'width': images.shape[2]}]

        with torch.no_grad(), torch.autocast(enabled=False, device_type='cuda', dtype=torch.float16):
            outputs = self.model.forward(batch_inputs, inference_task='inst_seg')

        inst_seg = outputs[-1]['instances']

        reject_categories_id = torch.LongTensor([
            idx for idx, cat in enumerate(self.set_categories)
            if cat in reject_categories]).to(inst_seg.pred_classes)

        seen_categories_id = torch.LongTensor([
            idx for idx, cat in enumerate(self.set_categories)
            if cat in self.seen_classes and cat != 'socks' and cat != 'sock']).to(inst_seg.pred_classes)

        high_conf_categories_id = torch.LongTensor([
            idx for idx, cat in enumerate(self.set_categories)
            if cat in [
                "slipper", "socks", "sock",
                "slippers"
            ]]).to(inst_seg.pred_classes)

        super_high_conf_categories_id = torch.LongTensor([
            idx for idx, cat in enumerate(self.set_categories)
            if cat in [
                "charging", "hoodie", "hoodies"
            ]]).to(inst_seg.pred_classes)


        nms_indices = nms(inst_seg.pred_boxes.tensor, inst_seg.scores, self.nms_threshold)

        masks = inst_seg.pred_masks[nms_indices]
        classes_ids = inst_seg.pred_classes[nms_indices]
        boxes = inst_seg.pred_boxes[nms_indices]
        scores = inst_seg.scores[nms_indices]

        selected_indices = torch.where(
            (
                (scores >= self.min_score_seen) &
                (torch.logical_not(torch.isin(classes_ids, reject_categories_id))) &
                (torch.logical_not(torch.isin(classes_ids, high_conf_categories_id))) &
                (torch.logical_not(torch.isin(classes_ids, super_high_conf_categories_id))) &
                (torch.isin(classes_ids, seen_categories_id))
            ) |
            (
                (scores >= self.min_score_unseen) &
                (torch.logical_not(torch.isin(classes_ids, reject_categories_id))) &
                (torch.logical_not(torch.isin(classes_ids, seen_categories_id))) &
                (torch.logical_not(torch.isin(classes_ids, high_conf_categories_id)))
            ) |
            (
                (scores >= 0.6) &
                (torch.logical_not(torch.isin(classes_ids, reject_categories_id))) &
                (torch.isin(classes_ids, high_conf_categories_id))
            ) |
            (
                (scores >= 0.75) &
                (torch.logical_not(torch.isin(classes_ids, reject_categories_id))) &
                (torch.isin(classes_ids, super_high_conf_categories_id))
            )
        )[0]

        masks = masks[selected_indices]
        classes_ids = classes_ids[selected_indices]
        boxes = boxes[selected_indices].tensor
        scores = scores[selected_indices]
        if masks.shape[0] > 0:
            masks = masks[:, : masks.shape[1], : masks.shape[2]].expand(1, -1, -1, -1)
            #print("Before interpolation", masks.shape)
            masks = F.interpolate(
                masks, size=(height, width), mode="nearest"
            )[0]
            #print("After interpolation", masks.shape)

            height_box = height
            width_box = width
            x1 = boxes[:,0]
            y1 = boxes[:,1]
            x2 = boxes[:,2]
            y2 = boxes[:,3]
            boxes = torch.stack([x1,y1,x2,y2]).permute(1,0)
            scale = torch.tensor([width_box/images.shape[2], height_box/images.shape[1], width_box/images.shape[2], height_box/images.shape[1]])[None,:].to(boxes.device)
            boxes = boxes*scale

        masks = masks.cpu().numpy().astype(np.uint8)
        classes_ids = classes_ids.cpu().tolist()
        classes_ids = [self.cats_to_cat_id[self.openseed_id_to_cat[i]] for i in classes_ids]
        classes_ids = np.array(classes_ids).astype(int)
        boxes = boxes.cpu().numpy().astype(int)
        scores = scores.cpu().numpy().astype(float)

        filtered_indices = []

        #print("Cats for clip", self.cats_for_clip)
        cats_ids_for_subset = [
            self.cats_to_cat_id[self.cat_id_to_cats[self.cats_with_features_to_cat_id[cat]]]
            for cat in self.cats_for_clip
        ]

        #print(cats_ids_for_subset)
        #print(self.cat_name_to_openseed_id)
        #print("Clasess ids", classes_ids)
        selected_indices = np.where(
            np.isin(classes_ids, cats_ids_for_subset)
        )[0]


        for pred_idx in selected_indices:
            pred_mask = masks[pred_idx]
            pred_box = boxes[pred_idx]
            x1 = int(pred_box[0])
            y1 = int(pred_box[1])
            x2 = int(pred_box[2])
            y2 = int(pred_box[3])
            #print(x1,x2,y1,y2)
            image = image_ori[y1:y2,x1:x2,:]
            pred_mask = pred_mask[y1:y2,x1:x2]
            mask = np.stack([pred_mask for i in range(3)], axis=2)
            image = (image * mask).clip(0, 255).astype(np.uint8)

            image = self.preprocess(Image.fromarray(image)).unsqueeze(0).cuda()
            with torch.no_grad():
                image_features = self.model_clip.encode_image(image)
                image_features /= image_features.norm(dim=-1, keepdim=True)
            
            text_probs = image_features @ self.text_features.T
            cat_clip = [
                cat for cat in self.cats_for_clip
                if self.cats_to_cat_id[self.cat_id_to_cats[self.cats_with_features_to_cat_id[cat]]] == classes_ids[pred_idx]
            ]
            cat_indices = torch.LongTensor([
                idx for idx, cat in enumerate(self.cats_for_clip)
                if self.cats_to_cat_id[self.cat_id_to_cats[self.cats_with_features_to_cat_id[cat]]] == classes_ids[pred_idx]
            ]).to(inst_seg.pred_classes)
            
            if torch.max(text_probs[0][cat_indices]) > self.similarity_threshold:
                new_class = int(torch.argmax(text_probs[0][cat_indices]).cpu())
                new_class = self.cats_with_features_to_cat_id[cat_clip[new_class]]
                classes_ids[pred_idx] = new_class
            else:
                filtered_indices.append(pred_idx)
        masks = np.delete(masks, filtered_indices, axis=0)
        classes_ids = np.delete(classes_ids, filtered_indices)
        boxes = np.delete(boxes, filtered_indices, axis=0)
        scores = np.delete(scores, filtered_indices)

        masks = 255*masks.astype(np.uint8)

        #print(scores, classes_ids)
        #print(masks.shape)
        #print(boxes.shape)
        #print(boxes)
        return scores, classes_ids, boxes, masks
