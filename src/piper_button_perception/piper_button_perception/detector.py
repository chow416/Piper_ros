"""Shared model and preprocessing contract for training and later ROS inference."""
from functools import partial

from PIL import Image
import torch
from torch import nn
from torchvision.models.detection import (
    SSDLite320_MobileNet_V3_Large_Weights,
    ssdlite320_mobilenet_v3_large,
)
from torchvision.models.detection.ssdlite import SSDLiteClassificationHead
from torchvision.transforms.functional import pil_to_tensor


CLASSES = {1: "button_blue", 2: "button_red", 3: "button_green"}
INPUT_SIZE = 320


def letterbox(image):
    """Return RGB tensor and actual scale/offset, retaining the original aspect."""
    width, height = image.size
    scale = INPUT_SIZE / max(width, height)
    resized = (round(width * scale), round(height * scale))
    left, top = (INPUT_SIZE - resized[0]) // 2, (INPUT_SIZE - resized[1]) // 2
    canvas = Image.new("RGB", (INPUT_SIZE, INPUT_SIZE), (114, 114, 114))
    canvas.paste(image.resize(resized, Image.Resampling.BILINEAR), (left, top))
    geometry = (resized[0] / width, resized[1] / height, left, top)
    return pil_to_tensor(canvas).float() / 255.0, geometry


def restore_boxes(boxes, geometry, image_size):
    sx, sy, left, top = geometry
    boxes = boxes.clone()
    boxes[:, [0, 2]] = ((boxes[:, [0, 2]] - left) / sx).clamp(0, image_size[0])
    boxes[:, [1, 3]] = ((boxes[:, [1, 3]] - top) / sy).clamp(0, image_size[1])
    return boxes


def build_model(pretrained=False, freeze_backbone=True):
    weights = SSDLite320_MobileNet_V3_Large_Weights.DEFAULT if pretrained else None
    model = ssdlite320_mobilenet_v3_large(
        weights=weights, weights_backbone=None,
        num_classes=None if pretrained else len(CLASSES) + 1,
        score_thresh=0.001, detections_per_img=100,
    )
    if pretrained:
        channels = [block[-1].in_channels for block in model.head.classification_head.module_list]
        model.head.classification_head = SSDLiteClassificationHead(
            channels, model.anchor_generator.num_anchors_per_location(), len(CLASSES) + 1,
            partial(nn.BatchNorm2d, eps=0.001, momentum=0.03),
        )
    if freeze_backbone:
        model.backbone.requires_grad_(False)
    return model


def load_detector(checkpoint_path):
    """Only load tensor/state dictionaries; shared with the future inference node."""
    checkpoint = torch.load(checkpoint_path, map_location="cpu", weights_only=True)
    if checkpoint["classes"] != CLASSES or checkpoint["preprocessing"] != "letterbox320_rgb_114":
        raise ValueError("Checkpoint class map or preprocessing does not match this detector")
    model = build_model()
    model.load_state_dict(checkpoint["model"])
    model.eval()
    return model, checkpoint
