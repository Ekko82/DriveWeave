"""CPU YOLOX-S RGB detector; no CARLA truth, LiDAR or actor data are inputs.

Preprocessing/grid decoding/NMS adapted from Megvii YOLOX official demo:
https://github.com/Megvii-BaseDetection/YOLOX
Copyright (c) Megvii, Inc. and its affiliates. Apache-2.0.
Changes: explicit RGB input contract, road-class filtering, strict validation,
CPU-only session, source/model provenance and immutable input arrays.
License copy: docs/licenses/YOLOX-APACHE-2.0.txt.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np

MODEL_URL = "https://github.com/Megvii-BaseDetection/YOLOX/releases/download/0.1.1rc0/yolox_s.onnx"
OFFICIAL_MODEL_SHA256 = "c5c2d13e59ae883e6af3b45daea64af4833a4951c92d116ec270d9ddbe998063"
ROAD_CLASSES = {0: "person", 1: "bicycle", 2: "car", 3: "motorcycle", 5: "bus", 7: "truck"}
INPUT_SHAPE = (640, 640)
NMS_IOU = 0.45
SOURCE_URLS = [
    "https://raw.githubusercontent.com/Megvii-BaseDetection/YOLOX/main/demo/ONNXRuntime/onnx_inference.py",
    "https://raw.githubusercontent.com/Megvii-BaseDetection/YOLOX/main/yolox/utils/demo_utils.py",
    "https://raw.githubusercontent.com/Megvii-BaseDetection/YOLOX/main/yolox/data/data_augment.py",
]


def preprocess_rgb(rgb: np.ndarray) -> tuple[np.ndarray, float]:
    """RGB uint8 HWC -> BGR float32 CHW, top-left pad114, no normalization."""
    if not isinstance(rgb, np.ndarray) or rgb.dtype != np.uint8:
        raise ValueError("rgb must be a uint8 numpy array")
    if rgb.ndim != 3 or rgb.shape[2] != 3 or min(rgb.shape[:2]) <= 0:
        raise ValueError("rgb must have nonempty HxWx3 shape")
    height, width = rgb.shape[:2]
    ratio = min(INPUT_SHAPE[0] / height, INPUT_SHAPE[1] / width)
    resized_width, resized_height = int(width * ratio), int(height * ratio)
    if min(resized_width, resized_height) < 1:
        raise ValueError("Image aspect ratio is too extreme for 640 preprocessing")
    bgr = np.ascontiguousarray(rgb[:, :, ::-1])
    resized = cv2.resize(bgr, (resized_width, resized_height), interpolation=cv2.INTER_LINEAR)
    padded = np.full((*INPUT_SHAPE, 3), 114, dtype=np.uint8)
    padded[:resized_height, :resized_width] = resized
    return np.ascontiguousarray(padded.transpose(2, 0, 1), dtype=np.float32), ratio


def decode_yolox(raw_output: np.ndarray) -> np.ndarray:
    """Decode official 8/16/32-stride raw model output, leaving it untouched."""
    raw = np.asarray(raw_output)
    if raw.shape != (1, 8400, 85) or not np.isfinite(raw).all():
        raise ValueError("Expected finite YOLOX-S raw output 1x8400x85")
    predictions = raw[0].astype(np.float32, copy=True)
    grids, strides = [], []
    for stride in (8, 16, 32):
        h, w = INPUT_SHAPE[0] // stride, INPUT_SHAPE[1] // stride
        xx, yy = np.meshgrid(np.arange(w), np.arange(h))
        grids.append(np.stack((xx, yy), axis=2).reshape(-1, 2))
        strides.append(np.full((h * w, 1), stride))
    grid = np.concatenate(grids).astype(np.float32)
    expanded = np.concatenate(strides).astype(np.float32)
    predictions[:, :2] = (predictions[:, :2] + grid) * expanded
    with np.errstate(over="ignore", invalid="ignore"):
        predictions[:, 2:4] = np.exp(predictions[:, 2:4]) * expanded
    if not np.isfinite(predictions).all():
        raise ValueError("Nonfinite YOLOX decoded boxes")
    return predictions


def nms(boxes: np.ndarray, scores: np.ndarray, threshold: float = NMS_IOU) -> list[int]:
    """Official NumPy class-agnostic NMS, including its +1 pixel area rule."""
    if not len(boxes):
        return []
    x1, y1, x2, y2 = boxes.T
    areas = (x2 - x1 + 1) * (y2 - y1 + 1)
    order = scores.argsort()[::-1]
    keep = []
    while order.size:
        current = int(order[0])
        keep.append(current)
        rest = order[1:]
        inter_w = np.maximum(0.0, np.minimum(x2[current], x2[rest]) - np.maximum(x1[current], x1[rest]) + 1)
        inter_h = np.maximum(0.0, np.minimum(y2[current], y2[rest]) - np.maximum(y1[current], y1[rest]) + 1)
        intersection = inter_w * inter_h
        overlap = intersection / (areas[current] + areas[rest] - intersection)
        order = rest[overlap <= threshold]
    return keep


def postprocess(raw_output: np.ndarray, ratio: float, image_shape: tuple[int, int], confidence: float) -> list[dict]:
    predictions = decode_yolox(raw_output)
    boxes = np.column_stack((predictions[:, :2] - predictions[:, 2:4] / 2,
                             predictions[:, :2] + predictions[:, 2:4] / 2)) / ratio
    scores_all = predictions[:, 4:5] * predictions[:, 5:]
    classes = scores_all.argmax(axis=1)
    scores = scores_all[np.arange(len(classes)), classes]
    # Argmax over all COCO classes first; never reinterpret a non-road class as a car.
    selected = scores > confidence
    boxes, scores, classes = boxes[selected], scores[selected], classes[selected]
    result = []
    height, width = image_shape
    for index in nms(boxes, scores):
        class_id = int(classes[index])
        if class_id not in ROAD_CLASSES:
            continue
        box = boxes[index].copy()
        box[[0, 2]] = np.clip(box[[0, 2]], 0, width)
        box[[1, 3]] = np.clip(box[[1, 3]], 0, height)
        if box[2] <= box[0] or box[3] <= box[1]:
            continue
        result.append({"bbox_xyxy": [float(v) for v in box], "class_id": class_id,
                       "class_name": ROAD_CLASSES[class_id], "score": float(scores[index])})
    return result


class Detector:
    """One reusable CPU inference session; detect accepts only RGB pixels.

    class_id uses zero-based COCO indices (car=2), not CARLA semantic IDs.
    bbox_xyxy uses original-image continuous pixel coordinates clipped to W/H.
    """
    def __init__(self, model_path, threads: int = 4, confidence: float = .3):
        if isinstance(threads, bool) or not isinstance(threads, int) or not 1 <= threads <= 64:
            raise ValueError("threads must be an integer in [1,64]")
        if not np.isfinite(confidence) or not 0 < confidence < 1:
            raise ValueError("confidence must be finite and between zero and one")
        self.model_path = Path(model_path).resolve(strict=True)
        self.model_sha256 = hashlib.sha256(self.model_path.read_bytes()).hexdigest()
        self.threads, self.confidence = threads, float(confidence)
        import onnxruntime as ort
        options = ort.SessionOptions()
        options.intra_op_num_threads = threads
        options.inter_op_num_threads = 1
        options.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL
        self.session = ort.InferenceSession(str(self.model_path), sess_options=options,
                                            providers=["CPUExecutionProvider"])
        inputs = self.session.get_inputs()
        outputs = self.session.get_outputs()
        if len(inputs) != 1 or inputs[0].shape != [1, 3, 640, 640] or inputs[0].type != "tensor(float)":
            raise ValueError("Model must have one float input 1x3x640x640")
        if len(outputs) != 1 or outputs[0].shape != [1, 8400, 85]:
            raise ValueError("Model must have one raw YOLOX-S output 1x8400x85")
        self.input_name = inputs[0].name
        self.output_name = outputs[0].name
        self.provenance = {
            "model": "YOLOX-S", "model_path": str(self.model_path), "model_sha256": self.model_sha256,
            "official_model_url": MODEL_URL, "matches_known_official_model_sha256": self.model_sha256 == OFFICIAL_MODEL_SHA256,
            "source_urls": SOURCE_URLS, "upstream_license": "Apache-2.0",
            "runtime": "onnxruntime", "runtime_version": ort.__version__, "providers": self.session.get_providers(),
            "threads": threads, "confidence_threshold": self.confidence, "nms_iou_threshold": NMS_IOU,
            "input_shape": [1, 3, 640, 640], "output_shape": [1, 8400, 85],
            "preprocessing": "RGB to BGR; top-left resize INTER_LINEAR; pad114; float32 CHW; no normalization",
            "class_id_namespace": "COCO zero-based 80-class indices", "road_classes": ROAD_CLASSES,
            "ground_truth_used_for_inference": False, "lidar_used_for_detection": False,
        }

    def detect(self, rgb_numpy: np.ndarray) -> list[dict]:
        tensor, ratio = preprocess_rgb(rgb_numpy)
        raw = self.session.run([self.output_name], {self.input_name: tensor[None]})[0]
        return postprocess(raw, ratio, rgb_numpy.shape[:2], self.confidence)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--image", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--threads", type=int, default=4)
    parser.add_argument("--confidence", type=float, default=.3)
    args = parser.parse_args()
    bgr = cv2.imread(str(args.image), cv2.IMREAD_COLOR)
    if bgr is None:
        raise ValueError("Image could not be decoded")
    detector = Detector(args.model, args.threads, args.confidence)
    payload = {"image_sha256": hashlib.sha256(args.image.read_bytes()).hexdigest(),
               "detections": detector.detect(bgr[:, :, ::-1]), "provenance": detector.provenance}
    args.out.write_text(json.dumps(payload, indent=2), encoding="utf-8")


if __name__ == "__main__":
    main()
