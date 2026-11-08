#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Ultralytics YOLO26-n & Vectorized Edge Inference Engine
=======================================================
CS532 - Advanced AI Robotics (Module Computer Vision - Hieu Me)
Academic Standards & Engineering Highlights:
  - Multi-Backend Inference Priority:
      1. TensorRT 7.1.3 Engine (`yolo26n.engine`, `yolov8n.engine`) trên Jetson GPU Maxwell.
      2. ONNX Runtime (`yolo26n.onnx`, `yolov8n.onnx`) với giải mã tensor thuần NumPy.
      3. Ultralytics PyTorch (`yolo26n.pt`).
      4. Synthetic/Mock Mode (Offline Test không cần runtime nặng).
  - 100% Vectorized Tensor Decoding cho output (1, 84, 8400) -> Độ trễ < 1ms (không dùng loop for chậm).
  - Letterbox Preprocessing (640x640) giữ nguyên tỉ lệ khung hình 4:3 của camera.
  - Tích hợp tính góc Azimuth ROS REP-103 CCW và Asymmetric Social Bubble cho người.
"""

import os
import sys
import time
import math
import json
from typing import Dict, List, Tuple, Optional, Any, Union
import numpy as np
import cv2

from .floor_ipm import FloorIPMTransformer
from .proxemics import pixel_to_azimuth_deg, compute_dynamic_proxemics

COCO_CLASSES = [
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat",
    "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat",
    "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack",
    "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball",
    "kite", "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket",
    "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple",
    "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair",
    "couch", "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse",
    "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink",
    "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier", "toothbrush"
]

TARGET_NAV_CLASSES = {"person", "chair", "backpack", "bottle", "suitcase", "box", "couch"}
MODEL_INPUT_SIZE = 640


def letterbox_blob(frame: np.ndarray, size: int = MODEL_INPUT_SIZE) -> Tuple[np.ndarray, float, Tuple[float, float]]:
    """
    Tiền xử lý Letterbox chuẩn Ultralytics: Giữ nguyên tỉ lệ khung hình, đệm viền xám (114, 114, 114).
    Trả về: (blob 1x3xSxS float32 RGB [0, 1], scale_ratio, (pad_left, pad_top)).
    """
    h, w = frame.shape[:2]
    ratio = min(size / float(h), size / float(w))
    new_w, new_h = int(round(w * ratio)), int(round(h * ratio))
    pad_x = (size - new_w) / 2.0
    pad_y = (size - new_h) / 2.0

    resized = cv2.resize(frame, (new_w, new_h), interpolation=cv2.INTER_LINEAR) if (new_w, new_h) != (w, h) else frame
    top, bottom = int(round(pad_y - 0.1)), int(round(pad_y + 0.1))
    left, right = int(round(pad_x - 0.1)), int(round(pad_x + 0.1))
    padded = cv2.copyMakeBorder(resized, top, bottom, left, right, cv2.BORDER_CONSTANT, value=(114, 114, 114))

    # Gộp BGR->RGB, scale 1/255 và HWC->NCHW trong một lần gọi C++
    blob = cv2.dnn.blobFromImage(padded, scalefactor=1.0 / 255.0, swapRB=True)
    return np.ascontiguousarray(blob, dtype=np.float32), ratio, (float(left), float(top))


def decode_yolo_output(
    raw_output: np.ndarray,
    orig_w: int,
    orig_h: int,
    ratio: float,
    pad: Tuple[float, float],
    conf_threshold: float = 0.40,
    iou_threshold: float = 0.45,
) -> List[Tuple[List[int], float, int]]:
    """
    Giải mã tensor (1, 84, 8400) hoàn toàn bằng NumPy vectorization.
    Trả về danh sách: [(bbox_xyxy, confidence, class_id), ...]
    """
    out = raw_output[0] if raw_output.ndim == 3 else raw_output
    if out.shape[0] != 84 and out.shape[-1] == 84:
        out = out.T

    scores = out[4:, :]
    confidences = scores.max(axis=0)
    keep = np.nonzero(confidences >= conf_threshold)[0]
    if keep.size == 0:
        return []

    confs = confidences[keep]
    cls_ids = scores[:, keep].argmax(axis=0)
    cx, cy, bw, bh = out[0, keep], out[1, keep], out[2, keep], out[3, keep]

    pad_x, pad_y = pad
    x1 = np.clip((cx - bw / 2.0 - pad_x) / ratio, 0, orig_w - 1)
    y1 = np.clip((cy - bh / 2.0 - pad_y) / ratio, 0, orig_h - 1)
    x2 = np.clip((cx + bw / 2.0 - pad_x) / ratio, 0, orig_w - 1)
    y2 = np.clip((cy + bh / 2.0 - pad_y) / ratio, 0, orig_h - 1)

    boxes_xywh = np.stack([x1, y1, np.maximum(x2 - x1, 1), np.maximum(y2 - y1, 1)], axis=1).astype(np.int32).tolist()
    confs_list = confs.astype(float).tolist()
    indices = cv2.dnn.NMSBoxes(boxes_xywh, confs_list, conf_threshold, iou_threshold)
    if len(indices) == 0:
        return []

    results = []
    for idx in np.array(indices).flatten():
        i = int(idx)
        results.append((
            [int(x1[i]), int(y1[i]), int(x2[i]), int(y2[i])],
            float(confs_list[i]),
            int(cls_ids[i]),
        ))
    return results


class YoloDetector:
    """Class điều phối nhận diện vật thể YOLO đa backend phục vụ điều hướng JetBot."""

    def __init__(
        self,
        model_path: Optional[str] = None,
        config_path: Optional[str] = None,
        conf_threshold: float = 0.40,
        iou_threshold: float = 0.45,
    ):
        self.conf_threshold = float(conf_threshold)
        self.iou_threshold = float(iou_threshold)
        self.ipm = FloorIPMTransformer(config_path=config_path)

        self.backend = "none"
        self.engine = None
        self.session = None
        self.model = None

        self._init_model(model_path)

    def _init_model(self, model_path: Optional[str]):
        base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        candidates = [
            model_path,
            os.path.join(base_dir, "models", "yolo26n.engine"),
            "/home/jetbot/yolo26n.engine",
            os.path.join(base_dir, "models", "yolov8n.engine"),
            "/home/jetbot/yolov8n.engine",
            os.path.join(base_dir, "models", "yolo26n.onnx"),
            os.path.join(base_dir, "yolo26n.onnx"),
            os.path.join(base_dir, "models", "yolo26n.pt"),
            os.path.join(base_dir, "yolo26n.pt"),
        ]

        # 1. Thử TensorRT trên Jetson Nano
        for c in candidates:
            if c and c.endswith(".engine") and os.path.exists(c):
                if self._init_tensorrt(c):
                    return

        # 2. Thử ONNX Runtime (CPU hoặc CUDA)
        for c in candidates:
            if c and c.endswith(".onnx") and os.path.exists(c):
                if self._init_onnx(c):
                    return

        # 3. Thử Ultralytics PyTorch
        for c in candidates:
            if c and c.endswith(".pt") and os.path.exists(c):
                if self._init_ultralytics(c):
                    return

        # 4. Fallback Mock mode
        print("[YoloDetector] Không tìm thấy model engine/onnx -> Kích hoạt Mock Detection Mode.")
        self.backend = "mock"

    def _init_tensorrt(self, engine_path: str) -> bool:
        try:
            import tensorrt as trt
            import ctypes

            self.cudart = ctypes.CDLL("libcudart.so")
            self.cudart.cudaMalloc.argtypes = [ctypes.POINTER(ctypes.c_void_p), ctypes.c_size_t]
            self.cudart.cudaMalloc.restype = ctypes.c_int
            self.cudart.cudaMemcpy.argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_size_t, ctypes.c_int]
            self.cudart.cudaMemcpy.restype = ctypes.c_int
            self.cudart.cudaFree.argtypes = [ctypes.c_void_p]

            logger = trt.Logger(trt.Logger.WARNING)
            with open(engine_path, "rb") as f:
                runtime = trt.Runtime(logger)
                self.engine = runtime.deserialize_cuda_engine(f.read())
            self.context = self.engine.create_execution_context()

            # Bộ đệm GPU (640x640x3 FP32 và 84x8400 FP32)
            self.input_bytes = 1 * 3 * MODEL_INPUT_SIZE * MODEL_INPUT_SIZE * 4
            self.output_bytes = 1 * 84 * 8400 * 4

            self.d_input = ctypes.c_void_p()
            self.d_output = ctypes.c_void_p()
            self.cudart.cudaMalloc(ctypes.byref(self.d_input), self.input_bytes)
            self.cudart.cudaMalloc(ctypes.byref(self.d_output), self.output_bytes)

            self.h_output = np.zeros((1, 84, 8400), dtype=np.float32)
            self.trt_bindings = [int(self.d_input.value), int(self.d_output.value)]
            self.backend = "tensorrt"
            print(f"[YoloDetector] Đã nạp thành công TensorRT Engine: {os.path.basename(engine_path)}")
            return True
        except Exception as e:
            print(f"[YoloDetector] Không thể nạp TensorRT ({e}).")
            return False

    def _init_onnx(self, onnx_path: str) -> bool:
        try:
            import onnxruntime as ort

            providers = ["CUDAExecutionProvider", "CPUExecutionProvider"]
            available = ort.get_available_providers()
            active_providers = [p for p in providers if p in available]

            self.session = ort.InferenceSession(onnx_path, providers=active_providers)
            self.input_name = self.session.get_inputs()[0].name
            self.backend = "onnxruntime"
            print(f"[YoloDetector] Đã nạp thành công ONNX Model: {os.path.basename(onnx_path)} (Providers: {active_providers})")
            return True
        except Exception as e:
            print(f"[YoloDetector] Không thể nạp ONNX Runtime ({e}).")
            return False

    def _init_ultralytics(self, pt_path: str) -> bool:
        try:
            from ultralytics import YOLO

            self.model = YOLO(pt_path)
            self.backend = "ultralytics"
            print(f"[YoloDetector] Đã nạp thành công PyTorch YOLO: {os.path.basename(pt_path)}")
            return True
        except Exception as e:
            print(f"[YoloDetector] Không thể nạp Ultralytics ({e}).")
            return False

    def detect(self, frame: np.ndarray) -> List[Dict[str, Any]]:
        """
        Nhận diện đối tượng trong khung hình BGR.
        Trả về danh sách dictionary các detections chuẩn hóa.
        """
        h, w = frame.shape[:2]
        raw_detections: List[Tuple[List[int], float, int]] = []

        if self.backend == "tensorrt" and self.engine is not None:
            blob, ratio, pad = letterbox_blob(frame, size=MODEL_INPUT_SIZE)
            import ctypes
            self.cudart.cudaMemcpy(self.d_input, blob.ctypes.data_as(ctypes.c_void_p), self.input_bytes, 1)
            self.context.execute_v2(self.trt_bindings)
            self.cudart.cudaMemcpy(self.h_output.ctypes.data_as(ctypes.c_void_p), self.d_output, self.output_bytes, 2)
            raw_detections = decode_yolo_output(self.h_output, w, h, ratio, pad, self.conf_threshold, self.iou_threshold)

        elif self.backend == "onnxruntime" and self.session is not None:
            blob, ratio, pad = letterbox_blob(frame, size=MODEL_INPUT_SIZE)
            outputs = self.session.run(None, {self.input_name: blob})[0]
            raw_detections = decode_yolo_output(outputs, w, h, ratio, pad, self.conf_threshold, self.iou_threshold)

        elif self.backend == "ultralytics" and self.model is not None:
            results = self.model(frame, verbose=False, imgsz=MODEL_INPUT_SIZE, conf=self.conf_threshold, iou=self.iou_threshold)
            if results and len(results) > 0 and results[0].boxes is not None:
                boxes = results[0].boxes
                xyxy = boxes.xyxy.cpu().numpy()
                confs = boxes.conf.cpu().numpy()
                clss = boxes.cls.cpu().numpy().astype(int)
                for b, c, cl in zip(xyxy, confs, clss):
                    raw_detections.append((b.astype(int).tolist(), float(c), int(cl)))

        elif self.backend == "mock":
            # Tạo detection mẫu mô phỏng người đứng trước mặt
            raw_detections = [([240, 120, 400, 440], 0.88, 0)]

        # Đóng gói và gắn thẻ phương vị, khoảng cách, proxemics
        results = []
        for bbox, conf, cid in raw_detections:
            cname = COCO_CLASSES[cid] if 0 <= cid < len(COCO_CLASSES) else f"class_{cid}"
            x1, y1, x2, y2 = bbox
            cx = (x1 + x2) / 2.0
            azimuth = pixel_to_azimuth_deg(cx, w, self.ipm.fov_deg)

            # Ước lượng khoảng cách từ điểm chân bbox qua IPM
            est_x, est_y = self.ipm.image_pixel_to_robot_metric(cx, y2)
            if est_x < 900.0:
                est_dist = round(math.hypot(est_x, est_y), 2)
            else:
                est_dist = round(220.0 / max(y2 - y1, 1), 2)

            margin, prox_env = compute_dynamic_proxemics(azimuth, cname, self.ipm.config.get("safety_margins_m"))

            det_obj = {
                "class_id": cid,
                "class_name": cname,
                "confidence": round(float(conf), 2),
                "azimuth_deg": azimuth,
                "estimated_dist": est_dist,
                "distance": est_dist,
                "safety_margin_m": margin,
                "bbox": [x1, y1, x2, y2]
            }
            if prox_env is not None:
                det_obj["proxemics"] = prox_env
            results.append(det_obj)

        return results

    def __del__(self):
        if hasattr(self, "cudart") and self.cudart is not None:
            try:
                if hasattr(self, "d_input") and self.d_input.value:
                    self.cudart.cudaFree(self.d_input)
                if hasattr(self, "d_output") and self.d_output.value:
                    self.cudart.cudaFree(self.d_output)
            except Exception:
                pass


# Alias tương thích ngược
YoloSegRunner = YoloDetector
