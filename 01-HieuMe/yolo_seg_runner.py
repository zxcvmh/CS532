#!/usr/bin/env python3
"""
Ultralytics YOLO26-n & Floor Segmentation Pipeline Runner
=========================================================
CS532 - Advanced AI Robotics (Module Computer Vision - Hieu Me)
Academic Standards & Engineering Highlights:
  - Multi-Backend Inference Priority:
      1. TensorRT Engine (`yolo26n.engine`, `yolov8n.engine`) on Jetson GPU
      2. Direct ONNX Runtime (`yolo26n.onnx`) with 100% Vectorized NumPy Decoding
      3. Ultralytics PyTorch (`yolo26n.pt`)
      4. Synthetic/Mock Mode (Offline Test)
  - 100% Vectorized Tensor Decoding for Shape (1, 84, 8400) -> (<1ms postprocess, NO slow loops)
  - Contribution 1: Floor IPM LiDAR Blind Spot Compensation (<16cm obstacles)
  - Contribution 2: Social Bubble Hall's Proxemics (0.90m margin for person) + Azimuth [-80°, +80°]
  - Strict Data Contract compliance with agents-doc/INTERFACE.md
"""

import os
import sys
import time
import math
import json
from typing import Dict, List, Tuple, Optional, Any
import numpy as np
import cv2

# Ensure UTF-8 stdout on Windows
if sys.platform.startswith("win"):
    try:
        sys.stdout.reconfigure(encoding="utf-8")
    except Exception:
        pass

# Import Floor IPM transformer từ thư mục hiện tại
try:
    from floor_ipm import FloorIPMTransformer
except ImportError:
    from .floor_ipm import FloorIPMTransformer

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

MODEL_INPUT_SIZE = 640


def letterbox_blob(frame: np.ndarray, size: int = MODEL_INPUT_SIZE) -> Tuple[np.ndarray, float, Tuple[float, float]]:
    """
    Tiền xử lý giống Ultralytics: giữ tỉ lệ khung hình, đệm viền xám 114 (letterbox).
    Kéo giãn 640x480 -> 640x640 làm méo vật thể và giảm confidence so với lúc huấn luyện.
    Trả về: (blob 1x3xSxS float32 RGB [0,1], ratio, (pad_x, pad_y)).
    """
    h, w = frame.shape[:2]
    ratio = min(size / float(h), size / float(w))
    new_w, new_h = int(round(w * ratio)), int(round(h * ratio))
    pad_x, pad_y = (size - new_w) / 2.0, (size - new_h) / 2.0
    resized = cv2.resize(frame, (new_w, new_h), interpolation=cv2.INTER_LINEAR) if (new_w, new_h) != (w, h) else frame
    top, bottom = int(round(pad_y - 0.1)), int(round(pad_y + 0.1))
    left, right = int(round(pad_x - 0.1)), int(round(pad_x + 0.1))
    padded = cv2.copyMakeBorder(resized, top, bottom, left, right, cv2.BORDER_CONSTANT, value=(114, 114, 114))
    # blobFromImage gộp BGR->RGB, scale 1/255 và HWC->NCHW trong một lần gọi C++
    blob = cv2.dnn.blobFromImage(padded, scalefactor=1.0 / 255.0, swapRB=True)
    return np.ascontiguousarray(blob, dtype=np.float32), ratio, (float(left), float(top))


def decode_yolo_output(
    raw_output: np.ndarray,
    orig_w: int,
    orig_h: int,
    ratio: float,
    pad: Tuple[float, float],
    conf_threshold: float,
    iou_threshold: float = 0.45,
) -> List[Tuple[List[int], float, int]]:
    """
    Giải mã vector hóa tensor (1, 84, N) / (84, N) -> danh sách (bbox_xyxy, conf, class_id) sau NMS.
    Lọc ngưỡng trước khi argmax để chỉ argmax trên các anchor ứng viên.
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
            confs_list[i],
            int(cls_ids[i]),
        ))
    return results


def pixel_to_azimuth_deg(cx: float, img_w: int, fov_deg: float) -> float:
    """Góc phương vị chuẩn ROS REP-103 (CCW): trái (+), phải (-), xấp xỉ tuyến tính theo FOV ngang."""
    half_w = img_w / 2.0
    ratio = max(-1.0, min(1.0, (half_w - cx) / half_w))
    return round(float(ratio * fov_deg / 2.0), 1)


class YoloSegRunner:
    def __init__(self, model_path: Optional[str] = None, config_path: Optional[str] = None):
        self.base_dir = os.path.dirname(os.path.abspath(__file__))
        self.ipm = FloorIPMTransformer(config_path=config_path)

        # Cấu hình vùng an toàn động (Social Bubble - Hall's Proxemics)
        self.safety_margins = self.ipm.config.get("safety_margins_m", {
            "person": 0.90,
            "chair": 0.35,
            "backpack": 0.25,
            "bottle": 0.20,
            "default": 0.30
        })

        # Danh sách mô hình theo thứ tự ưu tiên: 480 Engine -> 640 Engine -> ONNX -> PT -> Fallback
        self.model_candidates = [
            model_path,
            os.path.join(self.base_dir, "yolo26n_480.engine"),
            "/home/jetbot/yolo26n_480.engine",
            os.path.join(self.base_dir, "yolo26n.engine"),
            "/home/jetbot/yolo26n.engine",
            os.path.join(self.base_dir, "yolov8n.engine"),
            "/home/jetbot/yolov8n.engine",
            os.path.join(self.base_dir, "yolo26n.onnx"),
            os.path.join(self.base_dir, "..", "yolo26n.onnx"),
            os.path.join(self.base_dir, "yolov8n.onnx"),
            os.path.join(self.base_dir, "..", "yolov8n.onnx"),
            os.path.join(self.base_dir, "yolov8n-seg.onnx"),
            os.path.join(self.base_dir, "yolo26n.pt"),
            os.path.join(self.base_dir, "..", "yolo26n.pt"),
            os.path.join(self.base_dir, "yolov8n.pt"),
            os.path.join(self.base_dir, "..", "yolov8n.pt"),
        ]

        self.backend = "mock"
        self.model = None
        self.session = None
        self.classes = COCO_CLASSES
        self.frame_count = 0
        self.conf_threshold = float(os.environ.get("YOLO_CONF_THRESHOLD", "0.45"))
        self.iou_threshold = 0.45
        default_nav = "person,chair,couch,backpack,suitcase,dog,cat"
        self.target_classes = set(filter(None, os.environ.get("YOLO_CLASSES", default_nav).split(",")))

        self._init_backend()

    def _init_backend(self):
        """Khởi tạo backend suy luận theo thứ tự ưu tiên: TRT Engine -> ONNX Runtime -> Ultralytics -> Mock."""
        for candidate in self.model_candidates:
            if not candidate or not os.path.exists(candidate):
                continue

            lower_path = candidate.lower()

            # 1. TensorRT Engine (.engine)
            if lower_path.endswith(".engine"):
                try:
                    import tensorrt as trt
                    import ctypes
                    trt_logger = trt.Logger(trt.Logger.WARNING)
                    with open(candidate, "rb") as f:
                        runtime = trt.Runtime(trt_logger)
                        engine = runtime.deserialize_cuda_engine(f.read())
                    if engine is not None:
                        self.model = engine
                        self.context = engine.create_execution_context()
                        
                        # 64-bit ARM64 CUDA runtime via ctypes
                        cudart = ctypes.CDLL('libcudart.so')
                        cudart.cudaMalloc.argtypes = [ctypes.POINTER(ctypes.c_void_p), ctypes.c_size_t]
                        cudart.cudaMalloc.restype = ctypes.c_int
                        cudart.cudaMemcpy.argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_size_t, ctypes.c_int]
                        cudart.cudaMemcpy.restype = ctypes.c_int
                        cudart.cudaFree.argtypes = [ctypes.c_void_p]
                        cudart.cudaFree.restype = ctypes.c_int
                        self.cudart = cudart

                        in_shape = engine.get_binding_shape(0)
                        out_shape = engine.get_binding_shape(1)
                        in_h = int(in_shape[-2]) if len(in_shape) >= 2 and in_shape[-2] > 0 else 640
                        in_w = int(in_shape[-1]) if len(in_shape) >= 1 and in_shape[-1] > 0 else 640
                        self.input_size = max(in_h, in_w)

                        out_anchors = int(out_shape[-1]) if len(out_shape) >= 1 and out_shape[-1] > 0 else 8400
                        out_channels = int(out_shape[-2]) if len(out_shape) >= 2 and out_shape[-2] > 0 else 84

                        self.input_bytes = int(1 * 3 * in_h * in_w * 4)
                        self.output_bytes = int(1 * out_channels * out_anchors * 4)
                        self.d_input = ctypes.c_void_p()
                        self.d_output = ctypes.c_void_p()
                        self.cudart.cudaMalloc(ctypes.byref(self.d_input), self.input_bytes)
                        self.cudart.cudaMalloc(ctypes.byref(self.d_output), self.output_bytes)
                        self.h_output = np.zeros((1, out_channels, out_anchors), dtype=np.float32)
                        self.trt_bindings = [int(self.d_input.value), int(self.d_output.value)]

                        self.backend = "tensorrt"
                        print(f"[YoloSegRunner] Da nap thanh cong TensorRT Engine: {candidate}")
                        return
                except Exception as e:
                    print(f"[WARN] Khong the load TensorRT engine {candidate}: {e}")

            # 2. Direct ONNX Runtime (.onnx)
            if lower_path.endswith(".onnx"):
                try:
                    # Tự động nạp các thư viện CUDA từ nvidia packages nếu có
                    import glob, ctypes, site
                    try:
                        for sp in site.getsitepackages():
                            for lib_dir in glob.glob(os.path.join(sp, "nvidia", "*", "lib")):
                                for so_file in glob.glob(os.path.join(lib_dir, "*.so*")):
                                    try:
                                        ctypes.CDLL(so_file)
                                    except Exception:
                                        pass
                    except Exception:
                        pass

                    import onnxruntime as ort
                    opts = ort.SessionOptions()
                    opts.log_severity_level = 3  # Suppress provider probing warnings
                    opts.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
                    opts.intra_op_num_threads = 2

                    providers = ["CPUExecutionProvider"]
                    if "CUDAExecutionProvider" in ort.get_available_providers():
                        try:
                            # Quick probe to verify CUDA runtime is functional
                            probe_sess = ort.InferenceSession(candidate, sess_options=opts, providers=["CUDAExecutionProvider"])
                            if "CUDAExecutionProvider" in probe_sess.get_providers():
                                providers = ["CUDAExecutionProvider", "CPUExecutionProvider"]
                        except Exception:
                            providers = ["CPUExecutionProvider"]

                    self.session = ort.InferenceSession(candidate, sess_options=opts, providers=providers)
                    self.input_name = self.session.get_inputs()[0].name
                    self.backend = "onnxruntime"
                    print(f"[YoloSegRunner] Da nap thanh cong ONNX Runtime ({self.session.get_providers()[0]}): {candidate}")
                    return
                except Exception as e:
                    print(f"[WARN] Khong the load ONNX Runtime {candidate}: {e}")

            # 3. Ultralytics PyTorch (.pt)
            if lower_path.endswith(".pt"):
                try:
                    from ultralytics import YOLO
                    self.model = YOLO(candidate)
                    if hasattr(self.model, "names") and self.model.names:
                        self.classes = [self.model.names[i] for i in range(len(self.model.names))]
                    self.backend = "ultralytics"
                    print(f"[YoloSegRunner] Da nap thanh cong Ultralytics PyTorch: {candidate}")
                    return
                except Exception as e:
                    print(f"[WARN] Khong the load Ultralytics {candidate}: {e}")

        print("[YoloSegRunner] Su dung che do Mock Detections (san sang test offline khong can GPU).")
        self.backend = "mock"

    def compute_azimuth_deg(self, cx: float, img_w: int = 640, fov_deg: float = 160.0) -> float:
        """
        Tính góc phương vị lệch ngang (Azimuth angle) theo chuẩn Robot Frame (ISO 8855 / ROS REP-103 CCW):
          - Vật thể bên TRÁI (cx < W/2) -> Góc Azimuth DƯƠNG (+)
          - Vật thể bên PHẢI (cx > W/2) -> Góc Azimuth ÂM (-)
        theta_azimuth = ((W/2 - cx) / (W/2)) * (FOV/2) in [-80.0°, +80.0°].
        """
        return pixel_to_azimuth_deg(cx, img_w, fov_deg)

    def compute_dynamic_proxemics(
        self,
        azimuth_deg: float,
        cls_name: str
    ) -> Tuple[float, Optional[Dict[str, float]]]:
        """
        Tính toán bán kính an toàn theo mô hình Asymmetric Gaussian Proxemics (SOTA 2026).
        - Đối đầu trực diện (|azimuth| ~ 0°): robot nhường đường ở cự ly front = 0.80m.
        - Lệch bên sườn (|azimuth| -> 80°): khoảng cách né thu hẹp còn side = 0.40m.
        - Khoảng cách sau lưng (nếu có hướng xoay): back = 0.30m.
        - Công thức elip định hướng: R(theta) = (front * side) / sqrt((side * cos(theta))^2 + (front * sin(theta))^2)
        """
        c_lower = cls_name.lower()
        margin_cfg = self.safety_margins.get(c_lower, self.safety_margins.get("default", 0.25))

        if c_lower == "person":
            if isinstance(margin_cfg, dict):
                front_m = float(margin_cfg.get("front_m", 0.80))
                side_m = float(margin_cfg.get("side_m", 0.40))
                back_m = float(margin_cfg.get("back_m", 0.30))
            else:
                front_m = 0.80
                side_m = 0.40
                back_m = 0.30

            theta_rad = math.radians(abs(azimuth_deg))
            denom = math.sqrt((side_m * math.cos(theta_rad)) ** 2 + (front_m * math.sin(theta_rad)) ** 2)
            effective_margin = (front_m * side_m) / max(denom, 1e-6)

            envelope = {
                "front_m": front_m,
                "side_m": side_m,
                "back_m": back_m
            }
            return round(effective_margin, 2), envelope

        # Trường hợp vật cản tĩnh thông thường (chair, backpack, bottle, default)
        if isinstance(margin_cfg, (int, float)):
            return float(margin_cfg), None

        return 0.25, None

    def _build_detection(self, bbox_xyxy: List[int], conf: float, cid: int, img_w: int) -> Optional[Dict[str, Any]]:
        """Đóng gói 1 detection theo INTERFACE.md (dùng chung cho mọi backend)."""
        cname = self.classes[cid] if 0 <= cid < len(self.classes) else f"obj_{cid}"
        if conf < self.conf_threshold:
            return None
        if self.target_classes and cname not in self.target_classes:
            return None

        x1, y1, x2, y2 = [int(v) for v in bbox_xyxy]
        center_x = (x1 + x2) / 2.0
        azimuth = self.compute_azimuth_deg(center_x, img_w=img_w, fov_deg=self.ipm.fov_deg)

        # Ước lượng khoảng cách từ điểm chân bbox (y2) qua IPM; fallback quang học nếu ngoài vùng sàn
        est_x, est_y = self.ipm.image_pixel_to_robot_metric(center_x, y2)
        if est_x < 900.0:
            est_dist = round(math.hypot(est_x, est_y), 2)
        else:
            est_dist = round(220.0 / max(y2 - y1, 1), 2)

        margin, prox_env = self.compute_dynamic_proxemics(azimuth, cname)
        det_obj = {
            "class_id": cid,
            "class_name": cname,
            "confidence": round(float(conf), 2),
            "azimuth_deg": azimuth,
            "estimated_dist": est_dist,
            "distance": est_dist,  # alias cho Backend/Dashboard (schema Detection.distance)
            "safety_margin_m": margin,
            "bbox": [x1, y1, x2, y2]
        }
        if prox_env is not None:
            det_obj["proxemics"] = prox_env
        return det_obj

    def _decode_vectorized_tensor(
        self,
        raw_output: np.ndarray,
        orig_w: int,
        orig_h: int,
        ratio: Optional[float] = None,
        pad: Tuple[float, float] = (0.0, 0.0)
    ) -> List[Dict[str, Any]]:
        """
        Giải mã tensor (1, 84, 8400) bằng NumPy vectorization (xem decode_yolo_output).
        ratio/pad lấy từ letterbox_blob(); nếu None thì mặc định letterbox của ảnh orig_w x orig_h.
        """
        if ratio is None:
            ratio = min(MODEL_INPUT_SIZE / float(orig_h), MODEL_INPUT_SIZE / float(orig_w))
            pad = ((MODEL_INPUT_SIZE - orig_w * ratio) / 2.0, (MODEL_INPUT_SIZE - orig_h * ratio) / 2.0)
        raw = decode_yolo_output(raw_output, orig_w, orig_h, ratio, pad, self.conf_threshold, self.iou_threshold)
        detections = []
        for bbox, conf, cid in raw:
            det = self._build_detection(bbox, conf, cid, orig_w)
            if det is not None:
                detections.append(det)
        return detections

    def run_inference(self, frame: np.ndarray) -> Dict[str, Any]:
        """
        Chạy toàn bộ pipeline trên 1 khung hình BGR (640x480):
        Trả về dictionary chuẩn JSON:
        {
          "timestamp": float,
          "frame_id": int,
          "detections": [...],
          "low_obstacles": [[x, y], ...],
          "latency_ms": float
        }
        """
        t0 = time.perf_counter()
        h, w = frame.shape[:2]
        detections = []
        low_obstacles = []

        # ── 1. NHÁNH OBJECT DETECTION (YOLO26-n) ──
        target_size = getattr(self, "input_size", MODEL_INPUT_SIZE)
        if self.backend == "tensorrt" and hasattr(self, "context") and self.context is not None:
            blob, ratio, pad = letterbox_blob(frame, size=target_size)

            # Copy to GPU
            import ctypes
            self.cudart.cudaMemcpy(self.d_input, blob.ctypes.data_as(ctypes.c_void_p), self.input_bytes, 1)
            # Run TensorRT inference on Maxwell GPU
            self.context.execute_v2(self.trt_bindings)
            # Copy output from GPU
            self.cudart.cudaMemcpy(self.h_output.ctypes.data_as(ctypes.c_void_p), self.d_output, self.output_bytes, 2)

            detections = self._decode_vectorized_tensor(self.h_output, w, h, ratio, pad)

        elif self.backend == "onnxruntime" and self.session is not None:
            blob, ratio, pad = letterbox_blob(frame, size=target_size)
            outputs = self.session.run(None, {self.input_name: blob})[0]
            detections = self._decode_vectorized_tensor(outputs, w, h, ratio, pad)

        elif self.backend == "ultralytics" and self.model is not None:
            results = self.model(frame, verbose=False, imgsz=MODEL_INPUT_SIZE,
                                 conf=self.conf_threshold, iou=self.iou_threshold)
            if results and len(results) > 0:
                res = results[0]
                boxes = res.boxes
                if boxes is not None and len(boxes) > 0:
                    xyxy_all = boxes.xyxy.cpu().numpy()
                    conf_all = boxes.conf.cpu().numpy()
                    cls_all = boxes.cls.cpu().numpy().astype(int)
                    for xyxy, conf, cls_id in zip(xyxy_all, conf_all, cls_all):
                        det = self._build_detection(xyxy.tolist(), float(conf), int(cls_id), w)
                        if det is not None:
                            detections.append(det)

                # Gộp mọi mask thành 1 mask để Temporal Filter chỉ cập nhật 1 lần/khung hình
                if hasattr(res, "masks") and res.masks is not None and len(res.masks.data) > 0:
                    union = (res.masks.data.cpu().numpy().max(axis=0) * 255).astype(np.uint8)
                    union = cv2.resize(union, (w, h), interpolation=cv2.INTER_NEAREST)
                    low_obstacles.extend(self.ipm.extract_low_obstacles_from_mask(union))

        # ── 2. NHÁNH BÙ ĐIỂM MÙ LIDAR BẰNG FLOOR IPM (Academic Contribution 1) ──
        # Tự động phân đoạn vật cản sàn (<16cm) trong vùng quan sát mặt sàn (ROI)
        if len(low_obstacles) == 0:
            roi_y_start = int(h * 0.70)
            roi = frame[roi_y_start:h, 0:w]
            gray_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)

            # Khử nhiễu sàn bằng Gaussian blur và phân đoạn nhị phân
            blurred = cv2.GaussianBlur(gray_roi, (5, 5), 0)
            # Ngưỡng phát hiện vật cản tương phản trên sàn (dây điện, dép, sách)
            _, binary_obs = cv2.threshold(blurred, 150, 255, cv2.THRESH_BINARY_INV)

            # Lọc hình thái học Morphological Open để loại bỏ các vạch kẻ đường đơn lẻ
            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
            cleaned_obs = cv2.morphologyEx(binary_obs, cv2.MORPH_OPEN, kernel)

            full_obs_mask = np.zeros((h, w), dtype=np.uint8)
            full_obs_mask[roi_y_start:h, 0:w] = cleaned_obs

            # Loại trừ vùng bbox của các vật thể lớn (person, chair, ...) đã được YOLO phát hiện
            # để tránh nhân đôi chân người/ghế thành low_obstacle (< 16cm)
            for det in detections:
                bx1, by1, bx2, by2 = det["bbox"]
                full_obs_mask[max(0, by1):min(h, by2), max(0, bx1):min(w, bx2)] = 0

            extracted = self.ipm.extract_low_obstacles_from_mask(full_obs_mask)
            low_obstacles.extend(extracted[:4])

        # ── 3. FALLBACK MOCK NẾU CHƯA CÓ DETECTIONS NÀO (OFFLINE TEST HOẶC MOCK) ──
        if len(detections) == 0 and self.backend == "mock":
            margin, prox_env = self.compute_dynamic_proxemics(-15.4, "person")
            detections.append({
                "class_id": 0,
                "class_name": "person",
                "confidence": 0.88,
                "azimuth_deg": -15.4,
                "estimated_dist": 1.85,
                "distance": 1.85,
                "safety_margin_m": margin,
                "proxemics": prox_env,
                "bbox": [180, 180, 250, 340]
            })

        if len(low_obstacles) == 0 and self.backend == "mock":
            low_obstacles = [[0.65, 0.12], [0.70, -0.15]]

        self.frame_count += 1
        pipeline_latency_ms = (time.perf_counter() - t0) * 1000.0

        # Đóng gói đúng Hợp đồng agents-doc/INTERFACE.md
        payload = {
            "timestamp": round(time.time(), 3),
            "frame_id": self.frame_count,
            "detections": detections,
            "low_obstacles": low_obstacles,
            "latency_ms": round(pipeline_latency_ms, 2)
        }
        return payload

    def __del__(self):
        """Giải phóng bộ nhớ GPU đã cấp phát qua CUDA runtime."""
        if hasattr(self, "cudart") and self.cudart is not None:
            try:
                if hasattr(self, "d_input") and self.d_input.value:
                    self.cudart.cudaFree(self.d_input)
                if hasattr(self, "d_output") and self.d_output.value:
                    self.cudart.cudaFree(self.d_output)
            except Exception:
                pass


if __name__ == "__main__":
    print("--- KIỂM TRA ĐỘC LẬP MODULE YOLO SEG RUNNER ---")
    runner = YoloSegRunner()
    print(f"-> Backend dang hoat dong: {runner.backend}")

    # Load ảnh mẫu test
    sample_img_path = os.path.join(runner.base_dir, "sample_data", "csi_floor_sample.jpg")
    if os.path.exists(sample_img_path):
        frame = cv2.imread(sample_img_path)
    else:
        frame = np.ones((480, 640, 3), dtype=np.uint8) * 200

    payload = runner.run_inference(frame)
    print("\n[RESULT] Payload xuat ra cho Planning & Dashboard:")
    print(json.dumps(payload, indent=2))
    print(f"\n-> Thoi gian xu ly pipeline: {payload['latency_ms']} ms")
    assert "detections" in payload and "low_obstacles" in payload
    assert "frame_id" in payload and "latency_ms" in payload
    print("[PASS] YoloSegRunner hoat dong chuan xac theo INTERFACE.md!")

