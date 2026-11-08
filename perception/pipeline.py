#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
End-to-End Computer Vision Pipeline (Perception & Floor Obstacle Compensation)
=============================================================================
CS532 - Advanced AI Robotics (Perception Engine - Hieu Me)
Chức năng chính:
  1. Đọc luồng camera độ trễ cực thấp (Zero-Buffer Grabber).
  2. Phát hiện đối tượng YOLO (người, ghế, đồ đạc) kèm góc phương vị Azimuth CCW & Social Bubble.
  3. Bù điểm mù LiDAR sát sàn (<16cm) bằng Floor IPM & phân đoạn hình thái học ROI.
  4. Đóng gói payload JSON theo chuẩn giao tiếp chung của đồ án (agents-doc/INTERFACE.md).
"""

import os
import sys
import time
import math
import json
import argparse
from typing import Dict, List, Tuple, Optional, Any, Union
import numpy as np
import cv2

# Đảm bảo import được các module core
_MODULE_DIR = os.path.dirname(os.path.abspath(__file__))
if _MODULE_DIR not in sys.path:
    sys.path.insert(0, _MODULE_DIR)

from core.frame_grabber import FrameGrabber
from core.floor_ipm import FloorIPMTransformer
from core.yolo_detector import YoloDetector
from core.proxemics import ProxemicsEngine, pixel_to_azimuth_deg, compute_dynamic_proxemics


class CVPipeline:
    """Pipeline thị giác máy tính toàn diện cho JetBot."""

    def __init__(
        self,
        source: Optional[Union[str, int]] = None,
        model_path: Optional[str] = None,
        config_path: Optional[str] = None,
        conf_threshold: float = 0.40,
        enable_grabber: bool = True,
    ):
        self.config_path = config_path or os.path.join(_MODULE_DIR, "config", "camera_config.json")
        self.ipm = FloorIPMTransformer(config_path=self.config_path)
        self.detector = YoloDetector(
            model_path=model_path,
            config_path=self.config_path,
            conf_threshold=conf_threshold
        )
        self.proxemics = ProxemicsEngine(
            safety_margins=self.ipm.config.get("safety_margins_m"),
            fov_deg=self.ipm.fov_deg
        )

        self.grabber: Optional[FrameGrabber] = None
        if enable_grabber:
            self.grabber = FrameGrabber(
                source=source or "auto",
                width=self.ipm.img_w,
                height=self.ipm.img_h,
                fps=20
            )

        self.frame_count = 0

    def process_frame(self, frame: np.ndarray) -> Dict[str, Any]:
        """
        Xử lý 1 khung hình BGR (640x480):
          - Chạy YOLO Detection (TensorRT / ONNX).
          - Trích xuất Low-lying Obstacles (<16cm) qua Floor IPM Homography.
          - Tạo danh sách Social Bubbles cho người đi bộ.
          - Đóng gói JSON dictionary chuẩn hóa.
        """
        t0 = time.perf_counter()
        h, w = frame.shape[:2]
        self.frame_count += 1

        # 1. Nhận diện đối tượng (YOLO Detection)
        detections = self.detector.detect(frame)

        # 2. Phân đoạn vật cản sàn (<16cm) bù điểm mù LiDAR
        roi_y_start = int(h * 0.70)
        roi = frame[roi_y_start:h, 0:w]
        gray_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray_roi, (5, 5), 0)
        _, binary_obs = cv2.threshold(blurred, 150, 255, cv2.THRESH_BINARY_INV)

        # Lọc nhiễu hình thái học Morphological Open
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        cleaned_obs = cv2.morphologyEx(binary_obs, cv2.MORPH_OPEN, kernel)

        full_obs_mask = np.zeros((h, w), dtype=np.uint8)
        full_obs_mask[roi_y_start:h, 0:w] = cleaned_obs

        # Loại trừ bounding box của các đối tượng lớn đã nhận diện (tránh nhân đôi chân người/ghế)
        for det in detections:
            bx1, by1, bx2, by2 = det.get("bbox", [0, 0, 0, 0])
            full_obs_mask[max(0, by1):min(h, by2), max(0, bx1):min(w, bx2)] = 0

        low_obstacles = self.ipm.extract_low_obstacles_from_mask(full_obs_mask)

        # 3. Trích xuất Social Bubbles (dành cho người để nạp vào Costmap)
        social_bubbles = []
        for det in detections:
            cname = det.get("class_name", "")
            if cname == "person":
                azimuth_deg = det.get("azimuth_deg", 0.0)
                dist_m = det.get("distance", det.get("estimated_dist", 1.0))
                margin_m = det.get("safety_margin_m", 0.80)

                # Chuyển từ (azimuth, dist) sang tọa độ tương đối robot (x_forward, y_left)
                az_rad = math.radians(azimuth_deg)
                # az_rad > 0 là bên trái (+y), az_rad < 0 là bên phải (-y)
                xr = round(dist_m * math.cos(az_rad), 3)
                yr = round(dist_m * math.sin(az_rad), 3)

                social_bubbles.append({
                    "x": xr,
                    "y": yr,
                    "radius_m": margin_m,
                    "azimuth_deg": azimuth_deg,
                    "distance_m": dist_m,
                })

        latency_ms = round((time.perf_counter() - t0) * 1000.0, 2)

        payload = {
            "timestamp": round(time.time(), 3),
            "frame_id": self.frame_count,
            "detections": detections,
            "low_obstacles": low_obstacles[:8],
            "social_bubbles": social_bubbles,
            "latency_ms": latency_ms,
        }
        return payload

    def run_once(self) -> Optional[Dict[str, Any]]:
        """Đọc 1 khung hình từ grabber và thực thi pipeline."""
        if self.grabber is None:
            return None
        ret, frame = self.grabber.read()
        if not ret or frame is None:
            return None
        return self.process_frame(frame)

    def close(self):
        if self.grabber is not None:
            self.grabber.release()


def main():
    parser = argparse.ArgumentParser(description="CS532 Computer Vision Pipeline")
    parser.add_argument("-s", "--source", default="auto", help="Nguồn video: auto, webcam, http://.../video_feed, hoặc path ảnh/video")
    parser.add_argument("-c", "--conf", type=float, default=0.40, help="Ngưỡng tin cậy YOLO (mặc định: 0.40)")
    parser.add_argument("--no-display", action="store_true", help="Không mở cửa sổ GUI")
    args = parser.parse_args()

    print("=" * 68)
    print("🤖 CS532 COMPUTER VISION PIPELINE — MODULAR RUNNER")
    print(f"   Source: {args.source} | Conf: {args.conf}")
    print("=" * 68)

    pipeline = CVPipeline(source=args.source, conf_threshold=args.conf, enable_grabber=True)

    try:
        while True:
            ret, frame = pipeline.grabber.read()
            if not ret or frame is None:
                time.sleep(0.02)
                continue

            payload = pipeline.process_frame(frame)
            dets = payload["detections"]
            lows = payload["low_obstacles"]
            lat = payload["latency_ms"]

            status = f"Frame {payload['frame_id']} | Dets: {len(dets)} | Low Obs: {len(lows)} | Latency: {lat}ms"
            print(f"\r{status}", end="", flush=True)

            if not args.no_display:
                # Vẽ visualization overlay
                disp = frame.copy()
                for d in dets:
                    b = d["bbox"]
                    cname = d["class_name"]
                    conf = d["confidence"]
                    dist = d["distance"]
                    az = d["azimuth_deg"]
                    cv2.rectangle(disp, (b[0], b[1]), (b[2], b[3]), (0, 255, 0), 2)
                    label = f"{cname} {conf:.2f} ({dist}m, {az}°)"
                    cv2.putText(disp, label, (b[0], max(20, b[1] - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)

                cv2.imshow("CS532 CV Pipeline HUD", disp)
                if cv2.waitKey(1) & 0xFF in (27, ord('q')):
                    break

    except KeyboardInterrupt:
        print("\nĐã nhận tín hiệu dừng từ người dùng.")
    finally:
        pipeline.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
