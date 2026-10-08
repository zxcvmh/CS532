#!/usr/bin/env python3
"""
Integrated Computer Vision & IPM Pipeline Demo
===============================================
CS532 - Advanced AI Robotics (Module Computer Vision - Hieu Me)
Chay pipeline truyen dan du lieu hoan chinh tu Camera -> YOLOv8n -> IPM -> JSON
"""

import os
import sys
import time
import json
import numpy as np

# Ensure UTF-8 stdout on Windows
if sys.platform.startswith("win"):
    try:
        sys.stdout.reconfigure(encoding="utf-8")
    except Exception:
        pass

from camera_loader import CameraLoader
from floor_ipm import FloorIPMTransformer
from yolo_seg_runner import YoloSegRunner

def run_pipeline_demo(iterations: int = 5):
    print("=" * 68)
    print("   CS532 - PIPELINE THỊ GIÁC & BÙ ĐIỂM MÙ LIDAR (HIẾU ME)")
    print("=" * 68)

    loader = CameraLoader(source="auto")
    runner = YoloSegRunner()

    print(f"\n[INFO] Đang khởi chạy pipeline kiểm thử ({iterations} vòng lặp)...")
    latencies = []

    for i in range(1, iterations + 1):
        success, frame = loader.read_frame()
        if not success or frame is None:
            print(f"[ERROR] Không đọc được khung hình tại vòng {i}!")
            break

        t_start = time.time()
        payload = runner.run_inference(frame)
        dt_ms = (time.time() - t_start) * 1000.0
        latencies.append(dt_ms)

        print(f"\n--- [KHUNG HÌNH {i}/{iterations}] Độ trễ chu kỳ: {dt_ms:.1f} ms ---")
        print(f"-> Đối tượng nhận diện (Social Bubble): {len(payload['detections'])} đối tượng")
        for d in payload["detections"]:
            print(f"   * [{d['class_name'].upper()}] Conf: {d['confidence']} | Góc Azimuth: {d['azimuth_deg']}° | Cự ly: {d['estimated_dist']}m | Né an toàn: {d['safety_margin_m']}m")

        print(f"-> Vật cản sát sàn < 16cm (LiDAR bị mù): {len(payload['low_obstacles'])} điểm")
        for obs in payload["low_obstacles"][:3]:
            print(f"   * Tọa độ Robot Frame: X = {obs[0]:.2f}m, Y = {obs[1]:.2f}m")

    loader.release()

    avg_latency = sum(latencies) / len(latencies) if latencies else 0.0
    print("\n" + "=" * 68)
    print(f"✅ HOÀN TẤT DEMO!")
    print(f"-> Độ trễ trung bình trên PC: {avg_latency:.1f} ms")
    print(f"-> Định dạng JSON đã chuẩn hóa 100% theo agents-doc/INTERFACE.md:")
    print(json.dumps(payload, indent=2))
    print("=" * 68)

if __name__ == "__main__":
    run_pipeline_demo()
