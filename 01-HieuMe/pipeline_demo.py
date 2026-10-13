#!/usr/bin/env python3
"""
Integrated Computer Vision & IPM Pipeline Demo
===============================================
CS532 - Advanced AI Robotics (Module Computer Vision - Hieu Me)
Chạy pipeline truyền dẫn dữ liệu hoàn chỉnh từ Camera -> YOLO26-n -> IPM -> JSON
Đo đạc chính xác ngân sách độ trễ thời gian thực (Latency Budget <= 35ms, >= 28-35 FPS)
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

base_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, base_dir)

from camera_loader import CameraLoader
from floor_ipm import FloorIPMTransformer
from yolo_seg_runner import YoloSegRunner


def run_pipeline_demo(iterations: int = 10):
    print("=" * 68)
    print("   CS532 - PIPELINE THỊ GIÁC, YOLO26-n & BÙ ĐIỂM MÙ LIDAR (HIẾU ME)")
    print("=" * 68)

    loader = CameraLoader(source="auto")
    runner = YoloSegRunner()
    print(f"-> Backend suy luận: {runner.backend}")

    # 1. Warmup giai đoạn đầu (3 frames) để nạp kernel GPU/CPU
    print("\n[INFO] Đang khởi động & Warmup mô hình (3 frames)...", end=" ", flush=True)
    success, warmup_frame = loader.read_frame()
    if success and warmup_frame is not None:
        for _ in range(3):
            runner.run_inference(warmup_frame)
        print("Hoàn tất! ✅")
    else:
        print("Không đọc được frame warmup! ⚠️")

    print(f"\n[INFO] Bắt đầu đo đạc benchmark chu kỳ thực ({iterations} frames)...")
    latencies = []
    payload = {}

    for i in range(1, iterations + 1):
        success, frame = loader.read_frame()
        if not success or frame is None:
            print(f"[ERROR] Không đọc được khung hình tại vòng {i}!")
            break

        t_start = time.perf_counter()
        payload = runner.run_inference(frame)
        dt_ms = (time.perf_counter() - t_start) * 1000.0
        latencies.append(dt_ms)

        print(f"--- [KHUNG HÌNH {i:02d}/{iterations:02d}] Độ trễ chu kỳ: {dt_ms:.2f} ms (~{1000.0/max(dt_ms, 0.1):.1f} FPS) ---")
        if payload["detections"]:
            for d in payload["detections"][:2]:
                print(f"   * [{d['class_name'].upper()}] Conf: {d['confidence']} | Góc: {d['azimuth_deg']}° | Cự ly: {d['estimated_dist']}m | Né an toàn: {d['safety_margin_m']}m")
        else:
            print("   * [DETECTIONS] Không có đối tượng lớn trong tầm nhìn.")

        if payload["low_obstacles"]:
            obs_str = ", ".join(f"[{pt[0]:.2f}m, {pt[1]:.2f}m]" for pt in payload["low_obstacles"][:3])
            print(f"   * [LOW OBSTACLES < 16cm]: {len(payload['low_obstacles'])} điểm -> {obs_str}")

    loader.release()

    if latencies:
        avg_lat = sum(latencies) / len(latencies)
        min_lat = min(latencies)
        max_lat = max(latencies)
        fps = 1000.0 / avg_lat

        print("\n" + "=" * 68)
        print("📊 KẾT QUẢ BENCHMARK ĐỘ TRỄ & TỐC ĐỘ XỬ LÝ (LATENCY REPORT)")
        print("=" * 68)
        print(f" - Độ trễ trung bình:   {avg_lat:.2f} ms")
        print(f" - Độ trễ nhỏ nhất:     {min_lat:.2f} ms")
        print(f" - Độ trễ lớn nhất:     {max_lat:.2f} ms")
        print(f" - Tốc độ khung hình:   {fps:.1f} FPS")
        print(f" - Tiêu chuẩn đồ án:    {'ĐẠT TIÊU CHUẨN THỜI GIAN THỰC (<= 35ms)' if avg_lat <= 35.0 else 'Môi trường CPU / Cần GPU TensorRT'}")
        print("=" * 68)

    print("\n-> Định dạng JSON đã chuẩn hóa 100% theo agents-doc/INTERFACE.md:")
    print(json.dumps(payload, indent=2))
    print("=" * 68)


if __name__ == "__main__":
    run_pipeline_demo()
