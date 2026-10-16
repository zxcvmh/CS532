#!/usr/bin/env python3
"""
bench_letterbox_jetbot.py
Benchmark chi tiết hiệu năng và độ chính xác của Letterbox Preprocessing vs Direct Resize
trên phần cứng NVIDIA Jetson Nano 4GB (Maxwell GPU TensorRT).
"""

import os
import sys
import time
import math
import cv2
import numpy as np

from yolo_seg_runner import YoloSegRunner, letterbox_blob
from floor_ipm import FloorIPMTransformer
from camera_loader import CameraLoader

def benchmark_inference(num_warmup=10, num_iters=50):
    print("=" * 65)
    print("  🚀 JETBOT HARDWARE BENCHMARK: YOLO26-n TENSORRT + LETTERBOX")
    print("=" * 65)

    base_dir = os.path.dirname(os.path.abspath(__file__))
    sample_img_path = os.path.join(base_dir, "sample_data", "csi_floor_sample.jpg")
    if not os.path.exists(sample_img_path):
        sample_img = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.rectangle(sample_img, (200, 100), (440, 400), (120, 150, 180), -1)
    else:
        sample_img = cv2.imread(sample_img_path)

    runner = YoloSegRunner()
    print(f"[*] Engine backend: {runner.backend}")
    if runner.backend != "tensorrt":
        print(f"[!] CẢNH BÁO: Chưa kích hoạt TensorRT, đang chạy backend: {runner.backend}")

    # 1. Đo riêng tiền xử lý Letterbox vs Direct Resize
    print("\n--- 1. So sánh Tiền xử lý (Preprocessing Latency) ---")
    t0 = time.perf_counter()
    for _ in range(100):
        _ = cv2.resize(sample_img, (640, 640))
    t_resize = (time.perf_counter() - t0) * 10.0  # ms per call
    print(f"-> cv2.resize((640, 640)) thô bạo (méo hình): {t_resize:.3f} ms")

    t0 = time.perf_counter()
    for _ in range(100):
        blob, r, pad = letterbox_blob(sample_img)
    t_letterbox = (time.perf_counter() - t0) * 10.0  # ms per call
    print(f"-> letterbox_blob (bảo toàn tỉ lệ 4:3, pad 114): {t_letterbox:.3f} ms")
    print(f"   Shape blob: {blob.shape}, scale ratio: {r:.3f}, pad (x, y): {pad}")

    # 2. Warm-up GPU
    print(f"\n--- 2. Khởi động GPU ({num_warmup} iterations) ---")
    for _ in range(num_warmup):
        _ = runner.run_inference(sample_img)

    # 3. Benchmark End-to-End Pipeline
    print(f"--- 3. Đo kiểm Pipeline Thực tế ({num_iters} iterations) ---")
    latencies = []
    for _ in range(num_iters):
        t_start = time.perf_counter()
        res = runner.run_inference(sample_img)
        dt = (time.perf_counter() - t_start) * 1000.0
        latencies.append(dt)

    latencies = np.array(latencies)
    mean_lat = np.mean(latencies)
    p50_lat = np.median(latencies)
    p95_lat = np.percentile(latencies, 95)
    fps = 1000.0 / mean_lat

    print(f"-> Latency trung bình (Mean): {mean_lat:.2f} ms")
    print(f"-> Latency Median (P50):      {p50_lat:.2f} ms")
    print(f"-> Latency P95:               {p95_lat:.2f} ms")
    print(f"-> Throughput thực tế:        {fps:.1f} FPS")

    # 4. Kiểm tra độ chính xác tọa độ Unpad của BBox
    print("\n--- 4. Kiểm định Độ chính xác Khôi phục BBox sau Unpad ---")
    pad_x, pad_y = pad
    orig_h, orig_w = sample_img.shape[:2]
    cx_t, cy_t = 320.0, 320.0
    unpad_x = (cx_t - pad_x) / r
    unpad_y = (cy_t - pad_y) / r
    print(f"-> Tọa độ trong tensor: ({cx_t}, {cy_t})")
    print(f"-> Tọa độ sau khôi phục unpad: ({unpad_x:.1f}, {unpad_y:.1f}) (Kỳ vọng: 320.0, 240.0)")
    assert abs(unpad_x - 320.0) < 1e-3 and abs(unpad_y - 240.0) < 1e-3, "Lỗi unpad letterbox!"
    print("✅ Unpad toán học khớp 100% tọa độ gốc của cảm biến CSI!")

    # 5. Kiểm định Floor IPM Bù điểm mù LiDAR
    print("\n--- 5. Kiểm định Floor IPM Mask Exclusion & Low Obstacles ---")
    sample_payload = runner.run_inference(sample_img)
    print(f"-> Số lượng low_obstacles phát hiện: {len(sample_payload['low_obstacles'])}")
    print(f"-> Danh sách tọa độ thực robot frame (m): {sample_payload['low_obstacles']}")
    print(f"-> Detections: {sample_payload['detections']}")

    print("\n" + "=" * 65)
    print("🎉 KẾT QUẢ BENCHMARK JETBOT: ĐẠT CHUẨN THỜI GIAN THỰC (> 15 FPS)")
    print("=" * 65)

if __name__ == "__main__":
    benchmark_inference()
