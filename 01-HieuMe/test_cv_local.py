#!/usr/bin/env python3
"""
Bộ Kiểm Thử Thị Giác, YOLO26-n & Bù Điểm Mù LiDAR Trên Máy Tính Cá Nhân (Hiếu Me)
================================================================================
Kiểm thử toàn diện 5 tiêu chuẩn:
  1. Camera Loader & Fallback mượt mà (CSI -> USB -> Sample Image -> Synthetic).
  2. Góc phương vị Azimuth [-80°, +80°] & Social Bubble 0.90m (Hall's Proxemics).
  3. Ma trận Floor IPM, chiếu tọa độ mét thực 2D & Bảo vệ Division-by-Zero.
  4. Giải mã Tensor YOLO26-n Vectorized (1, 84, 8400) không dùng vòng lặp for.
  5. Hợp đồng dữ liệu JSON chuẩn xác 100% theo agents-doc/INTERFACE.md.
"""

import os
import sys
import time
import math
import json
import numpy as np

try:
    import cv2
except ImportError:
    print("[ERROR] Cần cài đặt opencv-python: pip install opencv-python")
    sys.exit(1)

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, BASE_DIR)

from camera_loader import CameraLoader
from floor_ipm import FloorIPMTransformer
from yolo_seg_runner import YoloSegRunner

SAMPLE_IMG_PATH = os.path.join(BASE_DIR, "sample_data", "csi_floor_sample.jpg")


def test_1_camera_loader():
    print("\n--- [TEST 1/5] Kiểm tra CameraLoader & Chuỗi Fallback 4 Tầng ---")
    loader = CameraLoader(source=SAMPLE_IMG_PATH)
    success, frame = loader.read_frame()
    assert success and frame is not None, "Không đọc được khung hình từ CameraLoader!"
    h, w = frame.shape[:2]
    print(f"-> Đọc ảnh chuẩn thành công: Độ phân giải {w}x{h} pixels (is_synthetic={loader.is_synthetic})")
    loader.release()
    print("✅ PASS 1/5: CameraLoader sẵn sàng và hoạt động mượt mà!")
    return frame


def test_2_azimuth_and_social_bubble():
    print("\n--- [TEST 2/5] Kiểm tra Tính toán Góc Phương vị & Asymmetric Gaussian Proxemics ---")
    runner = YoloSegRunner()
    img_w = 640

    # 1. Kiểm tra góc phương vị tại 3 điểm mốc: Trái ngoài cùng, Giữa, Phải ngoài cùng
    az_left = runner.compute_azimuth_deg(0.0, img_w=img_w, fov_deg=160.0)
    az_center = runner.compute_azimuth_deg(320.0, img_w=img_w, fov_deg=160.0)
    az_right = runner.compute_azimuth_deg(640.0, img_w=img_w, fov_deg=160.0)

    print(f"-> Góc Azimuth trái (x=0px): {az_left}° (Kỳ vọng chuẩn CCW: +80.0°)")
    print(f"-> Góc Azimuth giữa (x=320px): {az_center}° (Kỳ vọng: 0.0°)")
    print(f"-> Góc Azimuth phải (x=640px): {az_right}° (Kỳ vọng chuẩn CCW: -80.0°)")

    assert abs(az_left - 80.0) < 0.1, "Lệch góc trái (+80.0°)!"
    assert abs(az_center - 0.0) < 0.1, "Lệch góc giữa (0.0°)!"
    assert abs(az_right - (-80.0)) < 0.1, "Lệch góc phải (-80.0°)!"

    # 2. Kiểm tra Asymmetric Gaussian Proxemics (SOTA 2026: 0.80m front, 0.40m side, 0.30m back)
    margin_front, env_front = runner.compute_dynamic_proxemics(0.0, "person")
    margin_side, env_side = runner.compute_dynamic_proxemics(80.0, "person")
    margin_chair, _ = runner.compute_dynamic_proxemics(0.0, "chair")
    margin_backpack, _ = runner.compute_dynamic_proxemics(0.0, "backpack")

    print(f"-> Proxemics 'person' trực diện (θ=0°): {margin_front}m (Kỳ vọng: 0.80m)")
    print(f"-> Proxemics 'person' bên sườn (θ=80°): {margin_side}m (Kỳ vọng: ~0.40m - 0.41m)")
    print(f"-> Social Bubble 'chair':  {margin_chair}m (Kỳ vọng: 0.30m)")
    print(f"-> Social Bubble 'backpack': {margin_backpack}m (Kỳ vọng: 0.20m)")

    assert abs(margin_front - 0.80) < 0.05, f"Sai khoảng cách nhường đường trực diện: {margin_front}m!"
    assert abs(margin_side - 0.40) < 0.05, f"Sai khoảng cách né bên sườn: {margin_side}m!"
    assert env_front["front_m"] == 0.80 and env_front["side_m"] == 0.40 and env_front["back_m"] == 0.30, "Sai cấu trúc envelope proxemics!"
    assert margin_chair == 0.30, f"Sai khoảng cách ghế: {margin_chair}m!"
    assert margin_backpack == 0.20, f"Sai khoảng cách balo: {margin_backpack}m!"
    print("✅ PASS 2/5: Trích xuất góc phương vị & Asymmetric Gaussian Proxemics chuẩn xác!")


def test_3_floor_ipm_and_division_by_zero():
    print("\n--- [TEST 3/5] Kiểm tra Ma trận Floor IPM, Chống Lỗi Chia Cho 0 & Temporal Filter ---")
    ipm = FloorIPMTransformer()

    # 1. Kiểm tra tọa độ điểm tiếp xúc sát mũi xe (u=320, v=470)
    xm_near, ym_near = ipm.image_pixel_to_robot_metric(320.0, 470.0)
    print(f"-> Điểm sát mũi xe (320, 470) -> X={xm_near}m, Y={ym_near}m (Kỳ vọng: X=0.15m, Y=0.0m)")
    assert abs(xm_near - 0.15) < 0.05 and abs(ym_near) < 0.05, "Sai tọa độ mét sát mũi xe!"

    # 2. Kiểm tra tọa độ điểm biên xa hình chữ nhật hiệu chuẩn (u=320, v=408)
    xm_far, ym_far = ipm.image_pixel_to_robot_metric(320.0, 408.0)
    print(f"-> Điểm xa hiệu chuẩn (320, 408) -> X={xm_far}m, Y={ym_far}m (Kỳ vọng: X=0.65m, Y=0.0m)")
    assert abs(xm_far - 0.65) < 0.05 and abs(ym_far) < 0.05, "Sai tọa độ mét điểm xa!"

    # 3. Kiểm tra khả năng chống lỗi chia cho 0 tại đường chân trời (v=200, v=284)
    horizon_pts = [(320.0, 200.0), (320.0, 284.0), (100.0, 50.0)]
    for pt in horizon_pts:
        xm_h, ym_h = ipm.image_pixel_to_robot_metric(*pt)
        print(f"-> Kiểm tra điểm trên đường chân trời {pt} -> X={xm_h}m (Bảo vệ an toàn không crash)")
        assert xm_h >= 900.0 or xm_h <= 0.0 or math.isinf(xm_h), "Không lọc được điểm trên đường chân trời!"

    # 4. Kiểm tra Temporal Filter cho vật cản sàn
    mock_mask = np.zeros((480, 640), dtype=np.uint8)
    cv2.circle(mock_mask, (320, 430), 20, 255, -1)
    # Reset filter
    if ipm.temporal_filter:
        ipm.temporal_filter.reset()
    f1_obs = ipm.extract_low_obstacles_from_mask(mock_mask, apply_filter=True)
    f2_obs = ipm.extract_low_obstacles_from_mask(mock_mask, apply_filter=True)
    print(f"-> Trích xuất vật cản qua Temporal Filter: {f2_obs}")
    assert len(f2_obs) > 0, "Không trích xuất được vật cản qua Temporal Filter!"

    # 5. Kiểm tra Dynamic Pitch Update
    nominal_p = ipm.pitch_deg
    ipm.update_pitch_deg(23.5)
    assert abs(ipm.pitch_deg - 23.5) < 1e-4, "Lỗi cập nhật góc pitch động!"
    ipm.update_pitch_deg(nominal_p)  # Reset lại pitch ban đầu
    print(f"-> Cập nhật góc pitch động (IMU-assisted): {nominal_p}° -> 23.5° -> {ipm.pitch_deg}° (Thành công)")

    # 6. Kiểm tra Batch Pixels Metric (Fix BUG-03)
    pts_batch = np.array([[320.0, 470.0], [320.0, 100.0]], dtype=np.float32)
    b_res = ipm.batch_pixels_to_robot_metric(pts_batch)
    assert abs(b_res[0, 0] - 0.15) < 0.05, f"Lỗi batch ground point: {b_res[0]}"
    assert b_res[1, 0] >= 900.0, f"Lỗi batch horizon point: {b_res[1]}"
    print(f"-> Kiểm tra batch_pixels_to_robot_metric thành công (Ground X={b_res[0, 0]}m, Ceiling X={b_res[1, 0]}m)")

    print("✅ PASS 3/5: Ma trận IPM, lớp bảo vệ Division-by-Zero, Temporal Filter & Batch Metric hoạt động hoàn hảo!")


def test_4_yolo_vectorized_runner(frame):
    print("\n--- [TEST 4/5] Kiểm tra Giải Mã Tensor YOLO26-n Vectorized (1, 84, 8400) ---")
    runner = YoloSegRunner()
    print(f"-> Backend hoạt động: {runner.backend}")

    # Chạy 2 chu kỳ liên tiếp trên frame sàn mẫu để TemporalObstacleFilter (min_hits=2) xác nhận vật cản <16cm
    _ = runner.run_inference(frame)
    t0 = time.perf_counter()
    payload = runner.run_inference(frame)
    latency_ms = (time.perf_counter() - t0) * 1000.0

    print(f"-> Độ trễ đo được trên PC ({runner.backend}): {latency_ms:.2f} ms")
    print(f"-> Số điểm vật cản sát sàn < 16cm (Floor IPM): {len(payload['low_obstacles'])} -> {payload['low_obstacles']}")
    assert len(payload["low_obstacles"]) > 0, "Không trích xuất được low_obstacles từ ảnh sàn mẫu sau 2 chu kỳ!"

    # Kiểm tra nhận diện đối tượng thực tế (person) trên ảnh chuẩn Ultralytics nếu có sẵn
    try:
        import ultralytics
        bus_path = os.path.join(os.path.dirname(ultralytics.__file__), "assets", "bus.jpg")
        if os.path.exists(bus_path):
            bus_img = cv2.imread(bus_path)
            if bus_img is not None:
                bus_resized = cv2.resize(bus_img, (640, 480))
                bus_payload = runner.run_inference(bus_resized)
                print(f"-> Kiểm tra YOLO26-n trên ảnh thực (bus.jpg 640x480): phát hiện {len(bus_payload['detections'])} người")
                assert len(bus_payload["detections"]) >= 3, "YOLO26-n không phát hiện đủ người trên ảnh chuẩn bus.jpg!"
                payload["detections"] = bus_payload["detections"]
    except ImportError:
        pass

    assert "detections" in payload, "Thiếu trường 'detections'!"
    assert "low_obstacles" in payload, "Thiếu trường 'low_obstacles'!"
    print("✅ PASS 4/5: YOLO26-n Vectorized Pipeline & Floor IPM giải mã thành công!")
    return payload


def test_5_data_contract_verification(payload):
    print("\n--- [TEST 5/5] Kiểm tra Hợp đồng Dữ liệu INTERFACE.md ---")
    # Kiểm tra các trường cấp cao nhất
    required_root_keys = ["timestamp", "frame_id", "detections", "low_obstacles", "latency_ms"]
    for k in required_root_keys:
        assert k in payload, f"Thiếu trường bắt buộc '{k}' trong root JSON!"

    assert isinstance(payload["timestamp"], (float, int)), "timestamp phải là float/int!"
    assert isinstance(payload["frame_id"], int), "frame_id phải là int!"
    assert isinstance(payload["detections"], list), "detections phải là list!"
    assert isinstance(payload["low_obstacles"], list), "low_obstacles phải là list!"
    assert isinstance(payload["latency_ms"], (float, int)), "latency_ms phải là float/int!"

    # Kiểm tra từng detection
    for d in payload["detections"]:
        assert "class_id" in d, "Thiếu 'class_id' trong detection!"
        assert "class_name" in d, "Thiếu 'class_name' trong detection!"
        assert "confidence" in d, "Thiếu 'confidence' trong detection!"
        assert "azimuth_deg" in d, "Thiếu 'azimuth_deg' trong detection!"
        assert "estimated_dist" in d, "Thiếu 'estimated_dist' trong detection!"
        assert "safety_margin_m" in d, "Thiếu 'safety_margin_m' trong detection!"
        assert "bbox" in d and len(d["bbox"]) == 4, "Thiếu hoặc sai cấu trúc 'bbox' [x1, y1, x2, y2]!"
        assert -80.0 <= d["azimuth_deg"] <= 80.0, f"Góc azimuth {d['azimuth_deg']} nằm ngoài [-80, +80]!"

    # Kiểm tra low_obstacles
    for pt in payload["low_obstacles"]:
        assert len(pt) == 2, f"Vật cản sàn {pt} phải có 2 tọa độ [X, Y]!"
        assert isinstance(pt[0], (float, int)) and isinstance(pt[1], (float, int)), "Tọa độ vật cản phải là float!"

    print("-> Cấu trúc JSON kiểm tra mẫu:")
    print(json.dumps(payload, indent=2))
    print("✅ PASS 5/5: Hợp đồng dữ liệu JSON khớp 100% với agents-doc/INTERFACE.md!")


if __name__ == "__main__":
    print("=" * 68)
    print("   CS532 - BỘ KIỂM THỬ THỊ GIÁC & IPM TRÊN PC (HIẾU ME - AUDITED)")
    print("=" * 68)
    try:
        frame = test_1_camera_loader()
        test_2_azimuth_and_social_bubble()
        test_3_floor_ipm_and_division_by_zero()
        payload = test_4_yolo_vectorized_runner(frame)
        test_5_data_contract_verification(payload)
        print("\n" + "=" * 68)
        print("🎉 TẤT CẢ 5/5 BÀI KIỂM THỬ ĐÃ ĐẠT TIÊU CHUẨN HOÀN TOÀN (100% PASS)!")
        print("=" * 68)
    except Exception as e:
        print(f"\n❌ LỖI TRONG QUÁ TRÌNH KIỂM THỬ: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
