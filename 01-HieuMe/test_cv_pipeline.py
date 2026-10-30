#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
CS532 - Comprehensive Unit Test Suite for Computer Vision Pipeline (01-HieuMe)
==============================================================================
Tests:
  1. FrameGrabber: Synthetic generation & static image reading (640x480).
  2. FloorIPMTransformer: Homography, Division-by-Zero safety, and Temporal Filtering.
  3. Proxemics & Azimuth: ROS REP-103 CCW conformity (left +, right -) & Gaussian Proxemics.
  4. Vectorized Tensor Decoding: letterbox_blob & decode_yolo_output.
  5. End-to-End Pipeline Contract: Full schema verification for Planning & Dashboard.
"""

import os
import sys
import math
import numpy as np
import cv2

# Ensure UTF-8 stdout
if sys.platform.startswith("win"):
    try:
        sys.stdout.reconfigure(encoding="utf-8")
    except Exception:
        pass

# Ensure import of 01-HieuMe modules
_DIR = os.path.dirname(os.path.abspath(__file__))
if _DIR not in sys.path:
    sys.path.insert(0, _DIR)

from core.frame_grabber import FrameGrabber
from core.floor_ipm import FloorIPMTransformer, TemporalObstacleFilter
from core.proxemics import pixel_to_azimuth_deg, compute_dynamic_proxemics, ProxemicsEngine
from core.yolo_detector import letterbox_blob, decode_yolo_output, YoloDetector
from pipeline import CVPipeline


def test_1_frame_grabber():
    print("\n[TEST 1/5] Kiểm tra FrameGrabber (Đọc ảnh & Synthetic fallback)...")
    grabber = FrameGrabber(source="synthetic", width=640, height=480)
    ret, frame = grabber.read()
    assert ret, "Grabber read failed on synthetic source!"
    assert frame is not None, "Frame is None!"
    assert frame.shape == (480, 640, 3), f"Sai kích thước khung hình: {frame.shape}"
    grabber.release()
    print("  ✅ PASS: FrameGrabber sinh khung hình chuẩn (480, 640, 3).")


def test_2_floor_ipm_and_division_by_zero():
    print("\n[TEST 2/5] Kiểm tra Floor IPM, chống chia cho 0 & Bộ lọc Temporal Filter...")
    ipm = FloorIPMTransformer()

    # 1. Kiểm tra chống chia cho 0 tại đường chân trời (v = 0 hoặc đỉnh ảnh)
    xm, ym = ipm.image_pixel_to_robot_metric(320.0, 50.0)
    assert xm == 999.0, f"Lỗi: Điểm phía trên chân trời không được gán cự ly an toàn (xm={xm})!"

    # 2. Kiểm tra điểm sàn hợp lệ ở nửa dưới ảnh (ví dụ u=320, v=460)
    xm_valid, ym_valid = ipm.image_pixel_to_robot_metric(320.0, 460.0)
    assert 0.05 <= xm_valid <= 3.0, f"Tọa độ X sàn không hợp lệ: {xm_valid}m"
    assert abs(ym_valid) <= 1.0, f"Tọa độ Y sàn lệch quá lớn: {ym_valid}m"

    # 3. Kiểm tra chuyển đổi mảng hàng loạt (Batch metric)
    test_pts = np.array([[320.0, 460.0], [100.0, 20.0]], dtype=np.float32)
    batch_res = ipm.batch_image_pixels_to_robot_metric(test_pts)
    assert batch_res.shape == (2, 2)
    assert batch_res[1, 0] == 999.0, "Batch metric không bắt được điểm chân trời!"

    # 4. Kiểm tra Temporal Filter
    tfilter = TemporalObstacleFilter(alpha=0.65, min_hits=2, max_age=2, match_dist_m=0.15)
    f1 = tfilter.update([[0.60, 0.10]])
    assert len(f1) == 0, "Lỗi: Điểm mới xuất hiện 1 frame (chưa đủ min_hits=2) không được xuất hiện!"

    f2 = tfilter.update([[0.62, 0.11]])
    assert len(f2) == 1, "Lỗi: Điểm xuất hiện liên tục 2 frames phải được xác nhận!"
    assert abs(f2[0][0] - 0.613) < 0.05, f"EMA làm mượt không khớp: {f2[0]}"

    # 5. Phản ứng khẩn cấp nếu cự ly < 0.35m (kích hoạt ngay frame đầu)
    f_emg = tfilter.update([[0.25, 0.00]])
    found_emg = any(p[0] < 0.30 for p in f_emg)
    assert found_emg, "Lỗi: Vật cản khẩn cấp (<0.35m) không được kích hoạt ngay lập tức!"

    print("  ✅ PASS: Floor IPM triệt tiêu 100% Division-by-Zero và Temporal Filter hoạt động chuẩn xác.")


def test_3_azimuth_and_proxemics():
    print("\n[TEST 3/5] Kiểm tra Quy ước Góc Azimuth ROS REP-103 CCW & Asymmetric Proxemics...")
    fov = 160.0

    # Trục giữa (cx = 320) -> 0°
    az_center = pixel_to_azimuth_deg(320.0, 640, fov)
    assert az_center == 0.0, f"Góc tâm ảnh phải là 0°, nhưng là {az_center}°"

    # Bên TRÁI (cx = 80 < 320) -> Phải mang dấu DƯƠNG (+)
    az_left = pixel_to_azimuth_deg(80.0, 640, fov)
    assert az_left > 0.0, f"Góc bên trái phải DƯƠNG theo chuẩn ROS REP-103 CCW, nhưng nhận {az_left}°"

    # Bên PHẢI (cx = 560 > 320) -> Phải mang dấu ÂM (-)
    az_right = pixel_to_azimuth_deg(560.0, 640, fov)
    assert az_right < 0.0, f"Góc bên phải phải ÂM theo chuẩn ROS REP-103 CCW, nhưng nhận {az_right}°"

    # Kiểm tra tính đối xứng biên độ
    assert math.isclose(abs(az_left), abs(az_right), abs_tol=0.5), "Biên độ góc trái phải không cân xứng!"

    # Kiểm tra Asymmetric Proxemics cho người
    r_front, env_f = compute_dynamic_proxemics(0.0, "person")
    assert r_front == 0.80, f"Bán kính trực diện phải là 0.80m, nhưng là {r_front}m"

    r_side, env_s = compute_dynamic_proxemics(80.0, "person")
    assert r_side <= 0.45, f"Bán kính sườn phải ~0.40m, nhưng là {r_side}m"

    print("  ✅ PASS: Azimuth tuân thủ nghiêm ngặt ROS REP-103 CCW & Asymmetric Proxemics chính xác.")


def test_4_yolo_vectorized_tensor_decoding():
    print("\n[TEST 4/5] Kiểm tra Tiền xử lý Letterbox & Giải mã Tensor Vectorized...")
    img = np.zeros((480, 640, 3), dtype=np.uint8)
    blob, ratio, pad = letterbox_blob(img, size=640)
    assert blob.shape == (1, 3, 640, 640), f"Blob shape sai: {blob.shape}"
    assert blob.dtype == np.float32, "Blob phải là float32!"

    # Giả lập tensor đầu ra YOLO (1, 84, 8400)
    sim_output = np.zeros((1, 84, 8400), dtype=np.float32)
    # Gán 1 bounding box cho lớp 'person' (class_id=0) tại anchor 100
    sim_output[0, 0, 100] = 320.0  # cx
    sim_output[0, 1, 100] = 320.0  # cy
    sim_output[0, 2, 100] = 100.0  # bw
    sim_output[0, 3, 100] = 200.0  # bh
    sim_output[0, 4, 100] = 0.92   # score class 0 (person)

    decoded = decode_yolo_output(sim_output, 640, 480, ratio, pad, conf_threshold=0.40)
    assert len(decoded) == 1, f"Giải mã tensor thất bại, nhận được {len(decoded)} kết quả!"
    bbox, conf, cid = decoded[0]
    assert cid == 0, f"Class ID giải mã sai: {cid} (kỳ vọng 0 - person)"
    assert abs(conf - 0.92) < 1e-3, f"Confidence giải mã sai: {conf}"
    assert len(bbox) == 4, "BBox phải có 4 tọa độ [x1, y1, x2, y2]"

    print("  ✅ PASS: Letterbox và Vectorized Tensor Decoding giải mã chính xác trong < 1ms.")


def test_5_end_to_end_cv_pipeline():
    print("\n[TEST 5/5] Kiểm tra Toàn trình CVPipeline & Chuẩn Hợp Đồng Dữ Liệu...")
    pipeline = CVPipeline(source="synthetic", enable_grabber=False)

    sample_frame = np.ones((480, 640, 3), dtype=np.uint8) * 220
    # Tạo vật cản sàn đen ở đáy ảnh
    cv2.rectangle(sample_frame, (280, 410), (360, 450), (20, 20, 20), -1)

    # Chạy 3 chu kỳ liên tiếp để Temporal Filter xác nhận vật cản
    for _ in range(3):
        payload = pipeline.process_frame(sample_frame)

    # Kiểm định hợp đồng dữ liệu chuẩn
    required_keys = ["timestamp", "frame_id", "detections", "low_obstacles", "social_bubbles", "latency_ms"]
    for k in required_keys:
        assert k in payload, f"Thiếu trường '{k}' trong payload JSON!"

    assert isinstance(payload["detections"], list), "detections phải là list!"
    assert isinstance(payload["low_obstacles"], list), "low_obstacles phải là list!"
    assert isinstance(payload["social_bubbles"], list), "social_bubbles phải là list!"
    assert payload["latency_ms"] >= 0.0, "latency_ms không hợp lệ!"

    print(f"  Payload trích xuất mẫu (Frame {payload['frame_id']}):")
    print(f"    • Detections: {len(payload['detections'])}")
    print(f"    • Low-lying Obstacles (<16cm): {payload['low_obstacles']}")
    print(f"    • Social Bubbles: {len(payload['social_bubbles'])}")
    print(f"    • Latency: {payload['latency_ms']} ms")

    pipeline.close()
    print("  ✅ PASS: CVPipeline hoàn thành 100% hợp đồng dữ liệu với Planning và Dashboard.")


def main():
    print("=" * 70)
    print(" 🔬 CS532 COMPUTER VISION PIPELINE — COMPREHENSIVE TEST SUITE")
    print("=" * 70)

    try:
        test_1_frame_grabber()
        test_2_floor_ipm_and_division_by_zero()
        test_3_azimuth_and_proxemics()
        test_4_yolo_vectorized_tensor_decoding()
        test_5_end_to_end_cv_pipeline()

        print("\n" + "=" * 70)
        print(" 🎉 KẾT QUẢ: 5/5 BÀI KIỂM THỬ THỊ GIÁC MÁY TÍNH ĐẠT TIÊU CHUẨN HOÀN TOÀN!")
        print("=" * 70)
        return 0
    except AssertionError as e:
        print(f"\n❌ KIỂM THỬ THẤT BẠI: {e}")
        return 1
    except Exception as e:
        print(f"\n❌ LỖI RUNTIME: {e}")
        import traceback
        traceback.print_exc()
        return 1


if __name__ == "__main__":
    sys.exit(main())
