#!/usr/bin/env python3
"""
Kiểm Thử Trực Tiếp Bằng Camera Điện Thoại (Live Phone Camera Test)
==================================================================
CS532 - Advanced AI Robotics (Module Computer Vision - Hieu Me)
Chức năng:
  - Đọc luồng video trực tiếp từ điện thoại (qua Wi-Fi URL hoặc Iriun/USB Webcam)
  - Chạy mô hình YOLOv8 trích xuất người/vật cản, tính góc lệch Azimuth & Social Bubble
  - Phân đoạn mặt sàn IPM tìm dị vật thấp sát sàn (< 16cm)
  - Hiển thị trực quan cửa sổ OpenCV theo thời gian thực (nhấn 'q' để thoát)
"""

import os
import sys
import time
import argparse
import numpy as np
import cv2

# Ensure UTF-8 stdout on Windows
if sys.platform.startswith("win"):
    try:
        sys.stdout.reconfigure(encoding="utf-8")
    except Exception:
        pass

from camera_loader import CameraLoader
from yolo_seg_runner import YoloSegRunner
from floor_ipm import FloorIPMTransformer

def parse_args():
    parser = argparse.ArgumentParser(description="Chạy kiểm thử trực tiếp bằng Camera Điện thoại / Webcam.")
    parser.add_argument(
        "-s", "--source",
        type=str,
        default=None,
        help="Nguồn video: URL từ IP Webcam (vd: http://192.168.1.15:8080/video), hoặc số thứ tự webcam (vd: 0, 1), hoặc 'auto'"
    )
    return parser.parse_args()

def main():
    args = parse_args()
    source = args.source

    print("=" * 70)
    print("   CS532 - KIỂM THỬ TRỰC TIẾP CAMERA ĐIỆN THOẠI (HIẾU ME)")
    print("=" * 70)

    # Nếu người dùng chưa truyền tham số, gợi ý nhập URL hoặc chọn mặc định
    if source is None:
        print("\n[HƯỚNG DẪN KẾT NỐI CAMERA ĐIỆN THOẠI]:")
        print("  1. Cài app 'IP Webcam' trên điện thoại, nhấn 'Start Server'.")
        print("  2. Nhập URL hiển thị trên điện thoại (ví dụ: http://192.168.1.5:8080/video).")
        print("  3. Hoặc nhấn [ENTER] để dùng Webcam máy tính mặc định / Iriun Webcam.\n")
        try:
            user_input = input("Nhập URL camera điện thoại (hoặc ENTER để dùng webcam): ").strip()
            if user_input:
                source = user_input
            else:
                source = "webcam"
        except (EOFError, KeyboardInterrupt):
            source = "webcam"

    print(f"\n[INFO] Đang kết nối tới nguồn video: {source} ...")
    loader = CameraLoader(source=source)
    runner = YoloSegRunner()
    ipm = FloorIPMTransformer()

    window_name = "CS532 - JetBot Live Camera Perception (Hieu Me) - Nhan Q de thoat"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, 960, 720)

    fps_list = []
    print("\n✅ KẾT NỐI THÀNH CÔNG! Đang hiển thị cửa sổ camera...")
    print("-> Nhấn phím 'q' hoặc 'ESC' trên cửa sổ video để THOÁT.\n")

    while True:
        t_start = time.time()
        ret, frame = loader.read_frame()
        if not ret or frame is None:
            print("[WARN] Mất kết nối khung hình, đang thử lại...")
            time.sleep(0.1)
            continue

        h, w = frame.shape[:2]
        payload = runner.run_inference(frame)
        dt_ms = (time.time() - t_start) * 1000.0
        fps = 1000.0 / dt_ms if dt_ms > 0 else 30.0
        fps_list.append(fps)
        if len(fps_list) > 30:
            fps_list.pop(0)
        avg_fps = sum(fps_list) / len(fps_list)

        display_frame = frame.copy()

        # 1. Vẽ vùng hình thang hiệu chuẩn IPM trên sàn (Màu vàng)
        src_pts = ipm.src_pts.astype(np.int32)
        cv2.polylines(display_frame, [src_pts], isClosed=True, color=(0, 220, 255), thickness=2)
        cv2.putText(display_frame, "IPM Ground Region (Camera CSI)", (src_pts[0][0], src_pts[0][1] - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 220, 255), 1)

        # 2. Vẽ các vật cản thấp sát sàn (< 16cm) mà LiDAR bị mù (Màu đỏ chấm tròn)
        for obs in payload.get("low_obstacles", []):
            xm, ym = obs[0], obs[1]
            # Hiển thị text tọa độ mét
            label_obs = f"Obs: X={xm:.2f}m Y={ym:.2f}m"
            # Chiếu ngược để tìm điểm gần đúng trên màn hình vẽ
            cv2.putText(display_frame, f"* {label_obs}", (20, h - 30 - 20 * payload["low_obstacles"].index(obs)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        # 3. Vẽ Bounding Box, Góc phương vị và Social Bubble cho từng người/đối tượng
        for det in payload.get("detections", []):
            bbox = det.get("bbox", [])
            if len(bbox) == 4:
                x1, y1, x2, y2 = bbox
                cls_name = det.get("class_name", "obj").upper()
                azimuth = det.get("azimuth_deg", 0.0)
                margin = det.get("safety_margin_m", 0.3)
                conf = det.get("confidence", 0.0)
                dist = det.get("estimated_dist", 1.0)

                # Màu xanh lá cho người, màu tím cho đồ vật khác
                color = (0, 255, 0) if "PERSON" in cls_name else (255, 100, 200)

                # Vẽ khung Bbox
                cv2.rectangle(display_frame, (x1, y1), (x2, y2), color, 2)

                # Vẽ nhãn thông tin góc lệch Azimuth & Social Bubble
                info_text = f"{cls_name} {conf:.2f} | Azimuth: {azimuth}° | Bubble: {margin}m"
                cv2.putText(display_frame, info_text, (x1, max(20, y1 - 10)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.48, color, 2)

                # Vẽ tâm chân đối tượng tiếp xúc với sàn
                cx = int((x1 + x2) / 2)
                cv2.circle(display_frame, (cx, y2), 5, (0, 0, 255), -1)

        # 4. Hiển thị thông số FPS và trạng thái hệ thống góc trên trái
        cv2.rectangle(display_frame, (10, 10), (320, 75), (0, 0, 0), -1)
        cv2.putText(display_frame, f"FPS: {avg_fps:.1f} ({dt_ms:.1f} ms/frame)", (20, 35),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.putText(display_frame, f"Detections: {len(payload.get('detections', []))} | Low Obs: {len(payload.get('low_obstacles', []))}", (20, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)

        cv2.imshow(window_name, display_frame)

        # Thoát khi nhấn phím 'q' hoặc ESC
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q') or key == 27:
            print("\n[INFO] Đã nhận lệnh thoát từ người dùng.")
            break

    loader.release()
    cv2.destroyAllWindows()
    print("✅ ĐÃ ĐÓNG CỬA SỔ CAMERA AN TOÀN!")

if __name__ == "__main__":
    main()
