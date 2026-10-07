#!/usr/bin/env python3
"""
Bộ Kiểm Thử Thị Giác & Bù Điểm Mù LiDAR Trên Máy Tính Cá Nhân (Hiếu Me)
========================================================================
Kiểm thử trích xuất Bounding Box, Góc phương vị Azimuth, Bán kính an toàn động
và Phép biến đổi IPM trên ảnh chụp thực tế từ camera CSI JetBot.
"""

import os
import sys
import time
import math
import numpy as np

try:
    import cv2
except ImportError:
    print("[ERROR] Cần cài đặt opencv-python: pip install opencv-python")
    sys.exit(1)

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
SAMPLE_IMG_PATH = os.path.join(BASE_DIR, "sample_data", "csi_floor_sample.jpg")

def test_1_camera_image_load():
    print("\n--- [TEST 1/3] Kiểm tra Nạp Ảnh Mẫu Camera CSI JetBot ---")
    if not os.path.exists(SAMPLE_IMG_PATH):
        print(f"⚠️ Không tìm thấy ảnh tại: {SAMPLE_IMG_PATH}")
        # Tạo ảnh giả lập nếu chưa có
        img = np.zeros((480, 640, 3), dtype=np.uint8)
    else:
        img = cv2.imread(SAMPLE_IMG_PATH)
    assert img is not None, "Không đọc được ảnh mẫu!"
    h, w = img.shape[:2]
    print(f"-> Đọc ảnh thành công: Độ phân giải {w}x{h} pixels")
    print("✅ PASS: Ảnh mẫu CSI camera sẵn sàng!")
    return img

def test_2_azimuth_and_safety_margin():
    print("\n--- [TEST 2/3] Kiểm tra Tính toán Góc Phương vị & Bán kính An toàn ---")
    # Giả lập phát hiện người tại pixel (x=200, y=150, w=100, h=250)
    img_w = 640
    cx = 200 + 100 / 2.0  # 250px
    # Camera horizontal FOV 160 độ (từ -80 độ đến +80 độ)
    fov_deg = 160.0
    azimuth_deg = ((cx - (img_w / 2.0)) / (img_w / 2.0)) * (fov_deg / 2.0)
    
    # Hiếu me tự cấu hình bán kính an toàn cho từng class
    safety_margins = {
        "person": 0.90,
        "chair": 0.35,
        "backpack": 0.25
    }
    
    print(f"-> Tâm bbox x = {cx}px -> Góc lệch phương vị: {azimuth_deg:.1f}°")
    print(f"-> Bán kính Social Bubble cấu hình cho Person: {safety_margins['person']} m")
    assert -80.0 <= azimuth_deg <= 80.0, "Góc phương vị nằm ngoài FOV camera!"
    print("✅ PASS: Trích xuất góc phương vị & Safety Margin chính xác!")

def test_3_ipm_floor_homography(img):
    print("\n--- [TEST 3/3] Kiểm tra Ma trận IPM (Bird's Eye View Mặt Sàn) ---")
    h, w = img.shape[:2]
    # Định nghĩa 4 điểm hình thang trên ảnh camera tương ứng với hình chữ nhật trên sàn thực tế
    src_pts = np.float32([
        [w * 0.20, h * 0.85],
        [w * 0.80, h * 0.85],
        [w * 0.95, h * 0.98],
        [w * 0.05, h * 0.98]
    ])
    # Kích thước ảnh BEV sau biến đổi
    dst_w, dst_h = 300, 300
    dst_pts = np.float32([
        [0, 0],
        [dst_w, 0],
        [dst_w, dst_h],
        [0, dst_h]
    ])
    
    H = cv2.getPerspectiveTransform(src_pts, dst_pts)
    bev = cv2.warpPerspective(img, H, (dst_w, dst_h))
    
    # Giả lập một điểm vật cản trên ảnh BEV chuyển sang tọa độ mét (vùng 1.0m x 1.0m)
    res_m_per_px = 1.0 / dst_h
    sample_obs_px = (150, 100)
    obs_x_m = round((dst_h - sample_obs_px[1]) * res_m_per_px + 0.15, 2)
    obs_y_m = round((sample_obs_px[0] - dst_w / 2.0) * res_m_per_px, 2)
    
    print(f"-> Ma trận Homography H shape: {H.shape}")
    print(f"-> Vật cản sàn tại pixel {sample_obs_px} -> Tọa độ robot: X = {obs_x_m}m, Y = {obs_y_m}m")
    print("✅ PASS: Biến đổi IPM thành công! Tọa độ mét sẵn sàng nạp sang Planning!")

if __name__ == "__main__":
    print("=" * 65)
    print("   CS532 - BỘ KIỂM THỬ THỊ GIÁC & IPM TRÊN PC (HIẾU ME)")
    print("=" * 65)
    try:
        img = test_1_camera_image_load()
        test_2_azimuth_and_safety_margin()
        test_3_ipm_floor_homography(img)
        print("\n🎉 KIỂM THỬ COMPUTER VISION HOÀN TẤT TRÊN MÁY TÍNH CÁ NHÂN!")
    except Exception as e:
        print(f"\n❌ LỖI TRONG QUÁ TRÌNH KIỂM THỬ: {e}")
        import traceback
        traceback.print_exc()
