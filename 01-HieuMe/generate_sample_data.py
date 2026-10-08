#!/usr/bin/env python3
"""
Tu Dong Tao Du Lieu Anh San Mau (Synthetic Floor Generator & Asset Loader)
========================================================================
CS532 - Module Computer Vision (Hieu Me)
Muc dich:
    Tu dong chuan bi anh mau csi_floor_sample.jpg cho bo test test_cv_local.py
    khi thanh vien phat trien tren PC khong giu robot that va khong co camera.
"""

import os
import sys
import shutil
import numpy as np
import cv2

# Ensure UTF-8 output on Windows
if sys.platform.startswith("win"):
    try:
        sys.stdout.reconfigure(encoding="utf-8")
    except Exception:
        pass

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
SAMPLE_DIR = os.path.join(BASE_DIR, "sample_data")
SAMPLE_IMG_PATH = os.path.join(SAMPLE_DIR, "csi_floor_sample.jpg")
DEFAULT_REPO_ASSET = os.path.abspath(os.path.join(BASE_DIR, "..", "03-MinhHieu", "backend", "app", "robot", "default_camera.jpg"))

def generate_synthetic_perspective_floor(width=640, height=480):
    """
    Tao anh san nha phoi canh 3D nhan tao voi cac vach ke ca-ro
    va mot so vat the thap sat san (chieu cao < 16cm).
    """
    img = np.ones((height, width, 3), dtype=np.uint8) * 225

    # Diem chan troi (Vanishing point) mo phong camera chuc xuong 20 do
    vp_x = width // 2
    vp_y = int(height * 0.35)

    # 1. Cac vach ke doc hoi tu ve diem chan troi
    for x_base in range(-200, width + 400, 70):
        cv2.line(img, (x_base, height), (vp_x, vp_y), (190, 190, 190), 2)

    # 2. Cac vach ke ngang phan bo theo ti le phoi canh chieu sau
    y_lines = [475, 450, 420, 385, 345, 305, 270, 240, 215, 195]
    for y in y_lines:
        cv2.line(img, (0, y), (width, y), (190, 190, 190), 2)

    # 3. Ve 4 goc danh dau vung chu nhat hieu chuan IPM (0.5m x 0.5m)
    p1 = (int(width * 0.20), int(height * 0.85))
    p2 = (int(width * 0.80), int(height * 0.85))
    p3 = (int(width * 0.95), int(height * 0.98))
    p4 = (int(width * 0.05), int(height * 0.98))
    pts = np.array([p1, p2, p3, p4], np.int32)
    cv2.polylines(img, [pts], isClosed=True, color=(0, 180, 0), thickness=2)
    cv2.putText(img, "IPM Calibration Grid (0.5m x 0.5m)", (p1[0], p1[1] - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 140, 0), 1)

    # 4. Ve 1 vat can mau sat mat san (chiec dep / hop nho, h < 16cm)
    obs_box = (int(width * 0.52), int(height * 0.82), 65, 30)  # (x, y, w, h)
    cv2.rectangle(img, (obs_box[0], obs_box[1]),
                  (obs_box[0] + obs_box[2], obs_box[1] + obs_box[3]),
                  (60, 60, 180), -1)
    cv2.putText(img, "Low Obstacle (<16cm)", (obs_box[0] - 15, obs_box[1] - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (40, 40, 160), 1)

    # 5. Ve them hinh nguoi dung phia xa (test nhanh Detection)
    person_box = (int(width * 0.28), int(height * 0.38), 70, 160)
    cv2.rectangle(img, (person_box[0], person_box[1]),
                  (person_box[0] + person_box[2], person_box[1] + person_box[3]),
                  (200, 100, 30), 2)
    cv2.putText(img, "Person (~1.8m)", (person_box[0] - 10, person_box[1] - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (180, 80, 20), 1)

    return img

def setup_sample_data():
    os.makedirs(SAMPLE_DIR, exist_ok=True)
    
    # Tao anh san phoi canh tong hop
    synthetic_img = generate_synthetic_perspective_floor()
    cv2.imwrite(SAMPLE_IMG_PATH, synthetic_img)
    print(f"[OK] Da tao anh san phoi canh tong hop tai: {SAMPLE_IMG_PATH}")
    
    # Sao chep anh default_camera.jpg neu co
    if os.path.exists(DEFAULT_REPO_ASSET):
        repo_copy_path = os.path.join(SAMPLE_DIR, "jetbot_default_cam.jpg")
        shutil.copy2(DEFAULT_REPO_ASSET, repo_copy_path)
        print(f"[OK] Da sao chep anh default camera tu repo sang: {repo_copy_path}")

if __name__ == "__main__":
    setup_sample_data()
