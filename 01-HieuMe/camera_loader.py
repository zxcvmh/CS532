#!/usr/bin/env python3
"""
Camera Frame Provider (CameraLoader with Zero-Buffer Threaded Grabber)
=====================================================================
CS532 - Advanced AI Robotics (Module Computer Vision - Hieu Me)
Mục đích:
  Đọc khung hình camera đa nền tảng linh hoạt với cơ chế triệt tiêu độ trễ:
  - Khử hoàn toàn hiện tượng trễ tích lũy (Buffer Lag) bằng luồng Thread chạy ngầm.
  - Hỗ trợ kết nối cáp USB Type-C qua Webcam Index (0, 1, 2) hoặc IP Stream (http/rtsp).
  - Tự động fallback ảnh mẫu khi test trên PC không có camera.
"""

import os
import sys
import time
import threading
from typing import Optional, Tuple, Union
import numpy as np
import cv2

# Ensure UTF-8 stdout on Windows
if sys.platform.startswith("win"):
    try:
        sys.stdout.reconfigure(encoding="utf-8")
    except Exception:
        pass


class CameraLoader:
    def __init__(
        self,
        source: Optional[Union[str, int]] = None,
        width: int = 640,
        height: int = 480,
        fps: int = 30,
        flip_method: int = 0
    ):
        self.width = width
        self.height = height
        self.fps = fps
        self.flip_method = flip_method
        self.source = source or "auto"
        self.cap: Optional[cv2.VideoCapture] = None
        self.static_frame: Optional[np.ndarray] = None
        self.is_synthetic = False

        # Quản lý luồng đọc thời gian thực (Zero-Buffer Grabber)
        self._thread: Optional[threading.Thread] = None
        self._thread_running = False
        self._latest_frame: Optional[np.ndarray] = None
        self._frame_lock = threading.Lock()
        
        self._init_source()

    def _is_jetson(self) -> bool:
        """Kiểm tra xem mã có đang chạy trên NVIDIA Jetson hay không."""
        return os.path.exists("/etc/nv_tegra_release") or os.path.exists("/usr/lib/aarch64-linux-gnu/tegra")

    def _get_gstreamer_pipeline(self) -> str:
        """Pipeline GStreamer chuẩn NVMM cho IMX219 trên Jetson Nano."""
        return (
            f"nvarguscamerasrc sensor-id=0 ! "
            f"video/x-raw(memory:NVMM), width=(int){self.width}, height=(int){self.height}, "
            f"format=(string)NV12, framerate=(fraction){self.fps}/1 ! "
            f"nvvidconv flip-method={self.flip_method} ! "
            f"video/x-raw, width=(int){self.width}, height=(int){self.height}, format=(string)BGRx ! "
            f"videoconvert ! "
            f"video/x-raw, format=(string)BGR ! appsink drop=True max-buffers=1"
        )

    def _init_source(self):
        # 1. Hỗ trợ luồng IP Camera từ điện thoại (HTTP / RTSP)
        if isinstance(self.source, str) and (self.source.startswith("http://") or self.source.startswith("https://") or self.source.startswith("rtsp://")):
            print(f"[CameraLoader] Dang ket noi toi Camera dien thoai: {self.source}")
            # Cấu hình tối ưu FFmpeg & timeout cho OpenCV
            os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "fflags;nobuffer|flags;low_delay"
            self.cap = cv2.VideoCapture(self.source)
            self._start_zero_buffer_thread()
            return

        # 2. Hỗ trợ kết nối qua cáp USB Type-C (Webcam Index 0, 1, 2)
        if isinstance(self.source, int) or (isinstance(self.source, str) and self.source.isdigit()):
            cam_idx = int(self.source)
            print(f"[CameraLoader USB] Dang mo Camera cap USB index {cam_idx}...")
            backend = cv2.CAP_DSHOW if sys.platform.startswith("win") else cv2.CAP_ANY
            self.cap = cv2.VideoCapture(cam_idx, backend)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            self._start_zero_buffer_thread()
            return

        if self.source == "webcam":
            print("[CameraLoader] Dang mo Webcam USB mac dinh...")
            backend = cv2.CAP_DSHOW if sys.platform.startswith("win") else cv2.CAP_ANY
            self.cap = cv2.VideoCapture(0, backend)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            self._start_zero_buffer_thread()
            return

        # 3. Chế độ tự động
        if self.source == "auto":
            if self._is_jetson():
                print("[CameraLoader] Phat hien Jetson Nano -> Khoi tao GStreamer CSI Camera...")
                gst = self._get_gstreamer_pipeline()
                self.cap = cv2.VideoCapture(gst, cv2.CAP_GSTREAMER)
                if not self.cap.isOpened():
                    print("[WARN] Khong mo duoc GStreamer. Chuyen sang fallback.")
                    self._fallback_init()
            else:
                self._fallback_init()
        elif self.source == "gstreamer":
            gst = self._get_gstreamer_pipeline()
            self.cap = cv2.VideoCapture(gst, cv2.CAP_GSTREAMER)
        elif self.source == "synthetic":
            self.is_synthetic = True
        elif isinstance(self.source, str) and os.path.exists(self.source):
            if self.source.lower().endswith((".jpg", ".jpeg", ".png", ".bmp")):
                self.static_frame = cv2.imread(self.source)
                print(f"[CameraLoader] Da nap anh tinh: {self.source}")
            else:
                self.cap = cv2.VideoCapture(self.source)
                self._start_zero_buffer_thread()
        else:
            self._fallback_init()

    def _fallback_init(self):
        """Khởi tạo dự phòng khi chạy trên PC hoặc không có camera thật."""
        base_dir = os.path.dirname(os.path.abspath(__file__))
        sample_path = os.path.join(base_dir, "sample_data", "csi_floor_sample.jpg")

        if os.path.exists(sample_path):
            self.static_frame = cv2.imread(sample_path)
            print(f"[CameraLoader PC] Dang su dung anh mau: {sample_path}")
        else:
            print("[CameraLoader PC] Khong tim thay anh mau -> Chuyen sang che do tu sinh anh synthetic.")
            self.is_synthetic = True

    def _start_zero_buffer_thread(self):
        """Khởi động luồng đọc nền liên tục xả bộ đệm để giữ frame mới nhất."""
        if self.cap is not None and self.cap.isOpened():
            self._thread_running = True
            self._thread = threading.Thread(target=self._grabber_loop, daemon=True)
            self._thread.start()
            print("[CameraLoader] Da kich hoat che do Zero-Buffer Grabber (Triet tieu 100% do tre!).")

    def _grabber_loop(self):
        """Vòng lặp đọc nền tốc độ cao, luôn giữ frame mới nhất."""
        while self._thread_running and self.cap is not None and self.cap.isOpened():
            ret, frame = self.cap.read()
            if ret and frame is not None:
                if frame.shape[1] != self.width or frame.shape[0] != self.height:
                    frame = cv2.resize(frame, (self.width, self.height))
                with self._frame_lock:
                    self._latest_frame = frame
            else:
                time.sleep(0.01)

    def read_frame(self) -> Tuple[bool, Optional[np.ndarray]]:
        """Đọc khung hình mới nhất tại thời điểm hiện tại."""
        if self.static_frame is not None:
            return True, self.static_frame.copy()

        if self.is_synthetic:
            from generate_sample_data import generate_synthetic_perspective_floor
            return True, generate_synthetic_perspective_floor(self.width, self.height)

        # Nếu có luồng zero-buffer đang chạy, lấy frame mới nhất ngay lập tức
        if self._thread_running:
            with self._frame_lock:
                if self._latest_frame is not None:
                    return True, self._latest_frame.copy()
            return False, None

        if self.cap is not None and self.cap.isOpened():
            ret, frame = self.cap.read()
            if ret and frame is not None:
                if frame.shape[1] != self.width or frame.shape[0] != self.height:
                    frame = cv2.resize(frame, (self.width, self.height))
                return True, frame

        return False, None

    def release(self):
        self._thread_running = False
        if self._thread is not None:
            self._thread.join(timeout=0.5)
            self._thread = None
        if self.cap is not None:
            self.cap.release()
            self.cap = None
