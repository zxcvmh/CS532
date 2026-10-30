#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Multi-Source Frame Grabber (Zero-Buffer Threaded Pipeline)
==========================================================
CS532 - Advanced AI Robotics (Module Computer Vision - Hieu Me)
Tính năng:
  - Triệt tiêu tích lũy hàng đợi khung hình (Buffer Lag) bằng luồng Thread chạy nền.
  - Hỗ trợ đa nguồn đọc linh hoạt:
      1. CSI Camera IMX219 (GStreamer nvarguscamerasrc trên Jetson Nano).
      2. HTTP / RTSP MJPEG Stream (ví dụ: http://<jetbot-ip>:8080/api/stream/mjpeg).
      3. USB Webcam (Index 0, 1, 2) trên PC/Laptop.
      4. Tệp ảnh tĩnh hoặc video ghi lại (Offline Evaluation).
      5. Bộ tạo ảnh sàn giả lập 3D (Synthetic Mode) phục vụ unit test không cần camera.
"""

import os
import sys
import time
import threading
from typing import Optional, Tuple, Union
import numpy as np
import cv2


class FrameGrabber:
    """Thread-safe, Zero-Buffer Frame Grabber hỗ trợ đa nguồn dữ liệu."""

    def __init__(
        self,
        source: Optional[Union[str, int]] = None,
        width: int = 640,
        height: int = 480,
        fps: int = 20,
        flip_method: int = 0
    ):
        self.width = int(width)
        self.height = int(height)
        self.fps = int(fps)
        self.flip_method = int(flip_method)
        self.source = source or "auto"

        self.cap: Optional[cv2.VideoCapture] = None
        self.static_frame: Optional[np.ndarray] = None
        self.is_synthetic = False

        self._thread: Optional[threading.Thread] = None
        self._running = False
        self._latest_frame: Optional[np.ndarray] = None
        self._frame_lock = threading.Lock()

        self._init_source()

    @staticmethod
    def is_jetson() -> bool:
        """Kiểm tra môi trường có phải phần cứng NVIDIA Jetson (Tegra) hay không."""
        return os.path.exists("/etc/nv_tegra_release") or os.path.exists("/usr/lib/aarch64-linux-gnu/tegra")

    def _get_gstreamer_pipeline(self) -> str:
        """Pipeline GStreamer chuẩn NVMM cho IMX219 trên Jetson Nano."""
        return (
            f"nvarguscamerasrc sensor-id=0 ! "
            f"video/x-raw(memory:NVMM), width=(int)1280, height=(int)720, "
            f"format=(string)NV12, framerate=(fraction){self.fps}/1 ! "
            f"nvvidconv flip-method={self.flip_method} ! "
            f"video/x-raw, width=(int){self.width}, height=(int){self.height}, format=(string)BGRx ! "
            f"videoconvert ! "
            f"video/x-raw, format=(string)BGR ! appsink drop=True max-buffers=1"
        )

    def _init_source(self):
        # 1. Nguồn HTTP / RTSP Stream (Camera điện thoại hoặc Native MJPEG từ JetBot)
        if isinstance(self.source, str) and (
            self.source.startswith("http://") or self.source.startswith("https://") or self.source.startswith("rtsp://")
        ):
            print(f"[FrameGrabber] Đang kết nối tới HTTP/RTSP Stream: {self.source}")
            os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "fflags;nobuffer|flags;low_delay"
            self.cap = cv2.VideoCapture(self.source)
            self._start_reader_thread()
            return

        # 2. USB Camera qua Index số (0, 1, 2)
        if isinstance(self.source, int) or (isinstance(self.source, str) and self.source.isdigit()):
            idx = int(self.source)
            print(f"[FrameGrabber] Đang mở Camera Index {idx}...")
            backend = cv2.CAP_DSHOW if sys.platform.startswith("win") else cv2.CAP_ANY
            self.cap = cv2.VideoCapture(idx, backend)
            if self.cap.isOpened():
                self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
                self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
                self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                self._start_reader_thread()
                return
            self._fallback_init()
            return

        # 3. Nguồn webcam chỉ định
        if self.source == "webcam":
            backend = cv2.CAP_DSHOW if sys.platform.startswith("win") else cv2.CAP_ANY
            self.cap = cv2.VideoCapture(0, backend)
            if self.cap.isOpened():
                self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
                self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
                self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                self._start_reader_thread()
                return
            self._fallback_init()
            return

        # 4. Nguồn GStreamer chỉ định
        if self.source == "gstreamer":
            gst = self._get_gstreamer_pipeline()
            self.cap = cv2.VideoCapture(gst, cv2.CAP_GSTREAMER)
            if self.cap.isOpened():
                self._start_reader_thread()
                return
            self._fallback_init()
            return

        # 5. Nguồn tự động (Auto Discovery)
        if self.source == "auto":
            if self.is_jetson():
                gst = self._get_gstreamer_pipeline()
                self.cap = cv2.VideoCapture(gst, cv2.CAP_GSTREAMER)
                if self.cap.isOpened():
                    print("[FrameGrabber] Khởi tạo thành công GStreamer CSI Camera trên Jetson.")
                    self._start_reader_thread()
                    return
                # Thử USB Camera fallback trên Jetson
                self.cap = cv2.VideoCapture(0)
                if self.cap.isOpened():
                    self._start_reader_thread()
                    return
                self._fallback_init()
            else:
                # Trên PC/Laptop: Thử mở webcam có sẵn
                backend = cv2.CAP_DSHOW if sys.platform.startswith("win") else cv2.CAP_ANY
                try:
                    test_cap = cv2.VideoCapture(0, backend)
                    if test_cap.isOpened():
                        ret, test_frame = test_cap.read()
                        if ret and test_frame is not None and test_frame.size > 0:
                            self.cap = test_cap
                            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
                            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
                            self._start_reader_thread()
                            return
                        test_cap.release()
                except Exception:
                    pass
                self._fallback_init()
            return

        # 6. Tệp ảnh hoặc video có sẵn trên đĩa
        if isinstance(self.source, str) and os.path.exists(self.source):
            if self.source.lower().endswith((".jpg", ".jpeg", ".png", ".bmp")):
                self.static_frame = cv2.imread(self.source)
                print(f"[FrameGrabber] Đã nạp ảnh mẫu tĩnh: {self.source}")
            else:
                self.cap = cv2.VideoCapture(self.source)
                self._start_reader_thread()
            return

        # 7. Fallback cuối cùng
        self._fallback_init()

    def _fallback_init(self):
        """Khởi tạo fallback: Thử tìm ảnh mẫu sample_data hoặc chuyển sang Synthetic."""
        base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        candidates = [
            os.path.join(base_dir, "sample_data", "csi_floor_sample.jpg"),
            os.path.join(base_dir, "sample_data", "jetbot_default_cam.jpg"),
        ]
        for c in candidates:
            if os.path.exists(c):
                img = cv2.imread(c)
                if img is not None and img.size > 0:
                    self.static_frame = cv2.resize(img, (self.width, self.height))
                    print(f"[FrameGrabber] Fallback sang ảnh mẫu chuẩn: {os.path.basename(c)}")
                    return

        print("[FrameGrabber] Không tìm thấy ảnh mẫu -> Chuyển sang Synthetic Floor Frame Mode.")
        self.is_synthetic = True

    def _start_reader_thread(self):
        """Bắt đầu luồng đọc liên tục để vứt bỏ buffer cũ (Drop old frames)."""
        self._running = True
        self._thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._thread.start()

    def _reader_loop(self):
        while self._running and self.cap is not None and self.cap.isOpened():
            ret, frame = self.cap.read()
            if ret and frame is not None:
                if frame.shape[1] != self.width or frame.shape[0] != self.height:
                    frame = cv2.resize(frame, (self.width, self.height))
                with self._frame_lock:
                    self._latest_frame = frame
            else:
                time.sleep(0.01)

    def read(self) -> Tuple[bool, Optional[np.ndarray]]:
        """
        Đọc khung hình mới nhất. Trả về: (is_success, frame_bgr).
        """
        if self.static_frame is not None:
            return True, self.static_frame.copy()

        if self.is_synthetic:
            return True, self.generate_synthetic_floor_frame()

        if self._running and self._latest_frame is not None:
            with self._frame_lock:
                return True, self._latest_frame.copy()

        if self.cap is not None and self.cap.isOpened():
            ret, frame = self.cap.read()
            if ret and frame is not None:
                if frame.shape[1] != self.width or frame.shape[0] != self.height:
                    frame = cv2.resize(frame, (self.width, self.height))
                return True, frame

        return False, None

    def generate_synthetic_floor_frame(self) -> np.ndarray:
        """Sinh khung hình sàn gạch men giả lập kèm vật cản sàn để test offline."""
        frame = np.ones((self.height, self.width, 3), dtype=np.uint8) * 220
        # Vẽ các đường ron gạch phối cảnh nghiêng
        for y in range(int(self.height * 0.55), self.height, 40):
            cv2.line(frame, (0, y), (self.width, y), (180, 180, 180), 1)
        # Vẽ vật cản sàn nhỏ màu tối sát mép dưới (mô phỏng dây điện/dép < 16cm)
        cv2.rectangle(frame, (280, 410), (360, 450), (40, 40, 40), -1)
        cv2.putText(frame, "SYNTHETIC_FLOOR_OBS", (250, 400), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 200), 1)
        return frame

    def release(self):
        """Dừng luồng và giải phóng tài nguyên phần cứng."""
        self._running = False
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=0.5)
        if self.cap is not None:
            try:
                self.cap.release()
            except Exception:
                pass
            self.cap = None

    def __del__(self):
        self.release()


# Alias tương thích ngược với code cũ
CameraLoader = FrameGrabber
