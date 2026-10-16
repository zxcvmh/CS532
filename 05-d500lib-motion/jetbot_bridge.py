#!/usr/bin/env python3
"""
JetBot Hardware Bridge (Full Hardware: CSI Camera + D500 LiDAR + TensorRT YOLOv8n)
Hardware Configuration:
  - Robot:  NVIDIA JetBot (PCA9685 Motor Controller)
  - Camera: CSI Ribbon Cable (IMX219 via GStreamer nvarguscamerasrc)
  - LiDAR:  D500 / LD19 (47-byte packet sync @ 230400 baud, 360° Sliding Cache)
  - AI:     TensorRT 7.1.3 YOLOv8n Engine (/home/jetbot/yolov8n.engine)
"""

import sys
import os
import subprocess
import time
import math
import json
import base64
import asyncio
import threading
import ctypes
import numpy as np
from http.server import HTTPServer, BaseHTTPRequestHandler
import socketserver

# Import Floor IPM transformer for LiDAR blind spot compensation
try:
    from floor_ipm import FloorIPMTransformer
except ImportError:
    from .floor_ipm import FloorIPMTransformer

# ─────────────────────────────────────────────────────────────
# DEPENDENCY CHECKS
# ─────────────────────────────────────────────────────────────
try:
    import cv2
except ImportError:
    print("[ERROR] OpenCV (cv2) not found!")
    sys.exit(1)

try:
    import websockets
except ImportError:
    print("[ERROR] websockets not found! Run: pip3 install websockets")
    sys.exit(1)

try:
    import serial
except ImportError:
    print("[ERROR] pyserial not found! Run: pip3 install pyserial")
    sys.exit(1)

try:
    import tensorrt as trt
    HAS_TRT = True
except ImportError:
    HAS_TRT = False
    print("[WARN] TensorRT not found. AI detections disabled.")


# ─────────────────────────────────────────────────────────────
# MOTOR DRIVER (Pure I2C / Adafruit MotorHAT, NO PyTorch needed)
# ─────────────────────────────────────────────────────────────
HAS_MOTOR = False
left_motor_dev = None
right_motor_dev = None

try:
    from Adafruit_MotorHAT import Adafruit_MotorHAT
    motor_hat = Adafruit_MotorHAT(i2c_bus=1)
    left_motor_dev = motor_hat.getMotor(1)
    right_motor_dev = motor_hat.getMotor(2)
    HAS_MOTOR = True
    print("[OK] Adafruit MotorHAT initialized (No torch needed).")
except Exception:
    try:
        from jetbot import Robot
        jetbot_robot = Robot()
        HAS_MOTOR = True
        print("[OK] NVIDIA JetBot Robot driver initialized.")
    except Exception as e:
        HAS_MOTOR = False
        print(f"[INFO] Motor running in simulation log mode ({e}).")

def read_ina219_battery():
    """Reads 3S Li-ion battery voltage from INA219 sensor via I2C (Bus 1, Addr 0x41 or 0x40)."""
    for addr in (0x41, 0x40):
        try:
            from smbus2 import SMBus
            with SMBus(1) as bus:
                data = bus.read_i2c_block_data(addr, 0x02, 2)
                raw_v = (data[0] << 8) | data[1]
                voltage = (raw_v >> 3) * 0.004
                if 8.5 <= voltage <= 13.5:
                    pct = max(0.0, min(100.0, (voltage - 9.6) / (12.6 - 9.6) * 100.0))
                    return round(pct, 1), round(voltage, 2)
        except Exception:
            pass
    try:
        from ina219 import INA219
        ina = INA219(0.1, address=0x41, busnum=1)
        ina.configure()
        v = ina.voltage()
        if 8.5 <= v <= 13.5:
            pct = max(0.0, min(100.0, (v - 9.6) / 3.0 * 100.0))
            return round(pct, 1), round(v, 2)
    except Exception:
        pass
    return None, None



# ─────────────────────────────────────────────────────────────
# CONFIGURATION
# ─────────────────────────────────────────────────────────────
SERVER_HOST = (
    sys.argv[1] if len(sys.argv) > 1 and not sys.argv[1].startswith("-")
    else (os.environ.get("DASHBOARD_HOST") or os.environ.get("DASHBOARD_IP") or "192.168.1.8")
)
SERVER_PORT = int(sys.argv[2]) if len(sys.argv) > 2 and sys.argv[2].isdigit() else int(os.environ.get("DASHBOARD_PORT", "8000"))
WS_URL = f"ws://{SERVER_HOST}:{SERVER_PORT}/ws/robot"

LIDAR_PORT = os.environ.get("LIDAR_PORT", "/dev/ttyUSB0")
LIDAR_BAUDRATE = 230400

ROBOT_DYNAMICS = {
    "hardware": "JetBot-GA25-370",
    "floor_surface": "glossy_ceramic_tile",
    "tested_arena": "2m_x_1m",
    "min_drive_speed": 0.12,
    "cruise_speed": 0.14,
    "trim_factor": 1.015,
    "turn_speed": 0.14,
    "turn_time_90_deg_sec": 4.5,
    "turn_time_180_deg_sec": 9.0,
    "soft_start_ramp_sec": 0.4,
    "max_safe_speed_mps": 0.14,
    "max_angular_speed_rads": 0.35,
    "lidar_yaw_offset_deg": 0.0,
    "note": "Cau hinh chuan an toan san gach men va lidar D500 quay mui xe (offset 0 deg)",
    "motor_left_polarity": -1,
    "motor_right_polarity": -1
}

for _p in (
    "/home/jetbot/03-MinhHieu/test-hardware/robot_dynamics.json",
    "/home/jetbot/setup/robot_dynamics.json",
    os.path.join(os.path.dirname(__file__), "test-hardware", "robot_dynamics.json"),
    os.path.join(os.path.dirname(__file__), "..", "setup", "robot_dynamics.json"),
    "03-MinhHieu/test-hardware/robot_dynamics.json",
    "setup/robot_dynamics.json"
):
    try:
        if os.path.isfile(_p):
            with open(_p, "r", encoding="utf-8") as f:
                ROBOT_DYNAMICS.update(json.load(f))
            print(f"[BRIDGE] Loaded robot dynamics from {_p}")
            break
    except Exception:
        pass

LIDAR_YAW_OFFSET_DEG = float(ROBOT_DYNAMICS.get("lidar_yaw_offset_deg", 0.0))
MIN_DRIVE_SPEED = float(ROBOT_DYNAMICS.get("min_drive_speed", 0.12))
CRUISE_SPEED = float(ROBOT_DYNAMICS.get("cruise_speed", 0.14))
TRIM_FACTOR = float(ROBOT_DYNAMICS.get("trim_factor", 1.0145555))
TURN_SPEED = float(ROBOT_DYNAMICS.get("turn_speed", 0.14))
TURN_TIME_90_SEC = float(ROBOT_DYNAMICS.get("turn_time_90_deg_sec", 4.5))
SOFT_START_RAMP_SEC = float(ROBOT_DYNAMICS.get("soft_start_ramp_sec", 0.4))
MAX_SAFE_SPEED_MPS = float(ROBOT_DYNAMICS.get("max_safe_speed_mps", 0.14))
MAX_ANGULAR_SPEED_RADS = float(ROBOT_DYNAMICS.get("max_angular_speed_rads", 0.35))
MOTOR_LEFT_POLARITY = int(ROBOT_DYNAMICS.get("motor_left_polarity", -1))
MOTOR_RIGHT_POLARITY = int(ROBOT_DYNAMICS.get("motor_right_polarity", -1))

CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CAMERA_FPS = 20
CAMERA_FLIP = int(os.environ.get("CAMERA_FLIP", "0"))  # Set CAMERA_FLIP=2 if upside down

# Priority: YOLO26-n TensorRT Engine (2026), fallback to YOLOv8n
ENGINE_CANDIDATES = [
    os.environ.get("YOLO_ENGINE", ""),
    "/home/jetbot/yolo26n.engine",
    os.path.join(os.path.dirname(__file__), "yolo26n.engine"),
    "/home/jetbot/yolov8n.engine",
    os.path.join(os.path.dirname(__file__), "yolov8n.engine")
]
ENGINE_PATH = next((c for c in ENGINE_CANDIDATES if c and os.path.exists(c)), "/home/jetbot/yolo26n.engine")

COCO_CLASSES = [
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat",
    "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat",
    "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack",
    "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball",
    "kite", "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket",
    "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple",
    "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair",
    "couch", "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse",
    "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink",
    "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier", "toothbrush"
]

YOLO_CONF_THRESHOLD = float(os.environ.get("YOLO_CONF_THRESHOLD", "0.45"))
DEFAULT_NAV_CLASSES = "person,chair,couch,backpack,suitcase,dog,cat"
TARGET_CLASSES = set(filter(None, os.environ.get("YOLO_CLASSES", DEFAULT_NAV_CLASSES).split(",")))


# ─────────────────────────────────────────────────────────────
# 1. CSI CAMERA (Ribbon Cable - nvarguscamerasrc)
# ─────────────────────────────────────────────────────────────
def get_csi_pipeline(width=640, height=480, fps=20, flip=0):
    return (
        f"nvarguscamerasrc ! "
        f"video/x-raw(memory:NVMM), width=1280, height=720, format=NV12, framerate={fps}/1 ! "
        f"nvvidconv flip-method={flip} ! "
        f"video/x-raw, width={width}, height={height}, format=BGRx ! "
        f"videoconvert ! "
        f"video/x-raw, format=BGR ! appsink max-buffers=1 drop=true sync=false"
    )

def try_restart_nvargus_daemon():
    """Attempts to restart nvargus-daemon via passwordless sudo if CaptureSession failed."""
    try:
        res = subprocess.run(["sudo", "-n", "systemctl", "restart", "nvargus-daemon"],
                             capture_output=True, timeout=5)
        if res.returncode == 0:
            print("[OK] nvargus-daemon restarted automatically via sudo systemctl.")
            time.sleep(1.5)  # Allow daemon IPC socket to initialize
            return True
        else:
            print("[INFO] sudo without password not configured for systemctl restart nvargus-daemon.")
    except Exception as e:
        print(f"[INFO] Systemctl restart check: {e}")
    return False

class CSICameraThread(threading.Thread):
    def __init__(self):
        super().__init__(daemon=True)
        self.cap = None
        self.latest_frame = None
        self.latest_jpeg_b64 = ""
        self.running = True
        self._init_failures = 0

    def _release_cap(self):
        """Cleanly releases and deletes any existing VideoCapture instance to prevent GStreamer memory corruption."""
        if self.cap is not None:
            try:
                self.cap.release()
            except Exception:
                pass
            self.cap = None
        # Give GStreamer bus and kernel driver time to fully unload pipeline
        time.sleep(0.3)

    def _open_camera(self):
        self._release_cap()
        print(f"[INFO] Opening CSI Camera (flip={CAMERA_FLIP})...")
        pipeline = get_csi_pipeline(CAMERA_WIDTH, CAMERA_HEIGHT, CAMERA_FPS, CAMERA_FLIP)
        try:
            cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
            if cap and cap.isOpened():
                self._init_failures = 0
                return cap
            else:
                if cap is not None:
                    try:
                        cap.release()
                    except Exception:
                        pass
        except Exception as e:
            print(f"[WARN] Error initializing GStreamer VideoCapture: {e}")

        # Fallback to USB camera ONLY if explicitly configured by env var
        if os.environ.get("USE_USB_CAMERA") == "1":
            print("[INFO] Trying fallback USB /dev/video0...")
            try:
                cap = cv2.VideoCapture(0)
                if cap and cap.isOpened():
                    self._init_failures = 0
                    return cap
                if cap is not None:
                    cap.release()
            except Exception:
                pass

        self._init_failures += 1
        return None

    def run(self):
        # Initial camera open attempt
        self.cap = self._open_camera()
        if self.cap and self.cap.isOpened():
            print("[OK] Camera opened successfully!")
        else:
            print("[WARN] Camera could not be opened initially. Entering resilient reconnect loop...")
            # Try auto-restarting nvargus-daemon on initial failure
            if try_restart_nvargus_daemon():
                self.cap = self._open_camera()

        # Quality 48 reduces base64 payload size by ~4x without visible degradation, eliminating WiFi lag
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 48]
        failed_reads = 0

        while self.running:
            if not self.cap or not self.cap.isOpened():
                # Progressive backoff based on failure count
                if self._init_failures <= 3:
                    wait_time = 2.0
                elif self._init_failures <= 6:
                    wait_time = 5.0
                    print("[WARN] nvargus-daemon may be hung. If needed, run on Jetson: 'sudo systemctl restart nvargus-daemon'")
                else:
                    wait_time = 15.0  # Slow backoff to prevent GStreamer C-level thrashing

                time.sleep(wait_time)
                if not self.running:
                    break

                self.cap = self._open_camera()
                if self.cap and self.cap.isOpened():
                    print("[OK] Camera reconnected successfully!")
                    failed_reads = 0
                continue

            try:
                ret, frame = self.cap.read()
            except Exception as e:
                print(f"[WARN] Exception during cap.read(): {e}")
                ret, frame = False, None

            if ret and frame is not None and frame.size > 0:
                failed_reads = 0
                self.latest_frame = frame
                # Video is streamed independently via MJPEG HTTP server (/api/stream/mjpeg).
                # Continuous base64 encoding removed to keep telemetry payload lightweight (<10KB) and eliminate CPU lag.
                # No sleep here: cap.read() is hardware-synchronized to 20 FPS (prevents FIFO queue buildup)
            else:
                failed_reads += 1
                if failed_reads > 15:
                    print("[WARN] Camera stream interrupted. Safely releasing GStreamer pipeline...")
                    self._release_cap()
                    failed_reads = 0
                    # On stream stall, attempt to restart daemon once if possible
                    if self._init_failures == 0:
                        try_restart_nvargus_daemon()
                else:
                    time.sleep(0.05)

    def stop(self):
        self.running = False
        self._release_cap()


# ─────────────────────────────────────────────────────────────
# 1.1 NATIVE MJPEG CAMERA HTTP SERVER (Port 8080 - Decoupled from WebSocket)
# ─────────────────────────────────────────────────────────────
class MJPEGStreamHandler(BaseHTTPRequestHandler):
    camera_thread = None

    def do_GET(self):
        if self.path in ('/api/stream/mjpeg', '/video_feed', '/stream', '/'):
            self.send_response(200)
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 50]
            while True:
                try:
                    if self.camera_thread and self.camera_thread.latest_frame is not None:
                        frame = self.camera_thread.latest_frame
                        ret, jpeg = cv2.imencode('.jpg', frame, encode_param)
                        if ret:
                            self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + jpeg.tobytes() + b'\r\n')
                    time.sleep(0.066)  # ~15 FPS native stream
                except Exception:
                    break
        elif self.path == '/snapshot':
            if self.camera_thread and self.camera_thread.latest_frame is not None:
                ret, jpeg = cv2.imencode('.jpg', self.camera_thread.latest_frame, [int(cv2.IMWRITE_JPEG_QUALITY), 65])
                if ret:
                    self.send_response(200)
                    self.send_header('Content-Type', 'image/jpeg')
                    self.send_header('Access-Control-Allow-Origin', '*')
                    self.end_headers()
                    self.wfile.write(jpeg.tobytes())
                    return
            self.send_response(503)
            self.end_headers()
        else:
            self.send_response(404)
            self.end_headers()

    def log_message(self, format, *args):
        pass  # Suppress continuous access logs in console


def start_camera_http_server(camera_thread: CSICameraThread, port: int = 8080):
    MJPEGStreamHandler.camera_thread = camera_thread
    class ThreadedHTTPServer(socketserver.ThreadingMixIn, HTTPServer):
        daemon_threads = True
        allow_reuse_address = True
    try:
        server = ThreadedHTTPServer(('0.0.0.0', port), MJPEGStreamHandler)
        print(f"[OK] Native Camera MJPEG Stream active at http://0.0.0.0:{port}/api/stream/mjpeg (also /video_feed)")
        t = threading.Thread(target=server.serve_forever, daemon=True)
        t.start()
    except Exception as e:
        print(f"[WARN] Could not bind camera HTTP stream on port {port}: {e}")


# ─────────────────────────────────────────────────────────────
# 2. OFFICIAL D500 LIDAR DRIVER (Table-Driven CRC-8 + Revolution Sync)
# ─────────────────────────────────────────────────────────────
from d500_driver import D500OfficialDriver

class D500LidarThread:
    """Official LDROBOT D500 LiDAR thread wrapper with CRC-8 and revolution sync."""
    def __init__(self, port="/dev/ttyUSB0", baudrate=230400):
        self.driver = D500OfficialDriver(
            port=port,
            baudrate=baudrate,
            min_distance_m=0.12,
            max_distance_m=6.00,
            min_intensity=15,
            yaw_offset_deg=LIDAR_YAW_OFFSET_DEG
        )
        self.port = port
        self.baudrate = baudrate
        self.running = True

    def start(self):
        self.driver.start()

    def run(self):
        self.driver.start()

    def stop(self):
        self.running = False
        self.driver.stop()

    @property
    def cache_lock(self):
        """Thread-safe lock proxy for backward compatibility with 20Hz reflex tasks."""
        return self.driver.lock

    @property
    def latest_scan(self):
        """Thread-safe clean 360-degree scan list [[angle, dist], ...]"""
        return self.driver.get_scan_list()

    @property
    def scan_cache(self):
        """Thread-safe dictionary view for backward-compatible lookups"""
        ranges = self.driver.get_ranges_360()
        now = time.time()
        return {ang: (d, now) for ang, d in enumerate(ranges) if d > 0.05}



# ─────────────────────────────────────────────────────────────
# 3. TENSORRT YOLOV8n INFERENCE (Pure ctypes on Jetson Nano GPU)
# ─────────────────────────────────────────────────────────────
class TensorRTYOLOThread(threading.Thread):
    def __init__(self, camera_thread: CSICameraThread, lidar_thread: D500LidarThread, engine_path: str):
        super().__init__(daemon=True)
        self.camera = camera_thread
        self.lidar = lidar_thread
        self.engine_path = engine_path
        self.latest_detections = []
        self.latest_low_obstacles = []
        self.running = True

        # Khởi tạo Floor IPM Transformer bù điểm mù LiDAR
        self.ipm = FloorIPMTransformer()
        self.safety_margins = {
            "person": {"front_m": 0.80, "side_m": 0.40, "back_m": 0.30},
            "chair": 0.30,
            "backpack": 0.20,
            "bottle": 0.15,
            "default": 0.25
        }

    def compute_dynamic_proxemics(self, azimuth_deg: float, cls_name: str):
        c_lower = cls_name.lower()
        margin_cfg = self.safety_margins.get(c_lower, self.safety_margins.get("default", 0.25))
        if c_lower == "person":
            if isinstance(margin_cfg, dict):
                front_m = float(margin_cfg.get("front_m", 0.80))
                side_m = float(margin_cfg.get("side_m", 0.40))
                back_m = float(margin_cfg.get("back_m", 0.30))
            else:
                front_m, side_m, back_m = 0.80, 0.40, 0.30
            theta_rad = math.radians(abs(azimuth_deg))
            denom = math.sqrt((side_m * math.cos(theta_rad)) ** 2 + (front_m * math.sin(theta_rad)) ** 2)
            effective = (front_m * side_m) / max(denom, 1e-6)
            return round(effective, 2), {"front_m": front_m, "side_m": side_m, "back_m": back_m}
        if isinstance(margin_cfg, (int, float)):
            return float(margin_cfg), None
        return 0.25, None

    def run(self):
        if os.environ.get("NO_YOLO") == "1":
            print("[INFO] AI YOLO disabled via NO_YOLO=1.")
            return

        if not HAS_TRT or not os.path.exists(self.engine_path):
            print(f"[WARN] TensorRT engine not found at {self.engine_path}.")
            return

        print(f"[INFO] Loading TensorRT YOLO engine from {self.engine_path}...")
        try:
            cudart = ctypes.CDLL('libcudart.so')

            # 64-bit ARM64 ctypes prototypes (Prevents Segfault)
            cudart.cudaMalloc.argtypes = [ctypes.POINTER(ctypes.c_void_p), ctypes.c_size_t]
            cudart.cudaMalloc.restype = ctypes.c_int
            cudart.cudaMemcpy.argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_size_t, ctypes.c_int]
            cudart.cudaMemcpy.restype = ctypes.c_int

            TRT_LOGGER = trt.Logger(trt.Logger.WARNING)
            with open(self.engine_path, 'rb') as f:
                runtime = trt.Runtime(TRT_LOGGER)
                engine = runtime.deserialize_cuda_engine(f.read())
            self.engine = engine

            context = engine.create_execution_context()
            print("[OK] YOLO TensorRT Engine active on GPU!")

            # Input: images (1, 3, 640, 640) float32
            # Output: output0 (1, 84, 8400) float32
            input_bytes = 1 * 3 * 640 * 640 * 4
            output_bytes = 1 * 84 * 8400 * 4

            d_input = ctypes.c_void_p()
            d_output = ctypes.c_void_p()
            cudart.cudaMalloc(ctypes.byref(d_input), input_bytes)
            cudart.cudaMalloc(ctypes.byref(d_output), output_bytes)

            h_output = np.zeros((1, 84, 8400), dtype=np.float32)
            bindings = [int(d_input.value), int(d_output.value)]
            last_log_time = 0.0

            while self.running:
                try:
                    frame = self.camera.latest_frame
                    if frame is None:
                        time.sleep(0.04)
                        continue

                    h_orig, w_orig = frame.shape[:2]
                    img_resized = cv2.resize(frame, (640, 640))
                    img_rgb = cv2.cvtColor(img_resized, cv2.COLOR_BGR2RGB)
                    img_norm = img_rgb.astype(np.float32) / 255.0
                    img_chw = np.transpose(img_norm, (2, 0, 1))
                    h_input = np.ascontiguousarray(np.expand_dims(img_chw, axis=0))

                    # Copy input to GPU (64-bit safe data_as)
                    cudart.cudaMemcpy(d_input, h_input.ctypes.data_as(ctypes.c_void_p), input_bytes, 1)

                    # Run inference on Jetson GPU
                    context.execute_v2(bindings)

                    # Copy output from GPU (64-bit safe data_as)
                    cudart.cudaMemcpy(h_output.ctypes.data_as(ctypes.c_void_p), d_output, output_bytes, 2)

                    # ── 1. GIẢI MÃ TENSOR VECTOR HÓA 100% NUMPY (Độ trễ < 1ms) ──
                    preds = h_output[0].transpose(1, 0)  # (8400, 84)
                    boxes_raw = preds[:, :4]
                    scores_matrix = preds[:, 4:]

                    class_ids = np.argmax(scores_matrix, axis=1)
                    confidences = np.max(scores_matrix, axis=1)

                    mask = confidences >= YOLO_CONF_THRESHOLD
                    boxes = []
                    filt_confs = []
                    filt_cids = []

                    if np.any(mask):
                        f_boxes = boxes_raw[mask]
                        filt_confs = confidences[mask].tolist()
                        filt_cids = class_ids[mask].tolist()

                        scale_x = w_orig / 640.0
                        scale_y = h_orig / 640.0
                        cx = f_boxes[:, 0]
                        cy = f_boxes[:, 1]
                        bw = f_boxes[:, 2]
                        bh = f_boxes[:, 3]

                        x1 = np.clip((cx - bw / 2.0) * scale_x, 0, w_orig - 1).astype(np.int32)
                        y1 = np.clip((cy - bh / 2.0) * scale_y, 0, h_orig - 1).astype(np.int32)
                        box_w = np.clip(bw * scale_x, 1, w_orig).astype(np.int32)
                        box_h = np.clip(bh * scale_y, 1, h_orig).astype(np.int32)
                        boxes = np.stack([x1, y1, box_w, box_h], axis=1).tolist()

                    dets = []
                    if boxes:
                        indices = cv2.dnn.NMSBoxes(boxes, filt_confs, YOLO_CONF_THRESHOLD, 0.45)
                        now_lidar = time.time()
                        if len(indices) > 0:
                            indices = np.array(indices).flatten()
                            for idx in indices:
                                i = int(idx)
                                cls_idx = filt_cids[i]
                                cname = COCO_CLASSES[cls_idx] if cls_idx < len(COCO_CLASSES) else f"obj_{cls_idx}"

                                # Lọc bỏ các vật thể không có ý nghĩa dẫn đường / né tránh
                                if TARGET_CLASSES and cname not in TARGET_CLASSES:
                                    continue

                                bx, by, bw, bh = boxes[i]

                                # ── SENSOR FUSION: Camera Angle + 2D LiDAR Laser Distance ──
                                x_center = bx + bw / 2.0
                                # Góc phương vị chuẩn Robot Frame (CCW): Trái (+), Phải (-) theo FOV 160°
                                azimuth_deg = round((((w_orig / 2.0) - x_center) / (w_orig / 2.0)) * 80.0, 1)

                                # Cửa sổ góc quét LiDAR tương ứng
                                min_ang = azimuth_deg - 5.0
                                max_ang = azimuth_deg + 5.0

                                lidar_hits = []
                                if self.lidar and hasattr(self.lidar, "scan_cache"):
                                    with self.lidar.cache_lock:
                                        cache_snapshot = list(self.lidar.scan_cache.items())
                                    for raw_ang, (dist_m, ts) in cache_snapshot:
                                        if now_lidar - ts > 0.8:
                                            continue
                                        signed_ang = (raw_ang - 360.0) if raw_ang > 180.0 else float(raw_ang)
                                        if min_ang <= signed_ang <= max_ang:
                                            if 0.12 <= dist_m <= 10.0:
                                                lidar_hits.append(dist_m)

                                if lidar_hits:
                                    dist = round(min(lidar_hits), 2)
                                else:
                                    # IPM Metric distance estimation từ điểm chạm đáy mặt sàn
                                    est_x, est_y = self.ipm.image_pixel_to_robot_metric(x_center, by + bh)
                                    if est_x < 900.0:
                                        dist = round(math.hypot(est_x, est_y), 2)
                                    else:
                                        dist = round(220.0 / max(bh, 1), 2)

                                margin, prox_env = self.compute_dynamic_proxemics(azimuth_deg, cname)

                                det_obj = {
                                    "class_id": cls_idx,
                                    "class_name": cname,
                                    "confidence": round(filt_confs[i], 2),
                                    "azimuth_deg": azimuth_deg,
                                    "estimated_dist": dist,
                                    "safety_margin_m": margin,
                                    "bbox": [bx, by, bx + bw, by + bh]
                                }
                                if prox_env is not None:
                                    det_obj["proxemics"] = prox_env

                                dets.append(det_obj)

                    self.latest_detections = dets

                    # ── 2. TRÍCH XUẤT VẬT CẢN SÁT SÀN BÙ ĐIỂM MÙ LIDAR (Floor IPM) ──
                    # Vô hiệu hóa Floor IPM: tránh bóng phản chiếu sàn gạch men gây ra báo động va chạm giả
                    self.latest_low_obstacles = []

                    # Terminal visual feedback when objects detected
                    if dets:
                        now_t = time.time()
                        if now_t - last_log_time > 1.2:
                            names = ", ".join(f"{d['class_name']} ({int(d['confidence']*100)}% | {d['estimated_dist']}m | {d['azimuth_deg']}° | Bubble: {d['safety_margin_m']}m)" for d in dets[:3])
                            print(f"[AI YOLO + LiDAR] {names}")
                            last_log_time = now_t

                    time.sleep(0.04)  # ~25 FPS inference rate

                except Exception as e:
                    time.sleep(0.05)

        except Exception as e:
            print(f"[WARN] TensorRT setup error: {e}")

    def stop(self):
        self.running = False


# ─────────────────────────────────────────────────────────────
# ─────────────────────────────────────────────────────────────
# 4. MOTOR CONTROL (NVIDIA JetBot) & DEADMAN WATCHDOG
# ─────────────────────────────────────────────────────────────
_last_cmd_time = 0.0
_is_motor_moving = False
_current_pwm_l = 0.0
_current_pwm_r = 0.0
_last_slew_time = 0.0

# Slew-Rate Filter State for Smooth Acceleration (Soft-Start Ramp)
_active_cmd_v = 0.0
_active_cmd_w = 0.0
_current_spin_pwm = 0.0
_last_vel_cmd_time = 0.0

def set_raw_motor_speeds(target_l: float, target_r: float):
    """Differential drive normalized motor control [-1.0 .. 1.0].
    Applies:
      1. Trim compensation factor (trim_factor = 1.0145555) to ensure straight line drive
      2. Physical wiring polarity correction
      3. Safe motor dispatch to Adafruit MotorHAT or jetbot.Robot
    """
    global _current_pwm_l, _current_pwm_r, _is_motor_moving

    _current_pwm_l = max(-1.0, min(1.0, float(target_l)))
    _current_pwm_r = max(-1.0, min(1.0, float(target_r)))

    # Apply trim balancing and polarity correction
    # Left motor is scaled by TRIM_FACTOR (calibrated 1.0145555)
    hw_left = _current_pwm_l * TRIM_FACTOR * MOTOR_LEFT_POLARITY
    hw_right = _current_pwm_r * MOTOR_RIGHT_POLARITY

    # Clamp duty cycles
    hw_left = max(-1.0, min(1.0, hw_left))
    hw_right = max(-1.0, min(1.0, hw_right))

    _is_motor_moving = (abs(_current_pwm_l) > 0.01 or abs(_current_pwm_r) > 0.01)

    if HAS_MOTOR:
        try:
            if left_motor_dev and right_motor_dev:
                l_pwm = int(round(min(255, max(0, abs(hw_left) * 255))))
                r_pwm = int(round(min(255, max(0, abs(hw_right) * 255))))
                left_motor_dev.setSpeed(l_pwm)
                right_motor_dev.setSpeed(r_pwm)
                left_motor_dev.run(1 if hw_left >= 0 else 2)
                right_motor_dev.run(1 if hw_right >= 0 else 2)
            elif 'jetbot_robot' in globals() and jetbot_robot:
                jetbot_robot.set_motors(hw_left, hw_right)
        except Exception as e:
            print(f"[ERROR] Motor drive: {e}")
    else:
        if _is_motor_moving:
            print(f"[MOTOR RAW] L={_current_pwm_l:+.3f} (hw={hw_left:+.3f}), R={_current_pwm_r:+.3f} (hw={hw_right:+.3f})")

def set_motor_speeds(linear: float, angular: float):
    """Convert linear (m/s) and angular (rad/s) to calibrated motor outputs.
    Adheres strictly to physical dynamics on glossy ceramic tile:
      1. Slew-Rate Soft-Start: Smoothly ramps speed up from 0 to target over SOFT_START_RAMP_SEC (0.4s)
         without resetting on continuous streaming commands.
      2. True Bidirectional Support: Full support for both FORWARD (linear > 0) and BACKWARD (linear < 0).
      3. Precise In-Place Turn: Spins at TURN_SPEED (0.16) with gentle spin acceleration.
      4. Differential Steering: Smooth curving without motor stall deadband.
      5. Trim Compensation: Keeps straight trajectory without lateral veer.
    """
    global _active_cmd_v, _active_cmd_w, _last_vel_cmd_time
    now = time.time()
    dt = 0.05
    if _last_vel_cmd_time > 0:
        dt = max(0.01, min(0.2, now - _last_vel_cmd_time))
    _last_vel_cmd_time = now

    # 1. Full stop
    if abs(linear) < 0.005 and abs(angular) < 0.02:
        _active_cmd_v = 0.0
        _active_cmd_w = 0.0
        set_raw_motor_speeds(0.0, 0.0)
        return

    # Slew-Rate Limiting for Linear Speed (Acc: cruise_speed / soft_start_ramp_sec)
    ramp_duration = max(0.1, SOFT_START_RAMP_SEC)  # 0.4s
    max_accel = max(0.2, CRUISE_SPEED / ramp_duration)  # ~0.35 m/s^2
    max_step_v = max_accel * dt

    # Smooth linear velocity tracking
    diff_v = linear - _active_cmd_v
    if abs(diff_v) <= max_step_v:
        _active_cmd_v = linear
    else:
        _active_cmd_v += math.copysign(max_step_v, diff_v)

    # Slew-Rate Limiting for Angular Speed
    # When transitioning from pivot turn (v=0) to linear drive (v>0), snap angular velocity immediately
    # to prevent residual 3.14 rad/s pivot turn rate from jerking the robot into the wall
    if abs(linear) > 0.01 and abs(_active_cmd_v) < 0.01:
        _active_cmd_w = angular
    else:
        max_step_w = (1.5 / 0.25) * dt
        diff_w = angular - _active_cmd_w
        if abs(diff_w) <= max_step_w:
            _active_cmd_w = angular
        else:
            _active_cmd_w += math.copysign(max_step_w, diff_w)

    v = _active_cmd_v
    w = _active_cmd_w
    # 2. In-place turn (Stop-Turn-Go / Soft Pivot Spin to prevent SLAM slip drift)
    global _current_spin_pwm
    if abs(v) < 0.01 and abs(w) >= 0.02:
        # Scale target spin PWM proportionally with |w| relative to max angular speed
        max_w = float(ROBOT_DYNAMICS.get("max_angular_speed_rads", 0.50))
        w_ratio = min(1.0, abs(w) / max_w)

        # Soft spin bounds: minimum PWM to overcome gearbox stiction (MIN_DRIVE_SPEED), up to TURN_SPEED
        min_spin_pwm = MIN_DRIVE_SPEED if "MIN_DRIVE_SPEED" in globals() else 0.10
        max_spin_pwm = TURN_SPEED if "TURN_SPEED" in globals() else 0.14
        target_spin = min_spin_pwm + (max_spin_pwm - min_spin_pwm) * w_ratio

        # Slew-Rate Limiting for Spin PWM (gentle ramp: max 0.40 PWM units/sec to prevent rotational slip)
        spin_slew_rate = 0.40
        max_spin_step = spin_slew_rate * dt
        diff_spin = target_spin - _current_spin_pwm
        if abs(diff_spin) <= max_spin_step:
            _current_spin_pwm = target_spin
        else:
            _current_spin_pwm += math.copysign(max_spin_step, diff_spin)

        u_spin = _current_spin_pwm

        if w > 0:
            # Turn Left (CCW)
            set_raw_motor_speeds(-u_spin, u_spin)
        else:
            # Turn Right (CW)
            set_raw_motor_speeds(u_spin, -u_spin)
        return
    else:
        _current_spin_pwm = 0.0

    # 3. Linear drive (Forward or Backward) with True Differential Drive Kinematics
    # Unicycle Kinematics: v_L = v - w * (L/2), v_R = v + w * (L/2) with track width L = 0.12m
    half_track = 0.06
    vl = v - (w * half_track)
    vr = v + (w * half_track)

    def velocity_to_pwm(wheel_v: float) -> float:
        if abs(wheel_v) < 0.005:
            return 0.0
        sign = 1.0 if wheel_v >= 0 else -1.0
        v_norm = min(1.0, abs(wheel_v) / max(0.05, MAX_SAFE_SPEED_MPS))
        # Deadband compensation: map [0, 1] to [MIN_DRIVE_SPEED, CRUISE_SPEED]
        return sign * (MIN_DRIVE_SPEED + (CRUISE_SPEED - MIN_DRIVE_SPEED) * v_norm)

    target_l = max(-1.0, min(1.0, velocity_to_pwm(vl)))
    target_r = max(-1.0, min(1.0, velocity_to_pwm(vr)))

    set_raw_motor_speeds(target_l, target_r)

def stop_motors():
    global _is_motor_moving, _current_pwm_l, _current_pwm_r, _last_slew_time
    global _active_cmd_v, _active_cmd_w, _current_spin_pwm, _last_vel_cmd_time
    was_moving = _is_motor_moving
    _current_pwm_l = 0.0
    _current_pwm_r = 0.0
    _last_slew_time = 0.0
    _active_cmd_v = 0.0
    _active_cmd_w = 0.0
    _current_spin_pwm = 0.0
    _last_vel_cmd_time = 0.0
    if HAS_MOTOR:
        try:
            if left_motor_dev and right_motor_dev:
                left_motor_dev.run(4)  # 4=RELEASE
                right_motor_dev.run(4)
            elif 'jetbot_robot' in globals() and jetbot_robot:
                jetbot_robot.stop()
        except Exception:
            pass
    _is_motor_moving = False
    if was_moving:
        print("[MOTOR] STOPPED")


# ─────────────────────────────────────────────────────────────
# 5. WEBSOCKET BRIDGE
# ─────────────────────────────────────────────────────────────
async def bridge_loop(camera: CSICameraThread, lidar: D500LidarThread, yolo: TensorRTYOLOThread):
    global _last_cmd_time, _is_motor_moving
    print(f"\n=======================================================")
    print(f"  JetBot Connecting to Dashboard at:")
    print(f"  {WS_URL}")
    print(f"=======================================================\n")

    while True:
        try:
            async with websockets.connect(WS_URL, ping_interval=10, ping_timeout=10) as ws:
                print(f"[SUCCESS] Connected to Dashboard at {WS_URL}!")

                async def send_telemetry():
                    last_bat_read = 0.0
                    current_bat = None
                    while True:
                        now = time.time()
                        if now - last_bat_read >= 3.0:
                            bpct, bv = read_ina219_battery()
                            if bpct is not None:
                                current_bat = bpct
                            last_bat_read = now

                        payload = {
                            "type": "robot_telemetry",
                            "timestamp": now,
                            "lidar": lidar.latest_scan,
                            "detections": yolo.latest_detections,
                            "low_obstacles": getattr(yolo, "latest_low_obstacles", []),
                            "battery": current_bat,
                            "yolo_active": HAS_TRT and getattr(yolo, "engine", None) is not None
                        }
                        await ws.send(json.dumps(payload))
                        await asyncio.sleep(0.045)  # ~22 FPS responsive rate

                async def receive_commands():
                    global _last_cmd_time, _is_motor_moving
                    while True:
                        raw = await ws.recv()
                        msg = json.loads(raw)
                        mtype = str(msg.get("type", "")).strip().lower()
                        cmd = str(msg.get("cmd") or msg.get("command") or "").strip().lower()

                        # UIT Tut 3 command: {"cmd": "drive", "left": f, "right": f}
                        if cmd == "drive":
                            l = float(msg.get("left", 0.0))
                            r = float(msg.get("right", 0.0))
                            set_raw_motor_speeds(l, r)
                            _last_cmd_time = time.time()
                            _is_motor_moving = (abs(l) > 0.01 or abs(r) > 0.01)

                        # CS532 command: {"type": "cmd_vel", "linear": v, "angular": w}
                        elif mtype == "cmd_vel":
                            v = float(msg.get("linear", 0.0))
                            w = float(msg.get("angular", 0.0))
                            set_motor_speeds(v, w)
                            _last_cmd_time = time.time()
                            _is_motor_moving = (abs(v) > 0.01 or abs(w) > 0.01)

                        # Stop / Emergency stop (both protocols)
                        elif cmd in ("stop", "emergency_stop") or mtype in ("stop", "emergency_stop"):
                            stop_motors()

                # Deadman Watchdog Task: 1.5 second timeout (UIT Tut 3 Safety Standard)
                # Auto-stops motors if connection drops or no command is received within 1.5s
                async def watchdog_task():
                    global _last_cmd_time, _is_motor_moving
                    while True:
                        await asyncio.sleep(0.1)  # 10 Hz check
                        if _is_motor_moving and (time.time() - _last_cmd_time > 1.5):
                            print("[WATCHDOG] 1.5s timeout with no drive command! Safety Auto-Stop triggered.")
                            stop_motors()

                # Internal Hardware Safety Reflex Task: 20 Hz (Dual-Tier Safety Architecture)
                # Dynamic Stopping Distance Profile: D_crit(v) = max(0.22, 0.18 + 0.35 * v)
                # Prevents mechanical collisions caused by sensor-actuator latency & slip on tile
                async def hardware_reflex_task():
                    global _is_motor_moving, _active_cmd_v
                    while True:
                        await asyncio.sleep(0.05)  # 20 Hz check
                        if _is_motor_moving and _active_cmd_v > 0.02:
                            curr_v = _active_cmd_v
                            # Dynamic critical distance based on real forward speed (forward drive protection)
                            d_crit = max(0.20, 0.16 + 0.35 * curr_v)
                            if lidar and hasattr(lidar, "scan_cache"):
                                with lidar.cache_lock:
                                    cache_items = list(lidar.scan_cache.items())
                                now_l = time.time()
                                frontal_dists = [
                                    dist_m for raw_ang, (dist_m, ts) in cache_items
                                    if (now_l - ts < 0.35) and (0.12 <= dist_m < d_crit) and
                                    (raw_ang <= 28.0 or raw_ang >= 332.0)
                                ]
                                if frontal_dists:
                                    min_d = min(frontal_dists)
                                    print(f"[FAIL-SAFE REFLEX] Frontal obstacle at {min_d:.2f}m < D_crit({d_crit:.2f}m)! Triggering Emergency Dynamic Brake.")
                                    stop_motors()
                                    try:
                                        await ws.send(json.dumps({
                                            "type": "hardware_emergency_brake",
                                            "reason": f"Obstacle at {min_d:.2f}m closer than D_crit({d_crit:.2f}m)",
                                            "timestamp": time.time()
                                        }))
                                    except Exception:
                                        pass

                await asyncio.gather(send_telemetry(), receive_commands(), watchdog_task(), hardware_reflex_task())

        except Exception as e:
            print(f"[DISCONNECTED] Lost connection ({e}). Reconnecting in 2 seconds...")
            stop_motors()
            await asyncio.sleep(2.0)


# ─────────────────────────────────────────────────────────────
# ENTRY POINT
# ─────────────────────────────────────────────────────────────
def main():
    global WS_URL, SERVER_HOST, SERVER_PORT
    import argparse
    parser = argparse.ArgumentParser(description="NVIDIA JetBot Hardware Bridge")
    parser.add_argument("pos_host", nargs="?", default=None, help="Dashboard host IP (positional)")
    parser.add_argument("pos_port", nargs="?", type=int, default=None, help="Dashboard port (positional)")
    parser.add_argument("--host", dest="flag_host", default=None, help="Dashboard host IP (--host)")
    parser.add_argument("--port", dest="flag_port", type=int, default=None, help="Dashboard port (--port)")
    args, _ = parser.parse_known_args()

    host = args.flag_host or args.pos_host or os.environ.get("DASHBOARD_HOST") or os.environ.get("DASHBOARD_IP") or "192.168.1.8"
    port = args.flag_port or args.pos_port or int(os.environ.get("DASHBOARD_PORT", "8000"))
    SERVER_HOST = host
    SERVER_PORT = port
    WS_URL = f"ws://{SERVER_HOST}:{SERVER_PORT}/ws/robot"

    print("=" * 65)
    print("  NVIDIA JETBOT HARDWARE BRIDGE")
    print(f"  Camera:  CSI (cáp dẹt) GStreamer (flip={CAMERA_FLIP})")
    print(f"  LiDAR:   D500 @ {LIDAR_PORT} (baud {LIDAR_BAUDRATE}, 360° Cache)")
    model_tag = os.path.basename(ENGINE_PATH).replace(".engine", "").upper()
    print(f"  AI:      TensorRT Engine [{model_tag}] ({ENGINE_PATH})")
    print(f"  Target:  {WS_URL}")
    print("=" * 65)

    cam = CSICameraThread()
    cam.start()
    start_camera_http_server(cam, port=8080)

    lidar = D500LidarThread(port=LIDAR_PORT, baudrate=LIDAR_BAUDRATE)
    lidar.start()

    yolo = TensorRTYOLOThread(camera_thread=cam, lidar_thread=lidar, engine_path=ENGINE_PATH)
    yolo.start()

    try:
        loop = asyncio.get_event_loop()
        loop.run_until_complete(bridge_loop(cam, lidar, yolo))
    except KeyboardInterrupt:
        print("\n[INFO] Exiting...")
    finally:
        stop_motors()
        cam.stop()
        lidar.stop()
        yolo.stop()
        print("[INFO] Done.")

if __name__ == "__main__":
    main()
