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
import time
import math
import json
import base64
import asyncio
import threading
import ctypes
import numpy as np

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
SERVER_HOST = os.environ.get("DASHBOARD_HOST") or os.environ.get("DASHBOARD_IP") or "192.168.168.156"
SERVER_PORT = int(os.environ.get("DASHBOARD_PORT", "8000"))
WS_URL = f"ws://{SERVER_HOST}:{SERVER_PORT}/ws/robot"

LIDAR_PORT = os.environ.get("LIDAR_PORT", "/dev/ttyUSB0")
LIDAR_BAUDRATE = 230400

CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CAMERA_FPS = 20
CAMERA_FLIP = int(os.environ.get("CAMERA_FLIP", "0"))  # Set CAMERA_FLIP=2 if upside down

ENGINE_PATH = os.environ.get("YOLO_ENGINE", "/home/jetbot/yolov8n.engine")

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
        f"video/x-raw, format=BGR ! appsink drop=1"
    )

class CSICameraThread(threading.Thread):
    def __init__(self):
        super().__init__(daemon=True)
        self.cap = None
        self.latest_frame = None
        self.latest_jpeg_b64 = ""
        self.running = True

    def _open_camera(self):
        print(f"[INFO] Opening CSI Camera (flip={CAMERA_FLIP})...")
        pipeline = get_csi_pipeline(CAMERA_WIDTH, CAMERA_HEIGHT, CAMERA_FPS, CAMERA_FLIP)
        cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
        if not cap.isOpened():
            print("[WARN] GStreamer failed. Trying USB /dev/video0...")
            cap = cv2.VideoCapture(0)
        return cap

    def run(self):
        self.cap = self._open_camera()
        if self.cap.isOpened():
            print("[OK] Camera opened successfully!")
        else:
            print("[WARN] Camera could not be opened initially. Entering auto-reconnect loop...")

        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 55]
        failed_reads = 0

        while self.running:
            if not self.cap or not self.cap.isOpened():
                time.sleep(1.5)
                self.cap = self._open_camera()
                failed_reads = 0
                continue

            ret, frame = self.cap.read()
            if ret and frame is not None and frame.size > 0:
                failed_reads = 0
                self.latest_frame = frame.copy()
                _, buffer = cv2.imencode('.jpg', frame, encode_param)
                self.latest_jpeg_b64 = base64.b64encode(buffer).decode('ascii')
                time.sleep(0.033)  # ~30 FPS live camera capture
            else:
                failed_reads += 1
                if failed_reads > 20:
                    print("[WARN] Camera stream interrupted. Re-initializing GStreamer pipeline...")
                    try:
                        self.cap.release()
                    except Exception:
                        pass
                    self.cap = None
                    failed_reads = 0
                    time.sleep(1.5)
                else:
                    time.sleep(0.05)

    def stop(self):
        self.running = False
        if self.cap:
            self.cap.release()


# ─────────────────────────────────────────────────────────────
# 2. D500 LIDAR PARSER (47-byte sync + 360° Sliding Cache)
# ─────────────────────────────────────────────────────────────
class D500LidarThread(threading.Thread):
    def __init__(self, port="/dev/ttyUSB0", baudrate=230400):
        super().__init__(daemon=True)
        self.port = port
        self.baudrate = baudrate
        self.latest_scan = []  # list of [angle_deg, dist_meters]
        self.scan_cache = {}   # angle: (dist_m, timestamp)
        self.running = True

    def run(self):
        while self.running:
            try:
                print(f"[INFO] Connecting to D500 LiDAR on {self.port} @ {self.baudrate} baud...")
                ser = serial.Serial(self.port, baudrate=self.baudrate, timeout=0.5)
                print("[OK] D500 LiDAR Serial Port opened!")

                buffer = bytearray()
                last_flush_time = time.time()

                while self.running:
                    chunk = ser.read(ser.in_waiting or 47)
                    if chunk:
                        buffer.extend(chunk)

                    # Frame synchronization: look for 0x54 and process 47-byte packets
                    while len(buffer) >= 47:
                        if buffer[0] != 0x54:
                            try:
                                next_header = buffer.index(0x54, 1)
                                del buffer[:next_header]
                            except ValueError:
                                buffer.clear()
                                break

                        if len(buffer) < 47:
                            break

                        packet = buffer[:47]
                        del buffer[:47]

                        # Packet structure:
                        # Byte 4-5: Start angle (0.01 deg)
                        start_angle = (packet[4] | (packet[5] << 8)) / 100.0
                        # Byte 42-43: End angle (0.01 deg)
                        end_angle = (packet[42] | (packet[43] << 8)) / 100.0

                        if end_angle < start_angle:
                            end_angle += 360.0

                        step = (end_angle - start_angle) / 11.0
                        now = time.time()

                        # 12 measurement points (each 3 bytes: dist_lo, dist_hi, intensity)
                        for i in range(12):
                            offset = 6 + (i * 3)
                            dist_mm = packet[offset] | (packet[offset + 1] << 8)
                            raw_angle = (start_angle + (i * step)) % 360.0
                            dist_m = dist_mm / 1000.0

                            # Invert angle to standard CCW mathematical coordinate frame
                            angle_ccw = (360 - int(round(raw_angle))) % 360

                            if 0.05 < dist_m < 12.0:
                                self.scan_cache[angle_ccw] = (round(dist_m, 3), now)

                    # Flush 360° scan every 100ms, retaining points younger than 0.6s
                    now = time.time()
                    if now - last_flush_time >= 0.10:
                        fresh_points = [
                            [ang, dist] for ang, (dist, ts) in self.scan_cache.items()
                            if now - ts < 0.6
                        ]
                        if len(fresh_points) > 15:
                            self.latest_scan = fresh_points
                        last_flush_time = now

            except Exception as e:
                print(f"[WARN] D500 LiDAR error: {e}. Retrying in 2 seconds...")
                time.sleep(2.0)

    def stop(self):
        self.running = False


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
        self.running = True

    def run(self):
        if os.environ.get("NO_YOLO") == "1":
            print("[INFO] AI YOLO disabled via NO_YOLO=1.")
            return

        if not HAS_TRT or not os.path.exists(self.engine_path):
            print(f"[WARN] TensorRT engine not found at {self.engine_path}.")
            return

        print(f"[INFO] Loading TensorRT YOLOv8n engine from {self.engine_path}...")
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
            print("[OK] YOLOv8n TensorRT Engine active on GPU!")

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

                    # Parse predictions (1, 84, 8400) -> (8400, 84)
                    preds = h_output[0].transpose(1, 0)
                    boxes = []
                    confidences = []
                    class_ids = []

                    for row in preds:
                        cx, cy, w, h = row[:4]
                        classes_scores = row[4:]
                        class_id = int(np.argmax(classes_scores))
                        max_score = float(classes_scores[class_id])

                        # Sensitivity threshold set to 0.20
                        if max_score > 0.20:
                            x1 = int((cx - w / 2) * (w_orig / 640.0))
                            y1 = int((cy - h / 2) * (h_orig / 640.0))
                            box_w = int(w * (w_orig / 640.0))
                            box_h = int(h * (h_orig / 640.0))
                            boxes.append([x1, y1, box_w, box_h])
                            confidences.append(max_score)
                            class_ids.append(class_id)

                    dets = []
                    if boxes:
                        indices = cv2.dnn.NMSBoxes(boxes, confidences, 0.20, 0.45)
                        now_lidar = time.time()
                        for idx in indices:
                            i = int(idx[0] if isinstance(idx, (list, np.ndarray)) else idx)
                            bx, by, bw, bh = boxes[i]
                            cls_idx = class_ids[i]
                            cname = COCO_CLASSES[cls_idx] if cls_idx < len(COCO_CLASSES) else f"obj_{cls_idx}"

                            # ── SENSOR FUSION: Camera Angle + 2D LiDAR Laser Distance ──
                            # Camera horizontal FOV ~60° (+-30°). Center (320px) = 0°.
                            # Left (0px) = +30° (CCW). Right (640px) = -30° (CW).
                            x_left = bx
                            x_right = bx + bw
                            ang_left = ((320.0 - x_left) / 320.0) * 30.0
                            ang_right = ((320.0 - x_right) / 320.0) * 30.0
                            min_ang = min(ang_left, ang_right) - 4.0
                            max_ang = max(ang_left, ang_right) + 4.0

                            # Look up real LiDAR points in this bounding box angular window
                            lidar_hits = []
                            if self.lidar and self.lidar.scan_cache:
                                for raw_ang, (dist_m, ts) in self.lidar.scan_cache.items():
                                    if now_lidar - ts > 0.8:
                                        continue
                                    # Convert 0..360 to signed -180..180 where 0 is FRONT
                                    signed_ang = (raw_ang - 360.0) if raw_ang > 180.0 else float(raw_ang)
                                    if min_ang <= signed_ang <= max_ang:
                                        if 0.12 <= dist_m <= 10.0:
                                            lidar_hits.append(dist_m)

                            if lidar_hits:
                                # Laser distance (cm accuracy)
                                dist = round(min(lidar_hits), 2)
                            else:
                                # Optical fallback
                                dist = round(220.0 / max(bh, 1), 2)

                            # Calculate horizontal azimuth angle: center 320px is 0°, left is positive, right is negative
                            cx = bx + bw / 2.0
                            azimuth_deg = round(((320.0 - cx) / 320.0) * 30.0, 2)

                            dets.append({
                                "class": cname,
                                "class_name": cname,
                                "confidence": round(confidences[i], 2),
                                "bbox": [bx, by, bx + bw, by + bh],
                                "distance": dist,
                                "azimuth_deg": azimuth_deg
                            })

                    self.latest_detections = dets

                    # Terminal visual feedback when objects detected
                    if dets:
                        now_t = time.time()
                        if now_t - last_log_time > 1.2:
                            names = ", ".join(f"{d['class']} ({int(d['confidence']*100)}% | {d['distance']}m | {d['azimuth_deg']}°)" for d in dets[:3])
                            print(f"[AI YOLO + LiDAR] {names}")
                            last_log_time = now_t

                    time.sleep(0.10)  # ~10 FPS inference rate (balanced load)

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

def set_raw_motor_speeds(left_norm: float, right_norm: float):
    """Differential drive normalized motor control [-1.0 .. 1.0].
    Applies JetBot physical wiring polarity correction (negative duty cycle drives forward).
    """
    left_norm = max(-1.0, min(1.0, left_norm))
    right_norm = max(-1.0, min(1.0, right_norm))

    # Wiring polarity inversion: negative speed spins wheels forward on JetBot chassis
    hw_left = -left_norm
    hw_right = -right_norm

    # Scale down motor speed so robot moves gently (~38% duty cycle)
    SPEED_FACTOR = float(os.environ.get("SPEED_FACTOR", "0.38"))

    if HAS_MOTOR:
        try:
            if left_motor_dev and right_motor_dev:
                left_speed = int(abs(hw_left) * 255 * SPEED_FACTOR)
                right_speed = int(abs(hw_right) * 255 * SPEED_FACTOR)
                left_motor_dev.setSpeed(left_speed)
                right_motor_dev.setSpeed(right_speed)
                left_motor_dev.run(1 if hw_left >= 0 else 2)
                right_motor_dev.run(1 if hw_right >= 0 else 2)
            elif 'jetbot_robot' in globals() and jetbot_robot:
                jetbot_robot.set_motors(hw_left * SPEED_FACTOR, hw_right * SPEED_FACTOR)
        except Exception as e:
            print(f"[ERROR] Motor drive: {e}")
    else:
        if abs(left_norm) > 0.02 or abs(right_norm) > 0.02:
            print(f"[MOTOR RAW] L={hw_left*SPEED_FACTOR:.2f}, R={hw_right*SPEED_FACTOR:.2f}")

def set_motor_speeds(linear: float, angular: float):
    """Convert linear (m/s) and angular (rad/s) to differential drive normalized speeds.
    Standard differential drive equations without double negation.
    """
    track_width = 0.115
    max_speed = 0.5

    left = linear - (angular * track_width / 2.0)
    right = linear + (angular * track_width / 2.0)

    left_norm = max(-1.0, min(1.0, left / max_speed))
    right_norm = max(-1.0, min(1.0, right / max_speed))
    set_raw_motor_speeds(left_norm, right_norm)

def stop_motors():
    global _is_motor_moving
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
                            "camera": camera.latest_jpeg_b64,
                            "lidar": lidar.latest_scan,
                            "detections": yolo.latest_detections,
                            "battery": current_bat,
                            "yolo_active": HAS_TRT and getattr(yolo, "engine", None) is not None
                        }
                        await ws.send(json.dumps(payload))
                        await asyncio.sleep(0.066)  # 15 FPS

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

                await asyncio.gather(send_telemetry(), receive_commands(), watchdog_task())

        except Exception as e:
            print(f"[DISCONNECTED] Lost connection ({e}). Reconnecting in 2 seconds...")
            stop_motors()
            await asyncio.sleep(2.0)


# ─────────────────────────────────────────────────────────────
# ENTRY POINT
# ─────────────────────────────────────────────────────────────
def main():
    global WS_URL
    if len(sys.argv) > 1:
        ip = sys.argv[1]
        WS_URL = f"ws://{ip}:{SERVER_PORT}/ws/robot"

    print("=" * 65)
    print("  NVIDIA JETBOT HARDWARE BRIDGE")
    print(f"  Camera:  CSI (cáp dẹt) GStreamer (flip={CAMERA_FLIP})")
    print(f"  LiDAR:   D500 @ {LIDAR_PORT} (baud {LIDAR_BAUDRATE}, 360° Cache)")
    print(f"  AI:      TensorRT YOLOv8n ({ENGINE_PATH})")
    print(f"  Target:  {WS_URL}")
    print("=" * 65)

    cam = CSICameraThread()
    cam.start()

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
