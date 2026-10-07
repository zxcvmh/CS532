#!/usr/bin/env python3
"""
JetBot Lightweight Hardware Bridge (Client Mode)
================================================
Hardware Responsibilities on JetBot:
  - Camera: CSI (IMX219 via jetbot.Camera / GStreamer) or USB (/dev/video0) capture & JPEG encode
  - LiDAR:  D500 / LD19 (UART auto-detect /dev/ttyUSB*, 230400 baud, 47-byte sync)
  - Motors: Adafruit MotorHAT / PCA9685 I2C driver
  - Comms:  WebSocket client streaming raw sensor data to Laptop and receiving cmd_vel

Offloaded to Laptop:
  - All heavy processing: YOLOv8 AI object detection, LiDAR-Camera sensor fusion,
    2D SLAM Occupancy Grid mapping, A* path planning, and Web Dashboard.
"""

import sys
import os
import time
import glob
import math
import json
import base64
import asyncio
import threading

# ─────────────────────────────────────────────────────────────
# HEADLESS ENVIRONMENT FIX FOR JETSON NANO
# ─────────────────────────────────────────────────────────────
# Prevent libnvbuf_utils EGL display crash when running over SSH
if "DISPLAY" in os.environ and ("localhost" in os.environ["DISPLAY"] or ":" in os.environ["DISPLAY"]):
    os.environ.pop("DISPLAY", None)

# ─────────────────────────────────────────────────────────────
# DEPENDENCY CHECKS
# ─────────────────────────────────────────────────────────────
try:
    import cv2
except ImportError:
    print("[ERROR] OpenCV (cv2) not found! Run: sudo apt-get install python3-opencv")
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


# ─────────────────────────────────────────────────────────────
# MOTOR DRIVER (Adafruit MotorHAT / JetBot I2C)
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
    print("[OK] Adafruit MotorHAT driver initialized.")
except Exception:
    try:
        from jetbot import Robot
        jetbot_robot = Robot()
        HAS_MOTOR = True
        print("[OK] NVIDIA JetBot Robot driver initialized.")
    except Exception as e:
        HAS_MOTOR = False
        print(f"[INFO] Motor running in simulation log mode ({e}).")


# ─────────────────────────────────────────────────────────────
# CONFIGURATION
# ─────────────────────────────────────────────────────────────
SERVER_HOST = os.environ.get("DASHBOARD_IP", "192.168.168.150")
SERVER_PORT = int(os.environ.get("DASHBOARD_PORT", "8000"))
WS_URL = f"ws://{SERVER_HOST}:{SERVER_PORT}/ws/robot"

LIDAR_PORT = os.environ.get("LIDAR_PORT", "/dev/ttyUSB0")
LIDAR_BAUDRATE = 230400

CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CAMERA_FPS = 20
CAMERA_FLIP = int(os.environ.get("CAMERA_FLIP", "0"))  # Set CAMERA_FLIP=2 if upside down


# ─────────────────────────────────────────────────────────────
# 1. CSI / USB CAMERA THREAD (Multi-backend with EGL protection)
# ─────────────────────────────────────────────────────────────
def get_csi_pipeline(sensor_id=0, width=640, height=480, fps=20, flip=0):
    return (
        f"nvarguscamerasrc sensor-id={sensor_id} ! "
        f"video/x-raw(memory:NVMM), width={width}, height={height}, format=NV12, framerate={fps}/1 ! "
        f"nvvidconv flip-method={flip} ! "
        f"video/x-raw, width={width}, height={height}, format=BGRx ! "
        f"videoconvert ! "
        f"video/x-raw, format=BGR ! appsink drop=1"
    )

class CSICameraThread(threading.Thread):
    def __init__(self):
        super().__init__(daemon=True)
        self.cap = None
        self.jetbot_cam = None
        self.latest_jpeg_b64 = ""
        self.running = True

    def run(self):
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 55]

        # Method 1: Try official JetBot Camera class if installed
        try:
            from jetbot import Camera
            print("[INFO] Attempting to open camera via jetbot.Camera...")
            self.jetbot_cam = Camera.instance(width=CAMERA_WIDTH, height=CAMERA_HEIGHT, fps=CAMERA_FPS)
            print("[OK] Camera opened via jetbot.Camera driver!")
            while self.running:
                frame = self.jetbot_cam.value
                if frame is not None and len(frame) > 0:
                    _, buffer = cv2.imencode('.jpg', frame, encode_param)
                    self.latest_jpeg_b64 = base64.b64encode(buffer).decode('ascii')
                time.sleep(0.04)
            return
        except Exception as e:
            # Fall back to GStreamer pipeline
            pass

        # Method 2: Standard GStreamer CSI Pipeline
        print(f"[INFO] Opening CSI Camera via GStreamer (flip={CAMERA_FLIP})...")
        try:
            pipeline = get_csi_pipeline(sensor_id=0, width=CAMERA_WIDTH, height=CAMERA_HEIGHT, fps=CAMERA_FPS, flip=CAMERA_FLIP)
            self.cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
        except Exception as e:
            print(f"[WARN] CSI GStreamer exception: {e}")
            self.cap = None

        # Method 3: USB V4L2 (/dev/video0) fallback
        if self.cap is None or not self.cap.isOpened():
            print("[WARN] CSI Camera failed. Trying USB /dev/video0...")
            try:
                self.cap = cv2.VideoCapture(0)
            except Exception:
                self.cap = None

        if self.cap and self.cap.isOpened():
            print("[OK] Camera opened successfully!")
        else:
            print("=" * 60)
            print("[ERROR] KHÔNG THỂ MỞ CAMERA TRÊN JETSON NANO!")
            print(">> Hãy chạy lệnh sau trên JetBot để reset driver camera:")
            print("   sudo systemctl restart nvargus-daemon")
            print("=" * 60)
            return

        while self.running:
            ret, frame = self.cap.read()
            if ret and frame is not None:
                _, buffer = cv2.imencode('.jpg', frame, encode_param)
                self.latest_jpeg_b64 = base64.b64encode(buffer).decode('ascii')
                time.sleep(0.04)  # ~25 FPS
            else:
                time.sleep(0.05)

    def stop(self):
        self.running = False
        if self.cap:
            self.cap.release()
        if self.jetbot_cam:
            try:
                self.jetbot_cam.stop()
            except Exception:
                pass


# ─────────────────────────────────────────────────────────────
# 2. D500 LIDAR THREAD (Auto-detect port + 47-byte sync + 360° Cache)
# ─────────────────────────────────────────────────────────────
def find_available_lidar_port(default_port="/dev/ttyUSB0"):
    """Auto-detect LiDAR serial port on Jetson Nano."""
    if os.path.exists(default_port):
        return default_port
    # Search all possible USB/ACM serial ports
    candidates = glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*")
    if candidates:
        print(f"[INFO] Auto-detected LiDAR port: {candidates[0]}")
        return candidates[0]
    return default_port

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
            target_port = find_available_lidar_port(self.port)

            if not os.path.exists(target_port):
                print(f"[WARN] Chưa tìm thấy cổng LiDAR '{target_port}'.")
                print("       -> Vui lòng cắm cáp USB của LiDAR D500 vào JetBot.")
                time.sleep(3.0)
                continue

            try:
                print(f"[INFO] Connecting to D500 LiDAR on {target_port} @ {self.baudrate} baud...")
                ser = serial.Serial(target_port, baudrate=self.baudrate, timeout=0.5)
                print(f"[OK] D500 LiDAR opened on {target_port}!")

                buffer = bytearray()
                last_flush_time = time.time()

                while self.running:
                    chunk = ser.read(ser.in_waiting or 47)
                    if chunk:
                        buffer.extend(chunk)

                    # Frame synchronization: look for 0x54 header in 47-byte packets
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
                print(f"[WARN] D500 LiDAR read error: {e}. Thử lại sau 2 giây...")
                time.sleep(2.0)

    def stop(self):
        self.running = False


# ─────────────────────────────────────────────────────────────
# 3. MOTOR CONTROL (NVIDIA JetBot)
# ─────────────────────────────────────────────────────────────
def set_motor_speeds(linear: float, angular: float):
    # Invert linear direction for wiring polarity
    linear = -linear

    track_width = 0.115
    max_speed = 0.5

    left = linear - (angular * track_width / 2.0)
    right = linear + (angular * track_width / 2.0)

    left_norm = max(-1.0, min(1.0, left / max_speed))
    right_norm = max(-1.0, min(1.0, right / max_speed))

    SPEED_FACTOR = float(os.environ.get("SPEED_FACTOR", "0.38"))

    if HAS_MOTOR:
        try:
            if left_motor_dev and right_motor_dev:
                left_speed = int(abs(left_norm) * 255 * SPEED_FACTOR)
                right_speed = int(abs(right_norm) * 255 * SPEED_FACTOR)
                left_motor_dev.setSpeed(left_speed)
                right_motor_dev.setSpeed(right_speed)
                left_motor_dev.run(1 if left_norm >= 0 else 2)   # 1=FORWARD, 2=BACKWARD
                right_motor_dev.run(1 if right_norm >= 0 else 2)
            elif 'jetbot_robot' in globals() and jetbot_robot:
                jetbot_robot.set_motors(left_norm * SPEED_FACTOR, right_norm * SPEED_FACTOR)
        except Exception as e:
            print(f"[ERROR] Motor drive: {e}")
    else:
        if abs(linear) > 0.02 or abs(angular) > 0.02:
            print(f"[MOTOR] V={linear:.2f}, W={angular:.2f} -> L={left_norm*SPEED_FACTOR:.2f}, R={right_norm*SPEED_FACTOR:.2f}")

def stop_motors():
    if HAS_MOTOR:
        try:
            if left_motor_dev and right_motor_dev:
                left_motor_dev.run(4)  # 4=RELEASE
                right_motor_dev.run(4)
            elif 'jetbot_robot' in globals() and jetbot_robot:
                jetbot_robot.stop()
        except Exception:
            pass
    print("[MOTOR] STOPPED")


# ─────────────────────────────────────────────────────────────
# 4. WEBSOCKET HARDWARE BRIDGE LOOP
# ─────────────────────────────────────────────────────────────
async def bridge_loop(camera: CSICameraThread, lidar: D500LidarThread):
    print(f"\n=======================================================")
    print(f"  JetBot Connecting to Laptop at:")
    print(f"  {WS_URL}")
    print(f"  (All heavy tasks like YOLO and SLAM run on Laptop)")
    print(f"=======================================================\n")

    while True:
        try:
            async with websockets.connect(WS_URL, ping_interval=10, ping_timeout=10) as ws:
                print(f"[SUCCESS] Connected to Laptop Backend at {WS_URL}!")

                async def send_telemetry():
                    while True:
                        payload = {
                            "type": "robot_telemetry",
                            "timestamp": time.time(),
                            "camera": camera.latest_jpeg_b64,
                            "lidar": lidar.latest_scan,
                            "battery": 84.0
                        }
                        await ws.send(json.dumps(payload))
                        await asyncio.sleep(0.066)  # 15 FPS telemetry stream

                async def receive_commands():
                    while True:
                        raw = await ws.recv()
                        msg = json.loads(raw)
                        mtype = msg.get("type")
                        if mtype == "cmd_vel":
                            v = float(msg.get("linear", 0.0))
                            w = float(msg.get("angular", 0.0))
                            set_motor_speeds(v, w)
                        elif mtype in ("stop", "emergency_stop"):
                            stop_motors()

                await asyncio.gather(send_telemetry(), receive_commands())

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
    print("  NVIDIA JETBOT LIGHTWEIGHT HARDWARE BRIDGE")
    print(f"  Camera:  CSI / USB GStreamer (flip={CAMERA_FLIP})")
    print(f"  LiDAR:   D500 @ {LIDAR_PORT} (baud {LIDAR_BAUDRATE})")
    print(f"  Target:  {WS_URL}")
    print("  Heavy AI (YOLO) and SLAM: Offloaded completely to Laptop!")
    print("=" * 65)

    cam = CSICameraThread()
    cam.start()

    lidar = D500LidarThread(port=LIDAR_PORT, baudrate=LIDAR_BAUDRATE)
    lidar.start()

    try:
        loop = asyncio.get_event_loop()
        loop.run_until_complete(bridge_loop(cam, lidar))
    except KeyboardInterrupt:
        print("\n[INFO] Exiting...")
    finally:
        stop_motors()
        cam.stop()
        lidar.stop()
        print("[INFO] Done.")

if __name__ == "__main__":
    main()
