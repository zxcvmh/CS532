#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
=============================================================================
JetBot Hardware & System Health Check Diagnostic Tool
Compatible with NVIDIA Jetson Nano (JetPack 4.5+, Python 3.6.9+)
=============================================================================
Usage:
    python3 health.py              # Full diagnostic scan
    python3 health.py --quick      # Quick scan (skip slow hardware reads)
    python3 health.py --test-motor # Include a 0.2s physical motor pulse test
    python3 health.py --json       # Output machine-readable JSON format
=============================================================================
"""

import os
import sys
import time
import glob
import json
import socket
import argparse
import subprocess
import urllib.request
import urllib.error

# ANSI Color Codes for terminal
COLOR_RESET = "\033[0m"
COLOR_BOLD = "\033[1m"
COLOR_GREEN = "\033[92m"
COLOR_YELLOW = "\033[93m"
COLOR_RED = "\033[91m"
COLOR_CYAN = "\033[96m"
COLOR_GRAY = "\033[90m"

# Fallback if stdout is not a TTY
if not sys.stdout.isatty():
    COLOR_RESET = ""
    COLOR_BOLD = ""
    COLOR_GREEN = ""
    COLOR_YELLOW = ""
    COLOR_RED = ""
    COLOR_CYAN = ""
    COLOR_GRAY = ""

def badge(status: str) -> str:
    if status == "PASS":
        return "{}[PASS]{}".format(COLOR_GREEN, COLOR_RESET)
    elif status == "WARN":
        return "{}[WARN]{}".format(COLOR_YELLOW, COLOR_RESET)
    elif status == "FAIL":
        return "{}[FAIL]{}".format(COLOR_RED, COLOR_RESET)
    elif status == "INFO":
        return "{}[INFO]{}".format(COLOR_CYAN, COLOR_RESET)
    return "[{}]".format(status)

class JetBotHealthChecker:
    def __init__(self, quick=False, test_motors=False, verbose=False):
        self.quick = quick
        self.test_motors = test_motors
        self.verbose = verbose
        self.results = {}
        self.recommendations = []

    def log(self, section: str, message: str, status: str = "INFO"):
        print(" {} {}{:12s}{} | {}".format(badge(status), COLOR_BOLD, section, COLOR_RESET, message))

    # ─────────────────────────────────────────────────────────────
    # 1. SYSTEM & COMPUTE HEALTH
    # ─────────────────────────────────────────────────────────────
    def check_system(self):
        print("\n{}--- 1. SYSTEM & COMPUTE PLATFORM ---{}".format(COLOR_CYAN, COLOR_RESET))
        sys_res = {"status": "PASS", "details": {}}

        # Hostname & Python
        hostname = socket.gethostname()
        py_ver = "{}.{}.{}".format(sys.version_info.major, sys.version_info.minor, sys.version_info.micro)
        sys_res["details"]["hostname"] = hostname
        sys_res["details"]["python"] = py_ver
        self.log("Platform", "Host: {} | Python: {}".format(hostname, py_ver), "PASS")

        # L4T / JetPack Version
        jetpack_ver = "Unknown (Non-Tegra)"
        if os.path.exists("/etc/nv_tegra_release"):
            try:
                with open("/etc/nv_tegra_release", "r") as f:
                    content = f.read().strip()
                    jetpack_ver = content.splitlines()[0] if content else "Present"
                self.log("JetPack L4T", jetpack_ver, "PASS")
            except Exception as e:
                self.log("JetPack L4T", "Cannot read: {}".format(e), "WARN")
        else:
            self.log("JetPack L4T", "No Tegra release detected (Simulated / PC Mode)", "WARN")
        sys_res["details"]["jetpack"] = jetpack_ver

        # RAM & Swap (/proc/meminfo)
        try:
            mem = {}
            with open("/proc/meminfo", "r") as f:
                for line in f:
                    parts = line.split(":")
                    if len(parts) == 2:
                        key = parts[0].strip()
                        val = parts[1].strip().split()[0]
                        mem[key] = int(val)

            total_ram_mb = mem.get("MemTotal", 0) // 1024
            avail_ram_mb = mem.get("MemAvailable", mem.get("MemFree", 0)) // 1024
            used_ram_mb = total_ram_mb - avail_ram_mb
            ram_pct = (used_ram_mb / total_ram_mb * 100) if total_ram_mb > 0 else 0

            swap_total_mb = mem.get("SwapTotal", 0) // 1024
            swap_free_mb = mem.get("SwapFree", 0) // 1024
            swap_used_mb = swap_total_mb - swap_free_mb

            mem_msg = "RAM: {}MB / {}MB ({:.1f}% used)".format(used_ram_mb, total_ram_mb, ram_pct)
            if avail_ram_mb < 350:
                self.log("Memory", "{} - CRITICALLY LOW RAM!".format(mem_msg), "WARN")
                self.recommendations.append("RAM is below 350MB. Close any unused Jupyter or browser sessions.")
            else:
                self.log("Memory", mem_msg, "PASS")

            if swap_total_mb < 1000:
                self.log("Swap File", "Swap: {}MB / {}MB (Low/No swap!)".format(swap_used_mb, swap_total_mb), "WARN")
                self.recommendations.append("Jetson Nano needs at least 4GB Swap for YOLO. Run: sudo fallocate -l 4G /swapfile && sudo mkswap /swapfile && sudo swapon /swapfile")
            else:
                self.log("Swap File", "Swap: {}MB / {}MB configured".format(swap_used_mb, swap_total_mb), "PASS")

            sys_res["details"]["ram_total_mb"] = total_ram_mb
            sys_res["details"]["ram_avail_mb"] = avail_ram_mb
            sys_res["details"]["swap_total_mb"] = swap_total_mb
        except Exception as e:
            self.log("Memory", "Failed to inspect /proc/meminfo: {}".format(e), "WARN")

        # Thermal Zones
        thermal_zones = glob.glob("/sys/devices/virtual/thermal/thermal_zone*")
        temps = {}
        for tz in sorted(thermal_zones):
            try:
                with open(os.path.join(tz, "type"), "r") as tf, open(os.path.join(tz, "temp"), "r") as vf:
                    tz_type = tf.read().strip()
                    temp_c = float(vf.read().strip()) / 1000.0
                    temps[tz_type] = temp_c
            except Exception:
                continue

        if temps:
            temp_str = ", ".join(["{}: {:.1f}°C".format(k, v) for k, v in list(temps.items())[:3]])
            max_t = max(temps.values())
            if max_t > 80.0:
                self.log("Thermal", "{} (OVERHEATING WARNING!)".format(temp_str), "WARN")
                self.recommendations.append("Temperature is high ({:.1f}°C). Verify cooling fan is spinning.".format(max_t))
            else:
                self.log("Thermal", temp_str, "PASS")
            sys_res["details"]["temperatures"] = temps
        else:
            self.log("Thermal", "Thermal zones not exposed or readable", "INFO")

        # Disk Storage
        try:
            stat = os.statvfs("/")
            free_gb = (stat.f_bavail * stat.f_frsize) / (1024 ** 3)
            total_gb = (stat.f_blocks * stat.f_frsize) / (1024 ** 3)
            used_pct = (1.0 - (stat.f_bavail / stat.f_blocks)) * 100

            disk_msg = "Disk /: {:.1f}GB free of {:.1f}GB ({:.1f}% used)".format(free_gb, total_gb, used_pct)
            if free_gb < 2.0:
                self.log("Storage", "{} - DISK ALMOST FULL!".format(disk_msg), "WARN")
                self.recommendations.append("Free disk space is below 2GB. Clean up pip cache or build artifacts.")
            else:
                self.log("Storage", disk_msg, "PASS")
            sys_res["details"]["disk_free_gb"] = free_gb
        except Exception as e:
            self.log("Storage", "Could not check disk: {}".format(e), "WARN")

        self.results["system"] = sys_res

    # ─────────────────────────────────────────────────────────────
    # 2. CSI / USB CAMERA CHECK
    # ─────────────────────────────────────────────────────────────
    def check_camera(self):
        print("\n{}--- 2. CAMERA SUBSYSTEM (CSI / USB) ---{}".format(COLOR_CYAN, COLOR_RESET))
        cam_res = {"status": "FAIL", "type": None}

        # Check /dev/video*
        video_devs = glob.glob("/dev/video*")
        if video_devs:
            self.log("Video Devs", "Found video devices: {}".format(', '.join(video_devs)), "PASS")
        else:
            self.log("Video Devs", "No /dev/video* devices found! Check camera ribbon cable.", "FAIL")
            self.recommendations.append("No video devices detected. Re-seat the CSI ribbon cable with contacts facing the board.")

        # Try OpenCV capture
        try:
            import cv2
        except ImportError:
            self.log("OpenCV", "cv2 module not installed! Cannot test camera directly.", "FAIL")
            self.recommendations.append("Install OpenCV for Python: sudo apt-get install python3-opencv")
            self.results["camera"] = cam_res
            return

        if self.quick:
            self.log("Camera Read", "Skipped camera capture test (--quick mode)", "INFO")
            cam_res["status"] = "PASS" if video_devs else "FAIL"
            self.results["camera"] = cam_res
            return

        # 1. Test CSI nvarguscamerasrc pipeline
        csi_pipeline = (
            "nvarguscamerasrc ! "
            "video/x-raw(memory:NVMM), width=1280, height=720, format=NV12, framerate=20/1 ! "
            "nvvidconv flip-method=0 ! "
            "video/x-raw, width=640, height=480, format=BGRx ! "
            "videoconvert ! video/x-raw, format=BGR ! appsink drop=1"
        )

        self.log("CSI Test", "Testing nvarguscamerasrc GStreamer pipeline...", "INFO")
        cap = None
        opened = False
        try:
            cap = cv2.VideoCapture(csi_pipeline, cv2.CAP_GSTREAMER)
            if cap.isOpened():
                opened = True
                cam_res["type"] = "CSI (nvarguscamerasrc)"
        except Exception as e:
            if self.verbose:
                print("    GStreamer init error: {}".format(e))

        # 2. Test USB / V4L2 fallback if CSI failed
        if not opened:
            self.log("CSI Test", "CSI GStreamer failed. Trying USB /dev/video0 fallback...", "WARN")
            try:
                if cap:
                    cap.release()
                cap = cv2.VideoCapture(0)
                if cap.isOpened():
                    opened = True
                    cam_res["type"] = "USB (/dev/video0)"
            except Exception as e:
                if self.verbose:
                    print("    V4L2 init error: {}".format(e))

        if not opened or not cap:
            self.log("Camera", "Cannot open CSI or USB camera!", "FAIL")
            self.recommendations.append(
                "Camera capture failed. Try: sudo systemctl restart nvargus-daemon or check ribbon cable."
            )
            self.results["camera"] = cam_res
            return

        # Read sample frames to verify streaming & calculate FPS
        t0 = time.time()
        frames_read = 0
        last_frame = None
        for _ in range(5):
            ret, frame = cap.read()
            if ret and frame is not None:
                frames_read += 1
                last_frame = frame
            time.sleep(0.02)
        elapsed = time.time() - t0
        cap.release()

        if frames_read >= 2 and last_frame is not None:
            h, w, c = last_frame.shape
            mean_brightness = float(last_frame.mean())
            fps = frames_read / max(elapsed, 0.001)

            cam_res["status"] = "PASS"
            cam_res["resolution"] = "{}x{}".format(w, h)
            cam_res["fps"] = round(fps, 1)
            cam_res["brightness"] = round(mean_brightness, 1)

            self.log(
                "Camera Stream",
                "{} - {}x{} @ ~{:.1f} FPS (Brightness: {:.1f})".format(cam_res['type'], w, h, fps, mean_brightness),
                "PASS"
            )
            if mean_brightness < 2.0:
                self.log("Camera Lens", "Image is completely black! Remove lens cap or check lighting.", "WARN")
                self.recommendations.append("Camera lens cap may still be on, or lighting is pitch black.")
        else:
            self.log("Camera Stream", "Camera opened but failed to deliver video frames!", "FAIL")
            self.recommendations.append("Camera returned 0 frames. Restart nvargus-daemon: sudo systemctl restart nvargus-daemon")

        self.results["camera"] = cam_res

    # ─────────────────────────────────────────────────────────────
    # 3. 2D LIDAR SUBSYSTEM (LDROBOT D500 / LD19)
    # ─────────────────────────────────────────────────────────────
    def check_lidar(self):
        print("\n{}--- 3. 2D LIDAR SUBSYSTEM (D500 / LD19) ---{}".format(COLOR_CYAN, COLOR_RESET))
        lidar_res = {"status": "FAIL", "port": None}

        # Check serial module
        try:
            import serial
        except ImportError:
            self.log("Serial Mod", "pyserial not installed! Run: pip3 install pyserial", "FAIL")
            self.recommendations.append("Install pyserial: pip3 install pyserial")
            self.results["lidar"] = lidar_res
            return

        # Find serial port candidates
        ports = glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyTHS*") + glob.glob("/dev/ttyACM*")
        target_port = os.environ.get("LIDAR_PORT", "/dev/ttyUSB0")

        if target_port in ports:
            selected_port = target_port
        elif ports:
            selected_port = ports[0]
        else:
            selected_port = target_port

        self.log("LiDAR Port", "Target port: {} (Candidates: {})".format(selected_port, ports or 'None'), "INFO")

        if not os.path.exists(selected_port):
            self.log("LiDAR Port", "Port {} does NOT exist! LiDAR USB unplugged?".format(selected_port), "FAIL")
            self.recommendations.append("LiDAR port {} not found. Check USB cable connection.".format(selected_port))
            self.results["lidar"] = lidar_res
            return

        # Permission check
        if not os.access(selected_port, os.R_OK | os.W_OK):
            self.log("Permissions", "Cannot read/write {}! User not in 'dialout' group.".format(selected_port), "FAIL")
            self.recommendations.append("Permission denied on {}. Fix with: sudo usermod -aG dialout $USER (then re-login)".format(selected_port))
            self.results["lidar"] = lidar_res
            return
        else:
            self.log("Permissions", "Read/Write access to {} granted".format(selected_port), "PASS")

        if self.quick:
            self.log("LiDAR Stream", "Skipped live data packet capture (--quick mode)", "INFO")
            lidar_res["status"] = "PASS"
            lidar_res["port"] = selected_port
            self.results["lidar"] = lidar_res
            return

        # Read serial packets (Baudrate 230400 for D500)
        BAUDRATE = int(os.environ.get("LIDAR_BAUDRATE", "230400"))
        try:
            ser = serial.Serial(selected_port, baudrate=BAUDRATE, timeout=1.0)
            t_start = time.time()
            buffer = bytearray()
            packets_found = 0
            sample_points = 0
            closest_dist = 999.0

            while time.time() - t_start < 1.5:
                chunk = ser.read(256)
                if chunk:
                    buffer.extend(chunk)

                # Search for D500 / LD19 header: 0x54 0x2C
                while len(buffer) >= 47:
                    if buffer[0] == 0x54 and buffer[1] == 0x2C:
                        packet = buffer[:47]
                        buffer = buffer[47:]
                        packets_found += 1

                        # Parse points in packet (12 points per packet)
                        for i in range(12):
                            idx = 6 + i * 3
                            dist_mm = packet[idx] | (packet[idx + 1] << 8)
                            dist_m = dist_mm / 1000.0
                            if 0.12 < dist_m < 8.0:
                                sample_points += 1
                                if dist_m < closest_dist:
                                    closest_dist = dist_m
                    else:
                        buffer.pop(0)

                if packets_found > 10:
                    break

            ser.close()

            if packets_found > 0:
                lidar_res["status"] = "PASS"
                lidar_res["port"] = selected_port
                lidar_res["packets"] = packets_found
                lidar_res["points"] = sample_points
                lidar_res["closest_dist_m"] = round(closest_dist, 2) if sample_points > 0 else None

                self.log(
                    "LiDAR Stream",
                    "Port {} @ {} baud: Read {} D500 packets ({} valid points, nearest: {:.2f}m)".format(
                        selected_port, BAUDRATE, packets_found, sample_points, closest_dist
                    ),
                    "PASS"
                )
            else:
                self.log("LiDAR Stream", "Opened {} but no 0x54 0x2C packets received! Is motor spinning?".format(selected_port), "WARN")
                self.recommendations.append("LiDAR connected but no data packets received. Ensure motor spinning and baudrate is 230400.")
        except Exception as e:
            self.log("LiDAR Stream", "Serial communication error on {}: {}".format(selected_port, e), "FAIL")
            self.recommendations.append("Failed to open LiDAR serial port: {}".format(e))

        self.results["lidar"] = lidar_res

    # ─────────────────────────────────────────────────────────────
    # 4. MOTOR CONTROLLER & I2C CHECK
    # ─────────────────────────────────────────────────────────────
    def check_motors(self):
        print("\n{}--- 4. MOTOR DRIVER & I2C SUBSYSTEM ---{}".format(COLOR_CYAN, COLOR_RESET))
        motor_res = {"status": "FAIL", "driver": None}

        # 1. Check I2C bus
        i2c_bus = 1
        i2c_dev = "/dev/i2c-{}".format(i2c_bus)
        if os.path.exists(i2c_dev):
            self.log("I2C Bus", "Found {} interface".format(i2c_dev), "PASS")
        else:
            self.log("I2C Bus", "{} does not exist! I2C kernel module disabled?".format(i2c_dev), "WARN")
            self.recommendations.append("I2C bus not found. Ensure i2c-dev module is loaded: sudo modprobe i2c-dev")

        # 2. Check I2C address 0x60 (PCA9685 Motor HAT) using i2cdetect if available
        try:
            res = subprocess.run(
                ["i2cdetect", "-y", "-r", str(i2c_bus)],
                stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=2, universal_newlines=True
            )
            if "60" in res.stdout:
                self.log("I2C Detect", "Detected PCA9685 Motor Controller at I2C address 0x60", "PASS")
            elif "UU" in res.stdout and ("60" in res.stdout or "40" in res.stdout):
                self.log("I2C Detect", "Motor HAT chip is active and bound by driver (UU)", "PASS")
            else:
                self.log("I2C Detect", "No chip found at address 0x60 on I2C-1 (Power switch on MotorHAT ON?)", "WARN")
                self.recommendations.append("MotorHAT not detected at 0x60. Check if battery pack switch is turned ON.")
        except Exception:
            self.log("I2C Detect", "i2cdetect command not found (optional)", "INFO")

        # 3. Test Motor HAT driver library import
        try:
            from Adafruit_MotorHAT import Adafruit_MotorHAT
            mh = Adafruit_MotorHAT(i2c_bus=i2c_bus)
            motor_driver = "Adafruit_MotorHAT"
            motor_res["driver"] = motor_driver
            motor_res["status"] = "PASS"
            self.log("Motor Driver", "Adafruit_MotorHAT initialized successfully (Lightweight, No PyTorch)", "PASS")

            # Physical motor pulse test if explicitly requested
            if self.test_motors:
                self.log("Motor Test", "Running 0.2s physical twitch test on Left/Right motors...", "INFO")
                try:
                    left = mh.getMotor(1)
                    right = mh.getMotor(2)
                    left.setSpeed(40)
                    right.setSpeed(40)
                    left.run(Adafruit_MotorHAT.FORWARD)
                    right.run(Adafruit_MotorHAT.FORWARD)
                    time.sleep(0.2)
                    left.run(Adafruit_MotorHAT.RELEASE)
                    right.run(Adafruit_MotorHAT.RELEASE)
                    self.log("Motor Test", "Motor pulse completed and released!", "PASS")
                except Exception as me:
                    self.log("Motor Test", "Pulse execution failed: {}".format(me), "WARN")

        except Exception as e:
            try:
                from jetbot import Robot
                _ = Robot()
                motor_driver = "jetbot.Robot"
                motor_res["driver"] = motor_driver
                motor_res["status"] = "PASS"
                self.log("Motor Driver", "NVIDIA jetbot.Robot class initialized successfully", "PASS")
            except Exception as e2:
                self.log("Motor Driver", "Neither Adafruit_MotorHAT nor jetbot.Robot initialized ({})".format(e), "WARN")
                self.recommendations.append("Motor library not initialized. Run: pip3 install Adafruit-MotorHAT")

        self.results["motor"] = motor_res

    # ─────────────────────────────────────────────────────────────
    # 5. AI ENGINE & TENSORRT CHECK
    # ─────────────────────────────────────────────────────────────
    def check_ai(self):
        print("\n{}--- 5. AI & TENSORRT ENGINE SUBSYSTEM ---{}".format(COLOR_CYAN, COLOR_RESET))
        ai_res = {"status": "INFO", "trt": False, "engine_found": False}

        # Check TensorRT
        try:
            import tensorrt as trt
            ai_res["trt"] = True
            trt_ver = trt.__version__
            self.log("TensorRT", "TensorRT Python library found (v{})".format(trt_ver), "PASS")
        except ImportError:
            self.log("TensorRT", "TensorRT Python library NOT found. AI inference will be simulated.", "WARN")
            self.recommendations.append("TensorRT is missing. Ensure JetPack 4.5 default python3-libnvinfer is installed.")

        # Check YOLOv8n engine file
        engine_path = os.environ.get("YOLO_ENGINE", "/home/jetbot/yolov8n.engine")
        candidate_paths = [
            engine_path,
            os.path.abspath("yolov8n.engine"),
            os.path.abspath("../yolov8n.engine"),
            "/home/jetbot/yolov8n.engine",
            "/home/jetbot/cs532/yolov8n.engine"
        ]

        found_path = None
        for p in candidate_paths:
            if os.path.exists(p) and os.path.isfile(p):
                found_path = p
                break

        if found_path:
            sz_mb = os.path.getsize(found_path) / (1024 * 1024)
            ai_res["engine_found"] = True
            ai_res["engine_path"] = found_path
            ai_res["engine_size_mb"] = round(sz_mb, 2)
            self.log("YOLO Engine", "Found model engine at {} ({:.2f} MB)".format(found_path, sz_mb), "PASS")
            ai_res["status"] = "PASS"
        else:
            self.log("YOLO Engine", "yolov8n.engine not found at default locations", "WARN")
            self.recommendations.append(
                "yolov8n.engine not found. Build it with trtexec: /usr/src/tensorrt/bin/trtexec --onnx=yolov8n.onnx --saveEngine=/home/jetbot/yolov8n.engine --fp16"
            )

        self.results["ai"] = ai_res

    # ─────────────────────────────────────────────────────────────
    # 6. NETWORK & DASHBOARD SERVICE HEALTH
    # ─────────────────────────────────────────────────────────────
    def check_network_and_service(self):
        print("\n{}--- 6. NETWORK & WEB SERVICES ---{}".format(COLOR_CYAN, COLOR_RESET))
        net_res = {"status": "FAIL", "ip": [], "backend_up": False}

        # Get local IP addresses
        ips = []
        try:
            res = subprocess.run(
                ["hostname", "-I"], stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True
            )
            raw_ips = res.stdout.strip().split()
            ips = [ip for ip in raw_ips if not ip.startswith("127.") and not ip.startswith("172.17.")]
        except Exception:
            pass

        if ips:
            net_res["ip"] = ips
            self.log("Network IP", "Active JetBot IP(s): {}".format(', '.join(ips)), "PASS")
        else:
            self.log("Network IP", "No external Wi-Fi / Ethernet IP detected!", "WARN")
            self.recommendations.append("JetBot has no active IP. Check Wi-Fi connection with 'nmcli dev wifi'.")

        # Check port 8000 (FastAPI Backend)
        port = 8000
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(1.0)
        port_open = False
        try:
            result = sock.connect_ex(('127.0.0.1', port))
            if result == 0:
                port_open = True
        except Exception:
            pass
        finally:
            sock.close()

        if port_open:
            self.log("Port 8000", "Port 8000 is LISTENING", "PASS")
            net_res["backend_up"] = True

            # Query HTTP /health
            try:
                req = urllib.request.Request("http://127.0.0.1:8000/health", headers={"User-Agent": "health.py"})
                with urllib.request.urlopen(req, timeout=1.5) as response:
                    body = json.loads(response.read().decode('utf-8'))
                    if body.get("ok"):
                        self.log("API /health", "HTTP 200 OK - Health check passed: {}".format(body), "PASS")
                    else:
                        self.log("API /health", "HTTP 200 with unexpected response: {}".format(body), "WARN")
            except Exception as e:
                self.log("API /health", "Failed to query /health: {}".format(e), "WARN")

            # Query HTTP /version
            try:
                req = urllib.request.Request("http://127.0.0.1:8000/version", headers={"User-Agent": "health.py"})
                with urllib.request.urlopen(req, timeout=1.5) as response:
                    body = json.loads(response.read().decode('utf-8'))
                    self.log("API /version", "HTTP 200 OK - Agent: {} v{}".format(body.get('agent'), body.get('version')), "PASS")
            except Exception as e:
                self.log("API /version", "Failed to query /version: {}".format(e), "INFO")

            net_res["status"] = "PASS"
        else:
            self.log("Port 8000", "Port 8000 is NOT open. FastAPI backend server is not running.", "WARN")
            self.recommendations.append("Dashboard server is offline. Start it: python3 -m uvicorn app.main:app --host 0.0.0.0 --port 8000")

        self.results["network"] = net_res

    # ─────────────────────────────────────────────────────────────
    # 7. SUMMARY REPORT & RECOMMENDATIONS
    # ─────────────────────────────────────────────────────────────
    def print_summary(self):
        print("\n{}{}{}".format(COLOR_BOLD, '=' * 65, COLOR_RESET))
        print("{}             JETBOT HEALTH DIAGNOSTIC SUMMARY{}".format(COLOR_BOLD, COLOR_RESET))
        print("{}{}{}".format(COLOR_BOLD, '=' * 65, COLOR_RESET))

        categories = [
            ("Compute / OS", self.results.get("system", {}).get("status", "UNKNOWN")),
            ("CSI / USB Camera", self.results.get("camera", {}).get("status", "UNKNOWN")),
            ("2D LiDAR (D500)", self.results.get("lidar", {}).get("status", "UNKNOWN")),
            ("Motor Driver / I2C", self.results.get("motor", {}).get("status", "UNKNOWN")),
            ("AI / TensorRT", self.results.get("ai", {}).get("status", "UNKNOWN")),
            ("Web Service & Net", self.results.get("network", {}).get("status", "UNKNOWN")),
        ]

        for name, status in categories:
            print("  {}  {:<24s}".format(badge(status), name))

        print("{}{}{}".format(COLOR_BOLD, '=' * 65, COLOR_RESET))

        if self.recommendations:
            print("\n{}{}🔧 ACTIONABLE RECOMMENDATIONS:{}".format(COLOR_YELLOW, COLOR_BOLD, COLOR_RESET))
            for idx, rec in enumerate(self.recommendations, 1):
                print("  {}. {}".format(idx, rec))
            print()
        else:
            print("\n{}{}🎉 ALL SYSTEMS OPERATIONAL! Your JetBot is 100% ready.{}\n".format(COLOR_GREEN, COLOR_BOLD, COLOR_RESET))

    def run_all(self):
        t0 = time.time()
        print("{}================================================================={}".format(COLOR_BOLD, COLOR_RESET))
        print("{}  JETBOT HARDWARE & SERVICE HEALTH DIAGNOSTICS                   {}".format(COLOR_BOLD, COLOR_RESET))
        print("{}================================================================={}".format(COLOR_BOLD, COLOR_RESET))

        self.check_system()
        self.check_camera()
        self.check_lidar()
        self.check_motors()
        self.check_ai()
        self.check_network_and_service()

        elapsed = time.time() - t0
        print("\nDiagnostics completed in {:.2f} seconds.".format(elapsed))

        if not self.results.get("raw_json", False):
            self.print_summary()

        return self.results


def main():
    parser = argparse.ArgumentParser(description="JetBot Hardware & Service Diagnostic Tool")
    parser.add_argument("--quick", action="store_true", help="Perform quick scan without opening camera/serial streams")
    parser.add_argument("--test-motor", action="store_true", help="Execute safe 0.2s physical motor pulse test")
    parser.add_argument("--json", action="store_true", help="Output machine-readable JSON output")
    parser.add_argument("-v", "--verbose", action="store_true", help="Verbose error traces")
    args = parser.parse_args()

    checker = JetBotHealthChecker(
        quick=args.quick,
        test_motors=args.test_motor,
        verbose=args.verbose
    )

    results = checker.run_all()

    if args.json:
        print("\n" + json.dumps(results, indent=2))

if __name__ == "__main__":
    main()
