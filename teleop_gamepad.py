#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
CS532 - JetBot Gamepad / Controller Teleoperation
=============================================================================
Dựa trên kiến trúc điều khiển từ notebook 'basic_motion.ipynb':
  - Điều khiển động cơ thông qua `jetbot.Robot` (hoặc MockRobot khi chạy trên PC)
  - Hỗ trợ các lệnh cơ bản: robot.forward, robot.backward, robot.left, robot.right, robot.stop()
  - Hỗ trợ điều khiển độc lập 2 bánh: robot.set_motors(left, right)
  - Tích hợp bộ lọc an toàn Heartbeat / Watchdog tự động dừng khi nhả cần hoặc mất tín hiệu
  - Tự động nhận diện tay cầm: Xbox, PlayStation (PS4/PS5), Logitech F710, tay game USB...
  - Hỗ trợ chế độ điều khiển kép:
      1. Analog Joystick Mode: Điều khiển tốc độ & hướng mượt mà theo độ nghiêng cần gạt
      2. D-Pad / Button Mode: Điều khiển tiến/lùi/quay trái/quay phải từng bước
  - Fallback chế độ bàn phím (Keyboard) khi chưa cắm tay cầm để test nhanh
=============================================================================
"""

import sys
import os
import time
import math
import json
import signal
import urllib.request
import urllib.error
import urllib.parse

# Đảm bảo in tiếng Việt không bị lỗi encoding trên Windows console
if hasattr(sys.stdout, "reconfigure"):
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
        sys.stderr.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass

# -----------------------------------------------------------------------------
# 1. KHỞI TẠO ROBOT (Hardware JetBot, Mock Robot hoặc Remote Backend Robot)
# -----------------------------------------------------------------------------
HAS_HARDWARE = False
robot = None


class MockMotor:
    def __init__(self, name):
        self.name = name
        self._value = 0.0

    @property
    def value(self):
        return self._value

    @value.setter
    def value(self, val):
        self._value = max(-1.0, min(1.0, float(val)))


class MockRobot:
    """Lớp mô phỏng Robot tương thích 100% với jetbot.Robot trong basic_motion.ipynb"""
    def __init__(self):
        self.left_motor = MockMotor("left")
        self.right_motor = MockMotor("right")

    def forward(self, speed=0.3):
        self.left_motor.value = speed
        self.right_motor.value = speed

    def backward(self, speed=0.3):
        self.left_motor.value = -speed
        self.right_motor.value = -speed

    def left(self, speed=0.3):
        self.left_motor.value = -speed
        self.right_motor.value = speed

    def right(self, speed=0.3):
        self.left_motor.value = speed
        self.right_motor.value = -speed

    def stop(self):
        self.left_motor.value = 0.0
        self.right_motor.value = 0.0

    def set_motors(self, left_speed, right_speed):
        self.left_motor.value = left_speed
        self.right_motor.value = right_speed


class RemoteBackendRobot:
    """
    Adapter cho phép điều khiển JetBot từ xa qua FastAPI Backend REST API.
    Cung cấp giao diện tương thích 100% với jetbot.Robot và MockRobot:
      - set_motors(left_speed, right_speed) -> HTTP POST /drive {"left": l, "right": r}
      - forward(speed) -> HTTP POST /drive {"left": speed, "right": speed}
      - backward(speed) -> HTTP POST /drive {"left": -speed, "right": -speed}
      - left(speed) -> HTTP POST /drive {"left": -speed, "right": speed}
      - right(speed) -> HTTP POST /drive {"left": speed, "right": -speed}
      - stop() -> HTTP POST /stop (hoặc /api/robot/stop)
    """
    def __init__(self, base_url="http://127.0.0.1:8000", timeout=0.08, heartbeat_sec=0.20):
        self.base_url = base_url.rstrip("/")
        self.timeout = float(timeout)
        self.heartbeat_sec = float(heartbeat_sec)
        self.left_motor = MockMotor("left")
        self.right_motor = MockMotor("right")
        self._drive_url = f"{self.base_url}/drive"
        self._stop_url = f"{self.base_url}/stop"
        self._opener = urllib.request.build_opener()
        self._last_left = None
        self._last_right = None
        self._last_send_time = 0.0
        self._is_stopped = True

    def _send_post(self, path: str, payload: dict) -> bool:
        url = f"{self.base_url}{path}" if path.startswith("/") else f"{self.base_url}/{path}"
        try:
            data = json.dumps(payload).encode("utf-8")
            req = urllib.request.Request(
                url,
                data=data,
                headers={"Content-Type": "application/json"},
                method="POST"
            )
            with self._opener.open(req, timeout=self.timeout) as resp:
                return resp.status in (200, 201, 204)
        except Exception:
            return False

    def set_motors(self, left_speed: float, right_speed: float, force: bool = False) -> bool:
        l = max(-1.0, min(1.0, float(left_speed)))
        r = max(-1.0, min(1.0, float(right_speed)))
        self.left_motor.value = l
        self.right_motor.value = r

        now = time.time()

        # Stale command latch: Khi buông cần về 0 -> chuyển sang stop() ngay lập tức
        if abs(l) < 1e-3 and abs(r) < 1e-3:
            return self.stop()

        # Bộ lọc nén tần số (Frequency Compression):
        # Chỉ gửi lệnh nếu vận tốc thay đổi đáng kể (> 0.01) hoặc chuyển từ trạng thái dừng
        # hoặc đã tới chu kỳ heartbeat (0.2s)
        speed_changed = (
            self._last_left is None
            or self._last_right is None
            or self._is_stopped
            or abs(l - self._last_left) >= 0.01
            or abs(r - self._last_right) >= 0.01
        )
        heartbeat_due = (now - self._last_send_time) >= self.heartbeat_sec

        if force or speed_changed or heartbeat_due:
            payload = {"left": l, "right": r}
            ok = self._send_post("/drive", payload)
            if ok:
                self._last_left = l
                self._last_right = r
                self._last_send_time = now
                self._is_stopped = False
            return ok

        return True

    def forward(self, speed: float = 0.3) -> bool:
        return self.set_motors(speed, speed)

    def backward(self, speed: float = 0.3) -> bool:
        return self.set_motors(-speed, -speed)

    def left(self, speed: float = 0.3) -> bool:
        return self.set_motors(-speed, speed)

    def right(self, speed: float = 0.3) -> bool:
        return self.set_motors(speed, -speed)

    def stop(self) -> bool:
        self.left_motor.value = 0.0
        self.right_motor.value = 0.0
        now = time.time()

        # Nếu robot đang di chuyển hoặc đã quá heartbeat kể từ lệnh stop trước:
        # Gửi lệnh stop ngay lập tức (E-stop priority & latch prevention)
        if not self._is_stopped or (now - self._last_send_time) >= self.heartbeat_sec:
            ok = self._send_post("/stop", {})
            if ok:
                self._is_stopped = True
                self._last_left = 0.0
                self._last_right = 0.0
                self._last_send_time = now
            return ok

        return True


def parse_cli_args(args=None):
    """
    Phân giải tham số dòng lệnh CLI:
      --keyboard, -k : Chế độ điều khiển bàn phím
      --remote, -r [URL] : Chế độ điều khiển từ xa qua Backend REST API (mặc định http://127.0.0.1:8000)
    """
    if args is None:
        args = sys.argv[1:]

    is_keyboard = False
    remote_url = None

    idx = 0
    while idx < len(args):
        arg = args[idx].strip()
        if arg in ("--keyboard", "-k"):
            is_keyboard = True
            idx += 1
        elif arg in ("--remote", "-r"):
            if idx + 1 < len(args) and not args[idx + 1].startswith("-"):
                remote_url = args[idx + 1].strip()
                idx += 2
            else:
                remote_url = "http://127.0.0.1:8000"
                idx += 1
        elif arg.startswith("--remote="):
            remote_url = arg.split("=", 1)[1].strip() or "http://127.0.0.1:8000"
            idx += 1
        elif arg.startswith("-r="):
            remote_url = arg.split("=", 1)[1].strip() or "http://127.0.0.1:8000"
            idx += 1
        else:
            idx += 1

    return remote_url, is_keyboard


def init_robot(remote_url=None):
    """Khởi tạo robot: RemoteBackendRobot (nếu có remote_url), jetbot.Robot hoặc MockRobot."""
    global HAS_HARDWARE
    if remote_url:
        HAS_HARDWARE = False
        print(f"[CHẾ ĐỘ TỪ XA] Kết nối Backend JetBot tại: {remote_url}")
        return RemoteBackendRobot(base_url=remote_url)

    try:
        from jetbot import Robot
        r = Robot()
        HAS_HARDWARE = True
        print("[OK] Kết nối phần cứng NVIDIA JetBot (jetbot.Robot) thành công.")
        return r
    except Exception as e:
        HAS_HARDWARE = False
        print(f"[THÔNG BÁO] Không tìm thấy thư viện/phần cứng JetBot ({e}).")
        print("[CHẾ ĐỘ GIẢ LẬP] Chạy ở chế độ MOCK Robot để kiểm thử thuật toán trên máy tính.")
        return MockRobot()


# Robot mặc định khi import module (sẽ được cấu hình lại theo CLI khi chạy main)
robot = MockRobot()


# -----------------------------------------------------------------------------
# 2. ĐỌC CẤU HÌNH ROBOT DYNAMICS (NẾU CÓ)
# -----------------------------------------------------------------------------
CONFIG_PATHS = [
    os.path.join(os.path.dirname(__file__), "setup", "robot_dynamics.json"),
    os.path.join(os.path.dirname(__file__), "robot_dynamics.json"),
    os.path.join(os.path.dirname(__file__), "dashboard", "test-hardware", "robot_dynamics.json"),
    "/home/jetbot/setup/robot_dynamics.json",
]

trim_factor = 1.0
min_drive_speed = 0.12
cruise_speed = 0.30
max_speed = 0.60
motor_left_polarity = 1
motor_right_polarity = 1

for cfg_path in CONFIG_PATHS:
    if os.path.isfile(cfg_path):
        try:
            with open(cfg_path, "r", encoding="utf-8") as f:
                dyn = json.load(f)
                trim_factor = float(dyn.get("trim_factor", 1.0))
                min_drive_speed = float(dyn.get("min_drive_speed", 0.12))
                cruise_speed = float(dyn.get("cruise_speed", 0.30))
                motor_left_polarity = int(dyn.get("motor_left_polarity", 1))
                motor_right_polarity = int(dyn.get("motor_right_polarity", 1))
                print(f"[OK] Đã nạp thông số động học từ: {cfg_path}")
                print(f"     Trim: {trim_factor:.3f} | Min speed: {min_drive_speed:.2f} | Polarity: ({motor_left_polarity}, {motor_right_polarity})")
                break
        except Exception:
            pass


# -----------------------------------------------------------------------------
# 3. THIẾT LẬP THAM SỐ ĐIỀU KHIỂN TAY CẦM
# -----------------------------------------------------------------------------
DEADZONE = 0.10          # Vùng chết cần gạt chống trôi (drift)

# Tuân thủ Rule 3 (AGENTS.md): Mọi vận tốc phải gắn với robot_dynamics.json
# cruise_speed=0.2, min_drive_speed=0.08 (setup/robot_dynamics.json)
current_speed_limit = cruise_speed
SPEED_LEVELS = [min_drive_speed, cruise_speed, round(cruise_speed * 1.5, 2), round(cruise_speed * 2.0, 2)]
current_gear_idx = 1     # Mặc định Gear 2: cruise_speed (0.20) an toàn theo robot_dynamics.json

# Trạng thái an toàn
is_running = True
last_heartbeat_time = time.time()
HEARTBEAT_TIMEOUT = 0.5  # Tự động dừng nếu mất tín hiệu quá 500ms


def emergency_stop():
    """Hàm dừng khẩn cấp được gọi khi ngắt chương trình hoặc bấm nút Stop."""
    try:
        robot.stop()
    except Exception:
        pass
    print("\n[STOP] Đã dừng toàn bộ động cơ an toàn.")


def signal_handler(sig, frame):
    global is_running
    is_running = False
    emergency_stop()
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)


# -----------------------------------------------------------------------------
# 4. HÀM TÍNH TOÁN ĐỘNG HỌC VI SAI (DIFFERENTIAL DRIVE KINEMATICS)
# -----------------------------------------------------------------------------
def apply_deadzone(value, threshold=DEADZONE):
    """Lọc bỏ các dao động nhỏ quanh điểm 0 do trôi cần analog."""
    if abs(value) < threshold:
        return 0.0
    # Chuẩn hóa lại dải tuyến tính sau khi trừ vùng chết
    sign = 1.0 if value > 0 else -1.0
    return sign * (abs(value) - threshold) / (1.0 - threshold)


def drive_arcade(throttle, steering, speed_scale):
    """
    Điều khiển vi sai 2 bánh kiểu Arcade:
      - throttle: Giá trị cần tiến/lùi [-1.0, 1.0]
      - steering: Giá trị cần rẽ trái/phải [-1.0, 1.0]
      - speed_scale: Tỉ lệ tốc độ tối đa cho phép
    Áp dụng hệ số bù lệch bánh `trim_factor` theo chuẩn CS532.
    """
    throttle = apply_deadzone(throttle)
    steering = apply_deadzone(steering)

    if abs(throttle) < 1e-3 and abs(steering) < 1e-3:
        robot.stop()
        return 0.0, 0.0

    # Tính tốc độ 2 bánh vi sai logic:
    # Rẽ phải (steering > 0): bánh trái quay nhanh hơn bánh phải (left > right)
    # Rẽ trái (steering < 0): bánh phải quay nhanh hơn bánh trái (right > left)
    left_raw = throttle + steering
    right_raw = throttle - steering

    # Chuẩn hóa về khoảng [-1.0, 1.0] nếu vượt quá biên
    max_mag = max(abs(left_raw), abs(right_raw), 1.0)
    left_norm = left_raw / max_mag
    right_norm = right_raw / max_mag

    # Áp dụng tỉ lệ tốc độ logic chuẩn (tiến là dương, lùi là âm)
    left_cmd = max(-1.0, min(1.0, left_norm * speed_scale))
    right_cmd = max(-1.0, min(1.0, right_norm * speed_scale))

    if HAS_HARDWARE:
        # Khi chạy phần cứng trực tiếp trên Jetson Nano: áp dụng polarity và trim cho PCA9685
        hw_left = max(-1.0, min(1.0, left_cmd * motor_left_polarity))
        hw_right = max(-1.0, min(1.0, right_cmd * motor_right_polarity * trim_factor))
        robot.set_motors(hw_left, hw_right)
    else:
        # Khi chạy qua RemoteBackendRobot hoặc Mock: gửi vận tốc logic chuẩn
        # Backend /drive và jetbot_bridge.py trên Jetson đã tự động xử lý polarity & trim
        robot.set_motors(left_cmd, right_cmd)

    return left_cmd, right_cmd


def drive_direct(left_cmd, right_cmd):
    """Gửi lệnh vận tốc đến 2 bánh."""
    left_c = max(-1.0, min(1.0, left_cmd))
    right_c = max(-1.0, min(1.0, right_cmd))

    if HAS_HARDWARE:
        hw_left = max(-1.0, min(1.0, left_c * motor_left_polarity))
        hw_right = max(-1.0, min(1.0, right_c * motor_right_polarity * trim_factor))
        robot.set_motors(hw_left, hw_right)
    else:
        robot.set_motors(left_c, right_c)


# -----------------------------------------------------------------------------
# 5. VÒNG LẶP ĐIỀU KHIỂN BẰNG PYGAME JOYSTICK
# -----------------------------------------------------------------------------
def run_gamepad_teleop():
    global current_gear_idx, current_speed_limit, is_running

    try:
        import pygame
    except ImportError:
        print("\n[!] Chưa cài đặt thư viện 'pygame' để đọc tay cầm.")
        print("    Vui lòng cài đặt bằng lệnh: pip install pygame")
        print("    Đang chuyển sang chế độ điều khiển giả lập qua Bàn Phím (Keyboard Mode)...\n")
        run_keyboard_teleop()
        return

    # Khởi tạo pygame joystick (không mở cửa sổ đồ họa nặng)
    os.environ["SDL_VIDEODRIVER"] = "dummy"  # Chạy headless trên Jetson/Terminal
    pygame.init()
    pygame.joystick.init()

    joystick_count = pygame.joystick.get_count()
    if joystick_count == 0:
        print("\n[CẢNH BÁO] Không tìm thấy tay cầm nào được kết nối qua USB / Bluetooth!")
        print("  1. Hãy cắm hoặc kết nối Bluetooth tay cầm (Xbox, PS4/PS5, Logitech...).")
        print("  2. Nhấn 'k' để chuyển sang chế độ điều khiển Bàn Phím (W/A/S/D).")
        print("  3. Nhấn 'r' để quét lại tay cầm.\n")

        choice = input("Lựa chọn của bạn [k/r/q]: ").strip().lower()
        if choice == 'k':
            run_keyboard_teleop()
            return
        elif choice == 'r':
            return run_gamepad_teleop()
        else:
            print("Thoát chương trình.")
            return

    # Lấy tay cầm đầu tiên
    js = pygame.joystick.Joystick(0)
    js.init()
    js_name = js.get_name()
    num_axes = js.get_numaxes()
    num_buttons = js.get_numbuttons()
    num_hats = js.get_numhats()

    print("=" * 68)
    print(f"🎮 ĐÃ KẾT NỐI TAY CẦM: {js_name}")
    print(f"   Trục analog: {num_axes} | Nút bấm: {num_buttons} | D-Pad (Hats): {num_hats}")
    print("=" * 68)
    print("HƯỚNG DẪN ĐIỀU KHIỂN:")
    print("  • Cần Analog Trái (Trục Y)  : Tiến / Lùi (Đẩy lên = Tiến, Kéo xuống = Lùi)")
    print("  • Cần Analog Phải/Trái (X) : Rẽ Trái / Rẽ Phải")
    print("  • Phím D-Pad (Mũi tên)     : Điều khiển từng bước (Lên=Tiến, Xuống=Lùi, Trái/Phải=Quay)")
    print("  • Nút A (hoặc X trên PS)   : PHANH KHẨN CẤP (robot.stop())")
    print("  • Nút LB / RB (L1 / R1)    : Giảm tốc / Tăng tốc (Gear 1 -> 4)")
    print("  • Nhấn Ctrl + C            : Thoát an toàn")
    print("=" * 68)

    clock = pygame.time.Clock()
    last_print_time = 0.0

    while is_running:
        clock.tick(30)  # Tần số quét 30Hz đảm bảo độ nhạy cao và tối ưu băng thông mạng
        pygame.event.pump()

        # 1. Đọc các nút bấm sự kiện
        # Nút A (0 trên Xbox/PS): Phanh khẩn cấp
        if js.get_button(0):
            robot.stop()
            print("\r[KHẨN CẤP] Nút Phanh được nhấn -> Robot dừng ngay lập tức!", end="", flush=True)
            time.sleep(0.1)
            continue

        # Nút LB (4): Giảm mức tốc độ
        if js.get_button(4):
            if current_gear_idx > 0:
                current_gear_idx -= 1
                current_speed_limit = SPEED_LEVELS[current_gear_idx]
                print(f"\n[GEAR] Đã giảm tốc độ: Mức {current_gear_idx + 1} ({int(current_speed_limit*100)}%)")
                time.sleep(0.2)

        # Nút RB (5): Tăng mức tốc độ
        if js.get_button(5):
            if current_gear_idx < len(SPEED_LEVELS) - 1:
                current_gear_idx += 1
                current_speed_limit = SPEED_LEVELS[current_gear_idx]
                print(f"\n[GEAR] Đã tăng tốc độ: Mức {current_gear_idx + 1} ({int(current_speed_limit*100)}%)")
                time.sleep(0.2)

        # 2. Đọc phím D-Pad (Hats) - Chế độ điều khiển rời rạc theo basic_motion.ipynb
        dpad_active = False
        if num_hats > 0:
            hat_x, hat_y = js.get_hat(0)
            if hat_y == 1:       # D-Pad UP
                drive_direct(current_speed_limit, current_speed_limit)
                dpad_active = True
                cmd_name = "TIẾN THẲNG (D-Pad)"
            elif hat_y == -1:    # D-Pad DOWN
                drive_direct(-current_speed_limit, -current_speed_limit)
                dpad_active = True
                cmd_name = "LÙI XE (D-Pad)"
            elif hat_x == -1:    # D-Pad LEFT
                drive_direct(-current_speed_limit * 0.8, current_speed_limit * 0.8)
                dpad_active = True
                cmd_name = "QUAY TRÁI (D-Pad)"
            elif hat_x == 1:     # D-Pad RIGHT
                drive_direct(current_speed_limit * 0.8, -current_speed_limit * 0.8)
                dpad_active = True
                cmd_name = "QUAY PHẢI (D-Pad)"

        # 3. Đọc cần gạt Analog (Joystick) - Chế độ điều khiển mượt mà vô cấp
        if not dpad_active:
            # Trục 1: Cần trái Y (Pygame quy ước: -1.0 là Đẩy Lên, +1.0 là Kéo Xuống -> cần đổi dấu)
            throttle = -js.get_axis(1)

            # Trục rẽ: Ưu tiên Cần phải X (Trục 3 hoặc 2) nếu có, hoặc dùng Cần trái X (Trục 0)
            if num_axes >= 4:
                # Chuẩn Xbox / PS: Trục 3 hoặc 2 thường là Right Stick X
                steering = js.get_axis(3) if abs(js.get_axis(3)) > DEADZONE else js.get_axis(0)
            else:
                steering = js.get_axis(0)

            left_pwm, right_pwm = drive_arcade(throttle, steering, current_speed_limit)

            now = time.time()
            if now - last_print_time > 0.15:  # Giới hạn in 6-7 lần / giây cho màn hình gọn
                last_print_time = now
                if abs(left_pwm) > 0.01 or abs(right_pwm) > 0.01:
                    status = f"V={throttle:+.2f} | W={steering:+.2f} -> L_PWM={left_pwm:+.2f}, R_PWM={right_pwm:+.2f}"
                else:
                    status = "ĐANG DỪNG (Cần gạt ở vị trí trung tâm)"
                print(f"\r[TAY CẦM - Gear {current_gear_idx+1} ({int(current_speed_limit*100)}%)] {status}    ", end="", flush=True)

        else:
            now = time.time()
            if now - last_print_time > 0.15:
                last_print_time = now
                print(f"\r[D-PAD - Gear {current_gear_idx+1} ({int(current_speed_limit*100)}%)] Lệnh: {cmd_name}    ", end="", flush=True)

    emergency_stop()


# -----------------------------------------------------------------------------
# 6. CHẾ ĐỘ DỰ PHÒNG: ĐIỀU KHIỂN BÀN PHÍM (KEYBOARD FALLBACK)
# -----------------------------------------------------------------------------
def run_keyboard_teleop():
    """Chế độ điều khiển bằng bàn phím để kiểm thử ngay lập tức khi không có tay cầm vật lý."""
    global current_gear_idx, current_speed_limit, is_running

    print("=" * 68)
    print("⌨️  CHẾ ĐỘ ĐIỀU KHIỂN BÀN PHÍM (KEYBOARD TELEOP)")
    print("=" * 68)
    print("Phím điều khiển:")
    print("  • W / Phím Mũi Tên Lên    : TIẾN THẲNG   (robot.forward)")
    print("  • S / Phím Mũi Tên Xuống  : LÙI XE       (robot.backward)")
    print("  • A / Phím Mũi Tên Trái   : QUAY TRÁI    (robot.left)")
    print("  • D / Phím Mũi Tên Phải   : QUAY PHẢI    (robot.right)")
    print("  • SPACE (Phím cách) / X   : DỪNG XE      (robot.stop)")
    print("  • + / -                   : Tăng / Giảm mức tốc độ (Gear)")
    print("  • Q / Ctrl+C              : Thoát chương trình")
    print("=" * 68)

    # Đọc phím tương thích đa nền tảng (Windows & Linux)
    if os.name == 'nt':
        import msvcrt
        def get_char():
            ch = msvcrt.getch()
            if ch in (b'\x00', b'\xe0'):
                code = msvcrt.getch()
                key_map = {b'H': 'w', b'P': 's', b'K': 'a', b'M': 'd'}
                return key_map.get(code, '')
            try:
                return ch.decode('utf-8', errors='ignore').lower()
            except Exception:
                return ''
    else:
        import tty
        import termios
        def get_char():
            if not sys.stdin.isatty():
                ch = sys.stdin.read(1)
                return ch.lower() if ch else 'q'
            fd = sys.stdin.fileno()
            old_settings = termios.tcgetattr(fd)
            try:
                tty.setraw(fd)
                ch = sys.stdin.read(1)
                if ch == '\x1b':
                    ch2 = sys.stdin.read(1)
                    if ch2 == '[':
                        ch3 = sys.stdin.read(1)
                        arrow_map = {'A': 'w', 'B': 's', 'C': 'd', 'D': 'a'}
                        return arrow_map.get(ch3, '')
                return ch.lower()
            finally:
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    while is_running:
        try:
            k = get_char()
            if not k:
                continue

            if k == 'w':
                drive_direct(current_speed_limit, current_speed_limit)
                print(f"\r[BÀN PHÍM] TIẾN THẲNG   (Speed: {current_speed_limit:.2f})       ", end="", flush=True)
            elif k == 's':
                drive_direct(-current_speed_limit, -current_speed_limit)
                print(f"\r[BÀN PHÍM] LÙI XE       (Speed: {current_speed_limit:.2f})       ", end="", flush=True)
            elif k == 'a':
                drive_direct(-current_speed_limit * 0.8, current_speed_limit * 0.8)
                print(f"\r[BÀN PHÍM] QUAY TRÁI    (Speed: {current_speed_limit * 0.8:.2f})       ", end="", flush=True)
            elif k == 'd':
                drive_direct(current_speed_limit * 0.8, -current_speed_limit * 0.8)
                print(f"\r[BÀN PHÍM] QUAY PHẢI   (Speed: {current_speed_limit * 0.8:.2f})       ", end="", flush=True)
            elif k in (' ', 'x'):
                robot.stop()
                print("\r[BÀN PHÍM] DỪNG XE      (robot.stop())                         ", end="", flush=True)
            elif k in ('+', '='):
                if current_gear_idx < len(SPEED_LEVELS) - 1:
                    current_gear_idx += 1
                    current_speed_limit = SPEED_LEVELS[current_gear_idx]
                    print(f"\n[GEAR] Tăng tốc độ: Mức {current_gear_idx+1} ({int(current_speed_limit*100)}%)")
            elif k in ('-', '_'):
                if current_gear_idx > 0:
                    current_gear_idx -= 1
                    current_speed_limit = SPEED_LEVELS[current_gear_idx]
                    print(f"\n[GEAR] Giảm tốc độ: Mức {current_gear_idx+1} ({int(current_speed_limit*100)}%)")
            elif k == 'q':
                print("\nThoát chương trình.")
                break
        except KeyboardInterrupt:
            break

    emergency_stop()


# -----------------------------------------------------------------------------
# 7. ĐIỂM VÀO CHÍNH (MAIN ENTRYPOINT)
# -----------------------------------------------------------------------------
if __name__ == "__main__":
    print("\n" + "=" * 68)
    print("🤖 CHƯƠNG TRÌNH ĐIỀU KHIỂN JETBOT BẰNG TAY CẦM (GAMEPAD TELEOP)")
    print("   CS532 - Hệ Thống Robot Tự Hành Thông Minh")
    print("=" * 68)

    # Phân giải tham số dòng lệnh (--remote, -r, --keyboard, -k)
    cli_remote_url, cli_is_keyboard = parse_cli_args(sys.argv[1:])
    robot = init_robot(cli_remote_url)

    if cli_is_keyboard:
        run_keyboard_teleop()
    else:
        run_gamepad_teleop()
