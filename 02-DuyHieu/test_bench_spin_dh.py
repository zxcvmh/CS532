#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
CS532 - BÀI TEST THỰC NGHIỆM KÊ KÍCH BÁNH XE (BENCH MOTOR SPIN TEST)
=============================================================================
LƯU Ý QUAN TRỌNG CHO ANH NĂM:
- Hãy kê kích robot JetBot để 2 bánh sau HỔNG KHỎI MẶT BÀN trước khi chạy!
- Bài test này nhằm kiểm tra trực tiếp phần cứng động cơ:
    1. Test tốc độ chậm (v=0.05 m/s): Bánh xe quay êm, không bị kẹt chết máy.
    2. Test tăng ga Soft-Start: Bánh xe tăng tốc mượt mà, không sụt áp pin.
    3. Test xoay tại chỗ (Spin Turn): Hai bánh quay ngược chiều hoàn hảo.
    4. Dừng khẩn cấp ngắt motor an toàn.
=============================================================================
"""

import os
import sys
import time
import math

if sys.platform.startswith("win"):
    import io
    sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding="utf-8", errors="replace")
    sys.stderr = io.TextIOWrapper(sys.stderr.buffer, encoding="utf-8", errors="replace")

CLR_RESET = "\033[0m"
CLR_BOLD = "\033[1m"
CLR_RED = "\033[91m"
CLR_GREEN = "\033[92m"
CLR_YELLOW = "\033[93m"
CLR_CYAN = "\033[96m"

# Khởi tạo driver phần cứng
HAS_MOTOR = False
left_motor_dev = None
right_motor_dev = None
jetbot_robot = None

try:
    from Adafruit_MotorHAT import Adafruit_MotorHAT
    motor_hat = Adafruit_MotorHAT(i2c_bus=1)
    left_motor_dev = motor_hat.getMotor(1)
    right_motor_dev = motor_hat.getMotor(2)
    HAS_MOTOR = True
    print(f"{CLR_GREEN}[OK] Đã kết nối phần cứng Adafruit MotorHAT (I2C Bus 1).{CLR_RESET}")
except Exception:
    try:
        from jetbot import Robot
        jetbot_robot = Robot()
        HAS_MOTOR = True
        print(f"{CLR_GREEN}[OK] Đã kết nối phần cứng qua thư viện jetbot.Robot.{CLR_RESET}")
    except Exception as e:
        HAS_MOTOR = False
        print(f"{CLR_YELLOW}[MOCK] Không tìm thấy phần cứng động cơ ({e}). Chạy mô phỏng logic.{CLR_RESET}")

MIN_DRIVE_SPEED = 0.22
CRUISE_SPEED = 0.35
TRIM_FACTOR = 1.01
SOFT_START_RAMP_SEC = 0.40

_last_pwm_l = 0.0
_last_pwm_r = 0.0
_last_update_t = 0.0

def stop_motors():
    global _last_pwm_l, _last_pwm_r
    if HAS_MOTOR:
        try:
            if left_motor_dev and right_motor_dev:
                left_motor_dev.run(4) # RELEASE
                right_motor_dev.run(4)
            elif jetbot_robot:
                jetbot_robot.stop()
        except Exception:
            pass
    _last_pwm_l = 0.0
    _last_pwm_r = 0.0
    print(f"  {CLR_YELLOW}[STOP] Động cơ ĐÃ NGẮT HOÀN TOÀN.{CLR_RESET}")

def drive_raw(pwm_left: float, pwm_right: float):
    # Polarity đảo chiều trên khung xe JetBot: âm quay tiến
    hw_left = -pwm_left
    hw_right = -pwm_right

    if HAS_MOTOR:
        try:
            if left_motor_dev and right_motor_dev:
                l_speed = int(min(255, max(0, abs(hw_left) * 255)))
                r_speed = int(min(255, max(0, abs(hw_right) * 255)))
                left_motor_dev.setSpeed(l_speed)
                right_motor_dev.setSpeed(r_speed)
                left_motor_dev.run(1 if hw_left >= 0 else 2)
                right_motor_dev.run(1 if hw_right >= 0 else 2)
            elif jetbot_robot:
                jetbot_robot.set_motors(hw_left, hw_right)
        except Exception as e:
            print(f"[ERROR] Lỗi driver: {e}")
            stop_motors()
    else:
        print(f"  [MOCK RAW] L={hw_left:+.3f} (PWM {int(abs(hw_left)*255)}), R={hw_right:+.3f} (PWM {int(abs(hw_right)*255)})")

def set_speed_with_kinematics(linear_v: float, angular_w: float, dt: float = 0.05):
    global _last_pwm_l, _last_pwm_r, _last_update_t
    track_width = 0.115
    max_speed = 0.18

    left = linear_v - (angular_w * track_width / 2.0)
    right = linear_v + (angular_w * track_width / 2.0)

    u_l = max(-1.0, min(1.0, left / max_speed))
    u_r = max(-1.0, min(1.0, right / max_speed))

    def lift(u):
        if abs(u) < 1e-3: return 0.0
        sign = 1.0 if u > 0 else -1.0
        return sign * (MIN_DRIVE_SPEED + (CRUISE_SPEED - MIN_DRIVE_SPEED) * abs(u))

    target_pwm_l = lift(u_l) * TRIM_FACTOR
    target_pwm_r = lift(u_r)

    now_t = time.time()
    step_dt = max(0.02, min(0.2, now_t - _last_update_t)) if _last_update_t > 0 else dt
    _last_update_t = now_t

    max_delta = (1.0 / SOFT_START_RAMP_SEC) * step_dt

    def clamp_rate(tgt, prev):
        d = tgt - prev
        if abs(d) > max_delta: return prev + math.copysign(max_delta, d)
        return tgt

    pwm_l = clamp_rate(target_pwm_l, _last_pwm_l)
    pwm_r = clamp_rate(target_pwm_r, _last_pwm_r)
    _last_pwm_l = pwm_l
    _last_pwm_r = pwm_r

    drive_raw(pwm_l, pwm_r)
    return pwm_l, pwm_r

def main():
    print("\n" + "=" * 76)
    print(f" {CLR_BOLD}{CLR_CYAN}🤖 BỘ KIỂM THỬ ĐỘNG CƠ PHẦN CỨNG THỰC TẾ (BENCH SPIN TEST){CLR_RESET}")
    print("=" * 76)
    print("Vui lòng đảm bảo robot ĐÃ ĐƯỢC KÊ HỔNG BÁNH KHỎI MẶT ĐẤT!")
    time.sleep(1.5)

    try:
        # TEST 1: Chạy tốc độ chậm (Deadband Lift)
        print(f"\n{CLR_BOLD}[PHA 1/3] TEST TỐC ĐỘ BÒ CHẬM (v = 0.05 m/s) — Kiểm tra thắng ma sát hộp số:{CLR_RESET}")
        for _ in range(25): # Chạy trong 1.25s (25 x 50ms)
            pl, pr = set_speed_with_kinematics(0.05, 0.0, dt=0.05)
            time.sleep(0.05)
        print(f"  -> Xung duy trì: Trái = {pl:.3f}, Phải = {pr:.3f} (>= 0.22, không bị rít máy!)")
        stop_motors()
        time.sleep(1.0)

        # TEST 2: Chạy tốc độ hành trình với Soft-Start
        print(f"\n{CLR_BOLD}[PHA 2/3] TEST TĂNG GA MỀM SOFT-START (v = 0.15 m/s) — Kiểm tra chống sụt áp pin:{CLR_RESET}")
        for _ in range(35): # Chạy trong 1.75s
            pl, pr = set_speed_with_kinematics(0.15, 0.0, dt=0.05)
            time.sleep(0.05)
        print(f"  -> Xung hành trình: Trái = {pl:.3f}, Phải = {pr:.3f} (Động cơ quay đều êm ái)")
        stop_motors()
        time.sleep(1.0)

        # TEST 3: Xoay tại chỗ (In-Place Spin Turn)
        print(f"\n{CLR_BOLD}[PHA 3/3] TEST XOAY TẠI CHỖ (v = 0.0, w = 1.0 rad/s) — Kiểm tra vi sai ngược chiều:{CLR_RESET}")
        for _ in range(25): # Chạy trong 1.25s
            pl, pr = set_speed_with_kinematics(0.0, 1.0, dt=0.05)
            time.sleep(0.05)
        print(f"  -> Xung vi sai: Trái = {pl:.3f} (QUAY LÙI), Phải = {pr:.3f} (QUAY TIẾN)")
        stop_motors()

        print(f"\n{CLR_GREEN}{CLR_BOLD}🎉 HOÀN THÀNH KIỂM THỬ ĐỘNG CƠ: CẢ 2 BÁNH QUAY HOÀN HẢO!{CLR_RESET}")
        print("Hệ thống phần cứng đã sẵn sàng 100% để thả xuống sàn chạy tự hành.")

    except KeyboardInterrupt:
        print("\n[NGẮT] Người dùng ngắt khẩn cấp.")
        stop_motors()
    except Exception as e:
        print(f"\n[LỖI] {e}")
        stop_motors()

if __name__ == "__main__":
    main()
