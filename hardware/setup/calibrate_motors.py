#!/usr/bin/env python3
"""
CS532 - Bộ Công Cụ Hiệu Chuẩn Động Học Thực Nghiệm Cho JetBot
Tác giả: robot-setup-agent
Tính năng mới:
    - Cho phép người dùng TỰ DO TÙY CHỈNH mọi tham số (ga, thời gian, trim, góc xoay)
    - Không ép buộc thông số mặc định; có thể nhập số trực tiếp bằng tay
    - Test 3 sửa lỗi chạy lố: Cho phép tự chọn thời gian chạy hoặc đo bấm giờ linh hoạt
    - Có menu sửa trực tiếp toàn bộ giá trị trong file cấu hình
"""

import os
import sys
import time
import json

CONFIG_FILE = os.path.join(os.path.dirname(__file__), "robot_dynamics.json")

# Hướng động cơ mặc định (-1 = đảo cực theo chuẩn khung JetBot để TIẾN THẲNG)
MOTOR_LEFT_POLARITY = -1
MOTOR_RIGHT_POLARITY = -1

# Khởi tạo driver động cơ an toàn
HAS_HARDWARE = False
left_motor = None
right_motor = None
jetbot_robot = None

try:
    from Adafruit_MotorHAT import Adafruit_MotorHAT
    motor_hat = Adafruit_MotorHAT(i2c_bus=1)
    left_motor = motor_hat.getMotor(1)
    right_motor = motor_hat.getMotor(2)
    HAS_HARDWARE = True
    print("[OK] Kết nối Adafruit MotorHAT (I2C Bus 1) thành công.")
except Exception:
    try:
        from jetbot import Robot
        jetbot_robot = Robot()
        HAS_HARDWARE = True
        print("[OK] Kết nối qua thư viện jetbot.Robot thành công.")
    except Exception as e:
        HAS_HARDWARE = False
        print(f"[CẢNH BÁO] Không tìm thấy phần cứng động cơ ({e}). Chạy ở chế độ MOCK.")

def stop():
    """Dừng khẩn cấp toàn bộ động cơ."""
    if HAS_HARDWARE:
        try:
            if left_motor and right_motor:
                left_motor.run(4)   # 4 = RELEASE
                right_motor.run(4)
            elif jetbot_robot:
                jetbot_robot.stop()
        except Exception:
            pass
    print(" -> [STOP] Động cơ ĐÃ DỪNG.")

def drive_raw(left_speed: float, right_speed: float):
    """Cấp xung động cơ từ -1.0 đến +1.0."""
    hw_left = left_speed * MOTOR_LEFT_POLARITY
    hw_right = right_speed * MOTOR_RIGHT_POLARITY

    if not HAS_HARDWARE:
        print(f"   [MOCK MOTOR] Lệnh logic: L={left_speed:+.2f}, R={right_speed:+.2f} | Phần cứng: L={hw_left:+.2f}, R={hw_right:+.2f}")
        return

    try:
        if left_motor and right_motor:
            l_pwm = int(min(255, max(0, abs(hw_left) * 255)))
            r_pwm = int(min(255, max(0, abs(hw_right) * 255)))
            left_motor.setSpeed(l_pwm)
            right_motor.setSpeed(r_pwm)
            left_motor.run(1 if hw_left >= 0 else 2)
            right_motor.run(1 if hw_right >= 0 else 2)
        elif jetbot_robot:
            jetbot_robot.set_motors(hw_left, hw_right)
    except Exception as e:
        print(f"[LỖI ĐỘNG CƠ] {e}")
        stop()

def soft_start_drive(left: float, right: float, duration: float, ramp_time: float = 0.25):
    """
    Điều khiển hình thang (Trapezoidal Profile):
    - Soft Start: Tăng ga êm từ từ
    - Cruise: Duy trì tốc độ ổn định
    - Soft Stop: Hạ ga từ từ trước khi ngắt để TRIỆT TIÊU HIỆN TƯỢNG VẸO MŨI KHI DỪNG
    """
    ramp = min(ramp_time, duration * 0.35)
    steps = 5
    dt = ramp / steps if steps > 0 else 0

    # 1. Soft Start (Tăng ga dần)
    for i in range(1, steps + 1):
        frac = i / steps
        drive_raw(left * frac, right * frac)
        time.sleep(dt)

    # 2. Cruise (Duy trì ga hành trình)
    hold_time = max(0.0, duration - (2.0 * ramp))
    if hold_time > 0:
        time.sleep(hold_time)

    # 3. Soft Stop (Hạ ga êm ái về 0 để không bị quán tính vặn lệch thân xe)
    for i in range(steps - 1, 0, -1):
        frac = i / steps
        drive_raw(left * frac, right * frac)
        time.sleep(dt)

    # 4. Ngắt hẳn động cơ
    stop()

def load_current_config():
    global MOTOR_LEFT_POLARITY, MOTOR_RIGHT_POLARITY
    if os.path.exists(CONFIG_FILE):
        try:
            with open(CONFIG_FILE, "r", encoding="utf-8") as f:
                data = json.load(f)
                MOTOR_LEFT_POLARITY = data.get("motor_left_polarity", -1)
                MOTOR_RIGHT_POLARITY = data.get("motor_right_polarity", -1)
                return data
        except Exception:
            pass
    return {
        "hardware": "JetBot-GA25-370",
        "floor_surface": "glossy_ceramic_tile",
        "tested_arena": "2m_x_1m",
        "min_drive_speed": 0.22,
        "cruise_speed": 0.26,
        "trim_factor": 1.0,
        "turn_speed": 0.22,
        "turn_time_90_deg_sec": 0.95,
        "turn_time_180_deg_sec": 1.90,
        "actual_speed_mps": 0.18,
        "soft_start_ramp_sec": 0.3,
        "motor_left_polarity": -1,
        "motor_right_polarity": -1,
        "note": "Cau hinh thuc nghiem toi uu cho CS532"
    }

def save_config(cfg):
    cfg["motor_left_polarity"] = MOTOR_LEFT_POLARITY
    cfg["motor_right_polarity"] = MOTOR_RIGHT_POLARITY
    with open(CONFIG_FILE, "w", encoding="utf-8") as f:
        json.dump(cfg, f, indent=4, ensure_ascii=False)
    print(f"\n[THÀNH CÔNG] Đã lưu thông số vào: {CONFIG_FILE}")

def prompt_float(prompt_text: str, default_val: float) -> float:
    """Hàm nhập số thực linh hoạt: gõ Enter để giữ nguyên mặc định."""
    val_str = input(f"{prompt_text} [Hiện tại: {default_val}]: ").strip()
    if not val_str:
        return default_val
    try:
        return float(val_str)
    except ValueError:
        print(f"[!] Không phải số hợp lệ. Giữ nguyên giá trị {default_val}")
        return default_val

# ─────────────────────────────────────────────────────────────
# CÁC BÀI TEST TÙY BIẾN THAM SỐ
# ─────────────────────────────────────────────────────────────

def test_1_deadband(cfg):
    print("\n" + "="*65)
    print("BÀI TEST 1: TÌM GA LĂN BÁNH TIẾN THẲNG (TÙY CHỈNH THAM SỐ)")
    print("="*65)
    
    current_min = cfg.get("min_drive_speed", 0.22)
    duration = 1.0
    
    while True:
        spd = prompt_float("-> Nhập mức ga bạn muốn thử nghiệm (0.10 - 0.40)", current_min)
        duration = prompt_float("-> Nhập thời gian chạy thử (giây)", duration)
        
        input(f"Nhấn [ENTER] để chạy thử (Ga={spd:.2f}, Thời gian={duration:.1f}s)...")
        drive_raw(spd, spd)
        time.sleep(duration)
        stop()
        
        print("\nKết quả thử nghiệm:")
        print("  [1] Mức ga này RẤT TỐT (Lăn bánh êm, chốt giá trị này)")
        print("  [2] Thử lại với mức ga hoặc thời gian khác")
        print("  [0] Quay lại Menu chính")
        c = input("Chọn (1/2/0): ").strip()
        if c == '1':
            cfg["min_drive_speed"] = round(spd, 2)
            cfg["cruise_speed"] = round(spd * 1.2, 2)
            cfg["turn_speed"] = round(spd, 2)
            print(f"==> ĐÃ LƯU: Ga tối thiểu = {cfg['min_drive_speed']}, Ga hành trình = {cfg['cruise_speed']}")
            save_config(cfg)
            break
        elif c == '0':
            break

def test_2_straight_trim(cfg):
    print("\n" + "="*65)
    print("BÀI TEST 2: CÂN BẰNG 2 BÁNH TIẾN THẲNG (TÙY CHỈNH TRIM & THỜI GIAN)")
    print("="*65)
    
    base_spd = cfg.get("min_drive_speed", 0.22)
    trim = cfg.get("trim_factor", 1.0)
    duration = 1.5
    
    while True:
        print(f"\nThông số hiện tại: Ga={base_spd:.2f} | Trim={trim:.3f} | Thời gian chạy={duration:.1f}s")
        print("Các tùy chọn:")
        print(f"  [Enter] Chạy thử xe ngay với thông số trên")
        print(f"  [1] Xe đã TIẾN THẲNG CHUẨN MẠCH GẠCH (Lưu & Hoàn tất)")
        print(f"  [2] Xe bị lệch PHẢI -> Tăng nhẹ trim (+0.02)")
        print(f"  [3] Xe bị lệch TRÁI -> Giảm nhẹ trim (-0.02)")
        print(f"  [4] Tự tay nhập giá trị Trim mới")
        print(f"  [5] Tự tay đổi mức ga hoặc thời gian chạy thử")
        print(f"  [0] Quay lại Menu")
        c = input("Chọn hoặc nhấn Enter để chạy: ").strip()
        
        if c == "":
            l_spd = base_spd
            r_spd = base_spd * trim
            print(f"-> Đang chạy thử tiến thẳng trong {duration:.1f}s...")
            soft_start_drive(l_spd, r_spd, duration=duration, ramp_time=cfg.get("soft_start_ramp_sec", 0.3))
        elif c == '1':
            cfg["trim_factor"] = round(trim, 3)
            print(f"==> ĐÃ CHỐT: Trim Factor = {cfg['trim_factor']}")
            save_config(cfg)
            break
        elif c == '2':
            trim += 0.02
            print(f"-> Đã tăng trim lên {trim:.3f}")
        elif c == '3':
            trim -= 0.02
            print(f"-> Đã giảm trim xuống {trim:.3f}")
        elif c == '4':
            trim = prompt_float("Nhập hệ số Trim (ví dụ: 0.96 hoặc 1.04)", trim)
        elif c == '5':
            base_spd = prompt_float("Nhập mức ga", base_spd)
            duration = prompt_float("Nhập thời gian chạy thử (giây)", duration)
        elif c == '0':
            break

def test_3_measure_speed_1m(cfg):
    print("\n" + "="*65)
    print("BÀI TEST 3: ĐO VẬN TỐC TIẾN 1 MÉT (KHÔNG LO CHẠY LỐ)")
    print("="*65)
    print("Chọn phương thức kiểm tra:")
    print("  [1] TỰ NHẬP THỜI GIAN CHẠY (Khuyên dùng: ví dụ xe chỉ chạy 2.0s hoặc 3.0s)")
    print("  [2] TỰ TAY NHẬP TRỰC TIẾP VẬN TỐC (m/s) nếu bạn đã đo trước đó")
    print("  [0] Quay lại")
    sub_c = input("Chọn (1/2/0): ").strip()
    
    if sub_c == '2':
        curr_mps = cfg.get("actual_speed_mps", 0.18)
        new_mps = prompt_float("Nhập vận tốc thực tế (m/s) (ví dụ: 0.20)", curr_mps)
        cfg["actual_speed_mps"] = round(new_mps, 3)
        save_config(cfg)
        return
    elif sub_c != '1':
        return

    base_spd = cfg.get("cruise_speed", 0.26)
    trim = cfg.get("trim_factor", 1.0)
    test_sec = 2.5
    
    while True:
        print(f"\n[THÔNG SỐ TEST 3] Ga={base_spd:.2f} | Trim={trim:.3f} (Trái={base_spd:.2f}, Phải={base_spd*trim:.2f}) | Thời gian={test_sec:.1f}s")
        base_spd = prompt_float("-> Mức ga chạy thử", base_spd)
        trim = prompt_float("-> Hệ số Trim 2 bánh (nếu bị lệch, hãy chỉnh số này)", trim)
        test_sec = prompt_float("-> Số giây xe chạy (để xe KHÔNG CHẠY LỐ 1 mét)", test_sec)
        
        input(f"\nĐặt xe ở vạch 0m rồi nhấn [ENTER] để xe chạy đúng {test_sec:.1f}s...")
        soft_start_drive(base_spd, base_spd * trim, duration=test_sec, ramp_time=0.3)
        
        print("\nXe vừa dừng lại. Hãy lấy thước đo khoảng cách xe vừa đi được:")
        print("  [1] Xe đi ĐÚNG CHUẨN 1.0 mét (100 cm)")
        print("  [2] Xe đi chưa tới 1.0m (Cần tăng số giây lên)")
        print("  [3] Xe đi lố qua 1.0m (Cần giảm bớt số giây xuống)")
        print("  [4] Nhập khoảng cách thực tế (cm) xe vừa đi được để máy tự tính")
        print("  [0] Thoát bài test")
        c = input("Chọn (1/2/3/4/0): ").strip()
        
        if c == '1':
            actual_mps = round(1.0 / test_sec, 3)
            cfg["actual_speed_mps"] = actual_mps
            cfg["cruise_speed"] = round(base_spd, 2)
            cfg["trim_factor"] = round(trim, 3)
            print(f"==> Vận tốc tính được: {actual_mps} m/s | Trim đã lưu: {trim:.3f}")
            save_config(cfg)
            break
        elif c == '2':
            test_sec += 0.3
            print(f"-> Tăng thời gian lên {test_sec:.1f}s")
        elif c == '3':
            test_sec = max(0.5, test_sec - 0.3)
            print(f"-> Giảm thời gian xuống {test_sec:.1f}s")
        elif c == '4':
            cm_str = input("Nhập số cm xe vừa đi được (ví dụ 85 hoặc 110): ").strip()
            try:
                cm = float(cm_str)
                if cm > 10:
                    dist_m = cm / 100.0
                    actual_mps = round(dist_m / test_sec, 3)
                    cfg["actual_speed_mps"] = actual_mps
                    cfg["cruise_speed"] = round(base_spd, 2)
                    print(f"==> Vận tốc tính được: {actual_mps} m/s (Đi {dist_m:.2f}m trong {test_sec:.1f}s)")
                    save_config(cfg)
                    break
            except ValueError:
                print("[!] Số không hợp lệ.")
        elif c == '0':
            break

def test_4_point_turn(cfg):
    print("\n" + "="*65)
    print("BÀI TEST 4: ĐO THỜI GIAN XOAY TẠI CHỖ 90° & 180° (TÙY CHỈNH)")
    print("="*65)
    
    turn_spd = cfg.get("turn_speed", 0.22)
    trim = cfg.get("trim_factor", 1.0)
    t90 = cfg.get("turn_time_90_deg_sec", 0.95)
    
    while True:
        print(f"\nThông số xoay: Ga xoay={turn_spd:.2f} | Thời gian quay 90°={t90:.2f}s")
        print("  [Enter] Cho xe xoay thử ngay")
        print("  [1] Xe đã xoay CHUẨN GÓC VUÔNG 90° (Lưu & Hoàn tất)")
        print("  [2] Chưa tới 90° (Tự động tăng +0.08s)")
        print("  [3] Quay quá 90° (Tự động giảm -0.08s)")
        print("  [4] Tự tay nhập thời gian xoay 90° mong muốn")
        print("  [5] Tự tay đổi mức ga xoay")
        print("  [0] Quay lại")
        c = input("Chọn: ").strip()
        
        if c == "":
            print(f"-> Đang xoay tại chỗ trong {t90:.2f}s...")
            drive_raw(turn_spd, -turn_spd * trim)
            time.sleep(t90)
            stop()
        elif c == '1':
            cfg["turn_time_90_deg_sec"] = round(t90, 2)
            cfg["turn_time_180_deg_sec"] = round(t90 * 2.0, 2)
            cfg["turn_speed"] = round(turn_spd, 2)
            print(f"==> ĐÃ CHỐT: Xoay 90° = {cfg['turn_time_90_deg_sec']}s | Xoay 180° = {cfg['turn_time_180_deg_sec']}s")
            save_config(cfg)
            break
        elif c == '2':
            t90 += 0.08
            print(f"-> Tăng thời gian lên {t90:.2f}s")
        elif c == '3':
            t90 = max(0.2, t90 - 0.08)
            print(f"-> Giảm thời gian xuống {t90:.2f}s")
        elif c == '4':
            t90 = prompt_float("Nhập thời gian xoay 90 độ (giây)", t90)
        elif c == '5':
            turn_spd = prompt_float("Nhập mức ga xoay (0.15 - 0.35)", turn_spd)
        elif c == '0':
            break

def test_5_dry_run_rectangle(cfg):
    print("\n" + "="*65)
    print("BÀI TEST 5: CHẠY THỬ VÒNG HÌNH CHỮ NHẬT (TÙY CHỈNH KÍCH THƯỚC)")
    print("="*65)
    
    spd = cfg.get("cruise_speed", 0.26)
    trim = cfg.get("trim_factor", 1.0)
    mps = cfg.get("actual_speed_mps", 0.20)
    t90 = cfg.get("turn_time_90_deg_sec", 0.95)
    
    len_long = prompt_float("Nhập chiều dài cạnh dài (mét) [mặc định 0.8m]", 0.8)
    len_short = prompt_float("Nhập chiều dài cạnh ngắn (mét) [mặc định 0.4m]", 0.4)
    
    input(f"\n-> Đặt xe vào góc ô 2m x 1m rồi nhấn [ENTER] để chạy vòng {len_long}m x {len_short}m...")
    
    t_long = len_long / mps
    t_short = len_short / mps
    
    for leg in range(4):
        dist = len_long if leg % 2 == 0 else len_short
        t_run = t_long if leg % 2 == 0 else t_short
        print(f" [Cạnh {leg+1}/4] TIẾN THẲNG {dist}m trong {t_run:.1f}s...")
        soft_start_drive(spd, spd * trim, duration=t_run, ramp_time=0.25)
        time.sleep(0.4)
        
        print(f" [Cạnh {leg+1}/4] Xoay tại chỗ 90° ({t90:.2f}s)...")
        drive_raw(cfg["turn_speed"], -cfg["turn_speed"] * trim)
        time.sleep(t90)
        stop()
        time.sleep(0.4)
        
    print("\n==> HOÀN TẤT VÒNG THỬ NGHIỆM!")

def edit_all_params_manually(cfg):
    """Cho phép người dùng tự tay sửa bất kỳ tham số nào trực tiếp."""
    global MOTOR_LEFT_POLARITY, MOTOR_RIGHT_POLARITY
    while True:
        print("\n" + "="*65)
        print("          BẢNG TÙY CHỈNH TẤT CẢ THAM SỐ THỦ CÔNG")
        print("="*65)
        keys = [
            ("min_drive_speed", "Ga tối thiểu lăn bánh"),
            ("cruise_speed", "Ga hành trình chạy thẳng"),
            ("trim_factor", "Hệ số cân bằng 2 bánh (Trim)"),
            ("actual_speed_mps", "Vận tốc thực tế (m/s)"),
            ("turn_speed", "Ga khi xoay tại chỗ"),
            ("turn_time_90_deg_sec", "Thời gian xoay 90 độ (giây)"),
            ("turn_time_180_deg_sec", "Thời gian xoay 180 độ (giây)"),
            ("soft_start_ramp_sec", "Thời gian tăng ga mềm (Soft start)"),
        ]
        
        for idx, (k, desc) in enumerate(keys, 1):
            val = cfg.get(k, 0.0)
            print(f" [{idx}] {desc:<35} : {val}")
        print(" [9] Đảo chiều động cơ (Hiện tại: Left={}, Right={})".format(MOTOR_LEFT_POLARITY, MOTOR_RIGHT_POLARITY))
        print(" [0] Lưu và Thoát ra Menu chính")
        print("="*65)
        
        choice = input("Nhập số thứ tự tham số muốn sửa (0-9): ").strip()
        if choice == '0':
            save_config(cfg)
            break
        elif choice == '9':
            MOTOR_LEFT_POLARITY = -MOTOR_LEFT_POLARITY
            MOTOR_RIGHT_POLARITY = -MOTOR_RIGHT_POLARITY
            print(f"-> Đã đảo chiều: Left={MOTOR_LEFT_POLARITY}, Right={MOTOR_RIGHT_POLARITY}")
        else:
            try:
                num = int(choice)
                if 1 <= num <= len(keys):
                    key, desc = keys[num - 1]
                    curr_val = cfg.get(key, 0.0)
                    new_val = prompt_float(f"Nhập giá trị mới cho '{desc}'", curr_val)
                    cfg[key] = new_val
                    print(f"-> Đã cập nhật {key} = {new_val}")
            except ValueError:
                print("Lựa chọn không hợp lệ.")

def main_menu():
    while True:
        cfg = load_current_config()
        print("\n" + "="*65)
        print("     CS532 - MENU HIỆU CHUẨN ĐỘNG HỌC (TÙY CHỈNH THAM SỐ)")
        print(f"     [Chiều động cơ: Left={MOTOR_LEFT_POLARITY}, Right={MOTOR_RIGHT_POLARITY}]")
        print("="*65)
        print(f" [1] Test 1: Tìm ga lăn bánh TIẾN tối thiểu (Ga: {cfg.get('min_drive_speed')})")
        print(f" [2] Test 2: Cân 2 bánh TIẾN THẲNG (Trim: {cfg.get('trim_factor')})")
        print(f" [3] Test 3: Đo vận tốc TIẾN 1 mét (Hiện tại: {cfg.get('actual_speed_mps')} m/s)")
        print(f" [4] Test 4: Đo thời gian xoay 90° & 180° (Hiện tại: {cfg.get('turn_time_90_deg_sec')}s)")
        print(f" [5] Test 5: Chạy thử vòng chữ nhật trong ô 2x1m")
        print(f" [6] TỰ TAY SỬA TẤT CẢ CÁC THAM SỐ (Bảng chỉnh tay trực tiếp)")
        print(f" [7] Xem nội dung file cấu hình (robot_dynamics.json)")
        print(f" [0] Thoát")
        print("="*65)
        
        choice = input("Nhập số bài test bạn muốn chạy (0-7): ").strip()
        if choice == '1':
            test_1_deadband(cfg)
        elif choice == '2':
            test_2_straight_trim(cfg)
        elif choice == '3':
            test_3_measure_speed_1m(cfg)
        elif choice == '4':
            test_4_point_turn(cfg)
        elif choice == '5':
            test_5_dry_run_rectangle(cfg)
        elif choice == '6':
            edit_all_params_manually(cfg)
        elif choice == '7':
            print("\n" + json.dumps(cfg, indent=4, ensure_ascii=False))
        elif choice == '0':
            print("Tạm biệt! Dừng chương trình.")
            stop()
            break
        else:
            print("Lựa chọn không hợp lệ, vui lòng chọn lại.")

if __name__ == "__main__":
    try:
        main_menu()
    except KeyboardInterrupt:
        print("\n[HỦY] Người dùng dừng bằng Ctrl+C.")
        stop()
