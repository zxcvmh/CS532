#!/usr/bin/env python3
"""
CS532 - LiDAR D500 Verification & Orientation Calibration Tool
Phạm vi: Thư mục setup/
Cảm biến: LiDAR D500 (LD19 protocol @ 230400 bps)
Tác giả: robot-setup-agent

Chức năng:
    1. Kiểm tra nhận dạng cổng USB-UART và luồng gói tin LiDAR
    2. Radar la bàn 4 hướng (Trước 0°, Phải 90°, Sau 180°, Trái 270°)
    3. Tự động kiểm tra chiều 0° của LiDAR (Orientation Check)
    4. Thử nghiệm phản xạ phanh khẩn cấp (Emergency Reflex Test)
    5. Tự động ghi nhận thông số vào setup/robot_dynamics.json
"""

import os
import sys
import time
import glob
import json
import threading

CONFIG_FILE = os.path.join(os.path.dirname(__file__), "robot_dynamics.json")

try:
    import serial
except ImportError:
    print("[LỖI] Chưa cài đặt pyserial. Chạy lệnh: pip3 install pyserial")
    sys.exit(1)

def find_lidar_port():
    candidates = sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyTHS*"))
    if not candidates:
        return None
    return candidates[0]

class LidarD500Reader:
    def __init__(self, port, baudrate=230400):
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        self.running = False
        self.thread = None
        self.ranges = [0.0] * 360
        self.lock = threading.Lock()
        self.packet_count = 0

    def start(self):
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
            self.running = True
            self.thread = threading.Thread(target=self._read_loop, daemon=True)
            self.thread.start()
            return True
        except Exception as e:
            print(f"[LỖI KẾT NỐI LIDAR] Không mở được {self.port}: {e}")
            return False

    def stop(self):
        self.running = False
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass

    def _read_loop(self):
        buf = bytearray()
        while self.running:
            try:
                chunk = self.ser.read(128)
                if not chunk:
                    time.sleep(0.005)
                    continue
                buf.extend(chunk)

                # Tìm header 0x54, 0x2C (gói 47 byte LD19/D500)
                while len(buf) >= 47:
                    if buf[0] == 0x54 and buf[1] == 0x2C:
                        packet = buf[:47]
                        buf = buf[47:]
                        self._parse_packet(packet)
                    else:
                        buf.pop(0)
            except Exception:
                break

    def _parse_packet(self, packet):
        start_angle = (packet[4] | (packet[5] << 8)) / 100.0
        end_angle = (packet[42] | (packet[43] << 8)) / 100.0
        if end_angle < start_angle:
            end_angle += 360.0
        step = (end_angle - start_angle) / 11.0

        with self.lock:
            self.packet_count += 1
            for i in range(12):
                offset = 6 + (i * 3)
                dist_mm = packet[offset] | (packet[offset + 1] << 8)
                raw_angle = (start_angle + (i * step)) % 360.0
                dist_m = dist_mm / 1000.0
                angle_ccw = (360 - int(round(raw_angle))) % 360
                if 0.03 <= dist_m <= 12.0:
                    self.ranges[angle_ccw] = round(dist_m, 3)

    def get_ranges(self):
        with self.lock:
            return list(self.ranges)

def get_sector_distance(ranges, center_deg, span=15):
    """Lấy khoảng cách nhỏ nhất trong một dải góc [center - span, center + span]."""
    valid_dists = []
    for deg in range(center_deg - span, center_deg + span + 1):
        idx = deg % 360
        r = ranges[idx]
        if r > 0.05:
            valid_dists.append(r)
    if not valid_dists:
        return 9.99
    return round(min(valid_dists), 2)

def update_config_lidar_offset(offset_deg):
    data = {}
    if os.path.exists(CONFIG_FILE):
        try:
            with open(CONFIG_FILE, "r", encoding="utf-8") as f:
                data = json.load(f)
        except Exception:
            pass
    data["lidar_yaw_offset_deg"] = offset_deg
    with open(CONFIG_FILE, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=4, ensure_ascii=False)
    print(f" -> Đã cập nhật `lidar_yaw_offset_deg = {offset_deg}` vào {CONFIG_FILE}!")

# ─────────────────────────────────────────────────────────────
# CÁC BÀI TEST LIDAR
# ─────────────────────────────────────────────────────────────

def test_live_compass(reader):
    print("\n" + "="*65)
    print("CHẾ ĐỘ 1: LA BÀN KHOẢNG CÁCH THỜI GIAN THỰC (BẤM CTRL+C ĐỂ THOÁT)")
    print("="*65)
    print("Hiển thị khoảng cách vật cản gần nhất ở 4 hướng chính:\n")
    try:
        while True:
            r = reader.get_ranges()
            d_front = get_sector_distance(r, 0, 15)
            d_right = get_sector_distance(r, 90, 15)
            d_back  = get_sector_distance(r, 180, 15)
            d_left  = get_sector_distance(r, 270, 15)

            sys.stdout.write(
                f"\r [LA BÀN 4 HƯỚNG] "
                f"Trước (0°): {d_front:4.2f}m | "
                f"Phải (90°): {d_right:4.2f}m | "
                f"Sau (180°): {d_back:4.2f}m | "
                f"Trái (270°): {d_left:4.2f}m   "
            )
            sys.stdout.flush()
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\n\nĐã dừng chế độ la bàn.")

def test_orientation_check(reader):
    print("\n" + "="*65)
    print("CHẾ ĐỘ 2: KIỂM TRA HƯỚNG GẮN GÓC 0° CỦA LIDAR")
    print("Mục đích: Xác định LiDAR có quay đúng góc 0° ra trước mũi xe không.")
    print("="*65)
    print("HƯỚNG DẪN:")
    print(" 1. Không để vật cản nào gần robot trong bán kính 60cm.")
    print(" 2. Chuẩn bị đưa bàn tay đặt cách mũi xe khoảng 20cm - 30cm.")
    input("\nNhấn [ENTER] khi bạn đã sẵn sàng...")

    print("\n-> BẮT ĐẦU ĐO: Hãy ĐẶT BÀN TAY TRƯỚC MŨI XE ngay bây giờ...")
    time.sleep(1.0)

    # Đọc mẫu trong 3 giây
    detected_angles = []
    t_end = time.time() + 3.0
    while time.time() < t_end:
        ranges = reader.get_ranges()
        for deg in range(360):
            dist = ranges[deg]
            if 0.12 <= dist <= 0.40:
                detected_angles.append(deg)
        time.sleep(0.05)

    if not detected_angles:
        print("\n[!] Không phát hiện thấy bàn tay trong khoảng 12 - 40cm.")
        print("    Vui lòng thử lại và để tay gần mũi xe hơn một chút.")
        return

    # Tính góc trung vị / phổ biến nhất
    detected_angles.sort()
    median_angle = detected_angles[len(detected_angles)//2]
    print(f"\n==> CẢM BIẾN PHÁT HIỆN VẬT CẢN TẠI GÓC: {median_angle}°")

    if (median_angle <= 25) or (median_angle >= 335):
        print("🎉 CHUẨN XÁC 100%! Cụm LiDAR D500 đã quay đúng góc 0° về phía mũi xe.")
        update_config_lidar_offset(0)
    elif 155 <= median_angle <= 205:
        print("⚠️ CỤM LIDAR ĐANG BỊ GẮN NGƯỢC 180° (Góc 0° đang quay về đuôi xe)!")
        print("   -> Bạn KHÔNG CẦN tháo ốc vặn lại phần cứng!")
        print("   -> Script sẽ tự lưu offset = 180° để phần mềm tự động bù góc.")
        update_config_lidar_offset(180)
    elif 65 <= median_angle <= 115:
        print("⚠️ Cụm LiDAR đang lệch sang phải 90°.")
        update_config_lidar_offset(90)
    elif 245 <= median_angle <= 295:
        print("⚠️ Cụm LiDAR đang lệch sang trái 270°.")
        update_config_lidar_offset(270)
    else:
        print(f"⚠️ Cụm LiDAR bị lệch góc khoảng {median_angle}°. Lưu offset = {median_angle}°.")
        update_config_lidar_offset(median_angle)

def test_safety_reflex(reader):
    print("\n" + "="*65)
    print("CHẾ ĐỘ 3: KIỂM TRA PHẢN XẠ PHANH KHẨN CẤP THỰC TẾ (10Hz REFLEX)")
    print("Mục đích: Xác nhận xe lập tức phanh dừng khi phát hiện vật cản sát mũi.")
    print("="*65)
    print("Hướng dẫn: Kê bánh xe hổng khỏi sàn gạch (đặt trên hộp) để an toàn.")
    input("Nhấn [ENTER] để bắt đầu thử nghiệm phanh...")

    from calibrate_motors import drive_raw, stop
    print("-> Động cơ đang quay tiến ở ga 0.20...")
    drive_raw(0.20, 0.20)
    
    print("-> Hãy ĐƯA BÀN TAY CHẮN TRƯỚC MŨI XE (< 20cm) ngay bây giờ...")
    t_start = time.time()
    reflex_triggered = False

    while time.time() - t_start < 8.0:
        ranges = reader.get_ranges()
        d_front = get_sector_distance(ranges, 0, 25)
        if 0.05 < d_front <= 0.20:
            stop()
            reflex_triggered = True
            print(f"\n🛑 [PHANH KHẨN CẤP THÀNH CÔNG!] Phát hiện vật cản ở cự ly {d_front*100:.0f}cm!")
            break
        time.sleep(0.05)

    stop()
    if not reflex_triggered:
        print("\n[HẾT GIỜ] Không phát hiện vật cản trong 8 giây. Đã tự động ngắt động cơ.")

def main():
    port = find_lidar_port()
    if not port:
        print("❌ Không tìm thấy cổng USB-UART của LiDAR D500 (/dev/ttyUSB*).")
        print("   Hãy kiểm tra cáp cắm vào cổng USB của Jetson Nano.")
        sys.exit(1)

    print(f"[*] Tìm thấy cổng LiDAR tại: {port}")
    reader = LidarD500Reader(port)
    if not reader.start():
        sys.exit(1)

    print("[*] Đang kết nối và đồng bộ gói tin LiDAR D500 (230400 bps)...")
    time.sleep(1.0)
    if reader.packet_count == 0:
        print("⚠️ Chưa nhận được gói tin nào. Kiểm tra quyền cổng: sudo usermod -aG dialout $USER")
        reader.stop()
        sys.exit(1)

    print(f"✅ Đã kết nối thành công! Tốc độ nhận: ~{reader.packet_count} gói tin/giây.")

    try:
        while True:
            print("\n" + "="*65)
            print("         CS532 - MENU KIỂM TRA CẢM BIẾN LIDAR D500")
            print("="*65)
            print(" [1] La bàn khoảng cách thời gian thực (Xem 4 hướng: Trước/Phải/Sau/Trái)")
            print(" [2] Kiểm tra hướng gắn 0° của LiDAR (Xác nhận mũi xe)")
            print(" [3] Thử nghiệm phản xạ phanh khẩn cấp (< 20cm tự ngắt ga)")
            print(" [0] Thoát")
            print("="*65)
            c = input("Chọn chức năng (0-3): ").strip()
            if c == '1':
                test_live_compass(reader)
            elif c == '2':
                test_orientation_check(reader)
            elif c == '3':
                test_safety_reflex(reader)
            elif c == '0':
                break
            else:
                print("Lựa chọn không hợp lệ.")
    finally:
        reader.stop()
        print("Đã đóng kết nối LiDAR an toàn.")

if __name__ == "__main__":
    main()
