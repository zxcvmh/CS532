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

PHASE 0 TRIAGE (chạy trước khi chỉnh thông số Phase 1):
    python3 setup/test_lidar.py --raw-dump
        -> In nguyên văn sector trước ±25°: angle, dist_mm, confidence (kể cả những
           điểm bị ngưỡng intensity loại bỏ), kèm bộ đếm rejected_by_intensity /
           rejected_by_range. Đây là phép đo quyết định H2 (ngưỡng intensity) vs H3
           (sai offset góc) vs lỗi hình học thuần túy.
    python3 setup/test_lidar.py --box-sweep
        -> Nhập khoảng cách thật của hộp; in khoảng cách nhỏ nhất sector trước và số
           tia hợp lệ trong 0.6m tại 10/16/25/40/60cm.
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

def find_lidar_port(preferred_port=None):
    if preferred_port and os.path.exists(preferred_port):
        return preferred_port
    # Prioritize USB serial adapters (/dev/ttyUSB*) over internal GPIO UART (/dev/ttyTHS*)
    usb_ports = sorted(glob.glob("/dev/ttyUSB*"))
    if usb_ports:
        return usb_ports[0]
    ths_ports = sorted(glob.glob("/dev/ttyTHS*"))
    if ths_ports:
        return ths_ports[0]
    return None

class LidarD500Reader:
    """Đọc thô LD19/D500. Luôn lưu MỌI điểm đo (bằng chứng), đồng thời áp chính sách
    intensity của Phase 1 để đếm số điểm bị loại."""

    def __init__(self, port, baudrate=230400, min_intensity=15,
                 min_intensity_near=1, near_field_m=1.0,
                 min_dist=0.05, max_dist=6.0):
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        self.running = False
        self.thread = None
        self.ranges = [0.0] * 360
        self.intensities = [0] * 360
        self.lock = threading.Lock()
        self.packet_count = 0
        # Chính sách Phase 1 (giống 03-MinhHieu/d500_driver.py)
        self.min_intensity = int(min_intensity)
        self.min_intensity_near = int(min_intensity_near)
        self.near_field_m = float(near_field_m)
        self.min_dist = float(min_dist)
        self.max_dist = float(max_dist)
        # Bằng chứng thô: (wall_time, angle_ccw, dist_mm, confidence)
        self._samples = []
        self.rejected_by_intensity = 0
        self.rejected_by_range = 0
        self.accepted_points = 0

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
            now_w = time.time()
            for i in range(12):
                offset = 6 + (i * 3)
                dist_mm = packet[offset] | (packet[offset + 1] << 8)
                intensity = packet[offset + 2]
                raw_angle = (start_angle + (i * step)) % 360.0
                dist_m = dist_mm / 1000.0
                angle_ccw = (360 - int(round(raw_angle))) % 360
                if dist_m <= 0.0:
                    continue

                # Bằng chứng: LƯU TẤT CẢ, kể cả điểm mà ngưỡng sẽ loại bỏ.
                self._samples.append((now_w, angle_ccw, dist_mm, intensity, dist_m))
                if len(self._samples) > 4000:
                    del self._samples[:-2000]

                if not (self.min_dist <= dist_m <= self.max_dist):
                    self.rejected_by_range += 1
                    continue
                required = (
                    self.min_intensity_near
                    if dist_m <= self.near_field_m
                    else self.min_intensity
                )
                if intensity < required:
                    self.rejected_by_intensity += 1
                    continue
                self.accepted_points += 1
                self.ranges[angle_ccw] = round(dist_m, 3)
                self.intensities[angle_ccw] = intensity

    def get_ranges(self):
        with self.lock:
            return list(self.ranges)

    def get_intensities(self):
        with self.lock:
            return list(self.intensities)

    def get_diagnostics(self):
        with self.lock:
            return {
                "min_intensity": self.min_intensity,
                "min_intensity_near": self.min_intensity_near,
                "near_field_m": self.near_field_m,
                "min_dist": self.min_dist,
                "max_dist": self.max_dist,
                "accepted_points": self.accepted_points,
                "rejected_by_intensity": self.rejected_by_intensity,
                "rejected_by_range": self.rejected_by_range,
            }

    def get_frontal_samples(self, span=25, max_age=0.35):
        """Các điểm đo thô gần nhất trong sector trước ±span độ."""
        cutoff = time.time() - max_age
        with self.lock:
            return [
                smp for smp in self._samples
                if smp[0] >= cutoff and min(smp[1], 360 - smp[1]) <= span
            ]

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
# PHASE 0 TRIAGE (quyết định H2 vs H3 vs hình học thuần túy)
# ─────────────────────────────────────────────────────────────

def run_raw_dump(reader, seconds=15.0, span=25, refresh=0.5):
    """In nguyên văn sector trước ±span độ trong `seconds` giây.

    Mỗi dòng là một điểm đo thô: angle, dist_mm, confidence. Điểm có confidence dưới
    ngưỡng Phase 1 được đánh dấu `*` — đó chính là bằng chứng cho H2.
    """
    print("\n" + "=" * 78)
    print("PHASE 0 — RAW DUMP: sector trước ±%d°, KHÔNG lọc intensity" % span)
    print("=" * 78)
    print("Đặt hộp/tấm bìa ĐEN trước mũi xe ở khoảng cách đã biết rồi bấm CTRL+C để dừng.")
    print("Dấu `*` = confidence dưới ngưỡng Phase 1 (sẽ bị driver loại bỏ).")
    print("-" * 78)
    print(f"{'angle_deg':>9} {'dist_mm':>9} {'conf':>6}  {'dist_m':>7}  ghi chú")
    print("-" * 78)

    t_end = time.time() + seconds
    try:
        while time.time() < t_end:
            samples = reader.get_frontal_samples(span=span)
            diag = reader.get_diagnostics()
            rows = sorted(samples, key=lambda x: x[1])
            print("\033[2J\033[H", end="")  # clear screen
            print("PHASE 0 — RAW DUMP (CTRL+C để thoát)")
            print(
                f"  min_intensity(far)={diag['min_intensity']}  "
                f"min_intensity(near)={diag['min_intensity_near']}  "
                f"near_field_m={diag['near_field_m']}"
            )
            print(
                f"  accepted={diag['accepted_points']}  "
                f"rejected_by_intensity={diag['rejected_by_intensity']}  "
                f"rejected_by_range={diag['rejected_by_range']}"
            )
            print("-" * 78)
            print(f"{'angle_deg':>9} {'dist_mm':>9} {'conf':>6}  {'dist_m':>7}  ghi chú")
            print("-" * 78)
            for _ts, ang, dist_mm, conf, dist_m in rows[-40:]:
                required = (
                    diag["min_intensity_near"]
                    if dist_m <= diag["near_field_m"]
                    else diag["min_intensity"]
                )
                mark = "  * DƯỚI NGƯỠNG" if conf < required else ""
                print(f"{ang:>9} {dist_mm:>9} {conf:>6}  {dist_m:>7.3f}{mark}")
            print("-" * 78)
            print(
                "Nếu hộp ĐEN xuất hiện ở đây với conf ~5-15 => H2 ĐÚNG (ngưỡng intensity).\n"
                "Nếu KHÔNG có điểm nào dù conf=0 vẫn trống => H2 CHẾT => lỗi hình học/H3."
            )
            time.sleep(refresh)
    except KeyboardInterrupt:
        pass
    print("\n[KẾT THÚC RAW DUMP]")


def run_box_sweep(reader, span=25):
    """Đo sector trước tại các khoảng cách hộp do người vận hành nhập."""
    print("\n" + "=" * 78)
    print("PHASE 0 — BOX SWEEP: hộp trước mũi xe tại 10/16/25/40/60 cm")
    print("=" * 78)
    print("Với mỗi mốc: đặt hộp đúng khoảng cách, nhập số cm rồi Enter.")
    print("Bỏ trống để kết thúc sớm. Gõ `x` để bỏ qua một mốc.")
    print("-" * 78)
    print(f"{'thật(cm)':>9} {'min_front_m':>12} {'#tia<=0.6m':>11} {'#tia 0.6-1.0m':>14}"
          f" {'conf_min':>9} {'#tia 0':>7}")
    print("-" * 78)

    while True:
        raw = input("Khoảng cách thật của hộp (cm, Enter để thoát): ").strip()
        if raw == "":
            break
        if raw.lower() == "x":
            continue
        try:
            true_cm = float(raw)
        except ValueError:
            print("  [!] Giá trị không hợp lệ.")
            continue

        t_end = time.time() + 2.0
        min_front = 99.0
        n_close = 0
        n_mid = 0
        confs = []
        while time.time() < t_end:
            samples = reader.get_frontal_samples(span=span, max_age=0.4)
            for _ts, _ang, _dmm, conf, dist_m in samples:
                min_front = min(min_front, dist_m)
                if dist_m <= 0.6:
                    n_close += 1
                elif dist_m <= 1.0:
                    n_mid += 1
                confs.append(conf)
            time.sleep(0.05)

        ranges = reader.get_ranges()
        n_zero = sum(
            1
            for deg in range(-span, span + 1)
            if ranges[deg % 360] < 0.05
        )
        conf_min = min(confs) if confs else -1
        print(
            f"{true_cm:>9.0f} {min_front:>12.2f} {n_close:>11} {n_mid:>14} "
            f"{conf_min:>9} {n_zero:>7}"
        )

    print("-" * 78)
    print("Đọc bảng: nếu #tia<=0.6m > 0 với conf_min ~5-15 => H2 (ngưỡng intensity).")
    print("Nếu min_front vẫn lớn (~1m+) dù hộp ở 16-25cm => nghi vấn hình học/H3.")
    print("Nếu #tia 0 (bin trống) tăng mạnh khi hộp tới gần => điểm mù cận trường.")


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
            d_left  = get_sector_distance(r, 90, 15)
            d_back  = get_sector_distance(r, 180, 15)
            d_right = get_sector_distance(r, 270, 15)

            sys.stdout.write(
                f"\r [LA BÀN 4 HƯỚNG] "
                f"Trước (0°): {d_front:4.2f}m | "
                f"Trái (90°): {d_left:4.2f}m | "
                f"Sau (180°): {d_back:4.2f}m | "
                f"Phải (270°): {d_right:4.2f}m   "
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
        print("⚠️ Cụm LiDAR đang lệch sang trái 90° (ROS CCW).")
        update_config_lidar_offset(90)
    elif 245 <= median_angle <= 295:
        print("⚠️ Cụm LiDAR đang lệch sang phải 270° (hoặc -90° ROS CCW).")
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
    # Phase 0 triage modes (chạy trước, không cần menu)
    raw_dump = "--raw-dump" in sys.argv
    box_sweep = "--box-sweep" in sys.argv
    min_intensity = 15

    port_arg = None
    if "--port" in sys.argv:
        try:
            port_arg = sys.argv[sys.argv.index("--port") + 1]
        except IndexError:
            pass
    if not port_arg:
        port_arg = os.environ.get("LIDAR_PORT")

    port = find_lidar_port(port_arg)
    if not port:
        print("❌ Không tìm thấy cổng USB-UART của LiDAR D500 (/dev/ttyUSB* hoặc /dev/ttyTHS*).")
        print("   Hãy kiểm tra cáp cắm vào cổng USB của Jetson Nano, hoặc chỉ định rõ: --port /dev/ttyUSB0")
        sys.exit(1)

    print(f"[*] Tìm thấy cổng LiDAR tại: {port}")
    reader = LidarD500Reader(port, min_intensity=min_intensity)
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
        if raw_dump:
            run_raw_dump(reader)
            return
        if box_sweep:
            run_box_sweep(reader)
            return

        while True:
            print("\n" + "="*65)
            print("         CS532 - MENU KIỂM TRA CẢM BIẾN LIDAR D500")
            print("="*65)
            print(" [1] La bàn khoảng cách thời gian thực (Xem 4 hướng: Trước/Phải/Sau/Trái)")
            print(" [2] Kiểm tra hướng gắn 0° của LiDAR (Xác nhận mũi xe)")
            print(" [3] Thử nghiệm phản xạ phanh khẩn cấp (< 20cm tự ngắt ga)")
            print(" [4] PHASE 0: raw dump sector trước (không lọc intensity)")
            print(" [5] PHASE 0: box sweep theo khoảng cách thật")
            print(" [0] Thoát")
            print("="*65)
            c = input("Chọn chức năng (0-3): ").strip()
            if c == '1':
                test_live_compass(reader)
            elif c == '2':
                test_orientation_check(reader)
            elif c == '3':
                test_safety_reflex(reader)
            elif c == '4':
                run_raw_dump(reader)
            elif c == '5':
                run_box_sweep(reader)
            elif c == '0':
                break
            else:
                print("Lựa chọn không hợp lệ.")
    finally:
        reader.stop()
        print("Đã đóng kết nối LiDAR an toàn.")

if __name__ == "__main__":
    main()
