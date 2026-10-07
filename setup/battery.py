#!/usr/bin/env python3
"""
CS532 - JetBot Battery Monitor & Safety Protection Daemon
Tham khảo & Chuẩn hóa theo: UIT ITCourses #31 (Hardware & IoT)
https://aiclub.uit.edu.vn/itcourses/#t/Hardware%20&%20IoT/31

Tính năng:
    1. Đọc điện áp (V), dòng điện (mA), công suất (mW) từ cảm biến INA219 (I2C Bus 1, Addr 0x41).
    2. Tự động hỗ trợ 2 driver: 'pi-ina219' và fallback trực tiếp 'smbus2/smbus' (đọc thanh ghi 0x01, 0x02).
    3. Ước lượng % pin Li-ion 3S qua bảng đặc tuyến xả phi tuyến tính + nội suy.
    4. Giao diện CLI chuyên nghiệp: Thanh pin trực quan, hiển thị chi tiết từng cell, dòng xả/sạc, công suất, ước tính thời gian.
    5. CƠ CHẾ BẢO VỆ PIN & TẮT MÁY AN TOÀN (Safe Shutdown / Motor E-Stop):
       - Ngưỡng pin yếu (<= 10.2V): Cảnh báo âm thanh/visual.
       - Ngưỡng nguy hiểm (<= 9.6V): Tự động ngắt motor (nếu có) và kích hoạt shutdown hệ thống an toàn (chống hỏng cell Li-ion & thẻ nhớ).
    6. Tích hợp background daemon: Xuất file JSON (/tmp/jetbot_battery.json) cho Web Dashboard và lệnh CLI 'jetbot-battery'.
"""

import sys
import os
import time
import json
import argparse
import subprocess
from collections import deque
from typing import Dict, Any, Tuple, Optional

# --- CẤU HÌNH PHẦN CỨNG MẶC ĐỊNH (UIT ITCourses #31) ---
DEFAULT_I2C_BUS = 1          # I2C Bus 1 trên NVIDIA Jetson Nano
DEFAULT_I2C_ADDR = 0x41      # 0x41 (Waveshare JetBot Expansion Board)
DEFAULT_SHUNT_OHMS = 0.1     # Điện trở shunt R_SHUNT = 0.1 Ohm
DEFAULT_CELLS = 3            # Khối 3S Li-ion 18650
BATTERY_CAPACITY_MAH = 2600  # Dung lượng danh định 1 cell 18650 (~2600mAh)

# --- NGƯỠNG ĐIỆN ÁP BẢO VỆ AN TOÀN (PACK 3S) ---
VOLTAGE_FULL = 12.30         # >= 12.30V (~4.10V/cell): Pin đầy
VOLTAGE_WARN = 10.50         # <= 10.50V (~3.50V/cell): Mức nên sạc
VOLTAGE_LOW = 10.20          # <= 10.20V (~3.40V/cell): Cảnh báo pin yếu
VOLTAGE_CRITICAL = 9.60      # <= 9.60V  (~3.20V/cell): Nguy cơ sập nguồn, bắt đầu kích hoạt bảo vệ
VOLTAGE_SHUTDOWN = 9.00      # <= 9.00V  (~3.00V/cell): Ngưỡng cắt xả kiệt cell, buộc shutdown ngay

# Bảng tra đặc tuyến xả Li-ion chuẩn (V/cell, % dung lượng): ITCourses #31
_DISCHARGE_CURVE = [
    (4.20, 100),
    (4.10, 90),
    (4.00, 80),
    (3.90, 70),
    (3.80, 60),
    (3.70, 50),
    (3.60, 35),
    (3.50, 20),
    (3.40, 10),
    (3.20, 5),
    (3.00, 0),
]

# Màu sắc ANSI cho Terminal
C_RESET  = "\033[0m"
C_BOLD   = "\033[1m"
C_RED    = "\033[31m"
C_GREEN  = "\033[32m"
C_YELLOW = "\033[33m"
C_BLUE   = "\033[34m"
C_CYAN   = "\033[36m"
C_BLINK  = "\033[5m"


def calculate_battery_percent(pack_voltage: float, cells: int = DEFAULT_CELLS) -> int:
    """Ước lượng % pin Li-ion theo đường cong xả phi tuyến tính và nội suy."""
    if pack_voltage <= 0:
        return 0
    vc = pack_voltage / float(cells)
    if vc >= _DISCHARGE_CURVE[0][0]:
        return 100
    if vc <= _DISCHARGE_CURVE[-1][0]:
        return 0
    for (v1, p1), (v2, p2) in zip(_DISCHARGE_CURVE, _DISCHARGE_CURVE[1:]):
        if v2 <= vc <= v1:
            return int(round(p2 + (p1 - p2) * (vc - v2) / (v1 - v2)))
    return 0


def render_battery_bar(percentage: int, width: int = 20) -> str:
    """Tạo thanh hiển thị pin đồ họa trực quan: [████████░░]"""
    clamped = max(0, min(100, percentage))
    filled_len = int(round(width * clamped / 100.0))
    empty_len = width - filled_len
    bar = "█" * filled_len + "░" * empty_len

    if clamped >= 60:
        color = C_GREEN
    elif clamped >= 25:
        color = C_YELLOW
    else:
        color = C_RED

    return f"{color}[{bar}] {clamped:3d}%{C_RESET}"


class JetBotBatterySafe:
    """
    Quản lý giao tiếp cảm biến INA219 và cơ chế an toàn ngắt nguồn.
    """

    def __init__(
        self,
        i2c_bus: int = DEFAULT_I2C_BUS,
        i2c_addr: int = DEFAULT_I2C_ADDR,
        shunt_ohms: float = DEFAULT_SHUNT_OHMS,
        cells: int = DEFAULT_CELLS,
        mock: bool = False,
        filter_samples: int = 5,
        enable_auto_shutdown: bool = False,
    ):
        self.bus_num = i2c_bus
        self.addr = i2c_addr
        self.shunt_ohms = shunt_ohms
        self.cells = cells
        self.mock = mock
        self.enable_auto_shutdown = enable_auto_shutdown
        self.filter_samples = filter_samples
        self._v_history = deque(maxlen=max(1, filter_samples))
        self.driver_type = "mock" if mock else "none"
        self._ina = None
        self._smbus = None
        self._last_error = None
        self._critical_count = 0  # Đếm số lần liên tiếp chạm ngưỡng nguy hiểm

        if not self.mock:
            self._init_hardware()

    def _init_hardware(self):
        """Khởi tạo: Ưu tiên pi-ina219, fallback đọc trực tiếp qua smbus2/smbus."""
        # 1. Thử pi-ina219
        try:
            from ina219 import INA219
            self._ina = INA219(self.shunt_ohms, address=self.addr, busnum=self.bus_num)
            self._ina.configure()
            self.driver_type = "pi-ina219"
            return
        except ImportError as e:
            self._last_error = f"Chưa cài pi-ina219 ({e})"
        except Exception as e:
            self._last_error = e

        # 2. Thử smbus2 hoặc python3-smbus
        SMBusClass = None
        try:
            from smbus2 import SMBus as SMBusClass
            self.driver_type = "smbus2_raw"
        except ImportError:
            try:
                from smbus import SMBus as SMBusClass
                self.driver_type = "smbus_raw"
            except ImportError as e:
                self._last_error = f"Chưa cài smbus2 hoặc python3-smbus ({e})"

        if SMBusClass is not None:
            try:
                self._smbus = SMBusClass(self.bus_num)
                # Đọc thử 1 byte kiểm tra tồn tại địa chỉ
                self._smbus.read_byte(self.addr)
                return
            except Exception as e:
                self._last_error = e
                self._smbus = None

        self.driver_type = "unconnected"

    def _read_smbus_raw(self) -> Tuple[float, float, float]:
        """
        Đọc thanh ghi thô INA219 chuẩn ITCourses #31:
        Thanh ghi 0x02: Bus Voltage (LSB 4mV, bỏ 3 bit shift)
        Thanh ghi 0x01: Shunt Voltage (16 bit bù 2, LSB 10uV)
        """
        if self._smbus is None:
            raise RuntimeError("SMBus chưa được khởi tạo")

        hi, lo = self._smbus.read_i2c_block_data(self.addr, 0x02, 2)
        bus_raw = (hi << 8) | lo
        voltage = (bus_raw >> 3) * 0.004

        hi, lo = self._smbus.read_i2c_block_data(self.addr, 0x01, 2)
        s_raw = (hi << 8) | lo
        shunt_raw = s_raw - 65536 if s_raw & 0x8000 else s_raw
        v_shunt = shunt_raw * 1e-5
        current_a = v_shunt / self.shunt_ohms
        current_ma = current_a * 1000.0
        power_mw = voltage * abs(current_ma)

        return voltage, current_ma, power_mw

    def read(self) -> Dict[str, Any]:
        """Đọc và phân tích thông số pin thời gian thực."""
        if self.mock:
            v, i, p = 11.85, 480.0, 5688.0
        elif self.driver_type == "pi-ina219":
            try:
                v = self._ina.voltage()
                i = self._ina.current()
                p = self._ina.power()
            except Exception as e:
                raise RuntimeError(f"Lỗi đọc pi-ina219: {e}")
        elif self.driver_type in ("smbus2_raw", "smbus_raw"):
            try:
                v, i, p = self._read_smbus_raw()
            except Exception as e:
                raise RuntimeError(f"Lỗi đọc thanh ghi I2C: {e}")
        else:
            raise ConnectionError(
                f"Không kết nối được cảm biến INA219 (0x{self.addr:02X}) trên I2C bus {self.bus_num}.\n"
                f"   Chi tiết: {self._last_error}\n"
                f"   👉 Kiểm tra: Công tắc pin trên thân JetBot đã BẬT (ON) chưa? Dây I2C có tiếp xúc tốt không?"
            )

        self._v_history.append(v)
        v_filtered = sum(self._v_history) / len(self._v_history)
        cell_avg = v_filtered / float(self.cells)
        pct = calculate_battery_percent(v_filtered, self.cells)

        is_charging = i < -20.0  # Dòng điện âm nghĩa là dòng đi vào pin (đang cắm sạc)
        is_critical = v_filtered <= VOLTAGE_CRITICAL
        is_low = v_filtered <= VOLTAGE_LOW
        is_shutdown_level = v_filtered <= VOLTAGE_SHUTDOWN

        # Xác định trạng thái cảnh báo
        if is_charging:
            status = "ĐANG SẠC PIN 🔌"
        elif is_shutdown_level:
            status = "CẮT XẢ KHẨN CẤP (NGUY CƠ HỎNG PIN)"
        elif is_critical:
            status = "NGUY HIỂM (SẮP SẬP NGUỒN)"
        elif is_low:
            status = "PIN YẾU (CẦN SẠC)"
        elif v_filtered <= VOLTAGE_WARN:
            status = "NÊN SẠC"
        elif pct >= 80:
            status = "TỐT / ĐẦY"
        else:
            status = "HOẠT ĐỘNG ỔN ĐỊNH"

        # Ước lượng thời gian sử dụng còn lại (phút)
        est_minutes = None
        if not is_charging and abs(i) > 50.0:
            rem_mah = BATTERY_CAPACITY_MAH * (pct / 100.0)
            est_hours = rem_mah / abs(i)
            est_minutes = int(est_hours * 60)

        # Xử lý cơ chế an toàn
        if is_critical and not is_charging:
            self._critical_count += 1
            if self._critical_count >= 3:  # 3 lần đo liên tục dưới 9.6V
                self._trigger_safety_shutdown(v_filtered)
        else:
            self._critical_count = 0

        return {
            "voltage": round(v, 2),
            "voltage_filtered": round(v_filtered, 2),
            "cell_voltage": round(cell_avg, 2),
            "current_ma": round(i, 1),
            "power_mw": round(p, 1),
            "percentage": pct,
            "status": status,
            "is_charging": is_charging,
            "is_low": is_low,
            "is_critical": is_critical,
            "is_shutdown_level": is_shutdown_level,
            "est_minutes": est_minutes,
            "driver": self.driver_type,
            "timestamp": round(time.time(), 2),
        }

    def _trigger_safety_shutdown(self, current_v: float):
        """Kích hoạt cơ chế dừng xe khẩn cấp và tắt nguồn máy an toàn."""
        # 1. Dừng động cơ ngay lập tức qua lệnh I2C nếu có jetbot
        try:
            from jetbot import Robot
            robot = Robot()
            robot.stop()
        except Exception:
            pass

        # 2. Cảnh báo ra màn hình/log
        msg = (
            f"\n🚨 [CRITICAL BATTERY SAFETY CUTOFF] 🚨\n"
            f"Điện áp pin đo được: {current_v:.2f}V (<= {VOLTAGE_CRITICAL}V)!\n"
            f"Nguy cơ chai kiệt cell 18650 và sụt áp làm hỏng thẻ nhớ SD Jetson Nano.\n"
        )
        print(msg, file=sys.stderr)

        # 3. Tự động shutdown nếu bật cờ --auto-shutdown
        if self.enable_auto_shutdown:
            print("🛑 Kích hoạt quy trình tắt máy an toàn: sudo shutdown -h now...", file=sys.stderr)
            subprocess.run(["sudo", "shutdown", "-h", "now"])

    def close(self):
        if self._smbus is not None:
            try:
                self._smbus.close()
            except Exception:
                pass


def print_professional_dashboard(data: Dict[str, Any]):
    """In bảng giám sát pin phong cách Dashboard chuyên nghiệp."""
    v = data["voltage"]
    vc = data["cell_voltage"]
    i = data["current_ma"]
    p = data["power_mw"]
    pct = data["percentage"]
    status = data["status"]
    est = data["est_minutes"]

    print("\n" + "=" * 65)
    print(f"{C_BOLD}   🤖 JETBOT BATTERY MONITOR & POWER MANAGEMENT SYSTEM{C_RESET}")
    print("=" * 65)

    # Dòng 1: Thanh pin
    bar_str = render_battery_bar(pct, width=22)
    print(f" Dung lượng pin  : {bar_str}")

    # Dòng 2: Điện áp tổng & từng cell
    v_color = C_GREEN if v >= VOLTAGE_WARN else (C_YELLOW if v >= VOLTAGE_LOW else C_RED)
    print(f" Điện áp Pack 3S : {v_color}{C_BOLD}{v:5.2f} V{C_RESET}  (TB mỗi cell: {vc:.2f} V/cell)")

    # Dòng 3: Dòng tiêu thụ & Công suất
    if data["is_charging"]:
        i_str = f"{C_CYAN}ĐANG SẠC (+{abs(i):.1f} mA){C_RESET}"
    else:
        i_str = f"{abs(i):.1f} mA"
    print(f" Dòng điện tải   : {i_str}  |  Công suất: {p/1000.0:.2f} W ({p:.0f} mW)")

    # Dòng 4: Ước tính thời gian
    if est is not None:
        hours = est // 60
        mins = est % 60
        time_str = f"~{hours}h {mins}m" if hours > 0 else f"~{mins} phút"
        print(f" Ước tính còn lại: {C_BOLD}{time_str}{C_RESET} (theo dòng xả hiện tại)")
    elif data["is_charging"]:
        print(f" Ước tính còn lại: {C_CYAN}Đang cắm nguồn sạc adapter{C_RESET}")
    else:
        print(f" Ước tính còn lại: Đang phân tích tải...")

    # Dòng 5: Đánh giá trạng thái
    st_color = C_GREEN if "TỐT" in status or "ỔN ĐỊNH" in status else (C_CYAN if "SẠC" in status else C_RED)
    print(f" Đánh giá        : {st_color}{C_BOLD}{status}{C_RESET}")
    print("-" * 65)

    if data["is_critical"]:
        print(f"{C_RED}{C_BLINK}🚨 CẢNH BÁO NGUY HIỂM: PIN DƯỚI 9.6V! HÃY CẮM SẠC NGAY ĐỂ TRÁNH HỎNG THẺ NHỚ!{C_RESET}")
    elif data["is_low"]:
        print(f"{C_YELLOW}⚠️  CẢNH BÁO: Pin sắp cạn (< 10.2V). Chuẩn bị đưa robot về trạm sạc.{C_RESET}")
    else:
        print(f"{C_GREEN}✅ Hệ thống nguồn hoạt động trong dải điện áp an toàn.{C_RESET}")
    print("=" * 65 + "\n")


def main():
    parser = argparse.ArgumentParser(description="JetBot Battery Monitor & Safety Protection Daemon (CS532)")
    parser.add_argument("--addr", type=lambda x: int(x, 0), default=DEFAULT_I2C_ADDR, help="Địa chỉ I2C INA219 (mặc định: 0x41)")
    parser.add_argument("--bus", type=int, default=DEFAULT_I2C_BUS, help="I2C Bus ID (mặc định: 1)")
    parser.add_argument("--loop", action="store_true", help="Chạy chế độ nền liên tục")
    parser.add_argument("--interval", type=float, default=2.0, help="Chu kỳ lặp (giây) khi chạy nền (mặc định: 2.0s)")
    parser.add_argument("--mock", action="store_true", help="Chạy chế độ giả lập (dùng khi test trên máy tính)")
    parser.add_argument("--export-json", type=str, default="/tmp/jetbot_battery.json", help="Đường dẫn file JSON xuất dữ liệu")
    parser.add_argument("--auto-shutdown", action="store_true", help="Bật chế độ tự động tắt máy khi pin chạm ngưỡng nguy hiểm <= 9.6V")

    args = parser.parse_args()

    try:
        monitor = JetBotBatterySafe(
            i2c_bus=args.bus,
            i2c_addr=args.addr,
            mock=args.mock,
            enable_auto_shutdown=args.auto_shutdown,
        )
    except Exception as e:
        print(f"❌ Khởi tạo thất bại: {e}")
        sys.exit(1)

    try:
        if args.loop:
            while True:
                data = monitor.read()
                # Xuất file json cho service và web dashboard
                if args.export_json:
                    try:
                        tmp_f = f"{args.export_json}.tmp"
                        with open(tmp_f, "w") as f:
                            json.dump(data, f, indent=2)
                        os.replace(tmp_f, args.export_json)
                    except Exception:
                        pass

                # In 1 dòng tóm tắt ra console/journal
                ts = time.strftime("%H:%M:%S")
                v = data["voltage"]
                pct = data["percentage"]
                i = data["current_ma"]
                st = data["status"]
                print(f"[{ts}] Pin: {v:5.2f}V ({pct:3d}%) | {i:6.1f}mA | {st}", flush=True)
                time.sleep(args.interval)
        else:
            # Chế độ chạy 1 lần: In Dashboard chi tiết đẹp mắt
            data = monitor.read()
            print_professional_dashboard(data)
            if args.export_json:
                try:
                    tmp_f = f"{args.export_json}.tmp"
                    with open(tmp_f, "w") as f:
                        json.dump(data, f, indent=2)
                    os.replace(tmp_f, args.export_json)
                except Exception:
                    pass
    except KeyboardInterrupt:
        print("\nĐã dừng giám sát pin.")
    except Exception as e:
        print(f"❌ Lỗi: {e}")
        sys.exit(1)
    finally:
        monitor.close()


if __name__ == "__main__":
    main()
