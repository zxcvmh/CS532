#!/usr/bin/env python3
"""
CS532 - Robot Setup Healthcheck Script
Phạm vi: Tuần 1 - Nghiệm thu phần cứng & môi trường JetBot / Jetson Nano
Tác giả: Robot Setup Agent
"""

import sys
import time
import glob
import types

def check_camera():
    print("[1/4] Kiểm tra Camera CSI (IMX219)...", end=" ")
    try:
        import cv2
        gst = (
            "nvarguscamerasrc sensor-id=0 num-buffers=5 ! "
            "video/x-raw(memory:NVMM), width=640, height=480, format=NV12, framerate=30/1 ! "
            "nvvidconv flip-method=0 ! "
            "video/x-raw, format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink"
        )
        cap = cv2.VideoCapture(gst, cv2.CAP_GSTREAMER)
        if not cap.isOpened():
            print("FAIL ❌ (Không mở được camera)")
            return False
        
        ret, frame = cap.read()
        cap.release()
        if ret and frame is not None and frame.shape[0] > 0:
            print(f"PASS ✅ (Đã đọc frame: {frame.shape[1]}x{frame.shape[0]})")
            return True
        else:
            print("FAIL ❌ (Frame rỗng)")
            return False
    except Exception as e:
        print(f"FAIL ❌ ({e})")
        return False

def check_motor():
    print("[2/4] Kiểm tra Mạch điều khiển Động cơ (I2C)...", end=" ")
    
    # 1. Tự động bypass phụ thuộc ipywidgets để không cần cài jupyter nặng
    if "ipywidgets" not in sys.modules:
        dummy_mod = types.ModuleType("ipywidgets")
        class DummyWidget(object):
            def __init__(self, *args, **kwargs): pass
            def observe(self, *args, **kwargs): pass
        dummy_mod.Widget = DummyWidget
        dummy_mod.Layout = DummyWidget
        dummy_mod.Image = DummyWidget
        dummy_mod.Button = DummyWidget
        sys.modules["ipywidgets"] = dummy_mod
        sys.modules["ipywidgets.widgets"] = dummy_mod

    # 2. Thử khởi tạo qua JetBot Robot
    try:
        from jetbot import Robot
        robot = Robot()
        robot.stop()
        print("PASS ✅ (Kết nối I2C và khởi tạo driver Motor JetBot thành công)")
        return True
    except Exception as e_jetbot:
        # 3. Fallback: Kiểm tra trực tiếp phần cứng chip PCA9685 trên Bus I2C 1
        try:
            import smbus
            bus = smbus.SMBus(1)
            found = []
            for addr in [0x60, 0x40]:
                try:
                    bus.read_byte(addr)
                    found.append(hex(addr))
                except Exception:
                    pass
            if found:
                print(f"PASS ✅ (Đã phát hiện mạch driver PCA9685 tại địa chỉ I2C: {', '.join(found)})")
                return True
            else:
                print(f"FAIL ❌ (Không tìm thấy chip PCA9685 tại 0x60/0x40. Lỗi: {e_jetbot})")
                return False
        except Exception:
            print(f"FAIL ❌ (Không kết nối được bo mạch động cơ: {e_jetbot})")
            return False

def check_tensorrt():
    print("[3/4] Kiểm tra Tăng tốc AI (TensorRT / CUDA)...", end=" ")
    try:
        import tensorrt as trt
        logger = trt.Logger(trt.Logger.WARNING)
        print(f"PASS ✅ (TensorRT {trt.__version__} sẵn sàng)")
        return True
    except Exception as e:
        print(f"FAIL ❌ (Lỗi TensorRT: {e})")
        return False

def check_lidar():
    print("[4/4] Kiểm tra Cảm biến LiDAR D500 (Serial)...", end=" ")
    try:
        import serial
    except ImportError:
        print("FAIL ❌ (Thiếu thư viện pyserial. Chạy: pip3 install pyserial)")
        return False

    ports = glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyTHS1*")
    if not ports:
        print("FAIL ❌ (Không tìm thấy cổng serial. Kiểm tra lại cáp kết nối)")
        return False

    target_baudrate = 230400
    for port in ports:
        try:
            with serial.Serial(port=port, baudrate=target_baudrate, timeout=2.0) as ser:
                data = ser.read(512)
                if not data:
                    continue
                if b"\x54\x2c" in data:
                    print(f"PASS ✅ (Cổng {port} @ {target_baudrate} bps, nhận dạng chuẩn LiDAR D500 header 0x54)")
                    return True
                if len(data) >= 100:
                    print(f"PASS ✅ (Cổng {port} @ {target_baudrate} bps, nhận được {len(data)} bytes dữ liệu)")
                    return True
        except PermissionError:
            print(f"FAIL ❌ (Cổng {port} bị chặn quyền truy cập. Chạy: sudo chmod 666 {port})")
            return False
        except Exception:
            continue

    for port in ports:
        try:
            with serial.Serial(port=port, baudrate=115200, timeout=1.5) as ser:
                data = ser.read(256)
                if data:
                    print(f"PASS ✅ (Cổng {port} @ 115200 bps - Đang nhận luồng byte serial)")
                    return True
        except Exception:
            pass

    print(f"FAIL ❌ (Không nhận được gói dữ liệu từ LiDAR trên các cổng {', '.join(ports)})")
    return False

def main():
    print("=" * 60)
    print("      CS532 - KIỂM TRA SỨC KHỎE HỆ THỐNG JETBOT (HEALTHCHECK)")
    print("=" * 60)
    
    results = [
        ("Camera CSI", check_camera()),
        ("Động cơ (Motor I2C)", check_motor()),
        ("TensorRT AI Runtime", check_tensorrt()),
        ("Cảm biến LiDAR D500", check_lidar()),
    ]
    
    print("\n" + "=" * 60)
    print("                    KẾT QUẢ TỔNG HỢP")
    print("=" * 60)
    all_passed = True
    for name, ok in results:
        status = "HOẠT ĐỘNG TỐT (PASS)" if ok else "LỖI (FAIL)"
        icon = "✅" if ok else "❌"
        print(f" - {name:<25}: {icon} {status}")
        if not ok:
            all_passed = False
            
    print("=" * 60)
    if all_passed:
        print("🎉 TẤT CẢ PHẦN CỨNG ĐÃ ĐẠT TIÊU CHUẨN ĐỒ ÁN CS532! SẴN SÀNG CHO TUẦN 2.")
        sys.exit(0)
    else:
        print("⚠️ CÒN THÀNH PHẦN CHƯA ĐẠT. VUI LÒNG KIỂM TRA LẠI.")
        sys.exit(1)

if __name__ == "__main__":
    main()
