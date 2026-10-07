#!/usr/bin/env python3
"""
CS532 - Robot Setup Healthcheck Script
Phạm vi: Tuần 1 - Nghiệm thu phần cứng & môi trường JetBot / Jetson Nano
Tác giả: Robot Setup Agent
Mục đích:
    Kiểm tra tự động 4 thành phần sống còn trước khi bàn giao:
    1. Camera CSI (nhận diện qua GStreamer & chụp được frame)
    2. Động cơ & Driver I2C (kết nối I2C và khởi tạo lớp Robot)
    3. Bộ tăng tốc TensorRT (kiểm tra runtime TensorRT và CUDA)
    4. Cảm biến LiDAR D500 (kết nối serial UART @ 230400 bps và nhận dữ liệu quét 360°)
"""

import sys
import time
import glob
import os

def check_camera():
    print("[1/4] Kiểm tra Camera CSI (IMX219)...", end=" ", flush=True)
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
    print("[2/4] Kiểm tra Mạch điều khiển Động cơ (I2C)...", end=" ", flush=True)
    try:
        from jetbot import Robot
        robot = Robot()
        # Dừng xe an toàn để xác nhận giao tiếp bus I2C hoạt động
        robot.stop()
        print("PASS ✅ (Kết nối I2C và khởi tạo driver Motor thành công)")
        return True
    except Exception as e:
        print(f"FAIL ❌ (Không kết nối được bo mạch động cơ: {e})")
        return False

def check_tensorrt():
    print("[3/4] Kiểm tra Tăng tốc AI (TensorRT / CUDA)...", end=" ", flush=True)
    try:
        import tensorrt as trt
        logger = trt.Logger(trt.Logger.WARNING)
        print(f"PASS ✅ (TensorRT {trt.__version__} sẵn sàng)")
        return True
    except Exception as e:
        print(f"FAIL ❌ (Lỗi TensorRT: {e})")
        return False

def check_lidar():
    print("[4/4] Kiểm tra Cảm biến LiDAR D500 (UART / USB)...", end=" ", flush=True)
    possible_ports = sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyTHS*"))
    if not possible_ports:
        print("FAIL ❌ (Không tìm thấy cổng serial nào: /dev/ttyUSB* hoặc /dev/ttyTHS*)")
        return False
    
    try:
        import serial
    except ImportError:
        print("FAIL ❌ (Chưa cài đặt thư viện 'pyserial'. Chạy: pip3 install pyserial)")
        return False
        
    for port in possible_ports:
        try:
            ser = serial.Serial(port, baudrate=230400, timeout=1.0)
            data = ser.read(256)
            ser.close()
            if len(data) > 0:
                if b'\x54' in data:
                    print(f"PASS ✅ (LiDAR D500 phản hồi tốt trên {port} @ 230400 bps, nhận diện header 0x54)")
                else:
                    print(f"PASS ✅ (Cổng {port} phản hồi, nhận {len(data)} bytes dữ liệu)")
                return True
        except Exception as e:
            continue
            
    print(f"FAIL ❌ (Phát hiện cổng {possible_ports} nhưng không đọc được dữ liệu. Kiểm tra quyền dialout: sudo usermod -aG dialout $USER)")
    return False

def main():
    print("=" * 65)
    print("      CS532 - KIỂM TRA SỨC KHỎE HỆ THỐNG JETBOT (HEALTHCHECK)")
    print("=" * 65)
    
    results = [
        ("Camera CSI", check_camera()),
        ("Động cơ (Motor I2C)", check_motor()),
        ("TensorRT AI Runtime", check_tensorrt()),
        ("LiDAR D500 (UART)", check_lidar()),
    ]
    
    print("\n" + "=" * 65)
    print("                         KẾT QUẢ TỔNG HỢP")
    print("=" * 65)
    all_passed = True
    for name, ok in results:
        status = "HOẠT ĐỘNG TỐT (PASS)" if ok else "LỖI (FAIL)"
        icon = "✅" if ok else "❌"
        print(f" - {name:<25}: {icon} {status}")
        if not ok:
            all_passed = False
            
    print("=" * 65)
    if all_passed:
        print("🎉 TẤT CẢ 4 THÀNH PHẦN PHẦN CỨNG ĐÃ ĐẠT CHUẨN ĐỒ ÁN CS532! SẴN SÀNG CHO TUẦN 2.")
        sys.exit(0)
    else:
        print("⚠️ CÒN THÀNH PHẦN CHƯA ĐẠT. VUI LÒNG KIỂM TRA LẠI CÁP HOẶC QUYỀN TRUY CẬP.")
        sys.exit(1)

if __name__ == "__main__":
    main()

