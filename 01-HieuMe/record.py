import cv2
import time
import sys
import os
import signal
import threading
import argparse
from datetime import datetime

# Cờ điều khiển dừng
is_running = True

def get_gstreamer_pipeline(width=1280, height=720, fps=30, flip_method=0):
    return (
        f"nvarguscamerasrc ! "
        f"video/x-raw(memory:NVMM), width=(int){width}, height=(int){height}, framerate=(fraction){fps}/1 ! "
        f"nvvidconv flip-method={flip_method} ! "
        f"video/x-raw, width=(int){width}, height=(int){height}, format=(string)BGRx ! "
        f"videoconvert ! "
        f"video/x-raw, format=(string)BGR ! appsink drop=True"
    )

def wait_for_user_input():
    global is_running
    try:
        input("\n[HƯỚNG DẪN] Nhấn phím [ENTER] bất cứ lúc nào để DỪNG quay...\n\n")
        is_running = False
    except (EOFError, KeyboardInterrupt):
        is_running = False

def handle_sigint(sig, frame):
    global is_running
    print("\nNhận tín hiệu dừng (Ctrl+C)...")
    is_running = False

def parse_arguments():
    parser = argparse.ArgumentParser(description="Script quay video từ CSI Camera trên JetBot.")
    parser.add_argument(
        "-d", "--output-dir", 
        type=str, 
        default="/home/jetbot/DH216/media", 
        help="Thư mục muốn lưu video (mặc định: ./recordings)"
    )
    parser.add_argument(
        "-o", "--output-file", 
        type=str, 
        default=None, 
        help="Chỉ định chính xác tên file (ví dụ: my_video.mp4). Nếu không đặt, sẽ tự sinh theo ngày giờ."
    )
    parser.add_argument("--width", type=int, default=1280, help="Chiều rộng khung hình (mặc định 1280)")
    parser.add_argument("--height", type=int, default=720, help="Chiều cao khung hình (mặc định 720)")
    parser.add_argument("--fps", type=int, default=30, help="Tốc độ khung hình (mặc định 30)")
    parser.add_argument("--flip", type=int, default=0, help="Lật ảnh: 0 (bình thường), 2 (xoay 180 độ)")
    return parser.parse_args()

def main():
    global is_running
    args = parse_arguments()
    signal.signal(signal.SIGINT, handle_sigint)

    # 1. Xử lý đường dẫn và tên file lưu
    if args.output_file:
        # Nếu người dùng truyền cả tên file hoặc đường dẫn đầy đủ
        save_path = os.path.abspath(args.output_file)
        save_dir = os.path.dirname(save_path)
    else:
        # Nếu chỉ truyền thư mục (hoặc dùng mặc định), tự sinh tên theo timestamp
        save_dir = os.path.abspath(args.output_dir)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        save_path = os.path.join(save_dir, f"record_{timestamp}.mp4")

    # Tự động tạo thư mục nếu chưa có
    if save_dir and not os.path.exists(save_dir):
        os.makedirs(save_dir, exist_ok=True)
        print(f">> Đã tự động tạo thư mục: {save_dir}")

    # 2. Khởi tạo camera
    print("=" * 65)
    print("Đang kết nối camera CSI...")
    cap_pipeline = get_gstreamer_pipeline(args.width, args.height, args.fps, args.flip)
    cap = cv2.VideoCapture(cap_pipeline, cv2.CAP_GSTREAMER)

    if not cap.isOpened():
        print("LỖI: Không thể mở camera CSI!")
        return

    # 3. Khởi tạo bộ ghi video
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter(save_path, fourcc, args.fps, (args.width, args.height))

    if not out.isOpened():
        print(f"LỖI: Không thể ghi file vào: {save_path}")
        cap.release()
        return

    print(f"Bắt đầu quay video!")
    print(f" - Đường dẫn lưu: {save_path}")
    print(f" - Độ phân giải: {args.width}x{args.height} @ {args.fps} FPS")

    # Lắng nghe Enter ở thread phụ
    input_thread = threading.Thread(target=wait_for_user_input, daemon=True)
    input_thread.start()

    start_time = time.time()
    frame_count = 0

    try:
        while is_running:
            ret, frame = cap.read()
            if not ret:
                print("\nMất tín hiệu camera!")
                break

            out.write(frame)
            frame_count += 1

            elapsed = time.time() - start_time
            current_fps = frame_count / elapsed if elapsed > 0 else 0
            mins, secs = divmod(int(elapsed), 60)
            sys.stdout.write(
                f"\r>> [ĐANG QUAY] {mins:02d}:{secs:02d} | "
                f"Frames: {frame_count:05d} | {current_fps:.1f} FPS"
            )
            sys.stdout.flush()

    finally:
        print("\n\nĐang hoàn tất đóng file video...")
        cap.release()
        out.release()

        total_time = time.time() - start_time
        avg_fps = frame_count / total_time if total_time > 0 else 0

        print("=" * 65)
        print("HOÀN TẤT:")
        print(f" - Video đã được lưu tại: {save_path}")
        print(f" - Kích thước file: {os.path.getsize(save_path) / (1024*1024):.2f} MB")
        print(f" - Thời lượng: {total_time:.2f}s ({avg_fps:.1f} FPS trung bình)")
        print("=" * 65)

if __name__ == "__main__":
    main()