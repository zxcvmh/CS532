# CS532 AGENT BOOTSTRAP & SYSTEM CONTEXT HANDOFF (Phiên Bản 2026-10-13)

> **Hướng dẫn sử dụng:** Copy toàn bộ nội dung file này và dán vào lời nhắc (Prompt) đầu tiên của Session Agent mới để Agent nắm bắt 100% ngữ cảnh kỹ thuật, hiện trạng dự án, các bài toán đã giải quyết và tiếp tục công việc ngay lập tức mà không bị mất dấu vết.

---

## 1. TỔNG QUAN DỰ ÁN & KIẾN TRÚC HỆ THỐNG

- **Tên đồ án:** CS532 — Robot Di Động Tự Hành Trong Nhà (Autonomous Mobile Indoor JetBot).
- **Phần cứng vật lý:**
  - **Xe robot (JetBot):** NVIDIA Jetson Nano 4GB (ARM Cortex-A57 4 nhân, 128 Maxwell CUDA cores, OS: Ubuntu 18.04 / JetPack 4.5.1, Python 3.6).
  - **Cảm biến:** LiDAR D500 (DToF 360°, UART 230400 baud, tần số quét 10Hz, tầm xa 0.12m – 8.0m), Camera CSI IMX219 (160° FOV, chúc 15°-20°).
  - **Cơ cấu chấp hành:** Động cơ dẫn động vi sai 2 bánh (Differential Drive TT Motors) điều khiển qua I2C MotorHAT (PCA9685).
  - **Địa chỉ mạng:** IP JetBot: `192.168.1.11`, User: `jetbot`.
- **Máy trạm lập trình & Dashboard (PC Local):**
  - OS: Linux x86_64, Python 3.11 (Virtualenv tại `/home/zxcvmh/Projects/CS532/03-MinhHieu/backend/.venv/bin/python`).
  - Địa chỉ mạng: IP PC: `192.168.1.8`.
  - Giao diện: Web Dashboard visionOS glassmorphism (FastAPI Backend + React 19 UI @ `http://localhost:5173`), WebSocket đồng bộ bản đồ 10Hz, stream telemetry 10Hz và MJPEG camera.

---

## 2. HIỆN TRẠNG THỰC TẾ NGAY LÚC NÀY (CURRENT SITUATION)

> [!IMPORTANT]
> **TRẠNG THÁI PHẦN CỨNG JETBOT:**
> **Robot JetBot hiện đang được mang đi sạc pin / offline!**
> **Quy tắc tuyệt đối:** KHÔNG chạy lệnh `rsync` hoặc `ssh` lên `192.168.1.11` lúc này. Mọi thao tác phát triển và kiểm thử ĐỀU ĐÃ ĐƯỢC THỰC HIỆN VÀ VERIFY 100% TRÊN LOCAL PC THEO NGUYÊN TẮC PC-FIRST.

- **Trạng thái mã nguồn 4 Module Độc Lập:**
  1. `01-HieuMe/`: Module Computer Vision hoàn thiện toàn diện (YOLO26-n FP16 2.4M params, Letterbox `640x640`, Floor IPM bù mù LiDAR 16cm chống chia cho 0, Asymmetric Gaussian Proxemics). Đạt **5/5 PASS** trong `test_cv_local.py`.
  2. `02-DuyHieu/`: Module thuật toán A* mới của Duy Hiếu, đột phá tăng tốc bằng nhân C `astar_core.c` (`libastar_core.so`), đạt **0.16 ms** (nhanh gấp **39.38x** baseline cũ), khắc phục lỗi detour và trượt quay spinout.
  3. `03-MinhHieu/`: Web Dashboard hoàn thiện (React 19, FastAPI, WebSocket 10Hz, `jetbot_bridge.py` tự động khởi chạy, bộ công cụ hiệu chuẩn `test-hardware/calibrate_motors.py`).
  4. `05-d500lib-motion/`: Thư viện điều hướng thực địa & driver LiDAR D500: Likelihood Field SLAM, Lever-Arm Odometry bù tâm quay vi sai chống trượt gạch men, Single-Pass EDT, Kinodynamic RVO né cản động, Polar Gap Navigation dự phòng, và Pivot Clearance Gating triệt tiêu lỗi Actuation Deadlock ($v=0 \implies \omega=0$).
- **Hiệu chuẩn động học thực nghiệm (`setup/robot_dynamics.json`):**
  - `cruise_speed`: 0.10 m/s | `turn_speed`: 0.10 m/s | `max_angular_speed_rads`: 0.28 rad/s (an toàn tuyệt đối trên sàn gạch men, Likelihood Field SLAM và planning luôn theo kịp).

---

## 3. LỊCH SỬ TIẾN HÓA & CÁC BÀI TOÁN LỚN ĐÃ GIẢI QUYẾT

Dự án đã trải qua 5 giai đoạn đột phá kỹ thuật cốt tử:

### 3.1. Sự Cố 1: Trượt Bánh Xe Vi Sai & Bản Đồ Bị Vặn Xoắn (Wheel Slip & Map Distortion)
- **Hiện tượng:** Robot quay đầu trên gạch men bóng bị trượt vật lý $	o$ Odometry tích phân bánh xe trôi góc $	heta 	o$ Bản đồ Occupancy Grid bị méo mó.
- **Giải pháp:** Tích hợp mô-đun **2D LiDAR Odometry (Point-to-Line ICP)** chạy thời gian thực 10Hz (`lidar_odometry.py`), khóa góc quay $	heta$ theo khung tường phòng với sai số kỷ lục: **$0.003^\circ$** (độ trễ 2.1ms).

### 3.2. Sự Cố 2: Kẹt Thảm, Nhấc Bổng Xe (Kidnapped Robot) & Bản Đồ Xoay Mất Goal
- **Hiện tượng:** Robot đi vào thảm bị vấp hoặc bị nhấc sang vị trí khác $	o$ kẹt ở trạng thái `BLOCKED`, ô lưới dưới gầm xe ghi nhận vật cản chết (254).
- **Giải pháp:**
  - **Khóa bản đồ tường tĩnh (Anchor Map Locking):** Khung tường chuẩn phòng ban đầu được khóa cứng (`lock_anchor_map()`).
  - **Giải phóng gầm xe (`clear_footprint`):** Tự động xóa chi phí dưới gầm xe ($r = 0.18	ext{m}$) khi nhận Goal mới hoặc khi Replan.
  - **Tự phục hồi Kidnapped Robot:** Khi chọn Goal mới, hệ thống tự động mở khóa `BLOCKED -> PLANNING -> NAVIGATING`.

### 3.3. Sự Cố 3: Nghịch Lý Chọn Đường Vòng Qua Khe Hẹp Chân Bàn
- **Hiện tượng:** Trước mũi xe là khoảng trắng nhưng A* lại đi vòng xuống đáy bản đồ qua khe hẹp chân bàn.
- **Giải pháp:** Loại bỏ hoàn toàn `rev_penalty`, đưa `weight = 1.0` (chuẩn Admissible Heuristic Nav2 Smac Planner), bơm cụm chân bàn vào Costmap thành ô chết 254 (`insert_obstacle_clusters`), và nắn thẳng quỹ đạo theo Any-Angle String Pulling.

### 3.4. Sự Cố 4: Nút Thắt Cổ Chai A* Python & Bẫy Xoay 180° Spinout
- **Hiện tượng:** A* trên lưới ô vuông lớn bị chậm, robot quay quá gấp dẫn tới văng đuôi 180° trên sàn gạch men.
- **Giải pháp:**
  - Duy Hiếu viết nhân C tối ưu **`astar_core.c`** (`libastar_core.so`), giảm độ trễ tìm đường xuống **0.16 ms** (tăng tốc 39.38x).
  - Team Lead thiết lập bộ lọc an toàn **Control Barrier Function (CBF)** và **cosine-squared velocity attenuation**, giới hạn tốc độ góc ở **0.35 rad/s** (~20°/s) triệt tiêu hoàn toàn hiện tượng văng đuôi.

### 3.5. Sự Cố 5: Sai Lệch Tỷ Lệ Ảnh Bounding Box & Điểm Mù LiDAR 16cm
- **Hiện tượng:** Resize ảnh thô `cv2.resize((640, 640))` làm méo tỷ lệ 4:3, LiDAR quét phẳng ở độ cao 16cm nên không thấy dép/dây điện dưới sàn, ma trận IPM bị chia cho 0 khi pixel gần chân trời.
- **Giải pháp:**
  - Nâng cấp mô hình **Ultralytics YOLO26-n FP16** (2.4M params, 5.5 GFLOPs) với tiền xử lý **Letterbox Blob** (padding 114 giữ chuẩn tỷ lệ 4:3).
  - Chuẩn hóa góc Azimuth CCW (ROS REP-103: Trái dương $+80^\circ$, Phải âm $-80^\circ$).
  - Tích hợp lớp bảo vệ mẫu số $Z_{	ext{cam}} < 1	ext{e-}6 	o 999	ext{m}$ trong `floor_ipm.py`, trích xuất `low_obstacles` nạp sang Costmap bù điểm mù LiDAR.

---

## 4. CẤU TRÚC MÃ NGUỒN TRỌNG YẾU

```text
/home/zxcvmh/Projects/CS532/
├── 01-HieuMe/               # Module Thị giác máy tính (YOLO26-n, Letterbox, Floor IPM)
│   ├── yolo_seg_runner.py   # Runner suy luận YOLO26-n (TensorRT/ONNX) + Vectorized NMS
│   ├── floor_ipm.py         # Ma trận Homography bù mù LiDAR 16cm, chống chia cho 0
│   ├── bench_letterbox_jetbot.py # Benchmark đo đạc Letterbox trên JetBot
│   ├── test_cv_local.py     # 5 bài test kiểm thử PC-First (PASS 100%)
│   └── yolo26n.onnx / yolo26n.pt # Model weights thế hệ 2026
├── 02-DuyHieu/              # Module Thuật toán A* nhân C của Duy Hiếu
│   ├── planning/
│   │   ├── astar_core.c     # Nhân C tối ưu thuật toán A*
│   │   ├── astar_planner.py # Wrapper Python + làm mượt quỹ đạo
│   │   └── ...
│   ├── verify_step1_astar.py # Nghiệm thu A* C-core (0.16ms, 39x faster)
│   └── test_planning_local.py # Kiểm thử A* cục bộ
├── 03-MinhHieu/              # Module Web Dashboard & Backend Bridge
│   ├── backend/
│   │   ├── main.py          # Entrypoint tương thích Python 3.6 JetPack & auto bridge
│   │   └── app/planning/    # Đã đồng bộ nhân C astar_core.c
│   ├── frontend/            # React 19 UI visionOS glassmorphism
│   ├── test-hardware/       # calibrate_motors.py & robot_dynamics.json
│   └── jetbot_bridge.py     # Cầu nối phần cứng đọc LiDAR, Camera, Motor
├── 05-d500lib-motion/       # Module Điều hướng Thực địa & Driver LiDAR D500
│   ├── planning/
│   │   ├── astar_planner.py       # A* kết hợp nhân C
│   │   ├── cbf_safety_filter.py   # Lá chắn Control Barrier Function
│   │   ├── disparity_extender.py  # Né chướng ngại vật Disparity Extender
│   │   ├── kinodynamic_rvo.py     # Kinodynamic Reciprocal Velocity Obstacles
│   │   ├── controller.py          # Pivot Clearance Gating & Actuation Control
│   │   └── lidar_odometry.py      # Định vị quét đối sánh chống trượt sàn gạch
│   ├── d500_driver.py       # Driver LiDAR D500 qua UART 230400
│   └── jetbot_bridge.py     # Cầu nối phần cứng tích hợp soft u_spin slew-rate
└── agents-doc/
    ├── BOOTSTRAP.md         # File này (Context Handover cho Agent mới)
    ├── STATE.md             # Bảng trạng thái living state của dự án
    └── agents/              # 7 hồ sơ tác nhân nòng cốt
```

---

## 5. LỆNH CHẠY KIỂM THỬ TRÊN LOCAL PC

Hệ thống đã loại bỏ hoàn toàn các bài test lý thuyết vòng lặp mock LiDAR (vì vô dụng trên phần cứng thật). Quy trình kiểm thử tập trung vào:

```bash
# 0. Kiểm định Toàn diện Cú pháp AST & Tính Toàn vẹn Tài liệu (Zero-Hallucination)
python3 scripts/verify_agent_integrity.py

# 1. Kiểm thử Computer Vision & Floor IPM (01-HieuMe)
/home/zxcvmh/Projects/CS532/03-MinhHieu/backend/.venv/bin/python 01-HieuMe/test_cv_local.py

# 2. Benchmark A* C-Core Engine siêu tốc (02-DuyHieu)
/home/zxcvmh/Projects/CS532/03-MinhHieu/backend/.venv/bin/python 02-DuyHieu/verify_step1_astar.py
/home/zxcvmh/Projects/CS532/03-MinhHieu/backend/.venv/bin/python 02-DuyHieu/bench_hardware_dh.py
```

---

## 6. HƯỚNG DẪN TRIỂN KHAI LÊN JETBOT (KHI SẠC XONG PIN & BẬT NGUỒN)

Khi robot online (`192.168.1.11`):
1. **Kiểm tra ping:** `ping -c 2 192.168.1.11`
2. **Nạp Model YOLO26-n & Build TensorRT Engine:**
   ```bash
   scp 01-HieuMe/yolo26n.onnx jetbot@192.168.1.11:/home/jetbot/
   ssh jetbot@192.168.1.11 "/usr/src/tensorrt/bin/trtexec --onnx=/home/jetbot/yolo26n.onnx --saveEngine=/home/jetbot/yolo26n.engine --fp16"
   ```
3. **Đồng bộ mã nguồn planning:**
   ```bash
   rsync -avz --exclude=ALGORITHM_STATE.md --exclude=.venv --exclude=__pycache__      /home/zxcvmh/Projects/CS532/03-MinhHieu/backend/app/planning/      jetbot@192.168.1.11:~/03-MinhHieu/backend/app/planning/
   ```
4. **Khởi chạy hệ thống toàn diện:**
   ```bash
   ssh jetbot@192.168.1.11 "bash ~/CS532/start_all.sh"
   ```
