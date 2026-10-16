# CS532 — AUTONOMOUS JETBOT NAVIGATION & PERCEPTION SYSTEM

---

## 📂 1. Cấu Trúc Dự Án (Repository Structure)

Dự án được phân chia thành **các module độc lập** theo nguyên tắc **Một người / Một tính năng — Một thư mục** để tránh xung đột code (Merge Conflict) và tối ưu hóa quy trình kiểm thử PC-First:

```text
CS532/
├── 01-HieuMe/               # [THỊ GIÁC HOÀN THIỆN] Computer Vision, YOLO26-n, Letterbox, Floor IPM, Proxemics
│   ├── requirements.txt     # Thư viện Python (ultralytics, onnxruntime, opencv...)
│   ├── sample_data/         # Ảnh chụp thực tế từ camera CSI JetBot để căn chỉnh IPM
│   ├── floor_ipm.py         # Ma trận Homography H3x3 bù mù LiDAR 16cm & chống chia cho 0
│   ├── yolo_seg_runner.py   # Runner YOLO26-n FP16 / ONNX Runtime vectorized
│   ├── test_cv_local.py     # Script kiểm thử độc lập 5 bài test trên PC (Pass 100%)
│   └── ...
├── 02-DuyHieu/              # [ĐIỀU HƯỚNG C-CORE] Thuật toán A* nhân C, Occupancy Grid, Detour Fix của Duy Hiếu
│   ├── planning/            # Mã nguồn lõi thuật toán điều hướng
│   │   ├── astar_core.c     # Nhân thuật toán A* tối ưu bằng C
│   │   └── ...
│   ├── requirements.txt     # Thư viện Python cho máy tính cá nhân
│   ├── test_planning_local.py # Script kiểm thử A* trên PC
│   ├── verify_step1_astar.py  # Đánh giá benchmark A* C-core
│   └── ...
├── 03-MinhHieu/              # [GIAO DIỆN & CẦU NỐI] Web Dashboard visionOS & Backend Bridge
│   ├── frontend/            # Giao diện Apple visionOS glassmorphism (React 19 + Vite + Tailwind)
│   ├── backend/             # Server điều phối WebSocket 10Hz & REST API (FastAPI)
│   └── jetbot_bridge.py     # Cầu nối phần cứng đọc LiDAR D500, Camera CSI và điều khiển động cơ
├── 04-hieu-MotionControl/   # [ĐIỀU KHIỂN NÂNG CAO] Thuật toán riêng cải tiến của Team Lead
│   ├── planning/            # Lõi thuật toán DDE, CBF Safety Filter, LiDAR Odometry
│   │   ├── cbf_safety_filter.py   # Bộ lọc an toàn Control Barrier Function
│   │   ├── disparity_extender.py  # Thuật toán né chướng ngại vật Dynamic Disparity Extender
│   │   └── lidar_odometry.py      # Định vị tương đối bằng quét đối sánh LiDAR
│   ├── ALGORITHM_STATE.md   # Nhật ký ma trận giả thuyết và thông số sa bàn 2m x 1m
│   ├── test_closed_loop_continuous_navigation.py # Test vòng lặp điều khiển kín liên tục
│   └── ...
├── task-team/               # [KẾ HOẠCH CHI TIẾT] File giao việc và tiêu chuẩn nghiệm thu
│   ├── plan_algorithm_duyhieu.md      # Kế hoạch kỹ thuật chi tiết cho Duy Hiếu
│   └── plan_computer_vision_hieume.md  # Kế hoạch kỹ thuật chi tiết cho Hiếu me & CV
├── agents-doc/              # [HIẾN PHÁP DỰ ÁN] Hợp đồng dữ liệu & hồ sơ tác nhân
│   ├── INTERFACE.md         # Chuẩn dữ liệu LaserScan, SemanticDetections, low_obstacles
│   ├── STATE.md             # Bảng trạng thái living state của toàn bộ dự án
│   ├── WORKFLOW.MD          # Quy chế phân tầng đa tần số và an toàn phần cứng
│   └── agents/              # Đặc tả các Agent nòng cốt (Supervisor, Algorithm, Perception, Comms, Helpdesk)
└── setup/                   # Script kiểm tra phần cứng và đo đạc benchmark trên Jetson
```

---

## 🚀 2. Hướng Dẫn Dành Cho Thành Viên (Developer Quick Start)

> [!IMPORTANT]
> **NGUYÊN TẮC PC-FIRST:**
> Các thành viên phát triển và kiểm thử thuật toán độc lập trên PC cá nhân trước. Chỉ nạp lên JetBot (`192.168.1.11`) khi bộ test cục bộ đạt 100% PASS!

### 2.1. Dành cho Duy Hiếu (`02-DuyHieu/`)
* Đọc kỹ kế hoạch tại: [`task-team/plan_algorithm_duyhieu.md`](task-team/plan_algorithm_duyhieu.md)
* Chạy bộ test A* C-core và kiểm tra vượt vật cản:
  ```bash
  cd 02-DuyHieu
  python3 test_planning_local.py
  python3 verify_step1_astar.py
  ```

### 2.2. Dành cho Hiếu me (`01-HieuMe/`)
* Đọc kỹ kế hoạch tại: [`task-team/plan_computer_vision_hieume.md`](task-team/plan_computer_vision_hieume.md)
* Chạy bộ test thị giác trên PC (kiểm tra góc lệch Azimuth, Social Bubble Proxemics và ma trận IPM):
  ```bash
  cd 01-HieuMe
  python3 test_cv_local.py
  ```

### 2.3. Dành cho Team Lead (`04-hieu-MotionControl/` & Tích hợp)
* Chạy kiểm thử thuật toán điều khiển kín DDE + CBF:
  ```bash
  cd 04-hieu-MotionControl
  python3 test_closed_loop_continuous_navigation.py
  ```
* Khởi động Web Dashboard và Backend:
  ```bash
  cd 03-MinhHieu/backend
  python3 run.py --host 0.0.0.0 --port 8000
  ```

---

## 📡 3. Chuẩn Hợp Đồng Dữ Liệu (`agents-doc/INTERFACE.md`)

* **Perception $	o$ Planning / Backend (`SemanticDetections` & `low_obstacles`):**
  ```json
  {
    "timestamp": 1727500000.150,
    "frame_id": 1024,
    "detections": [
      {
        "class_name": "person",
        "azimuth_deg": 15.4,
        "confidence": 0.88,
        "safety_margin_m": 0.80,
        "distance": 1.85
      }
    ],
    "low_obstacles": [[0.65, 0.12], [0.70, -0.15]],
    "latency_ms": 21.5
  }
  ```

---

## 🔒 4. Quy Ước Git (Git Rules)

1. Mỗi thành viên chỉ chỉnh sửa trong thư mục được phân công (`01-HieuMe/`, `02-DuyHieu/`, `03-MinhHieu/`, `04-hieu-MotionControl/`).
2. Trước khi push code, bắt buộc chạy pass bộ test PC tương ứng.
3. Không commit file nhị phân nặng không cần thiết (`.engine`, `__pycache__`, `node_modules`).
