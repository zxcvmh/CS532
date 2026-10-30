# CS532 — AUTONOMOUS JETBOT NAVIGATION & PERCEPTION SYSTEM
### HỆ THỐNG ĐIỀU HƯỚNG TỰ HÀNH & NÉ VẬT CẢN JETBOT ĐA TẦNG (2026)

---

## 📌 1. Giới Thiệu Tổng Quan (Project Overview)

Đồ án **CS532** phát triển hệ thống robot tự hành thông minh trên nền tảng **NVIDIA JetBot** (Jetson Nano) kết hợp cảm biến **LiDAR D500** (quét 360° UART 230400 baud) và **Camera CSI IMX219** (góc rộng 160° FOV).

Hệ thống được thiết kế theo kiến trúc **Fog-Edge Decoupled Computing**:
* **Edge Node (JetBot - Jetson Nano):** Chạy `jetbot_bridge.py` đọc cảm biến phần cứng thời gian thực, phát luồng video native MJPEG (cổng `:8080`), đóng gói dữ liệu LiDAR qua giao thức nhị phân siêu nhẹ **JB-Binary Protocol** (1,130 bytes @ 10Hz) và kích hoạt phản xạ an toàn cục bộ.
* **Fog Brain (Laptop / Trạm điều khiển trung tâm):** Xử lý thuật toán điều hướng cấp cao (Likelihood Field SLAM, Goal-Conditioned DAgger Imitation Learning, A* C-core, Cartographer 2D, CBF-QP Safety Filter) và vận hành Web Dashboard visionOS React 19.

---

## 📂 2. Cấu Trúc Dự Án (Repository Structure)

Dự án được phân rã thành **các module độc lập** theo nguyên tắc **Một tính năng / Một gói phần mềm** nhằm tối ưu hóa quy trình kiểm thử PC-First và tránh xung đột mã nguồn:

```text
CS532/
├── 01-HieuMe/               # [THỊ GIÁC MÁY TÍNH & BÙ ĐIỂM MÙ LIDAR]
│   ├── config/              # Cấu hình camera, 4 điểm hiệu chuẩn Homography IPM & Safety Margins
│   ├── core/                # Thuật toán cốt lõi: FrameGrabber, FloorIPM, YoloDetector, Proxemics
│   ├── models/              # Trọng số YOLO26-n ONNX/PT & Hướng dẫn build TensorRT FP16 .engine
│   ├── sample_data/         # Ảnh chụp thực tế từ camera CSI JetBot để kiểm thử
│   ├── pipeline.py          # Pipeline hoàn chỉnh End-to-End: Frame -> Detections + IPM -> JSON
│   ├── test_cv_pipeline.py  # Bộ kiểm thử độc lập 5/5 bài test (Pass 100% không cần phần cứng)
│   └── requirements.txt     # Thư viện phụ thuộc (opencv-python, numpy, onnxruntime)
│
├── 02-DuyHieu/              # [ĐIỀU HƯỚNG C-CORE & BỘ SINH QUỸ ĐẠO CHUYÊN GIA]
│   ├── planning/            # Nhân thuật toán A* tối ưu bằng C (astar_core.c < 0.2ms)
│   ├── controller.py        # Curvature Velocity Scaling & Plateau Detour Splicing
│   └── test_planning_local.py # Script kiểm thử A* và né cản cục bộ trên PC
│
├── 03-MinhHieu/             # [GIAO DIỆN WEB VISIONOS & BACKEND ĐIỀU PHỐI]
│   ├── frontend/            # Web Dashboard Apple visionOS glassmorphism (React 19 + Vite + Tailwind)
│   ├── backend/             # Server điều phối FastAPI, WebSocket 10Hz & Telemetry Parser
│   ├── jetbot_bridge.py     # Cầu nối phần cứng Edge (LiDAR D500, Camera CSI, Động cơ, Mux 50Hz)
│   └── start_all.sh         # Script khởi động đồng bộ toàn bộ hệ thống
│
├── 05-d500lib-motion/       # [SLAM, ODOMETRY & AI IMITATION LEARNING]
│   ├── d500_driver.py       # Driver LiDAR LDROBOT D500 chuẩn bảng tra CRC-8 & Revolution Sync
│   ├── planning/            # Tầng quy hoạch: Likelihood Field SLAM, Lever-Arm Odometry, Kinodynamic RVO
│   │   ├── il/              # Goal-Conditioned DAgger Imitation Learning (149D 1D-CNN Policy)
│   │   └── slam/            # Multi-layer Decoupled Costmap (Static Anchor, Rolling, Cluster, Low-Obs)
│   └── robot_dynamics.json  # Cấu hình động học xe thật (tốc độ, deadband động cơ GA25-370)
│
├── 06-cartographer/         # [CARTOGRAPHER 2D SLAM & ROS 2 BRIDGE]
│   ├── cartographer_core/   # Pipeline thuần Python: Odds ProbabilityGrid, CSM, Submap2D, PoseGraph2D
│   ├── config/              # Cấu hình Cartographer 2D LUA (REP-103/REP-105 TF Tree)
│   └── scripts/             # Bridge kết nối ROS 2 và Web Dashboard
│
├── agents-doc/              # [TÀI LIỆU KIẾN TRÚC & QUẢN TRỊ DỰ ÁN]
│   ├── STATE.md             # Nguồn sự thật duy nhất (Single Source of Truth) ghi lại trạng thái hệ thống
│   ├── TICKETS.md           # Hệ thống phiếu công việc và tiêu chuẩn nghiệm thu (Definition of Done)
│   ├── README.md            # Tổng quan về kiến trúc và phân công vai trò tác nhân
│   ├── agents/              # Đặc tả 4 Core Agents (Lead Supervisor, AI Learning, Researcher, Auditor)
│   └── archive/             # Lưu trữ tài liệu nghiên cứu các giai đoạn trước
│
├── scripts/                 # [KIỂM ĐỊNH TOÀN VẸN TỰ ĐỘNG]
│   └── verify_agent_integrity.py # Quét AST tĩnh 100% cú pháp Python & thẩm định đường dẫn thật
│
├── setup/                   # [CẤU HÌNH & HIỆU CHUẨN PHẦN CỨNG XE THẬT]
│   ├── robot_dynamics.json  # Single Source of Truth thông số động lực học xe
│   └── test_lidar.py        # Script đo đạc tần số quét và kiểm tra cổng UART LiDAR
│
└── teleop_gamepad.py        # [ĐIỀU KHIỂN TAY CẦM / BÀN PHÍM]
    └── Điều khiển JetBot thời gian thực bằng Gamepad (Xbox, PS4/PS5) hoặc bàn phím kèm Watchdog an toàn
```

---

## 🚀 3. Hướng Dẫn Nhanh Dành Cho Nhà Phát Triển (Quick Start)

> [!IMPORTANT]
> **NGUYÊN TẮC PC-FIRST & HARDWARE SAFETY:**
> Mọi tính năng phải được kiểm thử đạt 100% PASS trên PC trước khi chuyển giao nạp lên phần cứng JetBot (`192.168.1.11`).

### 3.1. Kiểm Tra Tính Toàn Vẹn Mã Nguồn Toàn Dự Án
Chạy bộ thẩm định AST và liên kết tài liệu:
```bash
python scripts/verify_agent_integrity.py
```
*(Yêu cầu: 100% AST Syntax Valid và 0 Hallucinated Paths).*

---

### 3.2. Module Thị Giác Máy Tính (`01-HieuMe/`)
Phụ trách: **Hiếu Me**
* **Chạy bộ Unit Test độc lập (5/5 PASS):**
  ```bash
  python 01-HieuMe/test_cv_pipeline.py
  ```
* **Chạy thử nghiệm Pipeline thời gian thực:**
  ```bash
  # Test với ảnh mẫu mặt sàn có sẵn:
  python 01-HieuMe/pipeline.py -s 01-HieuMe/sample_data/csi_floor_sample.jpg

  # Hoặc đọc trực tiếp từ luồng camera MJPEG của JetBot:
  python 01-HieuMe/pipeline.py -s "http://192.168.1.11:8080/api/stream/mjpeg"
  ```

---

### 3.3. Module Tìm Đường C-Core (`02-DuyHieu/`)
Phụ trách: **Duy Hiếu**
* Chạy bộ test tìm đường A* C-core và kiểm tra vượt vật cản:
  ```bash
  cd 02-DuyHieu
  python test_planning_local.py
  python verify_step1_astar.py
  ```

---

### 3.4. Module Web Dashboard & Backend (`03-MinhHieu/`)
Phụ trách: **Minh Hiếu / Team Lead**
* **Khởi động trọn bộ Backend & Web Dashboard:**
  ```bash
  cd 03-MinhHieu
  bash start_all.sh
  ```
* **Hoặc khởi động riêng Backend:**
  ```bash
  cd 03-MinhHieu/backend
  uvicorn app.main:app --host 0.0.0.0 --port 8000 --reload
  ```
* Truy cập Web Dashboard tại: `http://localhost:5173` (Frontend visionOS) và API Docs tại `http://localhost:8000/docs`.

---

### 3.5. Module SLAM & AI Learning Policy (`05-d500lib-motion/`)
Phụ trách: **Team Lead / AI Learning Engineer**
* Chạy kiểm thử chính sách học máy Goal-Conditioned DAgger (8/8 PASS):
  ```bash
  cd 05-d500lib-motion/planning/il
  python test_dagger_policy.py
  ```
* Huấn luyện vòng lặp DAgger với môi trường mô phỏng 2D:
  ```bash
  python dagger_trainer.py
  ```

---

### 3.6. Điều Khiển Xe Bằng Gamepad / Bàn Phím (`teleop_gamepad.py`)
* Chế độ tự động nhận diện tay cầm USB / Bluetooth:
  ```bash
  python teleop_gamepad.py
  ```
* Chế độ dự phòng điều khiển bàn phím (W/A/S/D/Space):
  ```bash
  python teleop_gamepad.py --keyboard
  ```

---

## 📡 4. Hợp Đồng Dữ Liệu Tích Hợp (Data Contracts)

### 4.1. Dữ Liệu Thị Giác (`01-HieuMe` $\to$ Planning / Dashboard)
```json
{
  "timestamp": 1727500000.150,
  "frame_id": 142,
  "detections": [
    {
      "class_id": 0,
      "class_name": "person",
      "confidence": 0.88,
      "azimuth_deg": 14.5,
      "distance": 1.25,
      "estimated_dist": 1.25,
      "safety_margin_m": 0.77,
      "bbox": [200, 150, 320, 420]
    }
  ],
  "low_obstacles": [
    [0.45, 0.08],
    [0.55, -0.12]
  ],
  "social_bubbles": [
    {
      "x": 1.21,
      "y": 0.31,
      "radius_m": 0.77,
      "azimuth_deg": 14.5,
      "distance_m": 1.25
    }
  ],
  "latency_ms": 14.2
}
```

### 4.2. Giao Thức Nhị Phân JB-Binary v1.0 (JetBot Edge $\to$ Fog Brain)
* Kích thước khung cố định: **1,130 bytes** (`Header 18B` + `LiDAR 360 rays 720B` + `Intensities 360B` + `Telemetry 32B`).
* Tần số truyền tải: **10Hz qua WebSocket**, tiết kiệm $93\%$ băng thông so với JSON thô, độ trễ $< 15\text{ms}$.

---

## 🔒 5. Quy Ước Kỷ Luật Mã Nguồn (Repository Rules)

1. **Ranh giới module:** Mỗi thành viên chỉ chỉnh sửa trong thư mục được phân công (`01-HieuMe/`, `02-DuyHieu/`, `03-MinhHieu/`, `05-d500lib-motion/`).
2. **Kiểm tra trước khi commit:** Bắt buộc chạy pass bộ unit test của module và script `verify_agent_integrity.py`.
3. **Bảo mật & Tối ưu:** Tuyệt đối không commit tệp nhị phân nặng không cần thiết (`.engine`, `node_modules`, `__pycache__`) hoặc tệp cấu hình chứa secret.
4. **Quy chuẩn điều khiển:** Mọi thuật toán phải tuân thủ nghiêm ngặt các giới hạn an toàn trong [setup/robot_dynamics.json](setup/robot_dynamics.json) ($v \le 0.12\text{ m/s}$, $|\omega| \le 0.35\text{ rad/s}$ trên sàn gạch men).
