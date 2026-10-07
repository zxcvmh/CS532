# CS532 — AUTONOMOUS JETBOT NAVIGATION & PERCEPTION SYSTEM
### Trường Đại học Công nghệ Thông tin (UIT) — ĐHQG-HCM

> **Đồ án:** Robot Di động Tự hành trong nhà (Autonomous Mobile Robot)  
> **Nền tảng mục tiêu:** NVIDIA JetBot (Jetson Nano 4GB, Ubuntu 18.04 / JetPack 4.5)  
> **Cảm biến:** LiDAR D500 (360° DToF @ 230400 baud, 10Hz, cao 16cm so với sàn), Camera CSI Sony IMX219 (160° FOV)  
> **Mô hình AI:** YOLOv8n TensorRT FP16 (Nhận diện đối tượng & Phân đoạn mặt sàn bù góc mù LiDAR)

---

## 📂 1. Cấu Trúc Dự Án (Repository Structure)

Dự án được phân chia thành **các module độc lập** theo nguyên tắc **Một người — Một thư mục** để tránh xung đột code (Merge Conflict) khi `git pull / push`:

```text
CS532/
├── 01-HieuMe/               # [THỊ GIÁC] Module Computer Vision, YOLOv8n, Phân đoạn sàn IPM
│   ├── requirements.txt     # Thư viện Python cho máy tính cá nhân
│   ├── sample_data/         # Ảnh chụp thực tế từ camera CSI JetBot để căn chỉnh IPM
│   ├── test_cv_local.py     # Script kiểm thử độc lập trên PC (Không cần robot thật)
│   └── ...
├── 02-DuyHieu/              # [ĐIỀU HƯỚNG] Thuật toán A*, Occupancy Grid, Pure Pursuit, Gom cụm
│   ├── planning/            # Mã nguồn lõi thuật toán
│   ├── requirements.txt     # Thư viện Python cho máy tính cá nhân
│   ├── test_planning_local.py # Script kiểm thử giả lập trên PC (Không cần robot thật)
│   └── ...
├── 03-MinhHieu/              # [GIAO DIỆN & CẦU NỐI] Web Dashboard & Backend Bridge
│   ├── frontend/            # Giao diện điều khiển Apple glassmorphism (React + Vite + Tailwind)
│   ├── backend/             # Server điều phối WebSocket & REST API (FastAPI)
│   └── jetbot_bridge.py     # Cầu nối phần cứng đọc LiDAR, Camera và điều khiển động cơ
├── task-team/               # [KẾ HOẠCH CHI TIẾT] File giao việc và tiêu chuẩn nghiệm thu
│   ├── plan_algorithm_duyhieu.md      # Kế hoạch chi tiết cho Duy Hiếu
│   └── plan_computer_vision_hieume.md  # Kế hoạch chi tiết cho Hiếu me
├── agents-doc/              # [HIẾN PHÁP DỰ ÁN] Hợp đồng dữ liệu & chuẩn giao tiếp
│   ├── INTERFACE.md         # Chuẩn dữ liệu LaserScan, SemanticDetections, TelemetryState
│   └── WORKFLOW.MD          # Quy chế phân tầng đa tần số và an toàn phần cứng
├── setup/                   # Script kiểm tra phần cứng và đo đạc benchmark trên Jetson
├── .env.example             # File mẫu cấu hình biến môi trường
└── README.md                # Tài liệu hướng dẫn này
```

---

## 🚀 2. Hướng Dẫn Dành Cho Thành Viên (Developer Quick Start)

> [!IMPORTANT]
> **LƯU Ý:** Cả **Duy Hiếu** và **Hiếu me** hiện **KHÔNG GIỮ ROBOT THỰC TẾ**.  
> Tất cả công việc phát triển được thiết kế để các bạn **tự code, debug và test 100% trên Laptop/PC cá nhân**!

### 2.1. Dành cho Duy Hiếu (Phụ trách Thuật toán Điều hướng)
* Đọc kỹ kế hoạch tại: [`task-team/plan_algorithm_duyhieu.md`](task-team/plan_algorithm_duyhieu.md)
* Cài đặt môi trường trên PC:
  ```bash
  cd 02-DuyHieu
  pip install -r requirements.txt
  ```
* Chạy bộ test giả lập trên PC (tự sinh dữ liệu LiDAR 360 tia, kiểm tra A*, né hình thang Plateau, điều tốc vào cua):
  ```bash
  python test_planning_local.py
  ```
* **Khi xong việc:** Đóng gói code trong `planning/` và gửi script test cho Team Lead để nạp lên xe thật đo đạc.

---

### 2.2. Dành cho Hiếu me (Phụ trách Thị giác Máy tính)
* Đọc kỹ kế hoạch tại: [`task-team/plan_computer_vision_hieume.md`](task-team/plan_computer_vision_hieume.md)
* Cài đặt môi trường trên PC:
  ```bash
  cd 01-HieuMe
  pip install -r requirements.txt
  ```
* Chạy bộ test thị giác trên PC (dùng ảnh mẫu CSI camera trong `sample_data/` để test góc lệch Azimuth, Social Bubble và ma trận IPM):
  ```bash
  python test_cv_local.py
  ```
* **Khi xong việc:** Export mô hình `yolov8n-seg.onnx`, gửi file model và code cho Team Lead để build TensorRT FP16 trên Jetson Nano.

---

### 2.3. Dành cho Team Lead (Người giữ robot & Tích hợp)
* Cài đặt cấu hình môi trường:
  ```bash
  cp .env.example .env
  ```
* Khởi động Web Dashboard và Backend trên Laptop điều khiển:
  ```bash
  cd 03-MinhHieu
  ./run_laptop.sh
  ```
* Nạp cầu nối phần cứng trên JetBot (qua SSH):
  ```bash
  ssh jetbot-at-home "cd ~/mhieu && ./run_bridge.sh"
  ```

---

## 📡 3. Chuẩn Hợp Đồng Dữ Liệu (`agents-doc/INTERFACE.md`)

* **Hiếu me $\to$ Team Lead (`SemanticDetections` & `low_obstacles`):**
  ```json
  {
    "timestamp": 1727500000.150,
    "detections": [
      {"class_name": "person", "azimuth_deg": -15.4, "confidence": 0.88, "safety_margin_m": 0.90}
    ],
    "low_obstacles": [[0.65, 0.12], [0.70, -0.15]]
  }
  ```
* **Duy Hiếu $\to$ Team Lead (`TelemetryState`):**
  * `robot_pose`: $(x, y, \theta)$ tính bằng mét và radian.
  * `path`: Danh sách waypoints $[[x_1, y_1], [x_2, y_2], ...]$ đã làm mượt.
  * Phản xạ phanh khẩn cấp: Tự ngắt $v = 0, \omega = 0$ khi vật cản $< 0.20\text{ m}$.

---

## 🔒 4. Quy Ước Git (Git Rules)

1. Mỗi thành viên chỉ chỉnh sửa trong thư mục được phân công:
   * Hiếu me: chỉ sửa trong `01-HieuMe/`
   * Duy Hiếu: chỉ sửa trong `02-DuyHieu/`
2. Trước khi push code, đảm bảo đã chạy pass script test trên PC:
   * Duy Hiếu: `python 02-DuyHieu/test_planning_local.py`
   * Hiếu me: `python 01-HieuMe/test_cv_local.py`
3. Không commit file nặng không cần thiết (`.engine`, `__pycache__`, `node_modules`).
