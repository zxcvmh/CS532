# 06-CARTOGRAPHER: DISTRIBUTED 2D LIDAR SLAM FOR NVIDIA JETBOT CS532
=====================================================================

**Authors & References**:
- Wolfgang Hess, Damon Kohler, Holger Rapp, Daniel Andor (Google, 2016): *"Real-Time Loop Closure in 2D LIDAR SLAM"*, IEEE ICRA 2016.
- Edwin Olson (2009): *"Real-Time Correlative Scan Matching"*, IEEE ICRA 2009.
- ROS 2 REP-103 (Units & Coordinate Conventions) & REP-105 (Mobile Robot Coordinate Frames).

---

## 1. MÔ HÌNH HỆ THỐNG PHÂN TÁN (DISTRIBUTED ARCHITECTURE)

```
[ EDGE: NVIDIA Jetson Nano ]
  ├── LDROBOT D500 LiDAR: Single-Revolution Sweep (~100ms, không sliding cache, không gap-filling)
  ├── CSI Camera: Native HTTP MJPEG Streamer (Port 8080, /video_feed) — Tách kênh độc lập
  ├── YOLOv8 / YOLO26 TensorRT Engine: Edge inference (bounding boxes, low obstacles)
  └── jetbot_bridge.py: Phát JB-Binary (1,130 bytes) trực tiếp qua WebSocket/UDP đến Laptop
          │
          │ Wi-Fi LAN (Port 8000 / 8765) — Hoàn toàn không bị nghẽn bởi Base64 Camera
          ▼
[ COMPUTE SERVER: Laptop (Ubuntu / ROS 2) ]
  ├── 1. `jetbot_ros_bridge_node.py`:
  │       - Giải mã JB-Binary (1,130 bytes)
  │       - Xuất chuẩn `sensor_msgs/msg/LaserScan` lên topic `/scan` (QoS SENSOR_DATA)
  │       - Phát TF cố định REP-105: `base_link` -> `laser_frame` (+0.08m lever arm)
  │
  ├── 2. `cartographer_node` (hoặc `cartographer_core`):
  │       - Local SLAM: Real-Time Correlative Scan Matcher (CSM) + Ceres Submap Matching
  │       - ActiveSubmaps2D: 2 submap gối đầu nhau 50%, đóng gói sau 35 scans (~3.5s)
  │       - Global SLAM: Branch-and-Bound Loop Closure + Sparse Pose Adjustment (SPA)
  │       - Tự sinh odometry: `provide_odom_frame = true` (`map` -> `odom` -> `base_link`)
  │
  ├── 3. `cartographer_occupancy_grid_node`:
  │       - Xuất bản đồ chuẩn `nav_msgs/msg/OccupancyGrid` lên `/map` (độ phân giải 5cm)
  │
  └── 4. `map_bridge_to_web.py`:
          - Mã hóa bản đồ và robot pose thành Base64 gửi lên Web Dashboard thời gian thực
```

---

## 2. CẤU TRÚC THƯ MỤC `06-cartographer/`

```
06-cartographer/
├── package.xml                       # Định nghĩa package chuẩn ROS 2 (ament_python)
├── setup.py                          # Cài đặt entrypoints và share data
├── config/
│   └── jetbot_d500_2d.lua            # Cấu hình Cartographer 2D chuẩn tác giả (đã sửa lỗi published_frame)
├── launch/
│   └── cartographer.launch.py        # Launch file tự động khởi động toàn bộ pipeline
├── scripts/
│   ├── __init__.py
│   ├── jetbot_ros_bridge_node.py     # Receiver node nhận JB-Binary -> /scan + TF
│   ├── map_bridge_to_web.py          # Bridge /map & TF -> Web Dashboard
│   └── test_cartographer_pipeline.py # Bộ kiểm chứng tự động toàn bộ giải thuật
├── cartographer_core/                # Thư viện thuật toán Cartographer 2D nguyên bản (Hess et al. 2016)
│   ├── __init__.py
│   ├── probability_grid.py           # ProbabilityGrid2D với Odds (p_hit=0.55, p_miss=0.49)
│   ├── correlative_scan_matcher.py   # Real-Time Correlative Scan Matcher 2D (CSM)
│   ├── submap_2d.py                  # Submap2D & ActiveSubmaps2D (dual-overlapping submaps)
│   └── pose_graph_2d.py              # PoseGraph2D với Loop Closure & SPA Optimization
└── README.md
```

---

## 3. CÁC ĐIỂM SỬA CHỮA QUAN TRỌNG SO VỚI BẢN PHÁC THẢO BAN ĐẦU

1. **Sửa lỗi cấu hình TF trong `jetbot_d500_2d.lua`**:
   - *Phác thảo cũ trong plan*: `published_frame = "odom"`, `provide_odom_frame = true`. Đây là lỗi tuần hoàn TF (`odom` -> `odom`), làm Cartographer sụp đổ ngay khi khởi động.
   - *Chuẩn tác giả Cartographer (Scenario A)*:
     - `tracking_frame = "base_link"`
     - `published_frame = "base_link"`
     - `odom_frame = "odom"`
     - `provide_odom_frame = true`
     - Cây TF chuẩn xác tuyệt đối: `map` $\to$ `odom` $\to$ `base_link` $\to$ `laser_frame`.
2. **Loại bỏ hoàn toàn Leaky Sliding Cache và Fake Gap-filling trong `d500_driver.py`**:
   - `03-MinhHieu/d500_driver.py` và `05-d500lib-motion/d500_driver.py` chuyển sang **Pure Single-Revolution Sweep**: chỉ xuất các điểm thực sự đo được trong 100ms của vòng quay hiện tại. Không chắp vá scan cũ, không tự tính trung bình bịa vật cản ở khe hẹp.
3. **Tách biệt luồng Camera trong `jetbot_bridge.py`**:
   - `03-MinhHieu/jetbot_bridge.py` và `05-d500lib-motion/jetbot_bridge.py` không còn gửi Base64 JPEG (25KB) qua WebSocket của LiDAR/Telemetry.
   - Camera được truyền riêng qua máy chủ Native HTTP MJPEG ở cổng 8080 (`/video_feed`).
   - Gói JB-Binary 1,130 bytes đi trên WebSocket hoàn toàn không bị trễ hay nghẽn cổ chai (Zero Head-of-Line Blocking).
4. **Giải phóng Bayesian Raycaster và Anchor Lock**:
   - `bayesian_raycaster.py`: Bỏ cấm xóa tường (`free_mask[confirmed_walls] = False`), cho phép toàn bộ tia laser dọn sạch các sai số cũ theo đúng định lý Bayes.
   - `telemetry_parser.py`: Vô hiệu hóa `set_anchor_from_scan` để bản đồ luôn là thực thể động tích lũy.

---

## 4. HƯỚNG DẪN CHẠY VÀ KIỂM CHỨNG (STEP-BY-STEP)

### Cách 1: Chạy kiểm chứng giải thuật độc lập (Deterministic Audit)
```bash
.venv/bin/python 06-cartographer/scripts/test_cartographer_pipeline.py
```
*Kết quả*: Tất cả các bài kiểm tra giải thuật (JB-Binary, Odds Raycasting, CSM Convergence, Submaps Lifecycle, Pose Graph) đạt 100% PASS.

### Cách 2: Chạy đầy đủ trên hệ thống ROS 2 Laptop
1. Build package:
   ```bash
   colcon build --packages-select jetbot_cartographer
   source install/setup.bash
   ```
2. Khởi chạy toàn bộ hệ thống:
   ```bash
   ros2 launch jetbot_cartographer cartographer.launch.py \
       ws_url:=ws://<JETBOT_IP>:8000/ws/robot
   ```
