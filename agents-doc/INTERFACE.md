# INTERFACES.md — Hợp đồng Dữ liệu giữa các Module (CS532)

Tài liệu này quy định cấu trúc dữ liệu, kiểu dữ liệu, đơn vị đo và tần suất truyền tin giữa 4 module chính: **Setup/Hardware (LiDAR D500 & Motors)**, **Perception (Camera YOLOv8n)**, **Planning & Control**, và **Interface & Comms (Web Dashboard)**.

> **Quy tắc bất biến:** Mọi thay đổi về tên trường (field), đơn vị hoặc kiểu dữ liệu phải được các Agent liên quan đồng thuận và cập nhật tại đây trước khi chỉnh sửa mã nguồn.

---

## 1. Nguồn Dữ liệu Cảm biến

### 1.1. LiDAR D500 Driver → Planning & Control
- **Kênh truyền:** Serial UART (`/dev/ttyUSB0` @ 230400 bps) $\to$ Bộ đệm dữ liệu Python
- **Tần suất gửi:** **10 Hz** (chu kỳ $100\text{ ms}$)
- **Cấu trúc dữ liệu (`LaserScan`):**
  ```json
  {
    "timestamp": 1727500000.123,
    "scan_frequency": 10.0,
    "angle_min": 0.0,
    "angle_max": 360.0,
    "angle_increment": 1.0,
    "range_min": 0.03,
    "range_max": 12.0,
    "ranges": [1.25, 1.26, 0.0, "... (360 giá trị float, mét; 0.0 là out-of-range/lỗi)"],
    "intensities": [200, 205, 0, "... (360 giá trị int, 0 - 255)"]
  }
  ```

### 1.2. Perception (Camera YOLOv8n) → Planning & Control
- **Kênh truyền:** IPC / Shared Memory / Queue nội bộ
- **Tần suất gửi:** **15 – 20 Hz** (theo FPS thực tế của mô hình TensorRT)
- **Cấu trúc dữ liệu (`SemanticDetections`):**
  ```json
  {
    "timestamp": 1727500000.150,
    "frame_id": 1042,
    "detections": [
      {
        "class_id": 0,
        "class_name": "person",
        "confidence": 0.88,
        "bbox": [120, 80, 260, 420],
        "azimuth_deg": -15.4,
        "estimated_dist": 1.85
      }
    ]
  }
  ```
  - `azimuth_deg`: Góc lệch ngang của tâm bounding box so với trục giữa camera (FOV $160^\circ \implies [-80^\circ, +80^\circ]$). Dùng để chiếu góc và đồng bộ với chùm tia tương ứng của LiDAR.

---

## 2. Planning & Control → Interface & Comms (Web Dashboard)

- **Kênh truyền:** WebSocket / HTTP JSON Stream
- **Tần suất gửi:** **5 – 10 Hz** (tối ưu băng thông mạng WiFi và CPU Jetson Nano)
- **Cấu trúc dữ liệu (`TelemetryState`):**
  ```json
  {
    "timestamp": 1727500000.200,
    "robot_pose": {
      "x": 1.45,
      "y": 0.82,
      "theta": 0.7854
    },
    "status": "NAVIGATING",
    "speed": {
      "linear": 0.25,
      "angular": 0.10
    },
    "nearest_obstacle": {
      "distance": 0.65,
      "angle_deg": 45.0,
      "is_critical": false
    },
    "path": [
      [1.45, 0.82],
      [1.60, 0.95],
      [2.00, 1.20]
    ],
    "grid_map": {
      "resolution": 0.05,
      "width": 100,
      "height": 100,
      "origin": [-2.5, -2.5],
      "data": [0, 0, 100, -1, "... (10000 giá trị int8: -1 chưa quét, 0 trống, 100 có vật cản)"]
    },
    "laser_points": [
      [1.25, 0.0],
      [1.20, 0.08]
    ]
  }
  ```
  - `status`: Gồm 7 trạng thái chuẩn: `"IDLE"`, `"MAPPING"`, `"NAVIGATING"`, `"AVOIDING"`, `"REPLANNING"`, `"GOAL_REACHED"`, `"EMERGENCY_STOP"`.

---

## 3. Interface & Comms (Web Dashboard) → Planning & Control

- **Kênh truyền:** WebSocket Client $\to$ Server
- **Tần suất:** Theo sự kiện tương tác của người dùng (Event-driven)
- **Cấu trúc lệnh (`UserCommand`):**

### 3.1. Đặt điểm đích đến (Set Goal)
```json
{
  "command": "SET_GOAL",
  "goal": {
    "x": 2.50,
    "y": 1.80
  },
  "timestamp": 1727500005.000
}
```

### 3.2. Lệnh Dừng khẩn cấp / Hủy bỏ / Tiếp tục
```json
{
  "command": "EMERGENCY_STOP",
  "reason": "User clicked Web E-Stop button",
  "timestamp": 1727500006.100
}
```
*(Các command khác: `"RESUME"`, `"CANCEL_GOAL"`)*

---

## 4. Tầng Điều khiển Động cơ (Control Loop $\to$ Motor Driver PCA9685)

- **Kênh truyền:** Giao tiếp I2C bus 1 (địa chỉ `0x60`)
- **Tần suất vòng lặp điều khiển:** **10 Hz** (chu kỳ $100\text{ ms}$, đồng bộ hóa 1:1 với chu kỳ quét 360° của LiDAR D500)
- **Ngưỡng an toàn vật lý:**
  - Vận tốc tối đa: $|v_{max}| \le 0.35\text{ m/s}$ (tương đương giá trị PWM $0.3$ trong thư viện JetBot).
  - Vùng phanh khẩn cấp phản xạ: Nếu $\min(\text{ranges}[-30^\circ \dots +30^\circ]) < 0.20\text{ m} \implies$ ngắt tốc độ $v = 0, \omega = 0$ ngay lập tức.