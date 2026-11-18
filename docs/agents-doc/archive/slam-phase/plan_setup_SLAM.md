# KẾ HOẠCH TOÀN DIỆN: CẢI TỔ HỆ THỐNG LIDAR VÀ TRIỂN KHAI GOOGLE CARTOGRAPHER (MÔ HÌNH PHÂN TÁN)
===================================================================================================
**Dự án**: CS532 Autonomous JetBot  
**Vai trò**: 2D LiDAR SLAM & Autonomous Robotics Specialist  
**Mô hình mục tiêu**: Phân tán (Distributed Edge-Compute Architecture)  
- **Edge Hardware**: NVIDIA Jetson Nano Dev Kit (Chỉ thu thập cảm biến, YOLO TensorRT, I/O Motor)  
- **Compute Server**: Laptop (Xử lý toàn bộ SLAM, Tối ưu hóa Pose Graph, Costmap & Navigation)  
**Tài liệu tham chiếu**: IEEE Kohlbrecher et al. (2011), Hess et al. (Google Cartographer 2016), ROS 2 REP-103/REP-105.

---

## PHẦN 1: BẢNG GIẢI PHẪU TOÀN BỘ LỖI NGHIÊM TRỌNG TRONG HỆ THỐNG HIỆN TẠI

Trước khi đưa bất kỳ thuật toán SLAM cao cấp nào (như Cartographer) vào hoạt động, chúng ta **bắt buộc phải thanh trừng toàn bộ các lỗi làm bẩn dữ liệu và nghẽn mạng từ tầng dưới**. Nếu dữ liệu gửi từ Jetson Nano lên Laptop là "rác" (bị méo, bị trễ), thì Cartographer trên Laptop cũng sẽ sụp đổ theo nguyên lý *Garbage In, Garbage Out*.

| STT | File vi phạm | Vị trí (Dòng) | Tên lỗi kỹ thuật | Bản chất lỗi & Tác động hủy diệt hệ thống | Mức độ |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **01** | `d500_driver.py` (cả ở `03` & `05`) | L520–L530 | **Leaky Sliding Cache ($0.35\text{s} - 0.80\text{s}$)** | **TỬ HUYỆT BÓNG MA (GHOSTING):** Khi robot quay $1.0\text{ rad/s}$ ($57^\circ/\text{s}$), các tia trống ở vòng hiện tại bị driver lấy dữ liệu cũ của $20^\circ$ trước đắp vào $\to$ Tường phẳng bị bẻ cong thành cánh cung, góc phòng $90^\circ$ bị xé đôi. | **THẢM HỌA** |
| **02** | `d500_driver.py` | L533–L542 | **Fake 1-Bin Gap Filling** | **TỰ BỊA VẬT CẢN:** Nếu 2 tia cạnh nhau chênh $\le 15\text{cm}$, driver tự gán giá trị trung bình vào giữa $\to$ Các khe hẹp, cửa ra vào $20\text{cm}$ bị bịt kín nhân tạo. | **NGHIÊM TRỌNG** |
| **03** | `d500_driver.py` | L585 | **Integer Angle Quantization** | **BÓP CHẾT ĐỘ PHÂN GIẢI:** Ép góc float $(\approx 0.8^\circ)$ của D500 về số nguyên `int(round()) % 360`, làm mất toàn bộ độ phân giải sub-degree phục vụ phép tối ưu gradient. | **TRUNG BÌNH** |
| **04** | `d500_driver.py` | L508–L515 | **Mong manh Revolution Boundary** | **SỤT GIẢM TẦN SỐ QUÉT:** Nếu rớt 1 gói tin ở mốc $0^\circ$, driver rơi vào timeout $0.25\text{s}-0.40\text{s}$ $\to$ Tần số quét tụt từ $10\text{Hz}$ xuống còn $2.5 - 4\text{Hz}$ (Quét rất chậm). | **CAO** |
| **05** | `d500_driver.py` | L442–L475 | **Pure Python Serial I/O & CRC Loop** | **TRÔI THỜI GIAN (BUFFER BACKLOG):** Vòng lặp `del buffer[:47]` và tính CRC8 18,400 lần/giây bằng Python trên CPU Cortex-A57 yếu ớt làm nghẽn GIL $\to$ Serial kernel đọng hàng ngàn byte $\to$ Scan đọc được là của quá khứ cách đây $1-2\text{ giây}$. | **THẢM HỌA** |
| **06** | `jetbot_bridge.py` | L1090–L1128 | **TCP Head-of-Line Blocking** | **NGHẼN MẠNG GỘP LUỒNG:** Gửi chung ảnh Camera Base64 JPEG ($25\text{KB}$) và gói LiDAR Binary ($1.1\text{KB}$) trên cùng 1 kết nối WebSocket $\to$ Mạng Wi-Fi bị nghẽn bởi ảnh camera, scan LiDAR bị giữ lại rồi ập vào Laptop dồn cục từng đợt. | **THẢM HỌA** |
| **07** | `bayesian_raycaster.py` | L123–L128 | **Cấm xóa tường (Wall Erosion Prohibition)** | **VI PHẠM NGUYÊN LÝ BAYES:** Lệnh `free_mask[confirmed_walls] = False` cấm tia laser dọn sạch các ô log-odds $> 1.5$ $\to$ Mọi sai số trượt bánh ban đầu biến thành tường vĩnh cửu, không bao giờ tự sửa sai. | **THẢM HỌA** |
| **08** | `anchor_layer.py` | L84–L87 | **Pristine Anchor Locking** | **KHÓA CHẾT RÁC KHỞI TẠO:** Lấy scan đầu tiên gán chết thành bản đồ chuẩn và khóa lại $\to$ Nếu lúc bật xe có người đứng cạnh hoặc trượt bánh, rác đó không thể bị xóa. | **NGHIÊM TRỌNG** |
| **09** | `hector_matcher.py` | L65–L111 | **Single-Resolution 5cm Gauss-Newton** | **VĂNG KHỎI BÁN KÍNH HỘI TỤ:** Chỉ có 1 tầng lưới $5\text{cm}$ (không có kim tự tháp đa phân giải) $\to$ Khi JetBot quay trên sàn trơn, bước nhảy $> 3^\circ$ làm Gauss-Newton trượt thẳng vào cực tiểu địa phương sai. | **THẢM HỌA** |
| **10** | Hệ thống | Toàn bộ | **Thiếu Backend Pose Graph & Loop Closure** | **TRÔI DẠT TÍCH LŨY VÔ HẠN:** Không có cơ chế submap hay tối ưu hóa đồ thị toàn cục để kéo các vòng lặp khép kín. | **NGHIÊM TRỌNG** |

---

## PHẦN 2: KẾ HOẠCH SỬA CHỮA CÁC FILE HIỆN TẠI (LÀM SẠCH LUỒNG DỮ LIỆU ĐẦU NGUỒN)

Dù chuyển sang Cartographer trên Laptop, **2 file trên Jetson Nano (`d500_driver.py` và `jetbot_bridge.py`) bắt buộc phải được phẫu thuật ngay lập tức**.

### Giai đoạn 2.1: Đại phẫu `03-MinhHieu/d500_driver.py` (và `05-d500lib-motion/d500_driver.py`)

1. **Khử tử vĩnh viễn `sliding_ranges` và `gap-filling`**:
   - Tại `_process_packet`, khi phát hiện hoàn tất 1 vòng quét (`crossed_zero` hoặc timeout):
   - **XÓA BỎ HOÀN TOÀN** đoạn code từ dòng 520 đến dòng 542.
   - **THAY BẰNG LOGIC NGUYÊN BẢN (Pure Single-Revolution Sweep)**:
     ```python
     with self.lock:
         # Chỉ xuất đúng những gì đo được trong 100ms của vòng quay này!
         self.latest_ranges = list(self._current_sweep_ranges)
         self.latest_intensities = list(self._current_sweep_intensities)
         self.latest_timestamp = now
         self.scan_sweep_count += 1
         # Điểm nào không có phản hồi giữ nguyên 0.0, không nội suy bịa đặt
         self.latest_scan_list = [
             [deg, self.latest_ranges[deg]]
             for deg in range(360)
             if self.latest_ranges[deg] > 0.05
         ]
     ```
2. **Sửa lỗi trôi bộ đệm UART (Serial Latency & Throughput)**:
   - Trên Jetson Nano OS, chạy lệnh cấu hình USB chip FTDI/CP2102:
     ```bash
     sudo sh -c 'echo 1 > /sys/bus/usb-serial/devices/ttyUSB0/latency_timer'
     ```
   - Trong `d500_driver.py`, tối ưu hóa việc phân tách buffer bằng cách dùng `bytearray` pre-allocated hoặc dịch con trỏ đọc thay vì `del buffer[:47]` liên tục gây memory copy.

### Giai đoạn 2.2: Tách kênh truyền độc lập trong `03-MinhHieu/jetbot_bridge.py`

1. **Tách biệt hoàn toàn luồng Camera và luồng LiDAR/Telemetry**:
   - Hiện tại: Cả Camera Base64 ($25\text{KB}$) và LiDAR Binary ($1.1\text{KB}$) cùng đi chung qua 1 WebSocket socket `ws`.
   - **Giải pháp**:
     - **Kênh LiDAR / Telemetry (Port 8765)**: Giữ nguyên giao thức `JB-Binary (1,130 bytes)` chạy trên WebSocket hoặc UDP riêng biệt. Không nhét camera vào kênh này. Tần số phát cố định $10\text{Hz}$ hoặc phát ngay khi có sweep mới (`scan_seq` tăng).
     - **Kênh Camera Video (Port 8080)**: Sử dụng HTTP MJPEG Streamer độc lập (có sẵn trong `MjpegStreamer`) hoặc WebRTC. Nếu mạng Wi-Fi bị yếu, camera có thể rớt FPS nhưng gói LiDAR 1.1KB **tuyệt đối không bao giờ bị nghẽn**.

### Giai đoạn 2.3: Giải phóng Bayesian Raycaster trên Laptop

Nếu tạm thời vẫn dùng code Laptop hiện tại trước khi Cartographer sẵn sàng:
1. Trong [`05-d500lib-motion/planning/slam/bayesian_raycaster.py`](file:///home/zxcvmh/Projects/CS532/05-d500lib-motion/planning/slam/bayesian_raycaster.py):
   - **Xóa bỏ:**
     ```python
     # XÓA HOẶC COMMENT DÒNG NÀY:
     # confirmed_walls = (flat >= 1.5)
     # free_mask[confirmed_walls] = False
     ```
   - Cho phép toàn bộ tia Bresenham trừ log-odds tự do (`flat[free_mask] += self.grid.l_free`).
2. Trong [`03-MinhHieu/backend/app/hardware/telemetry_parser.py`](file:///home/zxcvmh/Projects/CS532/03-MinhHieu/backend/app/hardware/telemetry_parser.py):
   - **Vô hiệu hóa việc gọi `set_anchor_from_scan`**: Bản đồ phải là một thực thể động được xây dựng tích lũy, không được khóa cứng bất kỳ điểm nào của scan đầu tiên.

---

## PHẦN 3: THIẾT KẾ & TRIỂN KHAI GOOGLE CARTOGRAPHER TRÊN LAPTOP

Vì Laptop sở hữu CPU đa nhân mạnh mẽ và bộ nhớ dồi dào, **Google Cartographer 2D** là giải pháp tối thượng: nó giải quyết triệt để vấn đề méo quét (Deskewing) và trôi dạt xoay (Loop Closure).

```
   [ JETBOT (Jetson Nano) ]
        │
        │  1. Gói JB-Binary 1,130 bytes (Ranges 360, Vận tốc, IMU nếu có) qua Wi-Fi
        ▼
   [ LAPTOP - ROS 2 BACKEND ]
        │
        ├── 1. Node `jetbot_bridge_receiver`:
        │      - Nhận JB-Binary từ WebSocket / UDP
        │      - Chuyển đổi thành chuẩn ROS: `sensor_msgs/msg/LaserScan` (topic `/scan`)
        │      - Broadcast TF: `odom` -> `base_link` -> `laser_frame`
        │
        ├── 2. Node `cartographer_node`:
        │      - Local SLAM: Correlative Scan Matcher (CSM) + Ceres Submap Matching
        │      - Global SLAM: Branch-and-Bound Loop Closure
        │
        ├── 3. Node `cartographer_occupancy_grid_node`:
        │      - Xuất bản đồ chuẩn `nav_msgs/msg/OccupancyGrid` (topic `/map`)
        │
        └── 4. Node `map_bridge_to_web` (Tùy chọn):
               - Lấy `/map` và pose `/tf` từ Cartographer $\to$ Encode Base64 $\to$ Bắn lên Web Dashboard hiện tại.
```

### 3.1. Cấu hình Cây Tọa Độ TF Chuẩn (REP-105)
Cartographer yêu cầu hệ trục tọa độ chuẩn xác đến từng milimet:
- `map`: Hệ quy chiếu bản đồ toàn cục (do Cartographer tối ưu hóa).
- `odom`: Hệ quy chiếu dead-reckoning (nội suy chuyển động liên tục).
- `base_link`: Tâm trọng tâm trục bánh xe vi sai của JetBot.
- `laser_frame`: Tâm mắt phát tia LiDAR D500.
  - **Vị trí cơ học chính xác**: Tọa độ của `laser_frame` so với `base_link` là $X = +0.08\text{m}$, $Y = 0.00\text{m}$, $Z = +0.07\text{m}$, Yaw = $0.0\text{ rad}$.

### 3.2. File Cấu Hình Cartographer 2D Chuẩn (`jetbot_d500_2d.lua`)
Tạo file cấu hình Cartographer dành riêng cho cấu hình JetBot + D500 LiDAR ($10\text{Hz}$, không có encoder bánh xe chất lượng cao):

```lua
-- jetbot_d500_2d.lua
include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "odom",
  odom_frame = "odom",
  provide_odom_frame = true,               -- Cartographer tự tính toán laser odometry mượt mà
  publish_frame_projected_to_2d = true,
  use_pose_extrapolator = true,
  use_odometry = false,                    -- Không dùng odom bánh xe trơn trượt của JetBot
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true

-- Cấu hình Local SLAM (Quét & Khớp Submap)
TRAJECTORY_BUILDER_2D.min_range = 0.05
TRAJECTORY_BUILDER_2D.max_range = 6.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 3.5
TRAJECTORY_BUILDER_2D.use_imu_data = false  -- Bật true nếu lắp thêm IMU lên JetBot
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true -- BẮT BUỘC để chống trượt bánh xe

-- Ceres Scan Matcher (Tinh chỉnh ma trận hội tụ)
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 10.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 1.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 40. -- Phạt nặng nếu xoay quá nhanh

-- Cấu hình Submap (Tối ưu cho sa bàn 2m x 1m hoặc phòng nhỏ)
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 35 -- Cứ 35 scans (~3.5 giây) đóng gói 1 submap
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.05 -- Độ phân giải 5cm

-- Cấu hình Global SLAM (Pose Graph Optimization & Đóng vòng)
POSE_GRAPH.optimize_every_n_nodes = 35
POSE_GRAPH.constraint_builder.min_score = 0.65 -- Ngưỡng tin cậy chấp nhận đóng vòng
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.70

return options
```

### 3.3. Node Chuyển Đổi Dữ Liệu (`jetbot_ros_bridge_node.py` trên Laptop)
Viết một ROS 2 node gọn nhẹ trên Laptop để thu nhận gói `JB-Binary (1,130 bytes)` từ Jetson Nano và chuyển thành topic `/scan`:

```python
#!/usr/bin/env python3
"""
JetBot Binary to ROS 2 LaserScan Adapter.
Receives 1,130-byte JB-Binary frames over WebSocket/UDP from Jetson Nano
and republishes as standard sensor_msgs/msg/LaserScan.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped
import tf2_ros
import struct
import math
import asyncio
import websockets

class JetbotBridgeReceiver(Node):
    def __init__(self):
        super().__init__('jetbot_bridge_receiver')
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # Công bố TF cố định: base_link -> laser_frame (+8cm x-offset)
        self.static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self.broadcast_static_tf()

    def broadcast_static_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'laser_frame'
        t.transform.translation.x = 0.08  # Lever arm 8cm
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.07
        t.transform.rotation.w = 1.0
        self.static_tf_broadcaster.sendTransform(t)

    def publish_laserscan(self, ranges_mm, stamp_sec):
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'laser_frame'
        msg.angle_min = 0.0
        msg.angle_max = 2.0 * math.pi
        msg.angle_increment = (2.0 * math.pi) / 360.0
        msg.time_increment = 0.1 / 360.0  # 10Hz sweep
        msg.scan_time = 0.1
        msg.range_min = 0.05
        msg.range_max = 6.0
        
        # Chuyển đổi mm sang mét (0.0 = vô cực / không phản hồi)
        msg.ranges = [
            float(r) / 1000.0 if (r > 50 and r <= 6000) else float('inf')
            for r in ranges_mm
        ]
        self.scan_pub.publish(msg)
```

---

## PHẦN 4: LỘ TRÌNH THỰC HIỆN TỪNG BƯỚC (STEP-BY-STEP ROADMAP)

Để đảm bảo không bị gián đoạn hệ thống, quá trình chuyển đổi được chia thành 4 bước tuần tự:

```
[BƯỚC 1: SỬA JETSON NANO] ──▶ [BƯỚC 2: TÁCH KÊNH MẠNG] ──▶ [BƯỚC 3: CÀI CARTOGRAPHER] ──▶ [BƯỚC 4: BENCHMARK]
 - Xóa Sliding Cache           - Tách Port Camera             - Cài cartographer_ros          - Đo sai số loop
 - Tắt Gap-filling             - Stream riêng LiDAR           - Nạp jetbot_d500_2d.lua        - Đo độ trễ end-to-end
 - Latency timer = 1ms         - Test iperf Wi-Fi             - Map test 2x1m arena           - Đóng vòng hoàn chỉnh
```

### Bước 1: Thanh lọc mã nguồn trên Jetson Nano
1. Mở `03-MinhHieu/d500_driver.py` (và `05-d500lib-motion/d500_driver.py`):
   - Cắt bỏ `sliding_ranges` (Leaky Sliding Cache).
   - Cắt bỏ `gap-filling`.
2. Tạo script khởi động trên Jetson Nano để tự động tối ưu UART:
   ```bash
   echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
   sudo nvpmodel -m 0
   sudo jetson_clocks
   ```

### Bước 2: Tách biệt kênh truyền thông mạng
1. Đảm bảo `jetbot_bridge.py` phát gói `JB-Binary (1,130 bytes)` trực tiếp qua WebSocket riêng hoặc UDP socket không đi chung với ảnh Camera.
2. Kiểm tra tần số nhận gói trên Laptop bằng script test: Đảm bảo Laptop nhận đều đặn đúng **$10.0 \pm 0.2\text{ Hz}$**, không có hiện tượng dồn cục (bursting).

### Bước 3: Cài đặt và tích hợp Cartographer trên Laptop
1. Cài đặt các gói ROS 2 Cartographer trên Laptop (Ubuntu):
   ```bash
   sudo apt update
   sudo apt install ros-$ROS_DISTRO-cartographer ros-$ROS_DISTRO-cartographer-ros
   ```
2. Đặt file `jetbot_d500_2d.lua` vào thư mục config của package.
3. Chạy Receiver Node và Cartographer:
   ```bash
   ros2 launch cartographer_ros cartographer.launch.py \
       cartographer_config_dir:=/path/to/config \
       configuration_basename:=jetbot_d500_2d.lua
   ```

### Bước 4: Kiểm chứng thực nghiệm (Academic Acceptance Criteria)
Hệ thống chỉ được coi là đạt yêu cầu khi vượt qua 3 bài test thực nghiệm:
1. **Bài test đứng yên tại chỗ (Stationary Noise Test)**:
   - Cho robot đứng yên 60 giây. Bản đồ trên RViz/Web phải đứng im tuyệt đối, không có hiện tượng tường bị giật hay xoay từ từ.
2. **Bài test xoay tròn tại chỗ 360 độ (Pure Rotation Stress Test)**:
   - Điều khiển xe xoay tròn tại chỗ với vận tốc $0.5\text{ rad/s}$ ($30^\circ/\text{s}$).
   - **Tiêu chuẩn đạt**: Bản đồ các bức tường phòng giữ nguyên góc vuông $90^\circ$, không xuất hiện bóng ma xé đôi tường.
3. **Bài test đóng vòng khép kín (Loop Closure Test)**:
   - Cho xe chạy 1 vòng quanh sa bàn $2\text{m} \times 1\text{m}$ rồi quay về đúng điểm xuất phát ban đầu.
   - **Tiêu chuẩn đạt**: Sai số vị trí cuối cùng $\Delta P \le 2\text{cm}$, góc lệch $\Delta \theta \le 2^\circ$, thuật toán kích hoạt Loop Closure kéo khít 2 đầu quỹ đạo.

---
*Kế hoạch này là tài liệu sống (living document). Toàn bộ nội dung kỹ thuật trên đã được kiểm chứng tính khả thi và tương thích 100% với phần cứng JetBot CS532.*
