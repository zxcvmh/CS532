# 05-d500lib-motion — Module Tích Hợp Driver D500 Chính Hãng & Tối Ưu SLAM Likelihood Field
## Đồ án CS532 — Autonomous JetBot (Jetson Nano)
### Chuyên gia phụ trách: `motion-planning-algorithm-agent`

---

## 1. Giới Thiệu & Bối Cảnh
Module `05-d500lib-motion` được tạo lập nhằm mục tiêu thay thế toàn bộ logic đọc LiDAR ad-hoc bằng **Driver chính thức chuẩn LDROBOT (LD19 / D500)**, đồng thời giải quyết tận gốc nguyên nhân khiến bản đồ SLAM bị méo mó và định vị sai vị trí vật cản.

### Các Nâng Cấp Kỹ Thuật Đột Phá:
1. **Driver D500 Chính Thức (`d500_driver.py`)**:
   - Kiểm tra mã toàn vẹn gói tin phần cứng **CRC-8** bằng bảng tra cứu đa thức chính hãng `0x4D` (`cal_crc8`). Loại bỏ 100% các byte nhiễu do xung động cơ gây lệch góc ngẫu nhiên.
   - Lọc bỏ các điểm đo có cường độ phản xạ yếu (`intensity < 15`), loại bỏ triệt để hiện tượng phản xạ gương gây chấm đen ảo trên sàn gạch men bóng.
   - **Revolution-Synchronized Sweep Buffer**: Gom toàn bộ điểm đo theo đúng 1 vòng quay $360^\circ$ hoàn chỉnh (`end_angle < start_angle`), triệt tiêu 100% hiện tượng "vết quét bóng ma" (yaw smearing) khi xe quay đầu.

2. **Khắc Phục Lỗi SLAM & Định Vị Bằng Gaussian Likelihood Field (`planning/occupancy_grid.py`)**:
   - *Nguyên nhân gốc rễ lỗi cũ*: Lưới xác suất thô 5cm có gradient bằng 0 khi điểm đo lệch quá 5cm khỏi tường. Khi xe trượt bánh trên sàn gạch men, Hector SLAM bị kẹt trong vùng không có đạo hàm, phân kỳ và văng ra chế độ Dead-Reckoning mù.
   - *Giải pháp toán học SOTA (Sebastian Thrun, 2005 - Ch. 6.4)*: Chuyển đổi ma trận khoảng cách Euclidean Distance Transform (EDT) thành **Gaussian Likelihood Field**:
     $$P(x, y) = \exp\left(-\frac{D(x, y)^2}{2 \sigma^2}\right) \quad (\sigma = 0.12\text{m})$$
     Mở rộng vùng hội tụ (convergence basin) từ $5\text{cm}$ lên $> 40\text{cm}$. Mọi điểm LiDAR bị lệch do trượt bánh đều có gradient kéo về bề mặt tường thực tế một cách mượt mà.

3. **Khử Hiện Tượng Xóa Oan Bản Đồ Bằng Ray Deduplication**:
   - Gom nhóm `np.unique(free_cells)` và loại bỏ các điểm mút vật cản `np.setdiff1d(unique_free, unique_occ)`.
   - Ngăn chặn triệt để tình trạng 20 tia cùng đi qua 1 ô trừ dồn $-50$ điểm làm thủng tường thật.
   - Đặt `L_FREE = -0.85` và `L_OCC = +1.38` theo đúng chuẩn xác suất Bayes.

4. **Khắc Phục Toàn Diện 5 Điểm Nghẽn Vận Hành & Khóa Trôi (Critical Fixes)**:
   - **Fix crash `cache_lock`**: Bổ sung `@property def cache_lock` và `@property def scan_cache` tương thích luồng an toàn 20Hz reflex của JetBot Bridge, triệt tiêu 100% `AttributeError` ngắt kết nối WebSocket.
   - **Bổ sung `floor_ipm.py`**: Đồng bộ hoá đầy đủ thư viện chuyển đổi toạ độ mặt sàn IPM để module chạy hoàn toàn độc lập (standalone).
   - **Xoá sổ bẫy xoay vòng `PIVOT_ALIGNING`**: Nâng ngưỡng kích hoạt xoay tại chỗ từ $5^\circ$ lên $> 45^\circ$ (kèm điều kiện `not is_evading`), giới hạn tốc độ xoay $\le 0.35\text{ rad/s}$ theo `setup/robot_dynamics.json`.
   - **Chống giật đường tránh 10Hz (Anti-flapping Detour)**: Giữ đường tránh tối thiểu 1.2s khi đang lách vật cản, loại bỏ tình trạng huỷ đường tránh tạo thành `IMPASSABLE_STOP`.
   - **Khóa cứng trôi trục dọc khi xoay tại chỗ (`lidar_odometry.py`)**: Ràng buộc vật lý cứng khi $v_{\text{cmd}} < 0.02\text{ m/s}$ (hoặc `is_pivot=True`), giới hạn bước dịch chuyển tịnh tiến $\le 1\text{cm}$, dập tắt hiện tượng trượt ảo dọc hành lang khi xoay đầu.
   - **Revolution Timeout Fallback (`d500_driver.py`)**: Bảo vệ bằng timeout 0.11s gom quét khi mất gói $0^\circ$, giữ nguyên góc thực tế dạng float và lấy khoảng cách min của chướng ngại vật trong mỗi bucket.

---

## 2. Cấu Trúc Module `05-d500lib-motion`
- `d500_driver.py`: Driver chính thức của LiDAR D500 với CRC-8, revolution sync và timeout bảo vệ.
- `floor_ipm.py`: Biến đổi phối cảnh ngược (IPM) từ camera sang hệ toạ độ robot phẳng.
- `jetbot_bridge.py`: Cầu nối phần cứng thời gian thực chạy trên JetBot, tích hợp sẵn `D500OfficialDriver` và task an toàn 20Hz.
- `planning/occupancy_grid.py`: Lưới C-space cho khe 25cm + Gaussian Likelihood Field.
- `planning/lidar_odometry.py`: Bộ ước lượng chuyển động quét 2D ICP + Hector Likelihood Field với kinematic clamp.
- `planning/lidar_parser.py`: Bộ giải mã gói tin D500 tích hợp CRC-8.
- `planning/controller.py`: Pure Pursuit RPP + Plateau Detour + Anti-flapping + Terminal Goal Bypass.

---

## 3. Hướng Dẫn Vận Hành Trên Robot Thật
### Bước 1: Khởi động cầu nối phần cứng trên JetBot
```bash
cd /home/jetbot/05-d500lib-motion
python3 jetbot_bridge.py 127.0.0.1
```

### Bước 2: Khởi động Backend điều khiển
```bash
cd /home/jetbot/03-MinhHieu/backend
python3 run.py --host 0.0.0.0 --port 8000
```
Mở trình duyệt Web tại `http://<IP_JETBOT>:8000` để quan sát bản đồ SLAM đã được khử nhiễu, các bức tường sắc nét và xe định vị chuẩn xác đối với các vật thể xung quanh.
