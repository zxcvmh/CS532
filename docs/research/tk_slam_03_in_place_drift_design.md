# TK-SLAM-03: In-Place Pivot Drift Elimination Design Specification

**Status:** APPROVED (Consensus reached via Superpowers Brainstorming 2026-11-15)  
**Ticket:** `TK-SLAM-03: Map follows the robot during in-place turns (self-reinforcing drift)`  
**Role:** `slam` ([`.agents/workflows/slam.md`](file:///home/zxcvmh/Projects/CS532/.agents/workflows/slam.md)) + Interface Note bàn giao cho `integration`  

---

## 1. Ranh giới Phân vai & Thẩm quyền Thực thi (Role Governance & Scope)

Theo quy chế vận hành chuẩn ([`AGENTS.md`](file:///home/zxcvmh/Projects/CS532/AGENTS.md) & [`.agents/workflows/slam.md`](file:///home/zxcvmh/Projects/CS532/.agents/workflows/slam.md)):
- **File SLAM sở hữu & được phép can thiệp trực tiếp:**
  - `motion/planning/slam/**` ([`bayesian_raycaster.py`](file:///home/zxcvmh/Projects/CS532/motion/planning/slam/bayesian_raycaster.py), [`anchor_layer.py`](file:///home/zxcvmh/Projects/CS532/motion/planning/slam/anchor_layer.py)).
  - `motion/planning/occupancy_grid.py` (tầng bao đóng grid map của SLAM).
  - `motion/planning/lidar_odometry.py` (nằm trong ticket TK-SLAM-03 do Supervisor phân công cho role `slam`).
  - `dashboard/backend/app/navigation/cartographer_adapter.py`.
  - [`tests/test_hector_anchor_slam.py`](file:///home/zxcvmh/Projects/CS532/tests/test_hector_anchor_slam.py).
- **File SLAM KHÔNG ĐƯỢC PHÉP sửa:**
  - [`setup/robot_dynamics.json`](file:///home/zxcvmh/Projects/CS532/setup/robot_dynamics.json) (LOCKED theo [`agents-doc/LOCKED_PATHS.txt`](file:///home/zxcvmh/Projects/CS532/agents-doc/LOCKED_PATHS.txt)).
  - [`dashboard/backend/app/navigation/motion_adapter.py`](file:///home/zxcvmh/Projects/CS532/dashboard/backend/app/navigation/motion_adapter.py) (thuộc quyền sở hữu độc quyền của **Integration** — SLAM chỉ xuất Interface Note).
  - [`dashboard/backend/app/navigation/coordinator.py`](file:///home/zxcvmh/Projects/CS532/dashboard/backend/app/navigation/coordinator.py) (thuộc quyền Integration).
  - `dashboard/jetbot_bridge.py` (thuộc quyền Integration).

---

## 2. Bắt buộc: 5-Line Call Trace (Tuân thủ Điều 2 - SLAM Workflow)

Trước khi can thiệp mã nguồn, quy trình bắt buộc xác lập vết gọi hàm 5 dòng chính xác:
1. `dashboard/backend/app/navigation/coordinator.py:465` (`NavigationCoordinator.step_navigation`): Tiếp nhận mảng quét thô 360 tia `scan_ranges` và telemetry trạng thái xe từ bridge.
2. `dashboard/backend/app/navigation/motion_adapter.py:102` (`MotionAdapter.update_odometry`): Gọi `self.lidar_odometry.update(ranges, dt=dt, v_cmd=v_cmd, w_cmd=w_cmd)`.
3. `motion/planning/lidar_odometry.py:512` (`LidarOdometry.update`): Chạy bộ khớp Tier 1 Hector Gauss-Newton (`match_scan_to_map`), ước lượng pose bù trôi `(ox, oy, oth)`.
4. `dashboard/backend/app/navigation/motion_adapter.py:126`: Chuyển tiếp pose và ranges sang `self.grid_map.update_scan(ox, oy, oth, ...)` $\rightarrow$ `motion/planning/occupancy_grid.py:328` ủy quyền cho `motion/planning/slam/bayesian_raycaster.py:28` (`BayesianRaycaster.update_scan`) để cập nhật log-odds và trả về danh sách cell va chạm `latest_hit_cells`.
5. `dashboard/backend/app/navigation/coordinator.py:512`: Tiêu thụ pose `(ox, oy, oth)` và lưới costmap đã cập nhật để lập quỹ đạo A* tránh vật cản và bắn telemetry ra Dashboard.

---

## 3. Căn nguyên Hiện tượng "Map Quay Theo Xe" (Root Cause & Mathematical Derivation)

### 3.1. Ràng buộc Động học Vi sai khi Quay tại chỗ (Kinematic Invariant)
Khi robot thực hiện quay tại chỗ (`in-place pivot turn`: $v_{\text{cmd}} \approx 0, \omega_{\text{cmd}} \neq 0$):
- Về mặt động học vi sai, tâm trục bánh xe (`base_link`) có vận tốc tịnh tiến tuyến tính lý thuyết triệt tiêu: $|\Delta p_{\text{axle}}| = 0$.
- Cảm biến LiDAR D500 có cánh tay đòn thực tế $l_a = 0.01\text{ m}$ (đo thực địa tại TK-SLAM-02).
- Do đó, độ dịch chuyển vị trí tối đa của mắt quét trong một chu kỳ quét $\Delta t = 0.1\text{ s}$ bị chặn trên bởi:
  $$\Delta p_{\text{lidar}} \le l_a \cdot |\Delta \theta| + \epsilon_{\text{slip}} \approx 0.01 \cdot (5.24 \cdot 0.1) + 0.002 \approx 0.007\text{ m } (7\text{ mm})$$

### 3.2. Ba khiếm khuyết phần mềm kích hoạt Vòng lặp Phản hồi Dương (Runaway Positive Feedback)
1. **Hector Stage nới lỏng tịnh tiến lên tới 6 cm:**
   Tại `motion/planning/lidar_odometry.py:614-617`:
   `max_allowed_p = max(0.06, lever_arm * max(0.05, delta_th) * 1.5)` với `lever_arm = 0.08`!
   Ngay cả khi robot quay tại chỗ, bộ lọc vẫn cho phép nghiệm Gauss-Newton nhảy tự do tới $6\text{ cm}$ mỗi scan.
2. **Bayesian Raycaster ghi đè tức thì 1-frame vào bản đồ tĩnh:**
   Tại `motion/planning/slam/bayesian_raycaster.py:131-132`:
   `flat[unique_occ] = np.maximum(flat[unique_occ] + self.grid.l_occ, self.grid.l_occ)`
   Các tia quét bị méo khi quay ở tốc độ cao ($\omega \approx 5.24\text{ rad/s}$) cùng với sai số pose 6 cm được ép cứng ngay lập tức thành tường tĩnh màu đen chỉ sau một frame duy nhất.
3. **Hiện tượng Tự củng cố Trôi (Self-Reinforcing Drift):**
   Ở frame kế tiếp, Hector lại lấy bản đồ tường tĩnh (vừa bị ghi lệch) làm mẫu đối sánh. Sai số này tích lũy liên tục theo chiều quay, khiến toàn bộ bản đồ bị robot "kéo lê" xoay theo mình.

---

## 4. Giải pháp Kiến trúc Phân tầng (Tiered Architecture Implementation Plan)

### 4.1. Tầng 1: Đọc chuẩn Vật lý Xe Thực địa (Triệt tiêu Hardcode tại `lidar_odometry.py`)
Tuân thủ Quy tắc cứng số 3 (`AGENTS.md`), toàn bộ thông số phải đọc trực tiếp từ [`setup/robot_dynamics.json`](file:///home/zxcvmh/Projects/CS532/setup/robot_dynamics.json):
- Tại `motion/planning/lidar_odometry.py:117-125`:
  Xóa bỏ hardcode cũ `0.50 s / 3.1416 rad/s / 5.0`. Thay bằng:
  ```python
  # Read measured dynamics from setup/robot_dynamics.json
  t90 = float(cfg.get("turn_time_90_deg_sec", 0.30))  # 0.3s (robot_dynamics.json)
  self.turn_time_90_deg_sec = t90
  self.calibrated_pivot_yaw_rate = (math.pi / 2.0) / t90  # ~5.236 rad/s
  self.max_physical_yaw_rate = self.calibrated_pivot_yaw_rate * 1.25  # ~6.545 rad/s
  self.lever_arm_x = float(cfg.get("lidar_lever_arm_m", 0.01))  # 0.01m from TK-SLAM-02
  ```

### 4.2. Tầng 2: Khóa Tịnh tiến Động học khi Quay tại chỗ (`lidar_odometry.py`)
Tại `motion/planning/lidar_odometry.py:614-617`:
- Khi phát hiện robot quay tại chỗ (`is_pivot or abs(v_cmd) < 0.02`):
  ```python
  if is_pivot or abs(v_cmd) < 0.02:
      la = getattr(self, "lever_arm_x", 0.01)
      # Bounded strictly by physical lever-arm rotation: max 7mm
      max_allowed_p = max(0.003, la * abs(delta_th) + 0.002)
  else:
      max_allowed_p = max(0.08, self.max_physical_speed * dt * 2.2)
  ```
- Đồng thời áp dụng translation clamp cho nghiệm Tier 1 Hector trước khi gán `self.x, self.y`:
  ```python
  if is_pivot or abs(v_cmd) < 0.02:
      step_p = math.hypot(gn_x - self.x, gn_y - self.y)
      max_pivot_trans = max(0.002, abs(v_cmd) * dt + 0.001)
      if step_p > max_pivot_trans:
          gn_x = self.x + ((gn_x - self.x) / step_p) * max_pivot_trans
          gn_y = self.y + ((gn_y - self.y) / step_p) * max_pivot_trans
  ```

### 4.3. Tầng 3: Đóng băng Bản đồ Tĩnh khi Quay (`bayesian_raycaster.py` & `occupancy_grid.py`)
- **Trong `motion/planning/slam/bayesian_raycaster.py`:**
  - Bổ sung tham số `is_rotating: bool = False` vào `update_scan()`.
  - Khi `is_rotating == True`:
    - **Tạm dừng ghi đè log-odds tĩnh:** Bỏ qua khối lệnh cập nhật `flat[unique_occ]` và `flat[free_mask]` đối với bản đồ nền tĩnh.
    - **Vẫn theo dõi chướng ngại vật động:** Tiếp tục cập nhật `self.dynamic_obstacle_cells` với TTL ngắn để đảm bảo an toàn nếu có người bước qua khi xe đang quay.
- **Trong `motion/planning/occupancy_grid.py:328`:**
  - Nhận `is_rotating: bool = False` tại `OccupancyGridMap.update_scan` và truyền tiếp vào `self.raycaster.update_scan(..., is_rotating=is_rotating)`.

### 4.4. Tầng 4: Interface Note bàn giao cho Role Integration (`motion_adapter.py`)
Vì [`motion_adapter.py`](file:///home/zxcvmh/Projects/CS532/dashboard/backend/app/navigation/motion_adapter.py) thuộc quyền của **Integration Engineer**, SLAM Engineer lập Interface Note yêu cầu Integration cập nhật tại dòng 126:
```python
# Interface Note for Integration Role:
# Determine rotation status to freeze static map raycasting during in-place pivot:
is_rotating = (abs(oth_diff) > 0.02 or abs(w_cmd) > 0.05) and abs(v_cmd) < 0.02
self.grid_map.update_scan(
    ox, oy, oth, ranges=ranges, max_valid_range=min(6.0, Config.MAX_RANGE),
    defer_costmap=False, is_rotating=is_rotating
)
```

---

## 5. Kế hoạch Kiểm thử & Tiêu chuẩn Nghiệm thu (Verification & Acceptance Gates)

### 5.1. Khắc phục Lỗ hổng của Suite Kiểm thử Cũ (TK-SLAM-05)
Hiện tại `tests/test_hector_anchor_slam.py:25-44` dùng phòng tròn đối xứng bán kính 1.5m (`[1.5] * 360`), vốn là hình dạng bất biến quay (*rotation-invariant*), do đó không thể phát hiện lỗi trôi góc xoay và kéo lê bản đồ.

### 5.2. Thiết kế Test Suite Mới trong `tests/test_hector_anchor_slam.py`
Tạo ca kiểm thử `test_05_in_place_360_turn_rectangular_arena_no_drift`:
- Môi trường: Mô phỏng đấu trường chữ nhật thực tế 2×1m (`tested_arena="2m_x_1m"` trong `robot_dynamics.json`).
- Kịch bản vận hành:
  1. Khởi tạo bản đồ tham chiếu với robot đứng yên ($v=0, \omega=0$) trong 5 chu kỳ quét.
  2. Robot thực hiện xoay 360° tại chỗ với $v_{\text{cmd}} = 0.0\text{ m/s}$, $\omega_{\text{cmd}} = 5.24\text{ rad/s}$ (hoặc vận tốc thử nghiệm $0.5\text{ rad/s}$) qua 20 bước quét.
- **Tiêu chuẩn Nghiệm thu Định lượng (Quantitative Thresholds):**
  - **Metric 1 (Trôi tịnh tiến):** Độ lệch vị trí tâm trục bánh xe sau 360°:
    $$|\Delta p| = \sqrt{x^2 + y^2} \le 0.01\text{ m } (1\text{ cm}) \quad [\text{Ngưỡng chuẩn } \le 5\text{ cm}]$$
  - **Metric 2 (Sai số góc hướng):** Góc xoay tích lũy bám sát quỹ đạo điều khiển:
    $$|\theta_{\text{meas}} - \theta_{\text{ref}}| \le 3.0^\circ$$
  - **Metric 3 (Biến dạng bản đồ):** Độ xê dịch của các bức tường tĩnh trong world frame:
    $$\text{drift}_{\text{wall}} \le 0.02\text{ m } (2\text{ cm} < \text{resolution } 5\text{ cm})$$
  - **Metric 4 (Hiệu năng):** Thời gian xử lý mỗi scan:
    $$t_{\text{match}} \le 5.0\text{ ms/scan}$$
