# TK-SLAM-04: Stop-Then-Scan Design Specification (Direction C)

**Status:** PROPOSED (Approach 1 selected during Brainstorming 2026-11-15)  
**Ticket:** `TK-SLAM-04: Stop-then-scan — never map smeared sweeps, re-anchor pose after each turn (direction C)`  
**Role:** `slam` ([`.agents/workflows/slam.md`](file:///home/zxcvmh/Projects/CS532/.agents/workflows/slam.md))  

---

## 1. Intent & Context (Bối cảnh & Mục đích)

Trên robot thực tế CS532 JetBot (động cơ vi sai GA25-370), `min_drive_speed = 0.19` PWM là ngưỡng tối thiểu để thắng ma sát tĩnh, dẫn tới vận tốc quay tại chỗ thực tế là $\omega \approx 5.24\text{ rad/s}$ ($90^\circ$ trong $0.3\text{ s}$). Trong khi đó, cảm biến LiDAR D500 quay ở tần số $10\text{ Hz}$ ($0.1\text{ s/vòng}$).
- **Hệ quả vật lý:** Mỗi vòng quét 360 tia khi robot đang quay bị biến dạng quét (*scan smear*) lên tới $\approx 30^\circ$.
- **Lỗi hệ thống hiện tại:**
  - [`motion_adapter.py:126`](file:///home/zxcvmh/Projects/CS532/dashboard/backend/app/navigation/motion_adapter.py#L126) ghi đè quét méo vào [`grid_map.update_scan`](file:///home/zxcvmh/Projects/CS532/motion/planning/occupancy_grid.py#L324), sinh ra 76–129 ghost cells làm hỏng bản đồ tĩnh.
  - [`lidar_odometry.py:608`](file:///home/zxcvmh/Projects/CS532/motion/planning/lidar_odometry.py#L608) chạy bộ khớp Gauss-Newton trên đám mây điểm méo, khiến sai số góc tích lũy lên tới $14.0^\circ$.
- **Định hướng giải quyết đã chốt (Direction C):** Không vẽ bản đồ khi đang quay; chỉ ước lượng hướng bằng kinematics. Khi dừng lại, dùng vòng quét tĩnh không méo để re-anchor lại pose với bản đồ qua bộ khớp thô 1D CSM có sẵn, sau đó mới tiếp tục ghi bản đồ.

---

## 2. Goals, Non-Goals & Anti-Goals

### Goals
1. Triệt tiêu toàn bộ việc ghi bản đồ khi đang xoay: `grid_map.update_scan` = 0 trong suốt quá trình quay.
2. Vượt qua bộ kiểm thử chấp nhận [`tests/acceptance/test_slam_stop_turn_go.py`](file:///home/zxcvmh/Projects/CS532/tests/acceptance/test_slam_stop_turn_go.py) trên cả 2 chế độ (`anchor ON` và `production`):
   - Sai số góc hướng tại mọi điểm dừng: $|\Delta \theta| \le 3.0^\circ$.
   - Trôi vị trí tịnh tiến tại mọi điểm dừng: $|\Delta p| \le 5.0\text{ cm}$.
   - Tổng số ghost cells cuối bài kiểm tra: $\le 10$ cells.
   - Số lần ghi map khi dừng: $\ge 1$ lần tại mỗi điểm dừng.
3. Không làm suy thoái bộ test chấp nhận trước đó [`tests/acceptance/test_slam_inplace_turn.py`](file:///home/zxcvmh/Projects/CS532/tests/acceptance/test_slam_inplace_turn.py) (giữ vững 4/4 PASS).

### Non-Goals
- Không cài đặt thuật toán bù méo thời gian thực (*Software deskew* - Direction B được hoãn lại).
- Không can thiệp điều khiển xung hay giảm tốc động cơ dưới `min_drive_speed` (Direction A bất khả thi do phần cứng).

### Anti-Goals
- Tuyệt đối không đóng băng bản đồ vĩnh viễn (*freeze map forever* — vi phạm yêu cầu `map_writes >= 1` khi dừng).
- Không can thiệp vào các file ngoài phạm vi Allowed files.

---

## 3. Ranh giới Phân vai & Allowed Files

Tuân thủ nghiêm ngặt quy chế phân vai trong [`AGENTS.md`](file:///home/zxcvmh/Projects/CS532/AGENTS.md):
- **Allowed files:**
  1. [`motion/planning/lidar_odometry.py`](file:///home/zxcvmh/Projects/CS532/motion/planning/lidar_odometry.py): Xử lý bỏ qua khớp scan méo khi quay và re-anchor khi dừng.
  2. [`dashboard/backend/app/navigation/motion_adapter.py`](file:///home/zxcvmh/Projects/CS532/dashboard/backend/app/navigation/motion_adapter.py): **Chỉ sửa $\le 10$ dòng** để chặn `update_scan` và anchor locking khi xoay tại chỗ (ngoại lệ đã được Claude Supervisor phê duyệt).
- **Forbidden files:** Toàn bộ các file khác và các đường dẫn trong `agents-doc/LOCKED_PATHS.txt` (đặc biệt là `tests/acceptance/` và `setup/robot_dynamics.json`).

---

## 4. Các Phương án đã Cân nhắc & Lý do Chọn Phương án 1

- **Phương án 1 (Được chọn): Kinematic Bypass khi Quay + Re-anchor tự nhiên bằng CSM:**
  - Khi quay (`abs(w_cmd) > 0.05 and abs(v_cmd) < 0.02`): Bỏ qua `match_scan_to_map` và `match_scans`, cập nhật pose bằng `prior_dtheta`, không lưu scan méo vào `prev_points_2d`.
  - Khi dừng: Bộ tìm kiếm 1D CSM ở Stage 1 của `match_scan_to_map` đã có sẵn dải quét $\pm 16.0^\circ$ (bước $2^\circ$), hoàn toàn đủ để bắt trọn sai số tích lũy động cơ mở $\pm 10\%$ ($\pm 9^\circ$ trên góc quay $90^\circ$) ngay ở vòng quét đầu tiên.
  - *Lý do chọn:* Đơn giản, diff nhỏ nhất (Rule 7), không làm phình code `motion_adapter.py` (chỉ 5 dòng thay đổi), giải quyết triệt để vấn đề mà không sinh side-effect.
- **Phương án 2 (Bị loại): State Machine phân tầng (`TRACKING` $\rightarrow$ `SPINNING` $\rightarrow$ `REANCHORING`):**
  - Đòi hỏi cờ báo hiệu trạng thái giữa `lidar_odometry` và `motion_adapter`, gây vượt ngưỡng 10 dòng code cho phép tại `motion_adapter.py`.

---

## 5. Thiết kế Chi tiết Triển khai (Detailed Design)

### 5.1. Chặn ghi bản đồ quét méo trong `motion_adapter.py`
Tại [`dashboard/backend/app/navigation/motion_adapter.py:110-128`](file:///home/zxcvmh/Projects/CS532/dashboard/backend/app/navigation/motion_adapter.py#L110-L128):
Xác định cờ quay tại chỗ:
```python
is_spinning = (abs(w_cmd) > 0.05 and abs(v_cmd) < 0.02)
```
- Nếu `not is_spinning`:
  - Thực hiện khởi tạo anchor nếu chưa lock (bước 2).
  - Thực hiện gọi `self.grid_map.update_scan(...)` (bước 3).
- Nếu `is_spinning`:
  - Bỏ qua hoàn toàn bước 2 và bước 3 (đảm bảo 0 lượt ghi map khi quay).

### 5.2. Kinematic Dead-Reckoning khi Quay trong `lidar_odometry.py`
Tại [`motion/planning/lidar_odometry.py:538-645`](file:///home/zxcvmh/Projects/CS532/motion/planning/lidar_odometry.py#L538-L645):
- Cờ `is_pivot = (abs(v_cmd) < 0.02 and abs(w_cmd) > 0.05)`.
- Khi `is_pivot == True`:
  - Bỏ qua khối lệnh gọi `self.match_scan_to_map(...)` và fallback ICP `self.match_scans(...)`.
  - Cập nhật pose thuần kinematic prior:
    ```python
    dx_res = 0.0
    dy_res = 0.0
    dth_res = prior_dtheta
    self.theta = (self.theta + dth_res + math.pi) % (2.0 * math.pi) - math.pi
    self.last_status = "PIVOT_DEAD_RECKONING"
    is_reliable = True
    match_info = {"method": "KINEMATIC_PIVOT"}
    ```
  - **Không lưu scan méo:** Không cập nhật `self.prev_points_2d = curr_cloud` để tránh làm vấy bẩn đám mây điểm tham chiếu của ICP.
- Khi dừng quay (`is_pivot == False`):
  - `match_scan_to_map` được kích hoạt bình thường.
  - Stage 1 CSM đối sánh dải $\pm 16^\circ$ quanh góc `cand_th` (vốn đã được dead-reckoning bám sát trong khoảng $\pm 9^\circ$), tìm ra góc cực đại xác suất trên bản đồ tĩnh sạch và đưa Gauss-Newton vào hội tụ hoàn hảo.

---

## 6. Kế hoạch Kiểm thử & Tiêu chuẩn Nghiệm thu (Verification Gates)

1. **Acceptance Test:**
   ```bash
   .venv/bin/python tests/acceptance/test_slam_stop_turn_go.py -v
   ```
   - Cả 2 kịch bản `anchor=ON` và `anchor=OFF(prod)` đều PASS.
   - Tại mọi điểm dừng: `drift <= 5.0 cm`, `heading_err <= 3.0 deg`.
   - `ghost_cells <= 10`.
   - `map_writes`: spin = `[0, 0, 0, 0]`, stop = `[>=1, >=1, >=1, >=1]`.
2. **Regression Tests:**
   - `.venv/bin/python tests/acceptance/test_slam_inplace_turn.py -v` (phải tiếp tục 4/4 PASS).
   - `.venv/bin/python tests/test_hector_anchor_slam.py` (4/4 PASS).
   - `cd dashboard/backend && ../../.venv/bin/python -c "import app.main"` (khởi động không lỗi).
