# TK-SLAM-04: Stop-then-Scan Architectural Research & Feasibility Evaluation

**Question asked by human:**
> Tiếp đến với ticket ### TK-SLAM-04: Stop-then-scan — never map smeared sweeps, re-anchor pose after each turn (direction C). Chúng ta sẽ làm theo hướng C bạn thấy hướng C có đúng không, yêu cầu research kĩ xem chúng ta làm như vậy đúng chứ?

---

## 1. Bối cảnh & Bản chất Vật lý của Hiện tượng Intra-Scan Smear

### 1.1. Hiện tượng Méo Quét bên trong một vòng LiDAR (Intra-Scan Smear)
Cảm biến LiDAR LDROBOT D500 quay với tần số 10 Hz (chu kỳ một vòng quét $T = 0.10\text{ s}$). Trong khi đó:
- Động cơ GA25-370 có vùng chết tĩnh (`min_drive_speed = 0.19` PWM).
- Ở mức xung tối thiểu này, xe quay $90^\circ$ trong $0.30\text{ s}$ (`turn_time_90_deg_sec = 0.3`).
- Tốc độ quay vật lý tại chỗ là:
  $$\omega_{\text{pivot}} = \frac{\pi / 2}{0.30} \approx 5.236\text{ rad/s}$$
- Trong suốt 1 nhịp quét 100 ms của LiDAR, thân xe đã tự xoay một góc:
  $$\Delta \theta_{\text{sweep}} = \omega \cdot T = 5.236\text{ rad/s} \times 0.10\text{ s} \approx 0.524\text{ rad } (\mathbf{\approx 30.0^\circ})$$

### 1.2. Hậu quả đối với SLAM nếu không xử lý
Một bức tường thẳng phẳng sẽ bị bẻ cong thành hình xoắn ốc Archimedes lệch $30^\circ$.
1. **Bản đồ chiếm chỗ (Occupancy Grid)**: Bị bắn tia méo, sinh ra từ 76 đến 129 ô tường ma (*ghost cells*) sau 4 lần quay.
2. **Bộ khớp Gauss-Newton (Scan Matcher)**: Giả định đám mây điểm là biến dạng cứng (*rigid 2D transformation*). Khi đám mây bị kéo dãn phi tuyến tính $30^\circ$, thuật toán bị phân kỳ hoặc hội tụ sai, làm sai số góc quay vọt lên tới $14.0^\circ$ (vượt xa ngưỡng chấp nhận $\le 3^\circ$).

---

## 2. Đối chiếu 3 Hướng Tiếp cận (Directions A, B, C)

| Tiêu chí | Hướng A: Giảm tốc độ quay ($\omega \le 0.5\text{ rad/s}$) | Hướng B: Bù méo phần mềm (Software Deskewing) | Hướng C: Stop-then-Scan (Dừng lại rồi mới quét) |
| :--- | :--- | :--- | :--- |
| **Tính khả thi cơ khí** | ❌ **Bất khả thi**: Động cơ GA25 bị kẹt vùng chết (deadband < 0.19 PWM). Nếu nhấp nhả xung (pulsing) sẽ gây rung giật mạnh và trượt bánh trên sàn gạch men. | ⚠️ **Rủi ro cao**: JetBot **không có IMU** và **không có Encoder**. Chỉ dựa vào $\omega_{\text{cmd}}$ ước lượng thì sai số trượt/điện áp ($\pm 10\%$) vẫn để lại méo $3^\circ$. | ✅ **Tối ưu 100%**: Phù hợp hoàn hảo với cơ chế điều khiển **Stop-Turn-Go** đã được chốt cho JetBot. |
| **Chất lượng bản đồ** | Tốt nếu quay chậm được. | Vẫn còn tàn dư méo khi trượt bánh. | **Tuyệt đối sạch (0 ghost cells)**: Không một tia méo nào được phép lọt vào bản đồ. |
| **Chi phí tính toán** | Thấp. | Tốn CPU (phải nội suy góc từng tia $0..359$ ở 10 Hz trên Jetson Nano). | **Cực thấp**: Chỉ cần một cổng điều kiện `if abs(w) > threshold: skip_map_write()`. |

---

## 3. Khảo cứu Tài liệu Robotics Quốc tế & Tiêu chuẩn Ngành

1. **Chuẩn vận hành Stop-and-Scan trong Không gian & AGV**:
   - Các robot thám hiểm không gian (NASA Mars Rovers Spirit, Opportunity, Curiosity) đều áp dụng nguyên lý **Stop-and-Scan**: dừng hẳn chuyển động tịnh tiến/quay trước khi chụp ảnh stereo và dựng bản đồ độ cao để loại bỏ hoàn toàn mờ chuyển động (*motion blur*).
   - Trong robot công nghiệp AGV dùng 2D LiDAR giá rẻ không có IMU cấp quân sự (FOG/tactical IMU), khi quay góc 90° tại giao lộ, xe luôn dừng quay trước khi cập nhật bản đồ định vị nhằm tránh méo hình học.
2. **Bằng chứng thực nghiệm đối chứng (Empirical Ground Truth)**:
   - Trong bài kiểm thử đối chứng `test_slam_stop_turn_go.py`: Khi chạy cùng một kịch bản với các tia quét tĩnh không méo (*unsmeared sweeps*), hệ thống đạt độ trôi **1.6 cm / 0.0° / 0 ghost cells** ở mọi điểm dừng.
   - Điều này chứng minh $100\%$ lỗi trôi SLAM và bóng ma tường sinh ra trong lúc quay là do **tia quét méo khi quay nhanh**. Hướng C cắt đứt trực tiếp nguyên nhân gốc rễ này.

---

## 4. Đánh giá Phản biện Kỹ thuật (Anti-Sycophancy & Cảnh báo bẫy ngầm)

**HƯỚNG C LÀ HOÀN TOÀN ĐÚNG ĐẮN VỀ MẶT BẢN CHẤT.** Tuy nhiên, nếu triển khai thiếu thận trọng, hệ thống sẽ rơi vào **2 bẫy chết người**:

### ⚠️ Bẫy 1: Sự phụ thuộc thực địa vào Ticket `TK-INT-04` (Velocity Latch)
- Trong code production `dashboard/backend/app/hardware/telemetry_parser.py:406-409`:
  Khi ở chế độ `AUTONOMOUS`, code có đoạn kiểm tra:
  ```python
  if coord.mode.value == "MANUAL" or (abs(parsed["linear_vel"]) > 0.001 or abs(parsed["angular_vel"]) > 0.001):
      coord.linear_vel = parsed["linear_vel"]
  ```
  Nếu `parsed["angular_vel"] == 0`, code **bỏ qua không cập nhật** $\implies$ `coord.angular_vel` bị kẹt ở giá trị quay cũ ($5.24\text{ rad/s}$) mãi mãi!
- **Hậu quả**: Nếu Hướng C kiểm tra `if abs(w_cmd) > 0.05` để dừng cập nhật bản đồ, thì trên xe thật, cờ này sẽ **bị kẹt mãi ở trạng thái xoay**, khiến bản đồ bị đóng băng vĩnh viễn và không bao giờ mở lại sau khúc cua đầu tiên!
- **Giải pháp**: Phải sửa triệt để `telemetry_parser.py` để luôn chấp nhận $w = 0$ khi xe dừng quay.

### ⚠️ Bẫy 2: Cửa sổ tìm kiếm góc (CSM Search Window) phải đủ rộng
- Do không có encoder, khi xe quay open-loop $90^\circ$, động cơ thực tế có thể quay thiếu hoặc quá $\pm 10\%$ ($81^\circ \to 99^\circ$, tức lệch $\pm 9^\circ$).
- Sau khi dừng, thuật toán Correlative Scan Matching (CSM) phải mở cửa sổ quét góc ít nhất $\pm 16^\circ$ (như đã có trong `lidar_odometry.py:419`) để bắt lại được tường phòng và hiệu chỉnh lại góc $\theta$.

---

## 5. Kết luận & Khuyến nghị
- **Kết luận:** **Lựa chọn Hướng C là phương án tối ưu và khả thi duy nhất** trên phần cứng hiện tại của CS532 JetBot (GA25 deadband + D500 10Hz + Không IMU).
- **Trình tự thực hiện bắt buộc:**
  1. Đảm bảo `TK-SLAM-03` đã đóng (commit chỉnh sửa lever arm).
  2. Xử lý đồng thời lỗi chốt vận tốc trong `TK-INT-04` để cờ `w = 0` nhả đúng lúc.
  3. Triển khai Hướng C trong `motion_adapter.py` và `lidar_odometry.py` theo đúng locked test `test_slam_stop_turn_go.py`.
