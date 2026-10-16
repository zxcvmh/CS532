---
# =============================================================================
# CS532 JETBOT - MOTION PLANNING & CONTROL ALGORITHM LIVING STATE
# =============================================================================
module: "04-hieu-MotionControl"
scope: "goal_reaching_and_narrow_arena_navigation"
last_updated: "2026-10-11"
active_phase: "H1_AND_SCALE_OPTIMIZATION_VERIFIED_PC"
sync_to_jetbot: false  # LOCKED - STRICTLY FORBIDDEN OVER SSH

speed_profile:
  max_linear_speed_mps: 0.11
  cruise_speed_mps: 0.10
  approach_creep_speed_mps: 0.05
  max_angular_speed_rads: 0.35
  turn_speed_mps: 0.06
  soft_start_ramp_sec: 0.50

arena_constraints:
  arena_dimensions: "2m x 1m"
  lateral_corridor_half_width_m: 0.22
  lateral_detour_offset_m: 0.20
  collision_horizon_m: 0.75
  goal_tolerance_m: 0.12

hypotheses_matrix:
  - id: "H6"
    name: "Over-aggressive Angular Turn & 180° Spinout Trap on Glossy Ceramic Tile"
    probability: 0.60
    status: "RESOLVED_AND_VERIFIED_PC"
    symptom: "Robot turns too abruptly; violently spins 180° backwards upon meeting an obstacle, ruining localization"
    solution:
      - "Clamped max_angular_speed from 0.50 to 0.35 rad/s (~20 deg/s) based on robot_dynamics.json"
      - "Eliminated blind spinout pivot in cbf_safety_filter (min_h < 0.01 triggers safe halt v=0, w=0)"
      - "Narrowed greedy detour candidates to forward cones [-18, 18, -32, 32, -45, 45 deg], removing 90 deg wall crashes"
      - "Enforced cosine-squared velocity attenuation v = max(0.04, v_base * cos^2(alpha / 2))"
      - "Increased lookahead distance from 0.24m to 0.28m for gentle curvature transitions"

  - id: "H5"
    name: "Motor Deadband Stiction Under Low Speed & False Chassis Reflex Braking"
    probability: 0.50
    status: "RESOLVED_AND_VERIFIED"
    symptom: "Path found & trajectory displayed on dashboard, but robot stays still and prints no further updates"
    solution:
      - "Bumped MIN_PWM from 34 to 50 in jetbot_bridge.py to overcome GA25 static gearbox friction deadband"
      - "Filtered false-positive chassis reflections (< 0.12m) in check_safety_reflex and cbf_safety_filter"
      - "Integrated 2Hz high-density terminal & persistent file logging (nav_debug.log) into mock_robot.py"

  - id: "H1"
    name: "Pure Pursuit Goal Deceleration, Plane Crossing & Narrow Corridor Tuning"
    probability: 0.25
    status: "IMPLEMENTED_AND_VERIFIED_PC"
    solution:
      - Goal Approach Deceleration Ramp (v: 0.11 -> 0.05 m/s)
      - Dual-criteria Goal Plane Crossing (Dot-product w . u >= 0)
      - Behind-the-Goal trap protection (|alpha| > 90 deg -> E-stop at goal)
      - Narrow Corridor filter (|cy| > 0.22m side walls ignored as blockers)
      - Detour offset reduced from 0.52m to 0.20m for 1m arena width

  - id: "H2"
    name: "Physical Pose Drift (Wheel Slip on Ceramic Tile)"
    probability: 0.15
    status: "STANDBY_LIDAR_ODOM_READY"
    module: "planning/lidar_odometry.py"
    trigger: "Dashboard shows NAVIGATING while robot already reached physical goal"

  - id: "H3"
    name: "Mechanical Motor Trim & Deadband Asymmetry"
    probability: 0.08
    status: "TUNING_APPLIED"
    solution: "Updated robot_dynamics.json with safe min_drive_speed=0.08 and cruise_speed=0.11"

  - id: "H4"
    name: "Network Latency & Bridge Watchdog Lag"
    probability: 0.02
    status: "MONITORED"
    solution: "Bridge watchdog 1.5s -> recommend 0.3s if overshoot glides persist"
---

# BẢNG THEO DÕI THỰC NGHIỆM THUẬT TOÁN (ALGORITHM LIVING STATE)

Tài liệu này quản lý toàn bộ các giả thuyết kiểm chứng, trạng thái thuật toán và phương án khắc phục cho bộ điều hướng JetBot CS532 trong sa bàn hẹp $2\text{m} \times 1\text{m}$.

---

## 1. PHÂN TÍCH HIỆN TƯỢNG ĐI THẲNG BỊ VẼ ĐƯỜNG PHỨC TẠP
1. **Bản chất**:
   - Khi xe chạy dọc sa bàn $1\text{m}$, hai bức tường bên hông chỉ cách xe $0.40 - 0.45\text{m}$.
   - Bộ quét va chạm cũ (`horizon_m = 1.8m`) nhận vơ tường sa bàn thành vật cản chắn đường đi thẳng $\to$ kích hoạt A* Re-planner và vẽ đường né tránh dạt $0.52\text{m}$ (phi thẳng vào tường đối diện).
2. **Khắc phục đã thực hiện**:
   - Thu hẹp hành lang phát hiện cản: Bỏ qua mọi cụm có $|c_y| > 0.22\text{m}$ (tường sa bàn).
   - Rút ngắn tầm nhìn va chạm từ $1.8\text{m}$ xuống $0.75\text{m}$.
   - Giảm độ dạt né tránh từ $0.52\text{m}$ xuống $0.20\text{m}$ vừa khít lòng sa bàn $1\text{m}$.

---

## 2. KHẮC PHỤC HIỆN TƯỢNG XE LAO ĐI QUÁ NHANH KHI VỪA SET GOAL
1. **Bản chất**:
   - Ngay khi vừa click Set Goal, góc lệch hướng mũi xe $|\alpha|$ lớn ($> 25^\circ$), nhưng hệ thống cũ lập tức phát lệnh vừa bẻ lái vừa phóng tới $0.10\text{ m/s}$ $\to$ bánh xe bị trượt xoay vẹo mũi trên sàn gạch men bóng.
2. **Khắc phục đã thực hiện**:
   - Khi góc lệch lớn ($|\alpha| > 0.45\text{ rad}$), vận tốc được kìm hãm ở mức bò $0.06\text{ m/s}$ để xoay chỉnh hướng an toàn.
   - Hạ trần tốc độ toàn hệ thống từ $0.18\text{ m/s}$ xuống **$0.11\text{ m/s}$** và vận tốc tiếp cận đích xuống **$0.05\text{ m/s}$**.

---

## 3. KHẮC PHỤC HIỆN TƯỢNG GÓC QUAY QUÁ GẮT & QUAY GIẬT 180° KHI GẶP VẬT CẢN (H6)
1. **Bản chất & Phân tích nguyên nhân gốc rễ**:
   - **Hiện tượng quay giật 180°**: Trong `cbf_safety_filter.py`, khi vật cản đến quá gần ($min\_h < 0.01$), logic cũ cưỡng chế $w = \pm 0.38\text{ rad/s}$ để xoay thoát hiểm khẩn cấp. Trên sàn gạch men trơn trượt (`glossy_ceramic_tile`), mô-men xoắn đột ngột khiến bánh xe bị trượt văng đuôi $180^\circ$, phá hủy toàn bộ hệ tọa độ dead-reckoning.
   - **Góc bẻ lái quá gắt đâm tường**: `generate_greedy_detour` cũ đưa ra các góc ứng viên $\pm 75^\circ, \pm 90^\circ$. Trong sa bàn rộng vỏn vẹn $1\text{m}$, quay $90^\circ$ đâm thẳng mũi vào vách ngăn.
   - **Không đồng bộ động học phần cứng**: Thông số thực nghiệm trong `robot_dynamics.json` cho thấy tốc độ quay vật lý tại PWM=0.16 lên tới $1.43\text{ rad/s}$ ($82^\circ/\text{s}$). Lệnh $\omega=0.50\text{ rad/s}$ là quá lớn đối với xe không có encoder hồi tiếp.
2. **Khắc phục đã hoàn thành**:
   - Cắt giảm trần vận tốc góc: $\omega_{max}$ từ $0.50$ xuống **$0.35\text{ rad/s}$** ($\approx 20^\circ/\text{s}$).
   - Xóa bỏ hoàn toàn lệnh pivot khẩn cấp trong CBF: Khi $min\_h < 0.01$, xe thực hiện **phanh dừng an toàn** ($v=0, w=0$), không quay giật mù quáng.
   - Giới hạn góc né tham lam: Chỉ chấp nhận góc hướng về phía trước sa bàn $[-18^\circ, 18^\circ, -32^\circ, 32^\circ, -45^\circ, 45^\circ]$.
   - Tăng khoảng cách nhìn trước Pure Pursuit từ $0.24\text{m}$ lên $0.28\text{m}$ để làm mượt độ cong chuyển hướng $\kappa$.
   - Điều tốc suy giảm cosin mượt mà: $v = \max(0.04, v_{base} \cdot \cos^2(\alpha / 2))$.

---

## 4. NGHIÊN CỨU & KHẢO SÁT CÁC THUẬT TOÁN NÉ CẢN THỜI GIAN THỰC TRÊN GITHUB
Qua khảo sát các open-source repositories hàng đầu trên GitHub trong lĩnh vực robot vi sai:
1. **Dynamic Window Approach (DWA) / Trajectory Rollout**:
   - *Mã nguồn tham khảo*: `AtsushiSakai/PythonRobotics` (24k+ stars), ROS 2 `nav2_dwb_controller`.
   - *Nguyên lý*: Lấy mẫu $(v, \omega)$ trong cửa sổ động học dựa trên giới hạn gia tốc thực tế $(a_{v,max}, a_{\omega,max})$, dự phóng các cung tròn quỹ đạo và tối ưu hàm chi phí (khoảng cách tới goal, khoảng hở vật cản, vận tốc).
   - *Ưu điểm*: Đảm bảo 100% tính khả thi động học; xe không bao giờ nhận lệnh vượt quá quán tính vật lý.
2. **Follow The Gap / Disparity Extender (FGM / DDE)**:
   - *Mã nguồn tham khảo*: `f1tenth/f1tenth_gym`, `f1tenth-dev/gap_follow`, `MIT-Racecar`.
   - *Nguyên lý*: Thuật toán phản xạ trực tiếp trên tia LiDAR $360^\circ$ ($O(N)$, độ trễ $< 0.1\text{ms}$). Khi gặp chênh lệch khoảng cách, mở rộng mép an toàn $\Delta\theta = \arcsin((R_{bot} + \text{margin}) / d_{near})$, sau đó chọn khe hở hướng về mục tiêu.
   - *Ưu điểm*: Miễn nhiễm hoàn toàn với sai số trôi tọa độ (Mapless Reactive), phản ứng tức thì không độ trễ.
3. **Control Barrier Functions (CBF-QP)**:
   - *Mã nguồn tham khảo*: Caltech AMBER Lab, `HybridRobotics/CBF-CLF-QP`.
   - *Nguyên lý*: Rào chắn an toàn bất biến $h(x) \ge 0$, giải tích QP tối ưu lồi $(< 0.06\text{ms})$, chỉ can thiệp nắn chỉnh lệnh điều khiển khi quỹ đạo vi phạm ranh giới an toàn.
   - *Ưu điểm*: Bảo vệ an toàn toán học tuyệt đối mà không làm méo mó quỹ đạo danh định khi thông thoáng.
4. **Vector Field Histogram (VFH / VFH+)**:
   - *Mã nguồn tham khảo*: `fatimahtanvir/obstacle-avoider-ros2`.
   - *Nguyên lý*: Chuyển đổi dữ liệu cảm biến sang lược đồ mật độ vật cản theo góc cực (Polar Obstacle Density) và chọn cung hở thông thoáng.

---

## 5. CHECKLIST KIỂM THỬ TRÊN PC (PC-FIRST VERIFICATION)
```bash
/home/zxcvmh/Projects/CS532/03-MinhHieu/backend/.venv/bin/python 04-hieu-MotionControl/test_planning_local.py
/home/zxcvmh/Projects/CS532/03-MinhHieu/backend/.venv/bin/python 04-hieu-MotionControl/test_dde_cbf.py
```
- `test_planning_local.py`: PASS 4/4 (Grid Update: 4.16ms, Clustering: 0.16ms, Plateau Detour: hở 0.40m, Curvature Scaling: giảm 80%).
- `test_dde_cbf.py`: PASS 3/3 (DDE Latency: 0.095ms, CBF-QP: 0.057ms, Integrated Evasion: PASS).
Tất cả bộ test phải đạt PASS 100% trước khi triển khai thực nghiệm.
