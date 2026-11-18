# TK-SLAM-02: LiDAR Lever-Arm Offset Analysis & Empirical Calibration

**Question asked by human:**
> Kiểm tra xem lỗi này là thế nào? Do lidar nằm ở giữa robot nên cần phải trừ khoảng 8 cm (từ rìa robot đến tâm lidar) là 8 cm? Chúng ta cần làm gì cho vấn đề trên?
> *(Cập nhật đo đạc thực tế từ người dùng: "Tôi đo lại thì nó lệch khoảng 1 cm thôi")*

---

## 1. Kết quả Đo đạc Thực tế (Empirical Ground Truth)

- **Số đo thực tế do người dùng đo trên xe thật:** Khoảng cách từ trục bánh xe chủ động (wheel axle) đến tâm mắt LiDAR D500 chỉ là **$\approx 1\text{ cm}$ ($0.01\text{ m}$)**, hoàn toàn KHÔNG PHẢI **$8\text{ cm}$ ($0.08\text{ m}$)**.
- **Kích thước bao khung xe:** Bán kính bao hình học vẫn là `ROBOT_RADIUS = 0.09` m (9 cm) như định nghĩa tại `dashboard/backend/app/config.py:38`.

---

## 2. Phát hiện Then chốt: Nguồn gốc Ảo giác 8 cm (Root Cause Analysis)

### 2.1. Nghịch lý giữa Codebase và Phần cứng thật
Trước đây, các phân tích trong ticket `TK-SLAM-02` dựa trên giả định rằng tầng SLAM là "chuẩn" và tầng Dashboard (`telemetry_parser.py`, `motion_adapter.py`) bị "quên" 8 cm.

Tuy nhiên, với số đo thực tế $l_a \approx 1\text{ cm}$:
1. **Tầng SLAM đang bị "ảo giác" nặng nề:**
   - `motion/planning/lidar_odometry.py:69, 181-182`: Hardcode `lever_arm_x = 0.08` m (+8 cm).
   - `motion/planning/slam/bayesian_raycaster.py:18, 46-47`: Hardcode `lever_arm = 0.08` m.
   - `motion/planning/slam/anchor_layer.py:44, 53-54`: Hardcode `lever_arm = 0.08` m.
   $\rightarrow$ Tầng SLAM đang giả định LiDAR nằm cách trục bánh xe tới 8 cm (thừa ra 7 cm so với xe thật!).
2. **Tầng Dashboard thực ra lại gần thực tế hơn:**
   - `dashboard/backend/app/hardware/telemetry_parser.py:92-93` (`ranges_to_points`): dùng $l_a = 0.0$.
   - `dashboard/backend/app/navigation/motion_adapter.py:205` (`process_clusters`): dùng $l_a = 0.0$.
   $\rightarrow$ Sai số giữa $0.0\text{ m}$ và $0.01\text{ m}$ chỉ là **$1\text{ cm}$**, nhỏ hơn nhiều so với độ phân giải lưới bản đồ $5\text{ cm}$ (`MAP_RESOLUTION = 0.05` m).

### 2.2. Hậu quả của việc SLAM gán sai 8 cm trên xe thật
Khi robot quay tại chỗ 180° (`in-place turn`):
- **Trên phần cứng thực:** Tâm LiDAR chỉ dịch chuyển $2 \times 1\text{ cm} = 2\text{ cm}$ trong không gian (nhỏ hơn 1 ô lưới $5\text{ cm}$ và nằm trong phạm vi nhiễu $\pm 2\text{ cm}$ của cảm biến D500).
- **Trong thuật toán SLAM:** SLAM kỳ vọng tâm LiDAR vung theo cung tròn bán kính $8\text{ cm}$, tức là dịch chuyển $2 \times 8\text{ cm} = 16\text{ cm}$ (hơn 3 ô lưới!).
- **Hậu quả:** Khi xe quay trên thực tế, SLAM thấy các bức tường không dịch chuyển 16 cm như mô hình toán kỳ vọng, dẫn tới việc SLAM **tự bẻ cong quỹ đạo và làm trôi bản đồ** để cố khớp với cánh tay đòn 8 cm ma này! Đây chính là nguyên nhân gốc rễ gây ra lỗi xoay trôi bản đồ tại `TK-SLAM-03`.

---

## 3. Đề xuất Giải pháp Chuẩn xác cho Ticket TK-SLAM-02 (Tuân thủ Ranh giới Phân vai)

Theo quy định quản trị hệ thống (`AGENTS.md` & `.agents/workflows/slam.md`), vai trò **SLAM & Mapping Engineer** phải tuân thủ nghiêm ngặt ranh giới quyền hạn:
- **File SLAM sở hữu:** `motion/planning/slam/**`, `cartographer/**`, `dashboard/backend/app/navigation/cartographer_adapter.py`, `tests/test_hector_anchor_slam.py`.
- **File KHÔNG ĐƯỢC PHÉP sửa:** `setup/robot_dynamics.json` (LOCKED), `dashboard/backend/app/navigation/motion_adapter.py` (thuộc Integration), `dashboard/backend/app/hardware/telemetry_parser.py` (thuộc Integration), `coordinator.py`, `jetbot_bridge.py`.

Do đó, giải pháp được phân định rành mạch thành 3 phạm vi sau:

### 3.1. Yêu cầu chuyển Supervisor / Human (Cập nhật file LOCKED)
File `setup/robot_dynamics.json` nằm trong danh mục `LOCKED_PATHS.txt` (bất khả xâm phạm). Đề xuất Claude Supervisor hoặc Human cập nhật thông số đo đạc thực địa:
```json
"lidar_lever_arm_m": 0.01
```
*(Ghi chú: Đo đạc thực tế từ tâm trục bánh xe GA25-370 đến tâm mắt quét LiDAR D500).*

### 3.2. Phạm vi can thiệp trực tiếp của Role SLAM (Chỉ sửa file SLAM sở hữu)
Chỉ thực hiện trong thư mục `motion/planning/slam/`:
1. **`motion/planning/slam/bayesian_raycaster.py`:**
   - Loại bỏ hardcode `lever_arm: float = 0.08` tại `__init__`.
   - Chuyển sang đọc giá trị cấu hình `lever_arm` được truyền vào, với fallback mặc định là `0.01` (khớp với `lidar_lever_arm_m` trong `robot_dynamics.json`).
2. **`motion/planning/slam/anchor_layer.py`:**
   - Loại bỏ hardcode `lever_arm: float = 0.08` tại `set_anchor_from_scan`.
   - Chuyển sang nhận giá trị cấu hình `lever_arm` (hoặc fallback `0.01`).
3. **`motion/planning/lidar_odometry.py` (Nếu cần):**
   - File này nằm tại `motion/planning/` (ngoài `slam/`). Nếu Supervisor yêu cầu xử lý `self.lever_arm_x = 0.08`, SLAM Engineer sẽ yêu cầu cấp quyền Allowed files trước khi sửa.

### 3.3. Interface Note bàn giao cho Role Integration (Dashboard / Backend)
Ticket `TK-SLAM-02` trong `agents-doc/TICKETS.md` được gán cho `Role: integration`. SLAM Engineer bàn giao Interface Note kỹ thuật để Integration Engineer thực thi:
1. **`dashboard/backend/app/config.py`:**
   - Đọc cấu hình từ file dynamics: `Config.LIDAR_LEVER_ARM_M = float(_cfg_dyn.get("lidar_lever_arm_m", 0.01))`.
2. **`dashboard/backend/app/hardware/telemetry_parser.py:92-93` (`ranges_to_points`):**
   - Thay `lidar_x = px`, `lidar_y = py` bằng tọa độ bù lever arm:
     `lidar_x = px + Config.LIDAR_LEVER_ARM_M * math.cos(pth)`
     `lidar_y = py + Config.LIDAR_LEVER_ARM_M * math.sin(pth)`
3. **`dashboard/backend/app/navigation/motion_adapter.py:205` (`process_clusters`):**
   - Thay giá trị `la = 0.0` bằng `la = getattr(Config, "LIDAR_LEVER_ARM_M", 0.01)`.

### 3.4. Điều chỉnh định nghĩa Ticket TK-SLAM-02
- Cập nhật lại mô tả trong `agents-doc/TICKETS.md`: Thay vì hiểu nhầm là "Dashboard bỏ quên 8 cm", mục tiêu chuẩn của `TK-SLAM-02` là **đồng bộ toàn hệ thống theo khoảng cách thực tế 1 cm (0.01 m)** từ `setup/robot_dynamics.json`, loại bỏ hoàn toàn con số ảo giác 8 cm ở cả SLAM và Dashboard.
- **Đánh giá dung sai:** Với $l_a = 0.01\text{ m}$, sai số tối đa khi robot quay 180° chỉ là $2\text{ cm} < \text{resolution } 5\text{ cm}$. Việc đồng bộ chuẩn 0.01 m triệt tiêu bất đối xứng hình học và giải quyết tận gốc nguyên nhân trôi xoay tại `TK-SLAM-03`.

---

## 4. Danh mục Xác thực (UNVERIFIED Items)
- Hướng lệch của 1 cm: Mặc định là lệch về phía trước (+x theo hướng tiến của robot).
