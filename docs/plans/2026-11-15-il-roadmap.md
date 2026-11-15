# Kế hoạch lộ trình Imitation Learning (IL Roadmap) — CS532 JetBot (2026-11-15)

> **Mục tiêu:** Đồng bộ toàn diện môi trường mô phỏng (Simulation), chính sách chuyên gia (Expert Oracle) và mạng nơ-ron (Policy) với động lực học đo thực tế trên xe JetBot (sàn gạch men bóng, động cơ GA25-370, LiDAR LDROBOT D500).

---

## 1. Cơ sở thực nghiệm & Số liệu đo đạc ngày 2026-11-15

### A. Động lực học xe thật (`setup/robot_dynamics.json`)
> **Supervisor note (2026-11-16):** the numbers below are a snapshot of the file on 2026-11-15, for context only.
> The human plans new turn values (turn_speed / turn_time_90 / turn_time_180). Code and tests must READ the file;
> never copy these numbers into code, tests or thresholds.
- **Ngưỡng ga điều khiển:** `min_drive_speed = 0.19` (ga tối thiểu để lăn bánh), `cruise_speed = 0.20` (ga hành trình chạy thẳng).
- **Vận tốc vật lý thực tế:** `actual_speed_mps = 0.294 m/s` (xe đi $1.0\text{ m}$ mất $3.4\text{ s}$).
  - *Lưu ý quan trọng:* Ngưỡng ga `0.19` thực chất đã cho xe chạy **khá nhanh** (~$0.3\text{ m/s}$ trong sa hình $3\text{ m} \times 2.4\text{ m}$). Tên gọi `min_drive_speed` trong file cấu hình là mức ga thực tế thấp nhất có thể chạy, không phải tốc độ bò chậm.
  - Ngưỡng danh nghĩa cũ `max_safe_speed_mps = 0.14 m/s` không khả thi về cơ khí (ở mức ga tương ứng ~0.10, động cơ bị kẹt do stiction; băm xung PWM gây trượt và giật).
- **Tốc độ quay tại chỗ (Pivot):** `turn_speed = 0.19`, `turn_time_90_deg_sec = 0.3 s` $\implies \omega \approx 5.236\text{ rad/s}$ ($300^\circ/\text{s}$).
- **Quy tắc điều khiển:** Xe chỉ có một tốc độ tiến và một tốc độ quay. Không dùng bẻ lái vòng cung (arc steering) mà dùng cơ chế **Stop-Turn-Go**.

### B. Baseline hiện trạng (Chạy ngày 2026-11-15 trên `test_il_sim_success.py`)
- **Runtime Policy hiện tại:** Thành công **89.0%**, va chạm **2.0%**, timeout **9.0%** (100 seeds).
  - Failed seeds: `[17, 34, 38, 44, 48, 49, 54, 62, 69, 83, 86]`.
  - Tỉ lệ mạng nơ-ron (NN) thực sự tự lái: chỉ **37.1%** (hơn 60% còn lại là các hàm luật chắp vá như pivot thoát hiểm, yield, bypass latch trong `il_policy_runner.py`).
- **Expert cũ (`GeometricExpertPolicy`):** Thành công **48.0%**, va chạm **40.0%**. Thầy lái kém hơn trò vì dùng luật hình học thuần túy trên các tia LiDAR cục bộ, không xử lý kịp người đi bộ cắt ngang ở vận tốc cao.

---

## 2. Lộ trình 5 bước thực hiện (5-Step Execution Roadmap)

| Bước | Ticket | Nội dung kỹ thuật | Tiêu chí nghiệm thu (Gate / Locked Test) | Trạng thái |
| :---: | :---: | :--- | :--- | :---: |
| **1** | **TK-IL-03** | Cách ly pipeline test sang `tempfile.mkdtemp()`, chặn hoàn toàn việc ghi đè model runtime và chặn `[PASS]` giả khi success = 0%. | `git status --short imitation_learning/` sạch 100% sau test; `test_05` ném `AssertionError` nếu success = 0%. | **IN REVIEW** (audit 1: code OK, scope REJECT) |
| **2** | **TK-IL-04** | Cập nhật sim theo động lực học thật: đọc `actual_speed_mps` và `turn_time_90_deg_sec` từ `setup/robot_dynamics.json`. Chuyển action space sang 4 lệnh rời rạc `{0: STOP, 1: FORWARD, 2: PIVOT_L, 3: PIVOT_R}`. | `tests/acceptance/test_il_sim_dynamics.py` (chưa viết): Tiến 1 s đi `actual_speed_mps` × 1 s ± 10 %; quay 90° mất `turn_time_90_deg_sec` ± 10 % — giá trị đọc từ file lúc chạy test. Không hardcode hằng số. | **DRAFT** |
| **3** | **TK-IL-05** | Viết lại Expert đặc quyền (`PrivilegedExpertPolicy`): cho phép quan sát vị trí và vận tốc thật của người đi bộ để tính Time-to-Collision (TTC) chính xác, ra quyết định dứt khoát né hoặc dừng. | `tests/acceptance/test_il_expert.py`: Expert đạt Success $\ge 95\%$, Collision $\le 1\%$ trên 100 locked seeds. | **DRAFT** |
| **4** | **TK-IL-06** | Thu thập lại dữ liệu với Expert mới. Train Behavioral Cloning $\to$ DAgger phân loại 4 lớp (Cross-Entropy). Tinh gọn `il_policy_runner.py`. | `tests/acceptance/test_il_sim_success.py`: Success $\ge 90\%$, Collision $\le 2\%$, và số bước do NN tự lái $\ge 90\%$. | **DRAFT** |
| **5** | **TK-IL-07** | Sim-to-Real: Thêm nhiễu LiDAR đo từ thực địa (`.npz`). Đo độ trễ suy luận ONNX trên Jetson Nano và kiểm thử 10 lần chạy trên xe thật. | Jetson loop $\le 50\text{ ms}$, RAM $\le 1.5\text{ GB}$; Xe thật chạy $10/10$ lượt đến đích an toàn không va chạm. | **DRAFT** |

---

## 3. Thiết kế kỹ thuật chi tiết từng bước

### Bước 2: TK-IL-04 — Mô phỏng Stop-Turn-Go khớp động lực học thực tế
- **File cho phép sửa:** `imitation_learning/config.py`, `imitation_learning/mock_expert_simulator.py`.
- **Hành động rời rạc (Discrete Actions):**
  - `ACTION_STOP = 0`: $v = 0.0, \omega = 0.0$.
  - `ACTION_FORWARD = 1`: $v$ = `actual_speed_mps`, $\omega = 0.0$.
  - `ACTION_PIVOT_LEFT = 2`: $v = 0.0$, $\omega = +(\pi/2)$ / `turn_time_90_deg_sec`.
  - `ACTION_PIVOT_RIGHT = 3`: $v = 0.0$, $\omega = -(\pi/2)$ / `turn_time_90_deg_sec`.
- **Động học Simulator:**
  - Loại bỏ hoàn toàn mô hình nội suy liên tục và dải ga chết cũ (0.10 m/s).
  - Khi xe nhận lệnh `FORWARD`, xe di chuyển với `actual_speed_mps` đọc từ file (nếu thêm `slip_ratio` thì vẫn phải nằm trong ± 10 % của acceptance).
  - Khi xe nhận lệnh `PIVOT_L/R`, 90° mất đúng `turn_time_90_deg_sec` đọc từ file.

### Bước 3: TK-IL-05 — Expert đặc quyền (Privileged Teacher Oracle)
- **File cho phép sửa:** `imitation_learning/mock_expert_simulator.py` (chỉ lớp Expert).
- **Vấn đề cốt lõi:** Xe chạy $0.294\text{ m/s}$ trong sa hình $3\text{ m} \times 2.4\text{ m}$ là tốc độ rất nhanh (thời gian vượt sa hình chỉ ~10s). Thầy cũ chỉ nhìn 36 tia khoảng cách LiDAR nên không thể phân biệt được vật cản tĩnh với người đi bộ đang lao tới, dẫn đến va chạm tới 40%.
- **Giải pháp Teacher-Student:**
  - Theo chuẩn Robotics hiện đại (LeRobot / Sim-to-Real), Expert trong môi trường mô phỏng được cấp quyền quan sát trạng thái thế giới (privileged state: vị trí $(x_p, y_p)$, vector vận tốc $(v_{xp}, v_{yp})$ của người đi bộ).
  - Expert tính toán điểm cắt quỹ đạo (trajectory crossing point) và khoảng cách tiếp cận gần nhất (Distance of Closest Approach - DCA). Nếu dự báo va chạm trong $< 1.5\text{ s}$, Expert dừng xe (`STOP`) nhường đường hoặc xoay đầu (`PIVOT`) tìm hành lang thoát.
  - Khi hành lang thoáng, Expert tiến dứt khoát (`FORWARD`).
  - Học sinh (mạng nơ-ron học qua BC/DAgger) vẫn chỉ nhìn quan sát 41 chiều (36 tia LiDAR + thông tin mục tiêu), không được nhìn trạng thái đặc quyền.

### Bước 4: TK-IL-06 — Phân loại rời rạc 4 lớp (4-Class Discrete Policy)
- **File cho phép sửa:** `imitation_learning/{model.py, train_bc.py, dagger_session.py, il_policy_runner.py}`.
- **Kiến trúc mạng:**
  - Đầu ra lớp cuối cùng chuyển từ Linear(2) sang Linear(4) (tương ứng 4 logit cho 4 action).
  - Hàm mất mát: `nn.CrossEntropyLoss()`.
- **DAgger Session:**
  - Chạy tương tác với môi trường: mạng chọn action rời rạc, Expert gán nhãn action tối ưu.
  - Đóng gói checkpoint sang ONNX.
- **Tối giản Runtime Runner:**
  - Xóa bỏ các nhánh heuristic ghi đè (terminal approach, yield, escape pivot, bypass latch).
  - Mạng nơ-ron trực tiếp quyết định hành vi lái xe $\ge 90\%$ số bước. Chỉ giữ lại lá chắn phanh khẩn cấp duy nhất (Emergency Reflex Brake) khi phát hiện vật cản bất ngờ $< 0.12\text{ m}$.

### Bước 5: TK-IL-07 — Đóng gói Sim-to-Real & Đo kiểm trên Jetson Nano
- **Tích hợp:** `dashboard/backend/app/navigation/il_onnx_bridge.py`.
- **Kiểm thử Jetson Nano (Python 3.6.9, JP4.5):**
  - Đo latency suy luận mô hình ONNX Runtime trên Jetson Nano CPU.
  - Kiểm tra tiêu thụ bộ nhớ RAM ($\le 1.5\text{ GB}$).
  - Chạy thực địa trên sàn gạch men trong bài test né người đi bộ cắt ngang thật.
