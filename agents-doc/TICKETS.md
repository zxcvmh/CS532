# HỆ THỐNG PHIẾU CÔNG VIỆC (AI AGENT WORK TICKETS) — CS532

*Tài liệu này quản lý các Ticket công việc vi mô (Micro-sprints) của từng Agent. Mỗi ticket phải có Tiêu chuẩn Nghiệm thu (Definition of Done) rõ ràng để kiểm chứng độc lập.*

---

## 📌 SPRINT HIỆN TẠI: TỐI ƯU DASHBOARD & SỬA LỖI THUẬT TOÁN TÌM ĐƯỜNG

---

### Ticket TK-01: [PLANNING] Chẩn đoán & Viết lại Thuật toán A* Path Planning
- **Agent gán trách nhiệm:** `planning-control-agent`
- **Mức độ ưu tiên:** 🔴 **P0 — Critical Blocker**
- **Mô tả vấn đề:**
  Mã nguồn tìm đường hiện tại do bạn trong nhóm viết đang gặp lỗi. Cần chẩn đoán lỗi cụ thể (cắt góc chéo qua 2 vật cản, ngược trục tọa độ $X/Y$, hoặc hàm heuristic không tối ưu), sau đó hoàn thiện thuật toán **A* Search 8 hướng** trên ma trận NumPy kèm bộ làm mượt quỹ đạo (Path Smoothing) cho robot bánh vi sai.
- **Tiêu chuẩn Hoàn thành (Definition of Done):**
  - [ ] Thu thập và phân tích đoạn code tìm đường bị lỗi hiện tại.
  - [ ] Cài đặt thuật toán A* thuần trên Python/NumPy, hỗ trợ di chuyển 8 hướng ($d_{\text{straight}} = 1.0, d_{\text{diag}} = \sqrt{2}$).
  - [ ] Chống lỗi cắt góc vật cản (Corner Cutting Prevention): Nếu 2 ô kề nhau là vật cản thì không được đi chéo qua khe hở giữa chúng.
  - [ ] Tích hợp hàm làm mượt đường (loại bỏ các nút zigzag $90^\circ$).
  - [ ] Viết unit test tự động chạy trên grid mẫu $50 \times 50$, thời gian tìm đường $< 30\text{ ms}$.
  - [ ] Giải thích rõ mô hình toán học và mã nguồn để thành viên nhóm tự tin bảo vệ trước giảng viên.

---

### Ticket TK-02: [WEB] Khắc phục Triệt để Hiện tượng Dashboard Bị Lag
- **Agent gán trách nhiệm:** `interface-comms-agent`
- **Mức độ ưu tiên:** 🟢 **P1 — High (HOÀN THÀNH - 100%)**
- **Mô tả vấn đề:**
  Giao diện Web Dashboard hiện tại bị lag, giật khung hình khi chạy. Cần tối ưu hóa tầng truyền thông WebSocket và cơ chế vẽ trên Canvas 2D.
- **Tiêu chuẩn Hoàn thành (Definition of Done):**
  - [x] **Giới hạn tần số WebSocket:** Throttle luồng dữ liệu phát từ server về client đúng **10 Hz** (chu kỳ $100\text{ ms}$, khớp với LiDAR D500), không để đẩy vô tội vạ 30–60Hz gây nghẽn trình duyệt.
  - [x] **Tối ưu hóa vòng lặp Canvas 2D:**
    - Sử dụng `requestAnimationFrame` thay vì `setInterval`.
    - Dùng cơ chế **Off-screen Canvas** (vẽ trước toàn bộ lưới tĩnh $100 \times 100$ vào canvas ẩn trong bộ nhớ, chỉ dùng `ctx.drawImage()` 1 lần lên canvas chính thay vì chạy vòng lặp 10,000 lần mỗi frame).
    - Chỉ vẽ lại các điểm laser LiDAR và vị trí robot khi có gói tin mới.
  - [x] Tách luồng stream video MJPEG riêng biệt (`/video_feed`), không dùng chung WebSocket với Telemetry.
  - [x] Đo đạc FPS giao diện trên Chrome/Edge đạt ổn định $\ge 60\text{ FPS}$ (không giật lag).
  - [x] Tái thiết kế toàn bộ Dashboard theo chuẩn **Apple visionOS / macOS Glassmorphism**, tích hợp **macOS Control Center**, bộ điều khiển Magic Keyboard W/A/S/D tactile, thanh trượt tốc độ an toàn và E-Stop phát quang.

---

### Ticket TK-03: [PERCEPTION] Chuẩn hóa Output YOLOv8n theo Hợp đồng Dữ liệu
- **Agent gán trách nhiệm:** `perception-agent`
- **Mức độ ưu tiên:** 🟢 **P2 — Medium**
- **Mô tả vấn đề:**
  Đôi mắt thị giác AI đã phát hiện được người, nhưng cần chuẩn hóa cấu trúc đầu ra theo đúng hợp đồng dữ liệu trong `agents-doc/INTERFACE.md` để kết hợp với LiDAR.
- **Tiêu chuẩn Hoàn thành (Definition of Done):**
  - [ ] Chuyển đổi tọa độ tâm bounding box thành góc phương vị:
    $$\theta_{\text{azimuth}} = \frac{x_{\text{center}} - W/2}{W/2} \times 80^\circ$$
  - [ ] Đóng gói output đúng JSON schema `SemanticDetections`: `[{"class_name": "person", "bbox": [...], "azimuth_deg": ...}]`.
  - [ ] Đảm bảo inference time trên Jetson Nano $\le 50\text{ ms}$ ($\ge 20\text{ FPS}$).

---

### Ticket TK-04: [INTEGRATION] Kiểm định Tích hợp & Đối chiếu Latency
- **Agent gán trách nhiệm:** `integration-reviewer-agent`
- **Mức độ ưu tiên:** 🔵 **P3 — Verification**
- **Mô tả vấn đề:**
  Kiểm định chất lượng sau khi TK-01 và TK-02 hoàn thành, xác nhận 2 bên nói chuyện khớp nhau.
- **Tiêu chuẩn Hoàn thành (Definition of Done):**
  - [ ] Schema `TelemetryState` từ `planning` gửi sang `web` khớp 100% từng trường dữ liệu.
  - [ ] Click tọa độ trên Web gửi lệnh `SET_GOAL` xuống `planning` được giải mã chuẩn xác mà không bị lệch tọa độ.
  - [ ] Nút dừng khẩn cấp E-Stop phản hồi ngắt động cơ trong $< 50\text{ ms}$.

---

### Ticket TK-05: [SIMULATION] Xây dựng Môi trường Mô phỏng 2D SIL & LiDAR Raycasting
- **Agent gán trách nhiệm:** `simulation-agent`
- **Mức độ ưu tiên:** 🟡 **P1 — High (Blocker Bypass do thiếu giá đỡ cơ khí LiDAR)**
- **Mô tả vấn đề:**
  Robot thật chưa có giá đỡ cơ khí cho LiDAR D500 trên đầu xe nên không thể chạy thực địa. Cần tạo môi trường Software-in-the-Loop (SIL) 2D độc lập: mô phỏng phòng $5\text{m} \times 5\text{m}$ có vật cản, bắn 360 tia LiDAR ảo (raycasting) ở tần số chuẩn 10Hz kèm nhiễu đo, và mô phỏng động học xe bánh vi sai (Unicycle Kinematics) cùng giới hạn gia tốc hình thang ($|a| \le 0.3\text{ m/s}^2$).
- **Tiêu chuẩn Hoàn thành (Definition of Done):**
  - [ ] Tạo module bản đồ phòng 2D `simulation/virtual_world.py` định nghĩa tọa độ tường và chướng ngại vật mẫu.
  - [ ] Cài đặt thuật toán 2D Raycasting trong `simulation/lidar_simulator.py` xuất mảng `ranges[360]` đúng chuẩn `INTERFACES.md` ở tần số 10Hz kèm Gaussian noise $\sigma = 0.01\text{ m}$.
  - [ ] Cài đặt mô hình động học xe vi sai có ràng buộc gia tốc thực tế trong `simulation/virtual_robot.py`.
  - [ ] Cung cấp cờ cấu hình chuyển đổi (Mock toggle) để `planning` và `web` kiểm thử toàn bộ hệ thống ngay trên máy tính mà không cần phần cứng.

