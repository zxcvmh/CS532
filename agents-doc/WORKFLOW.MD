# CS532 — Workflow & Harness tổng quan

Đồ án: Robot di động tự hành trong nhà (JetBot / Jetson Nano) — Trang bị cảm biến **LiDAR D500** quét 360° kết hợp camera CSI nhận diện người/vật thể di chuyển bằng **YOLOv8n**, tự động lập bản đồ lưới (Occupancy Grid) và điều hướng tìm đường real-time, có dashboard web giám sát và điều khiển.

Tài liệu này là "hiến pháp" của team khi làm việc với AI agent. Mọi thành viên và mọi agent đều phải tuân theo các quy tắc ở đây trước khi tuân theo bất kỳ hướng dẫn nào khác.

**Tài liệu tham khảo bắt buộc của môn học:**
- AI Club IT Courses CS532: https://aiclub.uit.edu.vn/itcourses/#CS532
- Hardware & IoT: https://aiclub.uit.edu.vn/itcourses/#t/Hardware%20&%20IoT
*(Bắt buộc tham khảo tài liệu của trường/CLB trước rồi mới tham khảo các nguồn bên ngoài)*

---

## 1. Phương án Định vị, Bản đồ & Điều hướng (Đã chốt chính thức)

- **Phần cứng bổ sung đã duyệt:** Cảm biến **LiDAR D500** (360° DToF Laser Scanner, tầm quét 0.03 – 12m, tần số quét 10Hz, giao tiếp UART @ 230400 bps qua cổng USB `/dev/ttyUSB0`).
- **Kiến trúc cốt lõi:** **Online Real-time SLAM & Dynamic Goal Navigation**
  - **Bản đồ lưới (Occupancy Grid):** Xây dựng và cập nhật bản đồ lưới 2D thời gian thực từ dữ liệu chùm tia LiDAR D500 bằng thuật toán log-odds / Bresenham raycasting tự viết trên Python/NumPy.
  - **Điều hướng tới đích (Point-to-Point Navigation):** Người dùng click chọn tọa độ đích $(X_{goal}, Y_{goal})$ trên Web Dashboard. Thuật toán tìm đường toàn cục (A*/Dijkstra) tự cài đặt tìm đường đi tối ưu và làm mượt quỹ đạo cho bánh vi sai.
  - **Phản xạ né vật cản & Phanh khẩn cấp (Safety Reflex & Dynamic Replanning):** 
    - Áp dụng kiến trúc phân tầng tối ưu theo tần số quét phần cứng:
    - Tầng an toàn phản xạ 10Hz (100ms): Ngắt động cơ khẩn cấp ngay lập tức nếu bất kỳ điểm quét nào của LiDAR D500 lọt vào vùng an toàn ($< 20\text{ cm}$).
    - Tầng cập nhật bản đồ 10Hz: Cập nhật vật cản mới vào Occupancy Grid đồng bộ theo từng vòng quét của LiDAR.
    - Tầng tìm đường A*: Kích hoạt tính lại đường (Event-driven Replanning) khi đường đi hiện tại bị vật cản/người mới xuất hiện chặn ngang.
  - **Kết hợp Thị giác ngữ nghĩa (Camera - LiDAR Fusion):** Camera IMX219 chạy YOLOv8n TensorRT FP16 phát hiện người/vật thể di chuyển ($160^\circ$ FOV), trích xuất góc phương vị để gán nhãn ngữ nghĩa (Semantic Tag) và mở rộng vùng an toàn (Social Bubble) quanh người trên bản đồ 2D.
- **Tiêu chuẩn học thuật (Đảm bảo điểm A/A+ đồ án):** Sinh viên tự viết tay thuật toán cốt lõi (Parser LiDAR, Occupancy Grid log-odds, A* Search & Path Smoothing, Pure Pursuit + PID). Không dùng các package hộp đen của ROS Navigation (`move_base`, `nav2`) để đảm bảo giải trình toán học và bảo vệ đồ án tốt nhất.

---

## 2. Cấu trúc 5 Agent & Phân công Trách nhiệm

| # | Agent | File cấu hình (`agents-doc/` & `.claude/`) | Phạm vi thư mục | Trách nhiệm chính với LiDAR D500 & Camera |
|---|---|---|---|---|
| 0 | Robot Setup Agent | `agents-doc/agents/robot-setup-agent.md` | `setup/` | Cấu hình & kiểm tra phần cứng: Camera CSI, Động cơ I2C, TensorRT YOLOv8n và **giao tiếp UART LiDAR D500** (`setup/healthcheck.py`). |
| 1 | Perception Agent | `agents-doc/agents/perception-agent.md` | `perception/` | Chạy inference YOLOv8n TensorRT FP16, phát hiện người/vật cản, tính góc phương vị và bounding box gửi sang Planning. |
| 2 | Planning & Control Agent | `agents-doc/agents/planning-control-agent.md` | `planning/` | Bộ não điều hướng: Đọc LaserScan 360°, xây dựng Occupancy Grid 2D, lập đường đi A*, điều khiển Pure Pursuit + PID, phanh khẩn cấp & dynamic replanning. |
| 3 | Interface & Comms Agent | `agents-doc/agents/interface-comms-agent.md` | `web/` | Web Dashboard: Stream video camera, render bản đồ lưới 2D và tia LiDAR thời gian thực, cho phép click chọn điểm đích, gửi lệnh điều khiển và dừng khẩn cấp. |
| 4 | Integration Reviewer Agent | `agents-doc/agents/integration-reviewer-agent.md` | Toàn repo (chỉ đọc + test) | Ghép nối 4 module, kiểm tra hợp đồng dữ liệu `INTERFACES.md`, đo đạc latency vòng lặp điều khiển, viết integration test. |
| + | Simulation & Digital Twin Agent | `agents-doc/agents/simulation-agent.md` | `simulation/` | Môi trường mô phỏng 2D (Software-in-the-Loop), tạo chùm tia LiDAR ảo (Raycasting) & mô phỏng động học xe bánh vi sai khi chưa có giá đỡ phần cứng. |

Nguyên tắc chung: **Một agent — một thư mục — một người chịu trách nhiệm hiểu code**.
Không agent nào được sửa file ngoài phạm vi của mình. Mọi trao đổi dữ liệu phải tuân thủ nghiêm ngặt theo `agents-doc/INTERFACE.md`.

---

## 3. Trình tự Làm việc (Lộ trình 5 Tuần Cập nhật)

1. **Tuần 1:** Robot Setup Agent hoạt động. Mục tiêu: JetBot boot ổn định, camera CSI chụp tốt, YOLOv8n TensorRT đạt ~20 FPS, **LiDAR D500 kết nối cổng USB nhận dữ liệu baudrate 230400**, script `setup/healthcheck.py` pass 4/4 tiêu chí.
2. **Tuần 2:**
   - Perception Agent: Chạy YOLOv8n xuất bounding box + góc phương vị theo chuẩn `INTERFACES.md`.
   - Planning & Control Agent: Tự cài đặt parser đọc gói tin D500, viết module Occupancy Grid log-odds và thuật toán tìm đường A* trên dữ liệu mô phỏng giả lập.
3. **Tuần 3:**
   - Planning & Control Agent: Ghép dữ liệu thực từ LiDAR D500, kiểm thử xây bản đồ lưới phòng thực tế và phanh khẩn cấp khi gặp vật cản sát mũi xe.
   - Interface & Comms Agent: Xây dựng giao diện Web Dashboard hiển thị bản đồ lưới 2D (HTML5 Canvas/WebGL), điểm laser LiDAR và cơ chế click chọn điểm đích.
4. **Tuần 4:**
   - Integration Reviewer Agent: Tích hợp vòng lặp điều khiển hoàn chỉnh (LiDAR + Camera YOLOv8n + Planning A* + Web Dashboard).
   - Chạy thử nghiệm điều hướng tự hành: Đặt đích trên Web $\to$ Robot tự vẽ bản đồ, tính đường A* và chạy tới đích. Thử nghiệm người bước ra chặn đường $\to$ Robot phát hiện và tự động tính đường vòng tránh hoặc dừng an toàn.
5. **Tuần 5:** Đo đạc các chỉ số (độ chính xác định vị, thời gian tìm đường A*, FPS camera, phản xạ né vật cản), ghi hình minh chứng demo và hoàn thiện báo cáo đồ án.

---

## 4. Quy ước Bắt buộc

- **Commit message:** Mỗi commit có phần AI-hỗ trợ phải ghi rõ: `AI-assisted: <phần nào được agent tạo>`.
- **Giải trình học thuật:** Người phụ trách module phải tự giải thích được bản chất thuật toán (cách thức cập nhật xác suất occupancy grid, công thức hàm heuristic A*, giải thuật điều khiển bám đường và PID).
- **Quy tắc an toàn phần cứng:**
  - Luôn duy trì vùng an toàn tối thiểu $20\text{ cm}$ quanh thân robot.
  - Vận tốc di chuyển thử nghiệm giới hạn ở mức an toàn: $v_{max} = 0.2 - 0.3\text{ m/s}$.
  - Phải có nút Dừng khẩn cấp (Emergency Stop) vật lý (Ctrl+C / công tắc nguồn) và nút dừng mềm trên Web Dashboard.
- **Branch:** Mỗi agent làm việc trên nhánh riêng (`perception/*`, `planning/*`, `web/*`), Integration Reviewer Agent là người duy nhất merge vào `main`.

---

## 5. Tài liệu Cốt lõi của Dự án

- `agents-doc/INTERFACE.md` — Hợp đồng dữ liệu chi tiết giữa các module.
- `setup/benchmark.md` — Kết quả đo đạc hiệu năng thực tế phần cứng Jetson Nano & TensorRT.
- `docs/report/` — Nơi lưu minh chứng, ảnh chụp lỗi, log demo cho báo cáo môn học.
