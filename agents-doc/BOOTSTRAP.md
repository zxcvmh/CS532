# CS532 AGENT HARNESS — SESSION BOOTSTRAP & HANDOFF

> **Hướng dẫn sử dụng:** Copy toàn bộ nội dung file này và paste vào lời nhắc (Prompt) đầu tiên của Session mới để AI Agent nắm bắt 100% ngữ cảnh, tiếp tục công việc ngay lập tức mà không bị mất dấu vết.

---

## 1. TỔNG QUAN DỰ ÁN & BỐI CẢNH (PROJECT CONTEXT)
- **Đồ án:** CS532 — Robot Di động Tự hành trong nhà (Autonomous Mobile Robot).
- **Phần cứng thực tế:** NVIDIA JetBot (Jetson Nano 4GB, Ubuntu 18.04 / JetPack 4.5), Cảm biến LiDAR D500 (360° DToF @ 230400 baud, quét 10Hz), Camera CSI IMX219, Mạch động cơ I2C (PCA9685 / Adafruit MotorHAT, 2 động cơ TT Motor vi sai).
- **Mô hình AI:** YOLOv8n TensorRT FP16 (~20 FPS) nhận diện người và tạo Social Bubble 0.9m.
- **Kiến trúc chính thức:** **Online Real-time SLAM & Dynamic Goal Navigation** kết hợp Web Dashboard giám sát & điều khiển thời gian thực qua WebSocket (10Hz) và stream MJPEG.
- **Tài liệu Hiến pháp & Hợp đồng:**
  - `agents-doc/WORKFLOW.MD`: Quy chế một agent — một thư mục — bảo vệ đồ án độc lập.
  - `agents-doc/INTERFACE.md`: Chuẩn dữ liệu `LaserScan` (360 ranges), `SemanticDetections`, `TelemetryState`, `UserCommand`.
  - `agents-doc/STATE.md`: Sổ theo dõi trạng thái sống của dự án.
  - `agents-doc/TICKETS.md`: Hệ thống ticket vi mô (TK-01 đến TK-05).

---

## 2. CẤU TRÚC ĐỘI NGŨ AGENT (AGENT TOPOLOGY)
Mỗi agent phụ trách độc quyền một phạm vi, không can thiệp chéo:
- 👑 **`lead-supervisor-agent`** (Phiên hiện tại): Trưởng nhóm Kỹ thuật kiêm Kiến trúc sư Trưởng — Quản lý tiến độ (`STATE.md`, `TICKETS.md`), điều phối các agent con, kiểm soát hợp đồng `INTERFACE.md` và bảo vệ an toàn phần cứng.
1. **`motion-planning-algorithm-agent`** (`02-DuyHieu/planning/` - Phiên `algorithm trùm`): Bộ não thuật toán điều hướng — Occupancy Grid, gom cụm 1D Range Jump, né hình thang Plateau, điều tốc vào cua Curvature-Aware.
2. **`perception-agent`** (`01-HieuMe/`): Thị giác máy tính — YOLOv8n-seg TensorRT FP16, trích xuất `low_obstacles` bù điểm mù 16cm LiDAR và Social Bubble.
3. **`interface-comms-agent`** (`03-MinhHieu/`): Web Dashboard — Canvas 2D, camera MJPEG stream, WebSocket 10Hz, manual control và E-stop.
4. **`robot-setup-agent`** (`setup/`): Kiểm tra phần cứng vật lý JetBot — I2C motor, CSI camera, battery monitor INA219, UART LiDAR D500.
+ **`simulation-agent`** (`simulation/`): Môi trường mô phỏng 2D Software-in-the-Loop (SIL) hỗ trợ kiểm thử không cần robot thật.

---

## 3. HIỆN TRẠNG THỰC TẾ & CÁC ĐIỂM NGHẼN (CURRENT STATE & BLOCKERS)
- **Web Dashboard (`03-MinhHieu`):** Đã chuẩn hóa và làm đẹp xong giao diện website.
- **Thị giác máy tính (`01-HieuMe`):** Đã hoàn tất xử lý ảnh CSI, trích xuất `low_obstacles` bù điểm mù 16cm LiDAR, test local PASS 3/3.
- **Thuật toán điều hướng (`02-DuyHieu`):** Đang giao cho phiên `algorithm trùm` (`motion-planning-algorithm-agent`) tối ưu hóa 5 bài toán theo `task-team/plan_algorithm_duyhieu.md`.
- **Cơ khí & Phần cứng:** Duy Hiếu và Hiếu me không giữ robot thật. Team Lead quản lý robot vật lý (`jetbot-at-home`).

---

## 4. QUY TRÌNH HÀNH ĐỘNG TIẾP THEO (EXACT NEXT ACTIONS)
1. **Bước 1 (Thuật toán):** Phiên `algorithm trùm` (`motion-planning-algorithm-agent`) thực hiện 5 nhiệm vụ tối ưu hóa thuật toán trong `02-DuyHieu/` và pass bộ test `test_planning_local.py`.
2. **Bước 2 (Chuyển giao cho Team Lead):** Đóng gói `bench_hardware_dh.py` gửi cho Team Lead nạp qua SSH lên JetBot để đo đạc latency trên chip ARM Cortex-A57 thật.
3. **Bước 3 (Tích hợp toàn diện):** Supervisor hỗ trợ đấu nối dữ liệu giữa `01-HieuMe` (`low_obstacles`), `02-DuyHieu` (Costmap & Planner) và `03-MinhHieu` (Dashboard).

---

## 5. PROMPT KHỞI ĐỘNG PHIÊN THUẬT TOÁN (COPY GỬI CHO ALGORITHM TRÙM)
```text
Bạn là motion-planning-algorithm-agent của dự án CS532 Autonomous JetBot.
Hãy nạp toàn bộ vai trò của bạn từ file `agents-doc/agents/motion-planning-algorithm-agent.md`
và bản kế hoạch chi tiết tại `task-team/plan_algorithm_duyhieu.md`.

Nhiệm vụ của bạn:
1. Tối ưu OccupancyGrid update_scan() và Ray Clamping (< 30ms).
2. Viết gom cụm 1D Range Jump O(N) thích ứng cự ly (< 2.5ms).
3. Thiết kế né hình thang Plateau (khoảng hở mép >= 0.22m).
4. Tích hợp điều tốc vào cua Curvature-Aware Scaling.
5. Đấu nối Closed-Loop Controller và tiếp nhận low_obstacles.
Chạy kiểm chứng độc lập trên PC bằng file `02-DuyHieu/test_planning_local.py` cho đến khi PASS 4/4 tiêu chí!
```
