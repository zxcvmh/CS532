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

## 2. CẤU TRÚC ĐỘI NGŨ 5+1 AGENT (AGENT TOPOLOGY)
Mỗi agent phụ trách độc quyền một thư mục, không sửa code chéo:
1. `robot-setup-agent` (`setup/`): Kiểm tra phần cứng, I2C motor, CSI camera, battery monitor.
2. `perception-agent` (`perception/`): Inference YOLOv8n TensorRT FP16, tính góc lệch tâm \theta_{azimuth} \in [-80^\circ, +80^\circ].
3. `planning-control-agent` (`planning/`): Bộ não điều hướng — Lập Occupancy Grid Log-odds 2D, tìm đường A* 8 hướng, Path Smoothing, Pure Pursuit bám quỹ đạo, phanh khẩn cấp reflex 10Hz. *(Agent đang đối thoại chính)*.
4. `interface-comms-agent` (`web/`): Web Dashboard hiển thị bản đồ Canvas 2D, camera stream, click chọn goal.
5. `integration-reviewer-agent` (Toàn repo): Thẩm định viên độc lập, đo đạc latency trên Jetson Nano, kiểm tra hợp đồng dữ liệu, là người duy nhất merge vào `main`.
6. `simulation-agent` (`simulation/`): Môi trường mô phỏng 2D Software-in-the-Loop (SIL), raycasting LiDAR ảo giải quyết tình huống thiếu giá đỡ cơ khí.

---

## 3. HIỆN TRẠNG THỰC TẾ & CÁC ĐIỂM NGHẼN (CURRENT STATE & BLOCKERS)
- **Web Dashboard (`03-MinhHieu`):** Người dùng đã chuẩn hóa và làm đẹp xong giao diện website.
- **Cơ khí (Blocker B03):** Robot chưa gắn cố định được LiDAR D500 lên đầu xe (chờ gia cố giá đỡ). Đã có giải pháp tạm thời: kê bánh test bench, gá tạm bằng băng keo 3M/dây rút, hoặc chạy kiểm thử qua SIL của `simulation-agent`.
- **Mã nguồn trên Robot (`ssh://jetbot-at-home/home/jetbot/`):**
  - Đã có 2 nguồn: `obstacle_clustering.py` (chỉ gom cụm laser cục bộ) và thư mục `02-DuyHieu` (chứa toàn bộ gói điều hướng `planning/` và bài test `test_planning.py`).
  - **Quyết định đã chốt:** Đưa gói `02-DuyHieu` cho `integration-reviewer-agent` chạy kiểm thử và đánh giá độc lập.

---

## 4. QUY TRÌNH HÀNH ĐỘNG TIẾP THEO (EXACT NEXT ACTIONS)
1. **Bước 1 (Đánh giá):** Gọi `integration-reviewer-agent` chạy đánh giá gói `02-DuyHieu` (đặc biệt là script `02-DuyHieu/test_planning.py`), đo đạc latency thực thi của A* (< 30ms) và tính toàn vẹn của Occupancy Grid / Pure Pursuit.
2. **Bước 2 (Chuyển giao kết quả):** Đưa kết quả kiểm thử (Log / Báo cáo lỗi / Latency) lại cho `planning-control-agent`.
3. **Bước 3 (Lập Plan tối ưu):** `planning-control-agent` tiếp nhận kết quả để:
   - Tối ưu hóa thuật toán A* (chống cắt góc vật cản, làm mượt góc rẽ 90 độ).
   - Tích hợp giải thuật điều tốc khi vào cua **Curvature-Aware Velocity Scaling** ($v = \frac{v_{\max}}{1 + k|\omega|}$) giải quyết bài toán gia tốc khi rẽ hướng.
   - Đồng bộ hoàn tất vào thư mục chính thức `03-MinhHieu`.

---

## 5. PROMPT KHỞI ĐỘNG SESSION MỚI (COPY TO RUN)
```text
Tôi tiếp tục dự án CS532 Autonomous JetBot.
Hãy nạp toàn bộ ngữ cảnh từ file `agents-doc/BOOTSTRAP.md` và `agents-doc/WORKFLOW.MD`.
Hiện tại:
- Web Dashboard (03-MinhHieu) đã được làm đẹp và chuẩn hóa.
- Đang cần tiến hành đánh giá gói thuật toán điều hướng `02-DuyHieu` (chứa `02-DuyHieu/planning` và `test_planning.py`).
Hãy đóng vai trò phù hợp và hướng dẫn tôi các bước tiếp theo theo đúng workflow đã thống nhất!
```
