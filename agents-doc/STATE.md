# BẢNG TRẠNG THÁI DỰ ÁN (PROJECT LIVING STATE) — CS532

*Tài liệu này là "nguồn sự thật duy nhất" (Single Source of Truth) ghi lại trạng thái thực tế của toàn bộ 5 Agent, các điểm nghẽn (blockers) và lộ trình tiếp theo. Mọi Agent và thành viên bắt buộc cập nhật file này sau mỗi phiên làm việc.*

---

## 1. Trạng thái Tổng quan Vòng đời Dự án (Current Phase)

- **Giai đoạn hiện tại:** **Tuần 3–4 — Chuyển giao Tích hợp: Hoàn tất Perception, Đột phá Thuật toán Điều hướng & Né tránh.**
- **Tiến độ tổng thể:**
  - `[0] Robot Setup`: **HOÀN THÀNH (100%)** — JetBot boot OK, TensorRT FP16 YOLOv8n ~19.7 FPS, chuẩn bị giao tiếp LiDAR D500.
  - `[1] Perception (Hiếu me)`: **HOÀN THÀNH (100%)** — Đã hoàn tất pipeline Computer Vision: trích xuất `SemanticDetections` với bán kính an toàn động `safety_margin_m`, giải quyết điểm mù LiDAR 16cm qua phân đoạn mặt sàn IPM xuất danh sách `low_obstacles`.
  - `[2] Planning & Control (Team Lead & Duy Hiếu)`: **ĐANG TÍCH HỢP NÂNG CAO (75%)** — Đã chuẩn bị bộ test PC pass 4/4 tiêu chí; Team Lead tiếp quản phiên `algorithm trùm` để tích hợp dữ liệu cản từ CV vào Costmap, tối ưu quỹ đạo né hình thang Plateau và điều tốc vào cua.
  - `[3] Interface & Comms`: **HOÀN THÀNH (100%)** — Dashboard đạt 60fps mượt mà, phong cách Apple visionOS, WebSocket/MJPEG ổn định.
  - `[4] Integration Review`: **ĐANG TRIỂN KHAI** — Đã tổ chức repository module độc lập, chuẩn bị sẵn sàng cho kiểm thử tích hợp khép kín.

---

## 2. Bảng Theo dõi Trạng thái Chi tiết Từng Module

| Module / Thư mục | Phụ trách | Trạng thái hiện tại | Vấn đề tồn đọng | Bước tiếp theo cần làm |
|---|---|---|---|---|
| **01-HieuMe/** | Hiếu me | ✅ Hoàn thành | Cần Team Lead chụp 5-10 ảnh mẫu camera CSI trên sàn thật để calibrate ma trận IPM. | Bàn giao file `yolov8n-seg.onnx` cho Team Lead build TRT. |
| **02-DuyHieu/** | Team Lead (`algorithm trùm`) & Duy Hiếu | 🟡 In Progress | Cần ghép mảng `low_obstacles` vào Costmap và làm mượt đường né tránh Plateau. | Tinh chỉnh `controller.py` và `occupancy_grid.py` tại phiên `algorithm trùm`. |
| **03-MinhHieu/** | Minh Hiếu / Team Lead | ✅ Stable | Đã sẵn sàng kết nối WebSocket với module Planning. | Chờ ghép nối vòng lặp điều khiển kín. |
| **task-team/** | Team Lead | ✅ Stable | Đã xuất 2 bản kế hoạch chi tiết cho Duy Hiếu & Hiếu me. | Theo dõi nghiệm thu theo các tiêu chí đã đề ra. |

---

## 3. Sổ Nhật ký Vấn đề & Điểm nghẽn (Blocker & Bug Ledger)

### 🔴 Blocker B01: Thuật toán Path Finding trong `planning/` đang bị lỗi
- **Người báo cáo:** Thành viên nhóm
- **Module ảnh hưởng:** `planning/` (do `planning-control-agent` chịu trách nhiệm)
- **Triệu chứng:** Đường đi tìm được bị lỗi (đi xuyên góc vật cản, không hội tụ về đích, hoặc sai lệch hệ tọa độ pixel $\leftrightarrow$ metric).
- **Hành động khắc phục:** `planning-control-agent` tiếp nhận code, phân tích nguyên nhân gốc rễ (root cause analysis) và viết test case kiểm chứng trên grid giả lập.

### 🟡 Blocker B02: Web Dashboard đang bị lag / gián đoạn
- **Người báo cáo:** Thành viên nhóm
- **Module ảnh hưởng:** `web/` (do `interface-comms-agent` chịu trách nhiệm)
- **Triệu chứng:** Giao diện phản hồi chậm, giật khung hình khi render bản đồ hoặc nhận luồng dữ liệu.
- **Hành động khắc phục:** Tối ưu hóa vòng lặp render Canvas 2D (dùng off-screen canvas / throttle 10Hz) và nén luồng truyền tin.

### 🟠 Blocker B03: Thiếu giá đỡ cơ khí LiDAR D500 trên robot
- **Người báo cáo:** Thành viên nhóm
- **Module ảnh hưởng:** Thực địa phần cứng
- **Triệu chứng:** Chưa thể gắn LiDAR lên đầu robot để chạy thử nghiệm thực tế.
- **Hành động khắc phục:** Tách riêng `simulation-agent` phụ trách thư mục `simulation/` để tạo môi trường Software-in-the-Loop (SIL) 2D, phát dữ liệu `LaserScan` ảo giúp `planning` và `web` tiếp tục hoàn thiện mà không bị phụ thuộc vào tiến độ gia công cơ khí.

---

## 4. Nhật ký Cập nhật Gần nhất (Changelog)
- **2026-10-09 (Phiên 3):** Hoàn tất phân rã nhiệm vụ và bàn giao độc lập: Hiếu me hoàn thành pipeline Computer Vision (YOLOv8n-seg + IPM bù mù 16cm LiDAR). Đóng gói toàn bộ repository module độc lập, kiểm thử pass 4/4 bộ test PC cho Duy Hiếu. Team Lead tiếp quản phát triển thuật toán né tránh tại phiên `algorithm trùm`.
- **2026-10-02 (Phiên 2):** Tiếp thu phản hồi về phân tầng tác nhân, rạch ròi ranh giới `planning-control-agent` (chỉ tập trung thuật toán điều hướng A* / Pure Pursuit). Nghiên cứu chuẩn IEEE/ROS 2 và hoàn thiện hồ sơ cho `simulation-agent` phụ trách Software-in-the-Loop (SIL) giải quyết Blocker B03.
- **2026-10-02 (Phiên 1):** Thiết lập hệ thống Harness State, tạo bảng theo dõi tiến độ và phân loại 2 blocker trọng tâm (Path Finding bug & Web Dashboard lag).
