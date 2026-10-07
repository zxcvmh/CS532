# BẢNG TRẠNG THÁI DỰ ÁN (PROJECT LIVING STATE) — CS532

*Tài liệu này là "nguồn sự thật duy nhất" (Single Source of Truth) ghi lại trạng thái thực tế của toàn bộ 5 Agent, các điểm nghẽn (blockers) và lộ trình tiếp theo. Mọi Agent và thành viên bắt buộc cập nhật file này sau mỗi phiên làm việc.*

---

## 1. Trạng thái Tổng quan Vòng đời Dự án (Current Phase)

- **Giai đoạn hiện tại:** **Tuần 2–3 — Kích hoạt Planning & Control và Tinh chỉnh Web Dashboard / Perception.**
- **Tiến độ tổng thể:**
  - `[0] Robot Setup`: **HOÀN THÀNH (100%)** — JetBot boot OK, TensorRT FP16 YOLOv8n ~19.7 FPS, chuẩn bị giao tiếp LiDAR D500.
  - `[1] Perception`: **ĐANG TRIỂN KHAI (50%)** — Đã có luồng nhận diện cơ bản, cần chuẩn hóa output góc phương vị $\theta_{\text{azimuth}}$ và format `SemanticDetections`.
  - `[2] Planning & Control`: **ĐANG KÍCH HOẠT (35%)** — Bạn trong nhóm đã phác thảo thuật toán tìm đường (Path Finding) nhưng còn lỗi; cần fix thuật toán A*, làm mượt đường và tích hợp dữ liệu quét LiDAR D500.
  - `[3] Interface & Comms`: **HOÀN THÀNH (100%)** — Đã tái thiết kế toàn bộ Dashboard theo phong cách Apple visionOS / macOS Glassmorphism, tích hợp macOS Control Center, triệt tiêu lag giật 60fps qua Offscreen Canvas, tách video MJPEG khỏi WebSocket.
  - `[4] Integration Review`: **CHỜ PILOT (0%)** — Sẵn sàng kiểm định khi 3 module hoàn thành pilot đầu tiên.

---

## 2. Bảng Theo dõi Trạng thái Chi tiết Từng Module

| Module / Thư mục | Agent phụ trách | Trạng thái hiện tại | Vấn đề tồn đọng (Active Blockers) | Bước tiếp theo cần làm |
|---|---|---|---|---|
| **setup/** | `robot-setup-agent` | ✅ Stable | Cần kiểm tra cắm thực tế cổng `/dev/ttyUSB0` khi có phần cứng trên tay. | Chạy `python3 setup/healthcheck.py` khi cắm LiDAR D500. |
| **perception/** | `perception-agent` | 🟡 In Progress | Chưa đóng gói output theo schema `SemanticDetections` trong `INTERFACES.md`. | Tính góc lệch tâm $\theta_{\text{azimuth}} \in [-80^\circ, +80^\circ]$ của bounding box. |
| **planning/** | `planning-control-agent` | 🔴 Blocked/Buggy | **Thuật toán Path Finding bị lỗi** (cần chẩn đoán: sai hệ trục tọa độ, rẽ góc $90^\circ$, hoặc rò rỉ ô vật cản). | Thu thập mã nguồn path finding hiện tại, debug và chuẩn hóa thuật toán A* trên NumPy. |
| **web/** | `interface-comms-agent` | ✅ Stable | **Đã giải quyết hoàn toàn Blocker B02:** Dashboard đạt 60fps mượt mà, giao diện Apple visionOS hiện đại. | Tích hợp thử nghiệm với luồng LiDAR thực tế khi kết nối JetBot. |
| **simulation/** | `simulation-agent` | ⚪ Ready to Build | **Chưa có giá đỡ LiDAR cơ khí** $\to$ Cần SIL 2D để test thuật toán và web không cần phần cứng. | Viết `simulation/lidar_simulator.py` và `simulation/virtual_world.py` theo TK-05. |
| **root / tests** | `integration-reviewer-agent` | ⚪ Waiting | Chưa có integration test tự động cho luồng giả lập end-to-end. | Tạo kịch bản mock-test tích hợp các module. |

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
- **2026-10-02 (Phiên 2):** Tiếp thu phản hồi về phân tầng tác nhân, rạch ròi ranh giới `planning-control-agent` (chỉ tập trung thuật toán điều hướng A* / Pure Pursuit). Nghiên cứu chuẩn IEEE/ROS 2 và hoàn thiện hồ sơ cho `simulation-agent` phụ trách Software-in-the-Loop (SIL) giải quyết Blocker B03.
- **2026-10-02 (Phiên 1):** Thiết lập hệ thống Harness State, tạo bảng theo dõi tiến độ và phân loại 2 blocker trọng tâm (Path Finding bug & Web Dashboard lag).
