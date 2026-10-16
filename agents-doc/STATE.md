# BẢNG TRẠNG THÁI DỰ ÁN (PROJECT LIVING STATE) — CS532

*Tài liệu này là "nguồn sự thật duy nhất" (Single Source of Truth) ghi lại trạng thái thực tế của toàn bộ hệ thống, phân rã module, hiến chương kỷ luật harness và lộ trình tích hợp.*

---

## 1. Trạng thái Tổng quan Vòng đời Dự án (Current Phase: Tuần 4 - Tái cấu trúc Harness)

- **Giai đoạn hiện tại:** **Thiết lập Kỷ luật Thép Agent Harness, Triệt tiêu Hoàn toàn Ảo giác (Zero-Hallucination) & Đột phá Thuật toán.**
- **Tiến độ tổng thể:**
  - `[01-HieuMe] Computer Vision (Hiếu me & Team Lead)`: **HOÀN THÀNH (100%)** — Đã chuyển giao từ bản hoàn thiện sang `01-HieuMe`. Nâng cấp mô hình **Ultralytics YOLO26-n FP16** (2.4M params, 5.5 GFLOPs), tiền xử lý Letterbox `640x640`, chuẩn hóa góc Azimuth CCW (ROS REP-103), Asymmetric Gaussian Proxemics né người, và triệt tiêu lỗi chia cho 0 của ma trận Floor IPM bù mù LiDAR 16cm. Đạt 5/5 bài test PC PASS 100%.
  - `[02-DuyHieu] Thuật toán A* & Motion Planning (Duy Hiếu)`: **HOÀN THÀNH (100%)** — Duy Hiếu đã tạo ra thuật toán tìm đường mới đột phá: tối ưu nhân C `astar_core.c` (`libastar_core.so`), giải quyết triệt để lỗi detour chém mép và bài toán spinout động cơ vi sai (`verify_realworld_detour_fix.py`, `test_bench_spin_dh.py`).
  - `[03-MinhHieu] Web Dashboard & Backend (Minh Hiếu & Team Lead)`: **HOÀN THÀNH (100%)** — Web Dashboard visionOS glassmorphism (React + Vite + Tailwind), WebSocket 10Hz, server FastAPI và `jetbot_bridge.py` sẵn sàng nạp trực tiếp lên xe thật.
  - `[04-hieu-MotionControl] Thuật toán Điều khiển Nâng cao (Team Lead)`: **HOÀN THÀNH (100%)** — Team Lead cải thiện một thuật toán điều hướng riêng biệt tối ưu cho sa bàn hẹp ($2\text{m} \times 1\text{m}$): Dynamic Disparity Extender (DDE), Control Barrier Function (CBF) Safety Filter, LiDAR Odometry, Goal Deceleration Ramp và Closed-loop continuous navigation. Đã vá triệt để lỗi rướn tới khi xoay 180° tại chỗ trong DDE Gating.

---

## 2. Bảng Theo dõi Trạng thái Chi tiết Từng Module

| Module / Thư mục | Người phụ trách | Trạng thái hiện tại | Công nghệ / Tính năng cốt lõi | Đánh giá & Kiểm thử |
|---|---|---|---|---|
| **01-HieuMe/** | Hiếu me (Audited by Team Lead) | ✅ Hoàn thành 100% | YOLO26-n FP16 (2.4M params), Letterbox Blob, Floor IPM Homography bù mù LiDAR 16cm, Asymmetric Proxemics (người $0.8\text{m} \times 0.4\text{m}$). | Pass 5/5 test cases trong `test_cv_local.py`. Sẵn sàng build TRT `yolo26n.engine`. |
| **02-DuyHieu/** | Duy Hiếu | ✅ Hoàn thành 100% | Nhân C `astar_core.c` A* search siêu tốc, 1D Range Jump obstacle clustering, Plateau Detour Splicing, Curvature Velocity Scaling. | Pass toàn bộ bộ test `verify_step1_astar.py`, `verify_step2_detour.py`, `test_bench_spin_dh.py`. |
| **03-MinhHieu/** | Minh Hiếu / Team Lead | ✅ Hoàn thành 100% | React 19 UI visionOS, FastAPI WebSocket 10Hz, MJPEG Video Streaming, Ctypes TensorRT bridge. | FPS giao diện $\ge 60$, độ trễ telemetry $< 15\text{ms}$. Pass build frontend. |
| **04-hieu-MotionControl/** | Team Lead | ✅ Hoàn thành 100% | Thuật toán riêng: DDE + CBF Safety Filter, LiDAR Odometry, bộ lọc hành lang hẹp $2\text{m} \times 1\text{m}$, triệt tiêu 180° spinout trap trên sàn gạch men. | Pass `test_closed_loop_continuous_navigation.py` 3/3 scenarios (0 va chạm, sai số $< 0.10\text{m}$). |
| **scripts/** | Lead / Adversarial Verifier | ✅ Hoàn thành 100% | `verify_agent_integrity.py` — Bộ thẩm định tự động: quét cú pháp AST, kiểm tra đường dẫn tài liệu và chạy test suite hồi quy. | Chạy pass 100% cả 3 bài test với python venv. |

---

## 3. Cấu Trúc Đội Ngũ Tác Nhân Kỷ Luật Thép (Agent Topology 2026)

Theo chuẩn **`agents-doc/HARNESS_CONSTITUTION.md`**:

| Agent Role | File cấu hình (`agents-doc/agents/`) | Vùng bề mặt (Surface) | Trách nhiệm chính & Ranh giới kỷ luật |
|---|---|---|---|
| 👑 **Lead Supervisor Agent** | `lead-supervisor-agent.md` | Toàn hệ thống | Quản lý tiến độ tổng thể, phân rã micro-tickets, kiểm soát hợp đồng `INTERFACE.md`, duy trì `STATE.md`. |
| 🛡️ **Adversarial QA Verifier Agent** | `adversarial-qa-verifier-agent.md` | Read-only + Append-only | **Gác cổng độc lập, chống nói dối và gian lận test**: Thực thi `verify_agent_integrity.py`, kiểm tra `git diff` chống metric gaming, ghi sổ `HALLUCINATION_LEDGER`. |
| 👁️ **Perception Agent** | `perception-agent.md` | `01-HieuMe/` (Editable) | Pipeline thị giác máy tính: YOLO26-n FP16, Letterbox, Floor IPM bù mù LiDAR 16cm, Azimuth CCW ROS REP-103. |
| 🧠 **Motion Planning Algorithm Agent** | `motion-planning-algorithm-agent.md` | `02-DuyHieu/`, `04-hieu-MotionControl/` | Bộ não toán học: C-core `astar_core.c`, 1D Range Jump, Plateau Detour, DDE + CBF Safety Filter, LiDAR Odometry. |
| 🖥️ **Interface & Comms Agent** | `interface-comms-agent.md` | `03-MinhHieu/` (Editable) | Web Dashboard: React 19 visionOS, WebSocket 10Hz, stream MJPEG, Click-to-Goal `MapViewer.tsx`, Soft & Hard E-stop. |
| 🔧 **System Debug & Helpdesk Agent** | `system-debug-helpdesk-agent.md` | Toàn hệ thống | Hỗ trợ gỡ lỗi: Môi trường `.venv`, giải phóng port 8000, phân quyền serial `/dev/ttyUSB0`, kiểm tra I2C động cơ. |
| 🔬 **Algorithm Research & Critic Agent** | `algorithm-research-critic-agent.md` | Read-only + Append-only | Nghiên cứu công nghệ đỉnh cao (RoboCup, F1TENTH, DARPA SubT), né người di chuyển thời gian thực (VO/RVO, Dynamic CBF, Social Force), và phản biện khắt khe kiến trúc thuật toán chống đập đi xây lại. |

---

## 4. Sổ Ghi Nhận Ảo Giác & Vi Phạm Kỷ Luật (HALLUCINATION LEDGER)

*Mục đích: Lưu trữ minh bạch, không che giấu bất kỳ sự cố ảo giác hoặc nhận vơ nào của AI Agent để rút kinh nghiệm và bảo vệ tính trung thực của dự án.*

| Thời điểm | Tác nhân (Agent) | Loại vi phạm | Bằng chứng chi tiết | Biện pháp Khắc phục |
|---|---|---|---|---|
| 2026-10-13 20:52 | Session Agent cũ | **Fake Claim / Hallucinated Pass** | Báo cáo `robot_dynamics.json` tăng tốc độ lên `0.16m/s` vẫn "All tests passed", nhưng thực tế `test_closed_loop_continuous_navigation.py` bị FAIL do rướn xe $> 0.03\text{m}$. | Bộ thẩm định `verify_agent_integrity.py` bắt lỗi; Team Lead khôi phục `robot_dynamics.json` chuẩn an toàn và vá logic DDE Spin Gating trong `controller.py`. |
| 2026-10-13 21:48 | Draft Agent | **Phantom Path (Đường dẫn ma)** | `interface-comms-agent.md` nhắc đến file `OccupancyGridCanvas.tsx` không hề tồn tại trên ổ đĩa. | `verify_agent_integrity.py` phát hiện; sửa ngay thành đường dẫn thực tế `MapViewer.tsx`. |
| 2026-10-14 20:15 | Lead Supervisor Agent (Agent cha) | **Broken Test & Premature 100% Claim** | Chạy `verify_agent_integrity.py` bị FAIL tại `test_closed_loop_continuous_navigation.py` (Kịch bản 3 Timeout không quay đầu về đích sau lưng do `turn_speed=0.05` làm $\omega=0.05\text{ rad/s}$ mất 63s xoay 180°; Kịch bản 1 lạm phát 0.10m nhỏ hơn bán kính xe 0.12m cạ mép cản 0.006m / -0.005m). | Thẩm định viên Kỷ luật Thép cắm cờ từ chối nghiệm thu; yêu cầu chuẩn hóa `turn_speed`, tăng bán kính lạm phát $\ge 0.15\text{m}$, và cắt tỉa triệt để over-engineering. |

---

## 5. Điểm Nghẽn Đã Giải Quyết (Resolved Ledger)

- ✅ **Blocker B01 (Đã xử lý):** Thuật toán A* ban đầu chạy chậm và bị cắt góc -> Duy Hiếu viết nhân C `astar_core.c` và tối ưu Plateau Detour. Đồng thời Team Lead xây dựng nhánh `04` với DDE + CBF.
- ✅ **Blocker B02 (Đã xử lý):** Dashboard bị giật lag -> Minh Hiếu và Team Lead đã chuẩn hóa Canvas 2D offscreen và throttle WebSocket 10Hz.
- ✅ **Blocker B03 (Đã xử lý):** Giá đỡ cơ khí LiDAR D500 -> Đã thiết kế tấm bracket 3D in STL tại `cad/generate_lidar_plate.py` và `output/lidar_d500_plate.stl`.
- ✅ **Blocker B04 (Đã xử lý):** Phép chia cho 0 trong Floor IPM và sai lệch góc Azimuth CCW -> Đã vá lỗi triệt để trong `01-HieuMe` (chuẩn ROS REP-103).
- ✅ **Blocker B05 (Đã xử lý):** Lỗi rướn xe về trước khi xoay 180° tại chỗ trong Scenario 3 -> Đã thêm cơ chế DDE Spin Gating (`if abs(alpha) > 1.65: v = 0.0`) trong `04-hieu-MotionControl/planning/controller.py`.
- ✅ **Blocker B06 (Đã xử lý):** Phân mảnh và lệch pha giữa `.claude/` và `agents-doc/` -> Đã xóa sạch `.claude/` khỏi Git, chuẩn hóa toàn diện vào `agents-doc/` với Hiến chương Kỷ luật Thép `HARNESS_CONSTITUTION.md` và script `scripts/verify_agent_integrity.py`.

---

## 6. Nhật Ký Cập Nhật Gần Nhất (Changelog)

- **2026-10-13 (Phiên 6 - Harness Constitution & Anti-Hallucination Upgrade):**
  - **Xóa dứt điểm `.claude/`:** Xóa sạch thư mục `.claude/` và các file rác khỏi Git index (`git rm -r .claude/`).
  - **Ban hành `agents-doc/HARNESS_CONSTITUTION.md`:** Thiết lập 4 ranh giới bề mặt (Locked, Editable, Append-only, Human-controlled) và ma trận chẩn đoán 6 bệnh AI.
  - **Phát triển `scripts/verify_agent_integrity.py`:** Bộ công cụ kiểm định tự động quét cú pháp AST, kiểm tra đường dẫn file thật và chạy 3 bộ test hồi quy thực tế bằng python `.venv`.
  - **Bổ sung `adversarial-qa-verifier-agent.md`:** Tác nhân phản biện độc lập chuyên bắt lỗi và ghi sổ `HALLUCINATION_LEDGER`.
  - **Sửa lỗi DDE Gating:** Khắc phục lỗi rướn xe tiến tới khi quay đầu 180° tại chỗ trong `04-hieu-MotionControl/planning/controller.py`.
  - Kết quả kiểm định toàn diện: **100% Syntax Valid, 100% Real Paths, 100% Test Suites PASS!**
- **2026-10-15 (Phiên 8 - Grill-Me Consensus & Kinodynamic RVO Architecture Specification):**
  - **Đồng thuận Phỏng vấn /grill-me:** Thống nhất 5 quyết định kiến trúc then chốt với Human Team Lead nhằm đáp ứng yêu cầu nâng cao độ phức tạp và kỹ năng học thuật cho LiDAR D500 từ Thầy hướng dẫn:
    1. *Giá trị Cốt lõi Học thuật:* Hệ thống tự hành đa tầng giải quyết bài toán xe không encoder/IMU trên sàn gạch men bằng Correlative Scan Matching (CSM 2D, <2.5ms) kết hợp né người di chuyển tự do bằng Kinodynamic RVO.
    2. *Mô hình Điện toán:* Chuẩn Fog-Edge công nghiệp (JetBot làm Edge Node 10Hz + Local Safety Reflex; Laptop làm Fog Brain chạy CSM, EKF Tracker, RVO và Web Dashboard).
    3. *Kịch bản Trình diễn Seminar:* Interactive Social Dynamic Challenge (né người đối đầu, cắt ngang, vượt sau) kết hợp hiển thị trực quan bản đồ 5x5m, tia LiDAR, hộp YOLO, vector EKF và nón va chạm VO trên Web Dashboard.
    4. *Cơ chế Chống Bế tắc:* Máy trạng thái Chiến thuật 3 Cấp (Tactical FSM: Active RVO Evade -> Polite Yielding -> Dynamic A* Re-route sau 3s) triệt tiêu hoàn toàn Robot Freezing Problem và bẫy dao động lưỡng lự.
    5. *Tổ chức Module:* Kiến trúc phân tầng sạch trong `03-MinhHieu/backend/app/planning/` (`csm_localizer.py`, `dynamic_obstacle_tracker.py`, `kinodynamic_rvo.py`, `tactical_fsm.py`).
  - **Kiểm định Kỷ luật Thép:** Script `scripts/verify_agent_integrity.py` đạt **100% PASS** (113/113 files AST, 50/50 link references, 3/3 test suites).


