# HỆ THỐNG PHIẾU CÔNG VIỆC (AI AGENT WORK TICKETS) — CS532

*Tài liệu này quản lý các Ticket công việc vi mô (Micro-sprints) của từng Agent. Mỗi ticket bắt buộc có Tiêu chuẩn Nghiệm thu (Definition of Done) rõ ràng để kiểm chứng độc lập theo nguyên lý `harness-engineering`.*

---

## 📌 SPRINT HIỆN TẠI: TRIỂN KHAI GOAL-CONDITIONED DAGGER IMITATION LEARNING (RL-10)

---

### Ticket TK-AI-01: [RESEARCH] Chốt Kiến Trúc Goal-Conditioned DAgger & RVO Expert (RL-10)
- **Agent gán trách nhiệm:** `ai-robotics-researcher-agent`
- **Mức độ ưu tiên:** 🟢 **P0 — Critical (HOÀN THÀNH 100%)**
- **Kết quả nghiệm thu:**
  - [x] Khảo sát kiến trúc học máy chuẩn khóa học RL-10 (Imitation Learning: DAgger vs BC).
  - [x] Giảm thiểu tối đa sự phụ thuộc vào A*: A* chỉ làm nhiệm vụ gợi ý hướng đi hoặc Subgoal vĩ mô; toàn bộ việc lách né vật cản động/tĩnh giao cho DAgger Policy.
  - [x] Chọn `KinodynamicRVOPlanner` làm Expert Oracle tự động sinh nhãn $u^* = (v^*, \omega^*)$.
  - [x] Chốt cấu trúc State Vector 77 chiều: 72 tia LiDAR $[0, 1] + \rho_{\text{subgoal}} + \sin(\phi) + \cos(\phi) + v_{\text{prev}} + \omega_{\text{prev}}$.

---

### Ticket TK-AI-02: [SIM & DATA] Xây dựng Môi trường Sim 2D & Pipeline Tự Thu Thập Trajectory
- **Agent gán trách nhiệm:** `ai-learning-engineer-agent`
- **Mức độ ưu tiên:** 🟢 **P0 — Critical (HOÀN THÀNH 100%)**
- **Kết quả nghiệm thu:**
  - [x] Xây dựng `05-d500lib-motion/planning/il/sim_env.py`: Môi trường 2D mô phỏng động học unicycle vi sai, vật cản tĩnh và người đi bộ cắt ngang.
  - [x] Thuật toán raycasting 72 sectors LiDAR với tốc độ $> 10,000$ steps/s.
  - [x] Tích hợp `05-d500lib-motion/planning/il/expert_oracle.py`: Wrapper Expert Oracle từ `KinodynamicRVOPlanner`.
  - [x] Thu thập thành công 11,827 mẫu dữ liệu trạng thái — hành động chuẩn hóa.

---

### Ticket TK-AI-03: [IL POLICY] Huấn luyện Mạng Goal-Conditioned DAgger & Xuất ONNX
- **Agent gán trách nhiệm:** `ai-learning-engineer-agent`
- **Mức độ ưu tiên:** 🟢 **P1 — High (HOÀN THÀNH 100%)**
- **Kết quả nghiệm thu:**
  - [x] Xây dựng mạng nơ-ron `NavPolicyMLP` trong `05-d500lib-motion/planning/il/dagger_policy.py`: Linear(77, 128) -> LayerNorm -> ReLU -> Linear(128, 64) -> ReLU -> Linear(64, 2) -> Tanh.
  - [x] Chạy vòng lặp DAgger 4 iterations với cơ chế $\beta$-decay ($1.0 \to 0.6 \to 0.36 \to 0.22$) giảm triệt để lỗi Covariate Shift.
  - [x] Lưu checkpoint weights tại `05-d500lib-motion/planning/il/checkpoints/dagger_policy.pth`.
  - [x] Xuất mô hình ONNX tại `05-d500lib-motion/planning/il/checkpoints/dagger_policy.onnx`.
  - [x] Đo đạc thời gian suy luận: **0.082 ms** (nhanh gấp 12 lần mục tiêu < 1.0 ms).

---

### Ticket TK-AI-04: [INTEGRATION] Tích hợp Controller "dagger_il" vào Motion Adapter
- **Agent gán trách nhiệm:** `ai-learning-engineer-agent`
- **Mức độ ưu tiên:** 🟢 **P0 — Critical (HOÀN THÀNH 100%)**
- **Kết quả nghiệm thu:**
  - [x] Cập nhật `03-MinhHieu/backend/app/navigation/motion_adapter.py` hỗ trợ mode `"dagger_il"`.
  - [x] Tự động trích xuất Subgoal dọc theo lộ trình hoặc đích đến cách xe 1.0m (`_extract_subgoal`).
  - [x] Tích hợp bộ lọc an toàn phân tích **CBF-QP Safety Shield** (`CBFSafetyFilter`): Ép vận tốc $v \to 0$ khi gặp vật cản $< 0.16\text{m}$, kích hoạt Pivot Clearance thoát góc kẹt.
  - [x] Giới hạn vận tốc an toàn tuân thủ `setup/robot_dynamics.json`: $v \le 0.12\text{ m/s}$, $|\omega| \le 0.35\text{ rad/s}$ trên sàn gạch men.

---

### Ticket TK-AI-05: [EDGE & HARDWARE DEPLOY] Nạp ONNX / TensorRT Lên Jetson Nano & Test Cảm Biến Thật
- **Agent gán trách nhiệm:** `quality-auditor-debugger-agent`
- **Mức độ ưu tiên:** 🔴 **P0 — Next Immediate Step**
- **Mô tả công việc:**
  Biên dịch `dagger_policy.onnx` thành TensorRT FP16 engine trên Jetson Nano. Kết nối với luồng LiDAR D500 thời gian thực (UART 230400) và camera CSI IMX219 (YOLO26n).
- **Tiêu chuẩn Hoàn thành (Definition of Done):**
  - [ ] Nạp file `.onnx` lên Jetson Nano và build `dagger_policy.engine`.
  - [ ] Đo đạc FPS suy luận trên Jetson Nano đạt $\ge 50\text{ FPS}$ (Latency $\le 20\text{ms}$).
  - [ ] Kiểm tra RAM chiếm dụng trên Jetson Nano $\le 450\text{MB}$.
  - [ ] Chạy `scripts/verify_agent_integrity.py` đạt **100% PASS**.

---

### Ticket TK-AI-06: [DASHBOARD] Trực Quan Hóa Quỹ Đạo AI DAgger & Trạng Thái CBF Shield
- **Agent gán trách nhiệm:** `quality-auditor-debugger-agent`
- **Mức độ ưu tiên:** 🟢 **P2 — Medium**
- **Mô tả công việc:**
  Nâng cấp Web Dashboard React 19 để hiển thị vector vận tốc dự đoán $[v, \omega]$ của DAgger policy và trạng thái can thiệp của CBF Safety Shield lên Canvas 2D.
- **Tiêu chuẩn Hoàn thành (Definition of Done):**
  - [ ] Vẽ mũi tên vận tốc $[v, \omega]$ do AI đề xuất trực tiếp trên Canvas 2D.
  - [ ] Hiển thị badge trạng thái: `AI Policy: NOMINAL` hoặc `Safety Shield: OVERRIDE`.
  - [ ] Duy trì tần số cập nhật WebSocket 10Hz mượt mà.

---

### Ticket TK-AI-07: [SUPERVISOR] Nghiệm Thu Thực Nghiệm Xe Thật Né Người Di Chuyển
- **Agent gán trách nhiệm:** `lead-supervisor-agent`
- **Mức độ ưu tiên:** 🔴 **P0 — Master Gatekeeper**
- **Mô tả công việc:**
  Tổ chức thực nghiệm trên xe JetBot thật: Chạy xe trên sàn gạch men, người đi bộ cắt ngang đường. Đo đạc các chỉ số an toàn, tránh va chạm tuyệt đối và kiểm tra hiện tượng trượt bánh.
- **Tiêu chuẩn Hoàn thành (Definition of Done):**
  - [ ] Tỉ lệ né cản và người đi bộ thành công $\ge 95\%$.
  - [ ] Không có hiện tượng va chạm vật lý phần cứng.
  - [ ] Video demo và ghi nhận kết quả vào `agents-doc/STATE.md`.
