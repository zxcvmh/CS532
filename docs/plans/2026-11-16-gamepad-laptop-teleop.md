# Gamepad Laptop Teleoperation Implementation Plan (Phương án 1)

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Cho phép cắm tay cầm vào Laptop và điều khiển JetBot an toàn từ xa thông qua FastAPI Backend (`/drive`, `/stop`) mà không gây xung đột bus I2C với `jetbot_bridge.py` trên Jetson Nano.

**Architecture:** Client-Server Teleop Architecture. Script `teleop_gamepad.py` chạy trên Laptop đọc tín hiệu USB Gamepad qua `pygame` ở tần số 30Hz, tính toán động học vi sai (với deadzone, hệ số trim và cực âm động cơ), sau đó gửi lệnh HTTP POST `/drive` hoặc WebSocket `manual_control` tới Backend cục bộ `http://127.0.0.1:8000`. Backend chuyển tiếp lệnh qua kết nối WebSocket `/ws/robot` xuống `jetbot_bridge.py` trên Jetson để điều khiển động cơ qua bộ dồn kênh an toàn `CommandMux` (có phanh khẩn cấp và phanh phản xạ Reflex Stop).

**Tech Stack:** Python 3.11 (.venv), Pygame, Python standard library (`urllib.request`, `json`), FastAPI Backend (`dashboard/backend/app/api/routes.py`).

**Spec:** [[agents-doc/STATE.md](file:///home/zxcvmh/Projects/CS532/agents-doc/STATE.md)], [[dashboard/jetbot_bridge.py](file:///home/zxcvmh/Projects/CS532/dashboard/jetbot_bridge.py)], [[dashboard/backend/app/api/routes.py](file:///home/zxcvmh/Projects/CS532/dashboard/backend/app/api/routes.py)].

---

## Global Constraints

- `setup/robot_dynamics.json` là LOCKED (chỉ đọc, tuyệt đối không chỉnh sửa).
- Không cài thêm thư viện ngoài (sử dụng thư viện chuẩn `urllib.request` và `pygame` đã có).
- Giữ nguyên 100% tính tương thích ngược: khi không truyền cờ `--remote`, script vẫn hoạt động độc lập ở chế độ Direct Hardware/Mock như cũ.
- Tần số gửi lệnh mạng tối đa $\le 30\text{Hz}$ để tránh làm nghẽn Event Loop của FastAPI Backend.
- Watchdog an toàn: nếu ngắt tay cầm hoặc mất kết nối mạng quá $0.5\text{s}$, tự động phát lệnh `stop` xuống robot.
- Tuân thủ Quy tắc số 8 (`AGENTS.md`): Không thực thi lệnh SSH hay can thiệp phần cứng Jetson trực tiếp.

## Review Focus (5 bẫy lỗi cần phòng ngừa)

1. **Trôi lệnh khi nhả cần (Stale command latch):** Khi người dùng buông cần gạt về điểm 0, client phải gửi ngay lập tức gói tin `robot.stop()` thay vì dừng gửi gói tin làm robot tiếp tục chạy theo quán tính.
2. **Độ trễ mạng tích lũy (Network Buffer Bloat):** Nếu gửi HTTP request dạng chặn (blocking synchronous) ở tần số cao, độ trễ sẽ dồn ứ. Cần sử dụng cơ chế Non-blocking hoặc tối ưu timeout ngắn ($< 0.05\text{s}$).
3. **Phanh khẩn cấp tức thời (E-Stop Priority):** Khi nhấn Nút A (X), lệnh `stop` phải được ưu tiên gửi tức thì mà không qua hàng đợi trung gian.
4. **Sai lệch hệ số tỷ lệ vận tốc (Scale Mismatch):** Tốc độ gửi qua `/drive` phải được chuẩn hóa đúng dải $[-1.0, 1.0]$ và nhân đúng `cruise_speed` theo cấu hình `robot_dynamics.json`.
5. **Mất kết nối Backend đột ngột:** Script client không được crash/văng chương trình nếu Backend tạm thời bận hoặc chưa khởi động.

---

### Task 1: Thiết kế Lớp Adapter `RemoteBackendRobot` trong `teleop_gamepad.py`

**Files:**
- Modify: `teleop_gamepad.py:35-95`
- Test: `tests/test_remote_teleop_gamepad.py`

**Interfaces:**
- Consumes: URL Backend (ví dụ `http://127.0.0.1:8000`).
- Produces: Class `RemoteBackendRobot` cung cấp đầy đủ các interface giống hệt `jetbot.Robot`:
  - `set_motors(left_speed: float, right_speed: float)` -> gửi HTTP POST `/drive` với `{"left": l, "right": r}`
  - `forward(speed: float)` -> gửi HTTP POST `/drive` với `{"left": speed, "right": speed}`
  - `backward(speed: float)` -> gửi HTTP POST `/drive` với `{"left": -speed, "right": -speed}`
  - `left(speed: float)` -> gửi HTTP POST `/drive` với `{"left": -speed, "right": speed}`
  - `right(speed: float)` -> gửi HTTP POST `/drive` với `{"left": speed, "right": -speed}`
  - `stop()` -> gửi HTTP POST `/api/robot/stop` hoặc `/stop`

- [x] **Step 1: Viết Unit Test kiểm tra `RemoteBackendRobot` với Mock HTTP Server**
  Tạo file `tests/test_remote_teleop_gamepad.py` kiểm tra:
  - Khi gọi `set_motors(0.3, 0.3)`, HTTP request nhận được đúng payload `{"left": 0.3, "right": 0.3}`.
  - Khi gọi `stop()`, HTTP request nhận được lệnh dừng.
  - Xử lý timeout an toàn khi Backend không phản hồi.

- [x] **Step 2: Chạy test để xác nhận FAIL (RED)**
  Chạy: `.venv/bin/python -m unittest tests/test_remote_teleop_gamepad.py`
  Kết quả mong đợi: `ImportError` hoặc `NameError: RemoteBackendRobot not defined`.

- [x] **Step 3: Triển khai class `RemoteBackendRobot` vào `teleop_gamepad.py`**
  Sử dụng `urllib.request` với `timeout=0.08` và connection pooling để gửi lệnh siêu tốc không chiếm dụng CPU.

- [x] **Step 4: Chạy test để xác nhận PASS (GREEN)**
  Chạy: `.venv/bin/python -m unittest tests/test_remote_teleop_gamepad.py`
  Kết quả mong đợi: 3/3 tests PASS.

---

### Task 2: Bổ sung tham số dòng lệnh `--remote` và Tối ưu tần số gửi lệnh 30Hz

**Files:**
- Modify: `teleop_gamepad.py:440-475`
- Test: `tests/test_remote_teleop_gamepad.py`

**Interfaces:**
- Consumes: `sys.argv` (`--remote [URL]`, ví dụ `--remote http://127.0.0.1:8000`).
- Produces: Khởi tạo robot dạng `RemoteBackendRobot` thay vì `jetbot.Robot` phần cứng khi có tham số `--remote`.

- [x] **Step 1: Viết test kiểm tra phân giải tham số dòng lệnh CLI**
  Bổ sung test case vào `tests/test_remote_teleop_gamepad.py` kiểm tra cờ `--remote` và `-r`.

- [x] **Step 2: Triển khai logic chọn Robot Adapter theo cờ `--remote`**
  - Nếu có `--remote http://...`: Sử dụng `RemoteBackendRobot(remote_url)`.
  - Nếu không có: Giữ nguyên logic tìm `from jetbot import Robot` (Direct Hardware) hoặc `MockRobot` (Giả lập cục bộ).
  - Tích hợp bộ lọc nén tần số: Chỉ gửi lệnh nếu giá trị vận tốc thay đổi hoặc sau chu kỳ heartbeat $0.2\text{s}$ để tiết kiệm băng thông mạng.

- [x] **Step 3: Xác minh chạy thử nghiệm không lỗi cú pháp**
  Chạy: `.venv/bin/python -m py_compile teleop_gamepad.py`
  Kết quả mong đợi: Exit code 0, không có lỗi cú pháp.

---

### Task 3: Kiểm thử tích hợp End-to-End với Backend đang chạy

**Files:**
- Test: `tests/test_remote_teleop_gamepad.py`
- Manual Check: Terminal chạy `teleop_gamepad.py --remote http://127.0.0.1:8000`

- [x] **Step 1: Chạy toàn bộ test suite của Task 1 & Task 2**
  Chạy: `.venv/bin/python -m unittest discover -s tests -p "test_remote_teleop_*.py"`
  Kết quả mong đợi: 100% tests PASS.

- [x] **Step 2: Dry-run kiểm tra kết nối với Backend nội bộ**
  Chạy thử client ở chế độ bàn phím kết nối tới backend cục bộ:
  `.venv/bin/python teleop_gamepad.py --remote http://127.0.0.1:8000 --keyboard`
  Bấm phím 'w'/'s' và quan sát log phản hồi từ `/drive`.

---

## Tiêu chí Nghiệm thu (Acceptance Criteria)

1. Khi cắm tay cầm vào Laptop và chạy lệnh:
   ```bash
   .venv/bin/python teleop_gamepad.py --remote http://127.0.0.1:8000
   ```
   Chương trình nhận diện tay cầm, đọc cần gạt mượt mà, và gửi lệnh tới Backend với độ trễ $< 15\text{ms}$.
2. Trên Web Dashboard (`http://localhost:8000`), icon robot phản hồi tương ứng theo góc lái của tay cầm.
3. Khi `jetbot_bridge.py` đang chạy trên Jetson, robot di chuyển trên sàn thật theo đúng tay cầm trên Laptop mà **hoàn toàn không bị xung đột bus I2C**.
4. Nút A (X) trên tay cầm lập tức kích hoạt `POST /stop` và phanh dừng robot an toàn.
