# PLAYBOOK — Bạn điều khiển, agent làm việc

Bạn là người điều phối. Claude = trợ lý (viết ticket, viết test khóa, audit, commit, phân loại việc).
Antigravity = người code (`/perception` `/slam` `/il-policy` `/integration`) và tóm tắt paper (`/researcher`).

---

## 1. Vòng đời một ticket (luôn đúng thứ tự này)

```
 (1) Có vấn đề        →  Claude: /triage <mô tả>        → biết giao role nào
 (2) Mở ticket        →  Claude: /ticket <mô tả>        → ticket + locked test, status READY
 (3) Giao code        →  Antigravity: /<role> TK-XXX    → Flash code + REPORT
 (4) Kiểm tra         →  Claude: /audit TK-XXX          → dán REPORT kèm theo → PASS / REJECT
 (5a) PASS            →  Claude: /close TK-XXX          → commit local, ticket DONE
 (5b) REJECT          →  Antigravity: dán phần "Blocking findings" (mẫu §3.4) → quay lại (4)
```

**3 luật bạn tự giữ:**
1. Không giao ticket còn `DRAFT` (chưa có test khóa) — Flash sẽ "pass" mà không ai đo được.
2. Mỗi role chỉ chạy **1 ticket một lúc**. Hai role khác nhau chạy song song được nếu khác file.
3. Không để Antigravity commit. Chỉ commit sau `/audit` PASS.

---

## 2. Việc mới → giao ai? (tra bảng trước, không chắc thì `/triage`)

| Triệu chứng / file liên quan | Giao | Ghi chú |
|---|---|---|
| Map trôi, xoay bị lệch, tường "chạy theo" xe, `motion/planning/slam/`, `cartographer/` | `/slam` | |
| Né người kém, policy, train, ONNX, `imitation_learning/` | `/il-policy` | |
| Camera, YOLO, IPM, người không hiện trên map, `perception/` | `/perception` | |
| Backend không chạy, dashboard, lệnh motor, bridge, `dashboard/backend/`, `jetbot_bridge.py` | `/integration` | Chỉ role này được sửa `coordinator.py` |
| **Test cũ ghi cứng số / test sai** | **Claude** | Coder không bao giờ tự sửa thước đo của mình |
| `setup/robot_dynamics.json`, đo đạc xe | **Bạn** | Mở khóa: `chmod u+w setup/robot_dynamics.json`, sửa xong `chmod 444` |
| Copy file lên Jetson, SSH, chạy motor | **Claude** (bạn duyệt) | Chỉ Supervisor được đụng phần cứng |
| Đọc paper, hỏi "phương pháp X có hợp xe mình không" | Bạn tìm paper → `/researcher` → Claude kiểm trích dẫn | |
| Coder báo "file này ngoài Allowed files" | `/triage` | Thường thành ticket mới cho role sở hữu file đó |
| Không biết | `/triage` | |

---

## 3. Prompt mẫu (copy–paste)

### 3.1 Gửi Claude — có vấn đề mới
```
/triage Xe quay tại chỗ 360° xong thì map bị xoay lệch khoảng 20 độ, tường bị nhân đôi.
```

### 3.2 Gửi Claude — mở ticket
```
/ticket TK-SLAM-03 — viết locked test quay 360° (drift ≤ 5 cm / 3°) và chuyển READY.
```

### 3.3 Gửi Antigravity — giao việc
```
/slam TK-SLAM-03
Đọc ticket trong agents-doc/TICKETS.md. Làm đúng Coder loop trong AGENTS.md.
Chạy lệnh nghiệm thu TRƯỚC khi sửa và dán output thô. Kết thúc bằng khối ## REPORT.
```

### 3.4 Gửi Antigravity — bị REJECT, sửa lại
```
/slam TK-SLAM-03 — lần 2.
Auditor REJECT với các lỗi sau (phải sửa hết, không sửa file ngoài Allowed files):
<dán phần "Blocking findings" của auditor>
Chạy lại lệnh nghiệm thu, dán output thô, kết thúc bằng ## REPORT.
```

### 3.5 Gửi Claude — Flash làm xong
```
/audit TK-SLAM-03
<dán nguyên khối ## REPORT của Antigravity>
```

### 3.6 Gửi Claude — PASS rồi
```
/close TK-SLAM-03
```

### 3.7 Gửi Claude — xe thật chạy sai
Trước khi chạy, trên Jetson luôn ghi log:
```bash
cd ~/03-MinhHieu && ./run_bridge.sh 192.168.1.8 2>&1 | tee ~/bridge-$(date +%F-%H%M).log
```
Rồi:
```
/triage Xe thật: <triệu chứng, lúc nào xảy ra>. Log: ~/bridge-<ngày-giờ>.log trên Jetson.
```

### 3.8 Gửi Antigravity — nhờ tóm tắt paper
```
/researcher
Câu hỏi: <ví dụ: deskew scan 2D LiDAR khi xe quay nhanh, không có encoder>.
Nguồn: <link arXiv / PDF bạn tìm>.
Ghi vào docs/research/<chủ-đề>.md đúng mẫu.
```
Sau đó: `/audit research docs/research/<chủ-đề>.md` gửi Claude để kiểm trích dẫn.

---

## 4. Dấu hiệu Flash đang làm sai — dừng ngay

- Sửa `tests/acceptance/`, `setup/robot_dynamics.json`, `AGENTS.md`, `.agents/`, `.claude/`.
- Nói "PASS"/"đã xong" mà không có output thô của lệnh.
- Sửa file ngoài `Allowed files` của ticket.
- Thêm `try/except: pass`, `skip`, nới ngưỡng, đổi seed.
- Viết đường dẫn `01-HieuMe`, `03-MinhHieu`, `05-d500lib-motion` trong code repo (thư mục chết).
- Thử cùng một cách quá 2 lần.

---

## 5. Lệnh hay dùng

```bash
# Backend trên laptop (run_laptop.sh đang trỏ sai venv)
cd ~/Projects/CS532/dashboard/backend && AUTO_START_BRIDGE=0 ../../.venv/bin/python run.py --host 0.0.0.0 --port 8000
```
```bash
# Kiểm tra backend còn khởi động được
cd ~/Projects/CS532/dashboard/backend && ../../.venv/bin/python -c "import app.main"
```
```bash
# Test khóa IL
cd ~/Projects/CS532 && .venv/bin/python tests/acceptance/test_il_sim_success.py -v
```
```bash
# Test PWM + contour
cd ~/Projects/CS532/dashboard && PYTHONPATH=../motion:backend ../.venv/bin/python test_path_following_and_modular_architecture.py
```
```bash
# Hiệu chỉnh motor trên Jetson (mở khóa file trước nếu cần)
cd ~/setup && python3 calibrate_motors.py
```

Jetson: `jetbot@192.168.1.18` · bridge chạy từ `~/03-MinhHieu/` · config xe: `~/setup/robot_dynamics.json`.
