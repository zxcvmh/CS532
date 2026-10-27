# Báo cáo kiểm tra dự án CS532 — JetBot Navigation

- **Ngày kiểm tra:** 18/10/2026
- **Nhánh:** `feature/hieu-me-cv-ipm` (working tree sạch)
- **Commit mới nhất:** `08b15d5 fix(slam,planning): incorporate lever-arm odometry, static anchor field, pivot clearance gating & replan debounce`
- **Phạm vi:** toàn bộ repo, chỉ đọc và chạy kiểm tra, **không sửa code**

---

## 1. Tóm tắt

| Hạng mục | Trạng thái |
|---|---|
| Cú pháp Python (`compileall`) | ✅ Không có lỗi |
| Typecheck frontend (`tsc`) | ⚠️ Chưa chạy được vì thiếu `node_modules` |
| Test tự động (17 script, bỏ qua test cần phần cứng) | ❌ **6 qua / 11 lỗi** |
| Bảo mật (secrets, `.env`) | ✅ Không có thông tin nhạy cảm bị commit |
| Cấu trúc repo | ⚠️ Code bị nhân bản nhiều nơi; file nhị phân lớn đang được git theo dõi |

**Kết luận:** chưa sẵn sàng merge hoặc demo ổn định. Có **1 lỗi crash nghiêm trọng** trong odometry và nhiều test điều hướng đang thất bại.

---

## 2. Cấu trúc module

| Thư mục | Phụ trách | Nội dung |
|---|---|---|
| `01-HieuMe` | Hiếu Mè | Computer Vision: YOLO26n / YOLOv8-seg, IPM mặt sàn, camera |
| `02-DuyHieu` | Duy Hiếu | Planning: A*, occupancy grid, controller, DDE/CBF, RVO |
| `03-MinhHieu` | Minh Hiếu | Backend FastAPI + WebSocket, frontend React/Vite/Tailwind, bridge JetBot |
| `05-d500lib-motion` | — | Driver LiDAR D500, SLAM/odometry, planning tích hợp (MPPI) |
| `agents-doc`, `task-team` | — | Tài liệu agent, kế hoạch, ticket |
| `setup`, `scripts` | — | Hiệu chỉnh motor, pin, healthcheck |
| `cad`, `output` | — | Thiết kế tấm gá LiDAR (STEP/STL) |

---

## 3. Kết quả test chi tiết

> Lưu ý: test phải chạy với `PYTHONIOENCODING=utf-8`. Nếu không, các script in tiếng Việt sẽ crash vì `UnicodeEncodeError` trên console Windows.

### ✅ Qua (6)
- `05-d500lib-motion/test_dde_cbf.py`
- `02-DuyHieu/test_lidar_odometry.py`
- `02-DuyHieu/test_planning.py`
- `02-DuyHieu/test_planning_local.py`
- `03-MinhHieu/backend/test_lidar_fusion_local.py`
- *(compileall toàn repo)*

### ❌ Lỗi (11)

| # | Test | Lỗi | Loại |
|---|---|---|---|
| 1 | `05/test_lidar_odometry.py` | `UnboundLocalError: lever_arm` | 🔴 Bug code |
| 2 | `03/backend/test_recovery_and_anchor.py` | `UnboundLocalError: lever_arm` | 🔴 Bug code |
| 3 | `05/test_closed_loop_continuous_navigation.py` | Xe vẫn tiến tới 0.096m khi quay 180° | 🟠 Hành vi |
| 4 | `05/test_narrow_corridor_benchmark.py` | Quỹ đạo 1.05m vượt ngưỡng (kỳ vọng ~0.90m) | 🟠 Hành vi / ngưỡng |
| 5 | `05/test_planning_local.py` | Vật cản thấp không được nạp vào costmap | 🟠 Hành vi |
| 6 | `02/test_closed_loop_continuous_navigation.py` | Không về tới đích, còn cách 1.8m | 🟠 Hành vi |
| 7 | `02/test_dde_cbf.py` | Không kích hoạt né tránh phản xạ khi gặp vật cản bất ngờ | 🟠 Hành vi |
| 8 | `02/test_narrow_corridor_benchmark.py` | `OccupancyGridMap` không có `insert_obstacle_clusters` | 🟡 Test lỗi thời |
| 9 | `03/backend/test_closed_loop_continuous_navigation.py` | `ModuleNotFoundError: planning` | 🟡 Sai đường dẫn import |
| 10 | `03/backend/test_narrow_corridor_benchmark.py` | `ModuleNotFoundError: planning` | 🟡 Sai đường dẫn import |
| 11 | Tất cả test (encoding mặc định) | `UnicodeEncodeError` | 🟡 Môi trường |

---

## 4. Lỗi nghiêm trọng nhất 🔴

**File:** `05-d500lib-motion/planning/lidar_odometry.py` (và bản sao giống hệt ở `03-MinhHieu/backend/app/planning/lidar_odometry.py`)

```python
if has_map_geometry and len(curr_cloud) >= 15:
    ...
    lever_arm = 0.08          # dòng 567: chỉ được gán trong nhánh này
    ...
if not scan_to_map_succeeded:
    ...
    max_allowed_step_dist = max(0.06, lever_arm * ...)   # dòng 601: crash nếu nhánh trên không chạy
```

- **Hậu quả:** khi chưa có map (lúc khởi động) hoặc point cloud có ít hơn 15 điểm, odometry **crash** và robot mất định vị.
- **Cách sửa (1 dòng):** đưa `lever_arm = 0.08` lên trước khối `if`, ở cả 2 bản sao.

---

## 5. Vấn đề cấu trúc repo

### 5.1 Code bị nhân bản nhiều nơi
Có hơn 20 nhóm file trùng hệt nhau, ví dụ:

| File | Có mặt tại |
|---|---|
| `planning/*` (astar, controller, occupancy_grid, lidar_odometry, mppi…) | `02-DuyHieu/`, `02-DuyHieu/planning/`, `03-MinhHieu/backend/app/planning/`, `05-d500lib-motion/planning/` |
| `jetbot_bridge.py`, `d500_driver.py` | `03-MinhHieu/`, `05-d500lib-motion/` |
| `floor_ipm.py` | `01`, `03`, `03/backend/app/cv`, `05` |
| `battery.py` | thư mục gốc, `setup/` |

**Rủi ro:** các bản đã bắt đầu lệch nhau (ví dụ `controller.py` của `02` khoảng 33KB, của `03`/`05` khoảng 72KB). Sửa lỗi ở một nơi dễ quên nơi khác, như chính lỗi `lever_arm` ở trên.

### 5.2 File nhị phân lớn đang được git theo dõi
- `01-HieuMe/yolo26n.pt`, `yolo26n.onnx`, `yolov8n-seg.pt`, `yolov8n-seg.onnx` (tổng khoảng 36MB)
- `output/lidar_d500_plate.stl`, `.step`

→ Đề xuất dùng **Git LFS** hoặc tải model bằng script riêng.

### 5.3 Khác
- File `web` ở thư mục gốc chỉ chứa chuỗi `03-MinhHieu`, có vẻ là symlink bị lỗi trên Windows.
- Script dùng một lần còn nằm trong repo: `02-DuyHieu/fix_bridge.py`, `fix_goal_planning.py`, `finish_integration.py`, `update_robot.py`, `optimize_realtime.py`, `deploy_tier1_costmap.py`.
- `.gitignore` còn thiếu `desktop.ini`, `.pytest_cache/`, `.eslintcache`.

---

## 6. Đề xuất hành động

| Ưu tiên | Việc cần làm | Đề xuất người làm | Công sức |
|---|---|---|---|
| P0 | Sửa lỗi `lever_arm` (2 file) | Phụ trách SLAM/`05` | 5 phút |
| P1 | Sửa import `planning` trong test `03/backend`; cập nhật test `02` theo API mới | Minh Hiếu / Duy Hiếu | 30 phút |
| P1 | Điều tra các test hành vi thất bại (#3–#7) | Duy Hiếu | 1–2 ngày |
| P2 | Gom `planning/` về **một nguồn duy nhất**, các module khác import lại | Cả nhóm thống nhất | 0.5–1 ngày |
| P2 | Thêm `PYTHONIOENCODING=utf-8` hoặc `sys.stdout.reconfigure(encoding="utf-8")` vào test | Bất kỳ | 15 phút |
| P3 | Chuyển model/CAD sang Git LFS; dọn script dùng một lần; bổ sung `.gitignore` | Bất kỳ | 1 giờ |
| P3 | `npm ci` rồi `tsc --noEmit` cho frontend | Minh Hiếu | 15 phút |

---

## 7. Cách tái hiện kết quả

```powershell
$env:PYTHONIOENCODING = 'utf-8'
python -m compileall -q . -x "(node_modules|\.git|venv)"
cd 05-d500lib-motion; python test_lidar_odometry.py
```
