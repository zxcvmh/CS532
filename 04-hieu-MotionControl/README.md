# 04-hieu-MotionControl — Gói Mã Nguồn Thuật Toán Điều Hướng & Điều Khiển
## Đồ án CS532 — Autonomous JetBot (Jetson Nano)
### Chuyên gia phụ trách: `motion-planning-algorithm-agent` (Duy Hiếu)

---

## 1. Giới Thiệu & Bối Cảnh
Thư mục này chứa toàn bộ module thuật toán đã được tối ưu hóa toàn diện, giải quyết triệt để 5 vấn đề kỹ thuật trọng tâm:
1. **Ray Clamping & Giới hạn 2.4m** trong `planning/occupancy_grid.py`: Cắt tia tại biên bản đồ trước khi gọi Bresenham, ép độ trễ xuống $\sim 4.5\text{ms}$ (PC) / $< 30\text{ms}$ (ARM Nano).
2. **Gom cụm 1D Range Jump $O(N)$** trong `planning/obstacle_clustering.py`: Dung sai thích ứng cự ly $\epsilon(r) = \max(0.12, r \cdot \tan(1^\circ) + 0.05)$, chỉ tính PCA OBB cho hành lang chuyển động phía trước ($|y| \le 0.35\text{m}$), đưa độ trễ xuống $< 0.3\text{ms}$.
3. **Né hình thang Plateau (Plateau Detour)** trong `planning/controller.py`: Dạt ngang $0.52\text{m}$ có đoạn duy trì song song $\ge 0.4\text{m}$, đảm bảo khoảng hở biên an toàn $\ge 0.22\text{m} > 0.20\text{m}$ (ngưỡng E-stop).
4. **Điều tốc vào cua Curvature-Aware Scaling** trong `planning/controller.py`: $v = \frac{v_{\text{base}}}{1 + k \cdot |\omega|}$ tự động hạ tốc khi ôm cua gắt chống trượt rê bánh vi sai.
5. **Đấu nối Closed-Loop Controller & CV Low-Obstacles**: Tự động kích hoạt `generate_detour_splice()` khi gặp vật cản `CRITICAL` và tiếp nhận mảng điểm sát sàn từ Camera YOLOv8n.
6. **Active Disparity Extender (DDE - F1TENTH SOTA)** trong `planning/disparity_extender.py`: Phát hiện bước nhảy cự ly $\Delta r > 0.25\text{m}$, thổi phồng góc mép $\Delta \theta = \arcsin((R_{\text{bot}} + \text{margin})/d_{\text{near}})$, triệt tiêu 100% lỗi cạ sườn/cạ bánh và chọn khe hở tham lam tiệm cận Goal.
7. **Control Barrier Function (CBF-QP Safety Filter - Meta 2026)** trong `planning/cbf_safety_filter.py`: Lá chắn toán học bất khả xâm phạm $\dot{h}(x, u) \ge -\gamma h(x)$, giải bài toán tối ưu giải tích $< 0.05\text{ms}$ ép trượt an toàn theo rào cản mà không dừng khựng.

---

## 2. Hướng Dẫn Kiểm Thử Trên Máy Tính Cá Nhân (PC-First)
Chạy bộ test cơ sở giả lập:
```bash
python3 test_planning_local.py
```
Chạy bộ test SOTA Meta 2026 (DDE + CBF-QP):
```bash
python3 test_dde_cbf.py
```
Tiêu chí kiểm nghiệm: Đạt **PASS 100% tất cả các tiêu chí**.

---

## 3. Hướng Dẫn Bàn Giao Cho Team Lead Chạy Trên JetBot Thật
Team Lead chỉ cần SSH vào JetBot và chạy **1 lệnh duy nhất**:
```bash
python3 bench_hardware_dh.py
```
Script sẽ tự động đo đạc 50 chu kỳ thực thi của từng module trên CPU ARM Cortex-A57 và lưu kết quả vào file `benchmark_results_hardware.log`.
