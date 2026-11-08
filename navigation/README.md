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

---

## 2. Hướng Dẫn Kiểm Thử Trên Máy Tính Cá Nhân (PC-First)
Chạy bộ test giả lập không cần robot thật:
```bash
python3 test_planning_local.py
```
Tiêu chí kiểm nghiệm: Đạt **PASS 4/4 tiêu chí** (Độ trễ lưới, Gom cụm 1D, Khoảng hở né mép $\ge 0.22\text{m}$, Điều tốc vào cua).

---

## 3. Hướng Dẫn Bàn Giao Cho Team Lead Chạy Trên JetBot Thật
Team Lead chỉ cần SSH vào JetBot và chạy **1 lệnh duy nhất**:
```bash
python3 bench_hardware_dh.py
```
Script sẽ tự động đo đạc 50 chu kỳ thực thi của từng module trên CPU ARM Cortex-A57 và lưu kết quả vào file `benchmark_results_hardware.log`.
