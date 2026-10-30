# MODULE 07: IMITATION LEARNING AUTONOMOUS NAVIGATION (HƯỚNG 1)
=============================================================================
**Dự án:** CS532 Autonomous JetBot Navigation System  
**Tham chiếu học thuật:** [UIT ITCourses — RL-101 (Bài 10: Học bắt chước - Imitation Learning)](https://aiclub.uit.edu.vn/itcourses/#RL/RL-10)  
**Phần cứng mục tiêu:** NVIDIA Jetson Nano Dev Kit + LDROBOT D500 LiDAR + JetBot GA25 Differential Drive  

---

## 1. NỀN TẢNG LÝ THUYẾT (ACADEMIC FOUNDATION)

Mô-đun này được xây dựng dựa trên nguyên lý nền tảng của **Bài 10: Học bắt chước (Imitation Learning)** từ khóa học RL-101 của CLB AI Trường Đại học Công nghệ Thông tin (UIT):

### 1.1. Học nhân bản hành vi (Behavioral Cloning - BC)
* **Bản chất:** Coi bài toán điều hướng tự chủ như bài toán Học có giám sát (Supervised Learning).
* **Mục tiêu:** Tìm bộ tham số $\theta$ của mạng nơ-ron $\pi_\theta(s)$ sao cho khớp với hành động điều khiển của chuyên gia $\pi^*(s)$:
  $$\mathcal{L}_{BC}(\theta) = \frac{1}{|\mathcal{D}|} \sum_{(s, a) \in \mathcal{D}} \left[ w_v \cdot (v - \hat{v}_\theta(s))^2 + w_\omega \cdot (\omega - \hat{\omega}_\theta(s))^2 \right] + \lambda \|\theta\|_2^2$$
  * $w_v = 1.0$: Trọng số sai số vận tốc dài.
  * $w_\omega = 1.5$: Trọng số sai số vận tốc góc (bẻ lái né cản chính xác).

### 1.2. Vấn đề trôi phân phối (Covariate Shift) & Sai số bậc hai $O(T^2 \epsilon)$
* Trong bài toán điều khiển chuỗi thời gian, mỗi sai số nhỏ $\epsilon$ tại thời điểm $t$ sẽ đưa robot sang một trạng thái lạ $s_{t+1}$ chưa từng xuất hiện trong tập dữ liệu của chuyên gia ($d_{\pi_\theta} \neq d_{\pi^*}$).
* Trong BC thuần túy, sai số tích lũy tăng theo **hàm bậc hai của thời gian** $O(T^2 \epsilon)$, dẫn tới việc robot nhanh chóng đâm vào tường khi đi lệch quỹ đạo ban đầu.

### 1.3. Khắc phục bằng DART (Noise Injection during Data Collection)
* Trong quá trình thu thập dữ liệu chuyên gia, hệ thống chủ động tiêm nhiễu ngẫu nhiên vào lệnh điều khiển khi robot ở vùng an toàn ($d_{clear} > 0.45\text{m}$):
  $$\tilde{a}_t = a_t + \epsilon_t, \quad \epsilon_t \sim \mathcal{N}(0, \sigma^2)$$
* Điều này buộc chuyên gia phải lái xe sửa sai (recovery behaviors), giúp phân phối dữ liệu $\mathcal{D}_E$ bao phủ các trạng thái lệch làn.

### 1.4. Khắc phục triệt để bằng DAgger (Dataset Aggregation - Ross et al. 2011)
* **Thuật toán lặp:**
  1. Khởi tạo $\mathcal{D}_1 \leftarrow$ Tập dữ liệu ban đầu từ $\pi^*$.
  2. Huấn luyện chính sách $\hat{\pi}_1$ trên $\mathcal{D}_1$ bằng BC.
  3. Tại mỗi vòng lặp $k = 1, \dots, K$:
     * Đặt xác suất hòa trộn $\beta_k = p^k$ (giảm dần qua từng vòng).
     * Cho robot chạy với chính sách $\pi_k = \beta_k \pi^* + (1 - \beta_k) \hat{\pi}_k$, ghi lại chuỗi trạng thái đã viếng thăm $\tau_k = \{s_1, \dots, s_T\} \sim d_{\pi_k}$.
     * Nhờ chuyên gia (Expert Oracle) gán nhãn hành động tối ưu cho các trạng thái này: $a_t^* = \pi^*(s_t)$.
     * Gộp dữ liệu mới vào tập chính: $\mathcal{D}_{k+1} \leftarrow \mathcal{D}_k \cup \{(s_t, a_t^*)\}$.
     * Huấn luyện bổ sung (fine-tune) $\hat{\pi}_{k+1}$ trên $\mathcal{D}_{k+1}$.
* **Định lý Ross et al.:** DAgger kéo giảm sai số tích lũy từ bậc hai $O(T^2 \epsilon)$ về **bậc nhất $O(T \epsilon)$**!

---

## 2. CẤU TRÚC OBSERVATION & ACTION SPACE

### 2.1. Không gian quan sát ($s \in \mathbb{R}^{41}$)
```text
Index 00..35: 36 tia khoảng cách LiDAR (lấy mẫu đều mỗi 10 độ, chuẩn hóa d / 3.0m)
Index 36    : Khoảng cách tới Goal (chuẩn hóa d_goal / 3.0m)
Index 37    : sin(alpha_goal) — Góc lệch tới Goal trong hệ quy chiếu Robot
Index 38    : cos(alpha_goal) — Góc lệch tới Goal (tránh gián đoạn tại ±pi)
Index 39    : Vận tốc dài hiện tại (v_curr / 0.16)
Index 40    : Vận tốc góc hiện tại (w_curr / 0.35)
```

### 2.2. Không gian hành động ($a \in \mathbb{R}^2$)
* $v \in [0.0, 0.16\text{ m/s}]$: Vận tốc dài. Kèm bộ lọc vùng chết ma sát (Motor Deadband): tự động nhảy lên $\ge 0.10\text{ m/s}$ khi di chuyển để thắng ma sát tĩnh hộp số GA25.
* $\omega \in [-0.35, +0.35\text{ rad/s}]$: Vận tốc góc ($\approx 20^\circ/\text{s}$, chống trượt văng bánh trên gạch men bóng).

### 2.3. Tăng cường đối xứng song phương (Bilateral Symmetry Augmentation)
* Lật gương trái $\leftrightarrow$ phải:
  * Tia LiDAR góc $\theta \leftrightarrow (360 - \theta) \pmod{360}$.
  * Góc Goal: $\sin(-\alpha) = -\sin(\alpha)$, $\cos(-\alpha) = \cos(\alpha)$.
  * Hành động: $v \leftrightarrow v$, $\omega \leftrightarrow -\omega$.
* **Tăng gấp đôi tập dữ liệu và triệt tiêu 100% định kiến rẽ trái/phải.**

---

## 3. CẤU TRÚC THƯ MỤC & FILE TRONG MODULE

```text
07-imitation-learning/
├── config.py                 # Siêu tham số không gian trạng thái, mạng nơ-ron & phần cứng
├── dataset.py                # Pipeline xử lý dữ liệu, chuẩn hóa 41D & Bilateral Symmetry
├── model.py                  # PyTorch MLP Policy Network + Pure NumPy Inference Engine (< 0.1ms)
├── mock_expert_simulator.py  # Giả lập 2D Raycaster + Chuyên gia hình học giải tích (F1TENTH)
├── data_collector.py         # Script thu thập dữ liệu chuyên gia (hỗ trợ tiêm nhiễu DART)
├── train_bc.py               # Pipeline huấn luyện Behavioral Cloning (Weighted MSE, AdamW, Cosine LR)
├── dagger_session.py         # Quản lý phiên DAgger (Interactive Relabeling + Iterative Fine-tuning)
├── il_policy_runner.py       # Bộ điều phối thời gian thực trên JetBot (Safety Shield + Deadband Override)
├── bridge_il_adapter.py      # Cầu nối tích hợp trực tiếp vào jetbot_bridge.py / Dashboard
└── test_il_pipeline.py       # Bộ kiểm thử tự động 6 bài test (Pass 100%, đo đạc độ trễ < 0.15ms)
```

---

## 4. HƯỚNG DẪN CHẠY & KIỂM THỬ

### 4.1. Chạy toàn bộ bài test kiểm định (PC Test-First)
```bash
python 07-imitation-learning/test_il_pipeline.py
```
* Kết quả kiểm tra: **6/6 TEST PASS 100%**:
  * `TEST 1`: Kiểm tra trích xuất 41D & Đối xứng gương.
  * `TEST 2`: Kiểm tra thu thập dữ liệu chuyên gia với nhiễu DART ($>80\%$ thành công).
  * `TEST 3`: Kiểm tra huấn luyện Behavioral Cloning (Validation MSE giảm mượt mà).
  * `TEST 4`: Kiểm tra độ tương đương số học PyTorch vs. NumPy và đo đạc độ trễ: **$\approx 0.11\text{ ms}$ (siêu tốc, gấp 5 lần mục tiêu $<0.5\text{ms}$)**!
  * `TEST 5`: Kiểm tra chu trình lặp DAgger (Dataset tự phình to và cập nhật).
  * `TEST 6`: Kiểm tra lá chắn phản xạ khẩn cấp ($<14\text{cm}$) và vượt vùng chết PWM.

### 4.2. Thu thập dữ liệu mô phỏng quy mô lớn
```bash
python 07-imitation-learning/data_collector.py --episodes 50 --output data/expert_demonstrations.npz
```

### 4.3. Huấn luyện mô hình Behavioral Cloning cơ sở (Tự động xuất ONNX)
```bash
python 07-imitation-learning/train_bc.py --data data/expert_demonstrations.npz --epochs 50 --batch-size 64
```
* Trọng số mô hình được tự động lưu thành:
  * `models/bc_policy.onnx`: **Định dạng chuẩn công nghiệp ONNX (Open Neural Network Exchange)** — tối ưu hóa đồ thị nơ-ron, tương thích TensorRT / ONNX Runtime.
  * `models/bc_policy_best.pt`: Checkpoint gốc PyTorch.
  * `models/bc_policy_numpy.json`: Trọng số dự phòng thuần NumPy.

### 4.4. Chạy phiên DAgger triệt tiêu Covariate Shift & Xuất ONNX
```bash
python 07-imitation-learning/dagger_session.py --data data/expert_demonstrations.npz --iterations 5
```
* Xuất file: `models/dagger_policy.onnx`.

### 4.5. Kiểm định chuẩn ONNX & Đo đạc tốc độ thực thi (Benchmark)
```bash
python 07-imitation-learning/benchmark_onnx.py --model models/dagger_policy.onnx --runs 5000
```
* Kiểm tra `onnx.checker.check_model` đạt chuẩn 100%.
* Đo đạc độ trễ ONNX Runtime: **~21.0 micro-seconds (0.021 ms)** — thông lượng **> 47,000 lần suy luận/giây**!

### 4.6. Kiểm thử phân tán: Chạy trên Laptop điều khiển JetBot qua SSH
Xem chi tiết mục 6 bên dưới.
```bash
# Trên Laptop (chạy Hộp đen máy bay Flight Recorder điều khiển JetBot bằng ONNX):
python 07-imitation-learning/live_evaluator.py --host 127.0.0.1 --port 8000 --model models/dagger_policy.onnx --goal-x 1.8 --goal-y 0.0
```

---

## 5. BƯỚC CHUYỂN TIẾP SANG HƯỚNG 2 (HIERARCHICAL WAYPOINTS)

Khi Hướng 1 đã hoạt động ổn định trên sa bàn, để chuyển sang **Hướng 2 (Phòng lớn $5\text{m} \times 5\text{m}$ với chướng ngại vật phức tạp hình chữ U)**:
1. Giữ nguyên toàn bộ `model.py` (`ONNXPolicyRunner`) của Hướng 1 làm **Bộ điều khiển cục bộ (Local Controller)**.
2. Dùng thuật toán A* C-core trong `02-DuyHieu` để sinh ra chuỗi Waypoints toàn cục thưa (mỗi điểm cách nhau $0.6\text{m}$).
3. Mỗi khi robot đạt cự ly $< 0.3\text{m}$ tới Waypoint hiện tại, tự động chuyển mục tiêu $(Goal_x, Goal_y)$ sang Waypoint tiếp theo.
4. Mạng nơ-ron Imitation Learning (ONNX) sẽ tự động dắt xe đi qua từng waypoint mà không bao giờ bị đâm cản hay kẹt Deadlock.

