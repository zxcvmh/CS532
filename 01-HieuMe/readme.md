# MODULE THỊ GIÁC MÁY TÍNH & BÙ ĐIỂM MÙ LIDAR (COMPUTER VISION & FLOOR IPM)
## ĐỒ ÁN CS532 — AUTONOMOUS JETBOT NAVIGATION & PERCEPTION SYSTEM
**Thành viên phụ trách:** Hiếu me  
**Phạm vi:** Thư mục `01-HieuMe/` (Phát triển PC-First độc lập & Chuyển giao nạp lên JetBot)

---

## 📌 1. Sứ Mệnh & Đóng Góp Học Thuật (Core Contribution)

Cảm biến **LiDAR D500** trên robot quét ngang cố định ở độ cao **$z = 16\text{ cm}$**. Mọi vật cản sát mặt sàn có chiều cao **$< 16\text{ cm}$** (dây điện, ổ cắm, dép, đồ chơi, sách, gờ cửa...) hoàn toàn **"tàng hình"** trước tia laser!

**Module thị giác của Hiếu me** sử dụng camera CSI (IMX219, góc rộng $160^\circ$ FOV, gắn chúc xuống $15^\circ - 20^\circ$) để giải quyết triệt để 2 bài toán:
1. **Phát hiện đối tượng & Vùng an toàn xã hội (Social Bubble):** Nhận diện người (`person`), tính góc lệch phương vị $\theta_{\text{azimuth}} \in [-80^\circ, +80^\circ]$, cự ly ước lượng và gán bán kính né an toàn $0.90\text{ m}$ (theo quy chuẩn Hall's Proxemics).
2. **Bù điểm mù sàn nhà bằng Inverse Perspective Mapping (IPM):** Áp dụng ma trận biến đổi phối cảnh ngược $H_{3 \times 3}$ chuyển ảnh camera góc nghiêng thành ảnh nhìn từ trên xuống (Bird's Eye View - BEV), trích xuất danh sách tọa độ mét thực tế 2D của các vật cản thấp sát sàn:
   $$\text{low\_obstacles} = [[x_1, y_1], [x_2, y_2], ...]$$
   Dữ liệu này nạp trực tiếp vào **Costmap của Planning (Duy Hiếu)** để xe tự bẻ lái né tránh.

---

## 📂 2. Cấu Trúc Thư Mục `01-HieuMe/`

```text
01-HieuMe/
├── camera_config.json        # File cấu hình thông số camera, 4 điểm hiệu chuẩn IPM & Safety Margins
├── floor_ipm.py              # Class FloorIPMTransformer: Biến đổi phối cảnh ngược & chiếu tọa độ mét
├── camera_loader.py          # Class CameraLoader: Đọc video đa nguồn kèm Zero-Buffer Threaded Grabber
├── yolo_seg_runner.py        # Class YoloSegRunner: Suy luận YOLOv8 & đóng gói JSON theo INTERFACE.md
├── pipeline_demo.py          # Script chạy demo đo đạc độ trễ thời gian thực end-to-end
├── test_live_phone.py        # Script test trực tiếp bằng camera điện thoại (HUD thời gian thực)
├── test_cv_local.py          # Script unit test độc lập 3 bài kiểm tra trên PC (Pass 100%)
├── generate_sample_data.py   # Script tự động tạo ảnh sàn 3D giả lập kèm lưới hiệu chuẩn
├── record.py                 # Script quay video từ CSI camera trên JetBot
├── health.py                 # Script kiểm tra môi trường & thiết bị
├── requirements.txt          # Danh sách thư viện Python cần cài đặt
├── sample_data/              # Thư mục chứa ảnh mẫu sàn nhà
│   ├── csi_floor_sample.jpg  # Ảnh sàn phối cảnh chuẩn kèm vật cản mẫu
│   └── jetbot_default_cam.jpg# Bản sao ảnh camera mặc định
└── readme.md                 # Tài liệu hướng dẫn này
```

---

## ⚙️ 3. Cài Đặt Môi Trường (Setup)

Cài đặt các thư viện phụ thuộc trên máy tính cá nhân:
```bash
pip install -r 01-HieuMe/requirements.txt
```
*(Yêu cầu: `opencv-python`, `numpy`, `ultralytics`, `onnx`, `onnxruntime`).*

---

## 🚀 4. Hướng Dẫn Sử Dụng & Kiểm Thử

### 4.1. Chạy Unit Test Độc Lập Trên PC
Kiểm tra nạp ảnh mẫu, trích xuất góc Azimuth, Social Bubble và ma trận IPM:
```bash
$env:PYTHONIOENCODING="utf-8"; python 01-HieuMe/test_cv_local.py
```
*(Kết quả yêu cầu: Pass 3/3 bài kiểm tra).*

---

### 4.2. Chạy Demo Pipeline Đo Đạc Hiệu Năng
Chạy luồng xử lý hoàn chỉnh từ ảnh/camera $\to$ YOLOv8 $\to$ IPM $\to$ JSON:
```bash
$env:PYTHONIOENCODING="utf-8"; python 01-HieuMe/pipeline_demo.py
```
* **Độ trễ ổn định đo được:** $\approx 45.2\text{ ms}$ (tương đương $\sim 22\text{ FPS}$), thỏa mãn trọn vẹn ngân sách thời gian thực $\le 48\text{ ms}$ của đồ án!

---

### 4.3. Test Trực Tiếp Bằng Camera Điện Thoại (Live Stream HUD)
Bạn có thể đặt điện thoại sát sàn (cao $\approx 12\text{ cm}$, chúc xuống $\approx 20^\circ$) để mô phỏng camera JetBot:

* **Cách 1: Qua Cáp USB to Type-C (Khuyên dùng — Không có độ trễ):** //Chưa thử qua
  Cài app **Iriun Webcam** trên điện thoại và laptop $\to$ cắm cáp USB Type-C $\to$ chạy lệnh:
  ```bash
  $env:PYTHONIOENCODING="utf-8"; python 01-HieuMe/test_live_phone.py -s 1
  ```
  *(Đổi `-s 1` thành `-s 2` hoặc `-s 0` tùy số thứ tự webcam máy bạn).*

* **Cách 2: Qua Wi-Fi bằng app IP Webcam:** // Đã thử qua (Đề xuất test bằng cách này)
  Mở app **IP Webcam** trên điện thoại, bấm *Start Server*, rồi chạy lệnh:
  ```bash
  $env:PYTHONIOENCODING="utf-8"; python 01-HieuMe/test_live_phone.py -s "http://192.168.x.x:8080/video"
  ```

> Nhấn phím **'q'** hoặc **ESC** trên cửa sổ video để dừng chương trình.

---

## 📡 5. Chuẩn Hợp Đồng Dữ Liệu Bàn Giao (`agents-doc/INTERFACE.md`)

Mỗi chu kỳ, module xuất ra đúng 1 dictionary JSON sạch sẽ gửi cho Planning và Web Dashboard:

```json
{
  "timestamp": 1727500000.150,
  "detections": [
    {
      "class_name": "person",
      "confidence": 0.88,
      "azimuth_deg": -15.4,
      "estimated_dist": 1.85,
      "safety_margin_m": 0.90
    },
    {
      "class_name": "chair",
      "confidence": 0.75,
      "azimuth_deg": 22.1,
      "estimated_dist": 1.10,
      "safety_margin_m": 0.35
    }
  ],
  "low_obstacles": [
    [0.65, 0.12],
    [0.70, -0.15]
  ]
}
```

* `detections`: Kích hoạt **Social Bubble** để robot tự động giữ khoảng cách lịch sự $0.9\text{ m}$ khi gặp người.
* `low_obstacles`: Nạp trực tiếp tọa độ mét vào **Costmap của Duy Hiếu** để xe tự bẻ lái né các vật cản thấp sát sàn.

---

## 🛠️ 6. Cấu Hình Tùy Biến (`camera_config.json`)

Mọi thông số hiệu chuẩn vật lý được tách riêng tại [camera_config.json](camera_config.json):
* `mount_height_m`: Chiều cao camera đặt trên xe (mặc định: $0.12\text{ m}$).
* `pitch_angle_deg`: Góc nghiêng camera chúc xuống (mặc định: $20.0^\circ$).
* `source_points`: 4 tọa độ pixel của hình chữ nhật hiệu chuẩn trên sàn. Khi Team Lead chụp ảnh thực tế có thước đo trên sàn, chỉ cần cập nhật 4 tọa độ này là toàn bộ hệ thống khớp ngay lập tức mà không phải sửa mã nguồn!
