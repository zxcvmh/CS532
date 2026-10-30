# MODULE THỊ GIÁC MÁY TÍNH & BÙ ĐIỂM MÙ LIDAR (COMPUTER VISION & FLOOR IPM)
## ĐỒ ÁN CS532 — HỆ THỐNG ĐIỀU HƯỚNG TỰ HÀNH & NÉ VẬT CẢN JETBOT
**Thành viên phụ trách:** Hiếu Me  
**Phạm vi:** Gói phần mềm `01-HieuMe/` (Modular Perception & Floor Obstacle Compensation Engine)

---

## 📌 1. Sứ Mệnh & Đóng Góp Học Thuật

Cảm biến **LiDAR D500** quét ngang 360° cố định ở độ cao $z = 16\text{ cm}$. Mọi vật thể thấp sát mặt sàn ($z < 16\text{ cm}$) như **dây điện, dép, gờ cửa, sách, đồ chơi** hoàn toàn "vô hình" trước tia laser!

**Module thị giác máy tính của Hiếu Me** kết hợp Camera CSI IMX219 (góc rộng 160° FOV, chúc 20°) để giải quyết triệt để 2 bài toán:
1. **Bù điểm mù sàn nhà bằng Inverse Perspective Mapping (IPM):**
   * Áp dụng ma trận biến đổi phối cảnh ngược $H_{3 \times 3}$ chuyển ảnh camera nghiêng thành Bird's Eye View (BEV).
   * Phân đoạn vật cản sàn và chiếu trực tiếp sang tọa độ mét robot $(X, Y)$ với lớp bảo vệ triệt tiêu 100% lỗi chia cho 0 ($Z_{\text{cam}} < 10^{-6}$).
   * Bộ lọc làm mượt thời gian **Temporal Smoothing Filter (EMA $\alpha=0.65$, `min_hits=2`)** chống rung lắc khung gầm và phản xạ khẩn cấp nếu vật cản sát mũi xe $< 0.35\text{m}$.
   * Dữ liệu `low_obstacles` nạp trực tiếp vào Costmap của Planning (`05-d500lib-motion/planning/slam/costmap_inflation.py`) ở tầng Short-TTL để né tránh an toàn mà không làm bẩn bản đồ SLAM.
2. **Nhận diện đối tượng & Vùng an toàn xã hội (Asymmetric Gaussian Social Bubble):**
   * Nhận diện người (`person`) và vật thể dẫn đường bằng YOLO26-n.
   * Tính góc lệch phương vị $\theta_{\text{azimuth}} \in [-80^\circ, +80^\circ]$ chuẩn quốc tế **ROS REP-103 CCW** (bên trái dương, bên phải âm).
   * Gán vùng an toàn xã hội hình elip bất đối xứng: trực diện $0.80\text{ m}$, bên sườn $0.40\text{ m}$, phía sau $0.30\text{ m}$ (Hall's Proxemics).

---

## 📂 2. Cấu Trúc Thư Mục Chuẩn Hóa

```text
01-HieuMe/
├── config/
│   └── camera_config.json      # File cấu hình thông số camera, 4 điểm hiệu chuẩn IPM & Safety Margins
├── core/
│   ├── __init__.py             # Export các module cốt lõi
│   ├── frame_grabber.py        # FrameGrabber: Đọc đa nguồn (CSI, USB, IP MJPEG, ảnh mẫu) kèm Zero-Buffer Thread
│   ├── floor_ipm.py            # FloorIPMTransformer: Biến đổi IPM & trích xuất vật cản sàn (<16cm)
│   ├── yolo_detector.py        # YoloDetector: Suy luận YOLO đa backend (TensorRT/ONNX) + Giải mã tensor vector
│   └── proxemics.py            # Tính toán góc Azimuth (ROS REP-103) & Asymmetric Social Bubble
├── models/
│   ├── yolo26n.onnx            # Trọng số ONNX chuẩn (Input 1x3x640x640, Output 1x84x8400)
│   ├── yolo26n.pt              # Trọng số PyTorch gốc
│   └── README.md               # Hướng dẫn build TensorRT FP16 .engine bằng trtexec
├── sample_data/
│   ├── csi_floor_sample.jpg    # Ảnh sàn phối cảnh chuẩn kèm vật cản mẫu
│   └── jetbot_default_cam.jpg  # Ảnh camera JetBot mặc định
├── pipeline.py                 # Pipeline hoàn chỉnh End-to-End: Frame -> Detections + IPM -> JSON Payload
├── test_cv_pipeline.py         # Bộ kiểm thử Unit Test độc lập 5/5 bài kiểm tra
├── requirements.txt            # Danh sách thư viện Python cần thiết
└── README.md                   # Tài liệu hướng dẫn này
```

---

## 🚀 3. Hướng Dẫn Sử Dụng & Kiểm Thử

### 3.1. Cài Đặt Môi Trường
```bash
pip install -r 01-HieuMe/requirements.txt
```

### 3.2. Chạy Bộ Kiểm Thử Unit Test (Không Cần Phần Cứng Thật)
```bash
python 01-HieuMe/test_cv_pipeline.py
```
*(Kiểm tra 5/5 hạng mục: Frame Grabber, Floor IPM & Zero-Division, Azimuth CCW & Proxemics, Vectorized Tensor Decoding, và Hợp đồng dữ liệu JSON).*

### 3.3. Chạy Pipeline Thời Gian Thực
* **Test với ảnh mẫu / video tĩnh:**
  ```bash
  python 01-HieuMe/pipeline.py -s 01-HieuMe/sample_data/csi_floor_sample.jpg
  ```
* **Test trực tiếp từ luồng MJPEG của JetBot:**
  ```bash
  python 01-HieuMe/pipeline.py -s "http://192.168.1.11:8080/api/stream/mjpeg"
  ```
* **Test với USB Webcam trên Laptop:**
  ```bash
  python 01-HieuMe/pipeline.py -s 0
  ```

---

## 📡 4. Hợp Đồng Dữ Liệu Tích Hợp (Data Contract)

Mỗi chu kỳ, `CVPipeline.process_frame(frame)` xuất ra đúng 1 dictionary JSON sạch sẽ gửi cho Planning và Backend Dashboard:

```json
{
  "timestamp": 1727500000.150,
  "frame_id": 142,
  "detections": [
    {
      "class_id": 0,
      "class_name": "person",
      "confidence": 0.88,
      "azimuth_deg": 14.5,
      "distance": 1.25,
      "estimated_dist": 1.25,
      "safety_margin_m": 0.77,
      "proxemics": {
        "front_m": 0.80,
        "side_m": 0.40,
        "back_m": 0.30
      },
      "bbox": [200, 150, 320, 420]
    }
  ],
  "low_obstacles": [
    [0.45, 0.08],
    [0.55, -0.12]
  ],
  "social_bubbles": [
    {
      "x": 1.21,
      "y": 0.31,
      "radius_m": 0.77,
      "azimuth_deg": 14.5,
      "distance_m": 1.25
    }
  ],
  "latency_ms": 14.2
}
```
