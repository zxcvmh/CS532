# AI Model Weights & TensorRT Compilation
## CS532 Autonomous JetBot Perception Engine

Thư mục này lưu trữ các trọng số mô hình YOLO (Object Detection) phục vụ nhận diện người và vật thể dẫn đường cho JetBot.

### 1. Trọng Số Sẵn Có
* `yolo26n.onnx`: Mô hình YOLO26-n export định dạng ONNX tiêu chuẩn FP32/FP16 (Shape đầu vào `1x3x640x640`, đầu ra `1x84x8400`).
* `yolo26n.pt`: Mô hình PyTorch Ultralytics gốc.

### 2. Biên Dịch TensorRT FP16 Engine Trên Jetson Nano
Trên phần cứng NVIDIA Jetson Nano 4GB (Maxwell GPU 128 CUDA cores), chạy lệnh sau bằng công cụ `trtexec` để tạo engine tối ưu:

```bash
/usr/src/tensorrt/bin/trtexec \
    --onnx=yolo26n.onnx \
    --saveEngine=yolo26n.engine \
    --fp16 \
    --workspace=256
```

Sau khi biên dịch thành công, đặt tệp `yolo26n.engine` vào thư mục này hoặc `/home/jetbot/yolo26n.engine`. Hệ thống sẽ tự động kích hoạt chế độ GPU thuần đạt tốc độ ~20.4 FPS (độ trễ ~48.25 ms).
