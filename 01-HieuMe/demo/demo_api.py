# server.py
from fastapi import FastAPI, WebSocket
from fastapi.responses import StreamingResponse
import asyncio
import cv2
import json

app = FastAPI()

# Biến toàn cục lưu trữ trạng thái mới nhất từ robot
latest_data = {
    "robot_pose": {"x": 0.0, "y": 0.0, "yaw": 0.0},
    "map": {"width": 100, "height": 100, "resolution": 0.05, "data": []},
    "detections": []
}

@app.websocket("/ws/telemetry")
async def websocket_telemetry(websocket: WebSocket):
    await websocket.accept()
    try:
        while True:
            # Gửi thông tin map & bounding boxes định kỳ (ví dụ 4 Hz)
            await websocket.send_text(json.dumps(latest_data))
            await asyncio.sleep(0.25)
    except Exception:
        pass

def generate_video_stream():
    # Sử dụng GStreamer pipeline để decode phần cứng trên Jetson
    cap = cv2.VideoCapture(0) # Hoặc pipeline nvarguscamerasrc cho CSI camera
    while True:
        ret, frame = cap.read()
        if not ret:
            break
            
        # 1. Chạy YOLO TensorRT inference
        # 2. Vẽ Bounding box + Khoảng cách lên frame
        # cv2.rectangle(...), cv2.putText(...)
        
        _, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + jpeg.tobytes() + b'\r\n')

@app.get("/video_feed")
def video_feed():
    return StreamingResponse(generate_video_stream(), 
                            media_type="multipart/x-mixed-replace; boundary=frame")