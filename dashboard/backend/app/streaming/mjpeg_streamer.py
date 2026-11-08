"""
MJPEG HTTP Video Streaming Module.
Provides low-latency multipart/x-mixed-replace HTTP streaming and snapshot endpoints.
"""
import os
import io
import asyncio
from typing import Optional, Tuple, AsyncGenerator


def create_default_frame(width: int = 640, height: int = 480) -> bytes:
    """Generate default placeholder image for camera stream."""
    asset_path = os.path.join(os.path.dirname(__file__), "../robot/default_camera.jpg")
    if os.path.exists(asset_path):
        try:
            with open(asset_path, "rb") as f:
                return f.read()
        except Exception:
            pass

    try:
        from PIL import Image, ImageDraw
        img = Image.new("RGB", (width, height), color=(15, 18, 25))
        draw = ImageDraw.Draw(img)
        for x in range(0, width, 40):
            draw.line([(x, 0), (x, height)], fill=(25, 30, 42), width=1)
        for y in range(0, height, 40):
            draw.line([(0, y), (width, y)], fill=(25, 30, 42), width=1)
        cx, cy = width // 2, height // 2
        draw.line([(cx - 15, cy), (cx + 15, cy)], fill=(0, 212, 255), width=1)
        draw.line([(cx, cy - 15), (cx, cy + 15)], fill=(0, 212, 255), width=1)
        buf = io.BytesIO()
        img.save(buf, format="JPEG", quality=60)
        return buf.getvalue()
    except Exception:
        pass

    try:
        import cv2
        import numpy as np
        img = np.zeros((height, width, 3), dtype=np.uint8)
        img[:] = (25, 18, 15)
        for x in range(0, width, 40):
            cv2.line(img, (x, 0), (x, height), (42, 30, 25), 1)
        for y in range(0, height, 40):
            cv2.line(img, (0, y), (width, y), (42, 30, 25), 1)
        cx, cy = width // 2, height // 2
        cv2.line(img, (cx - 15, cy), (cx + 15, cy), (255, 212, 0), 1)
        cv2.line(img, (cx, cy - 15), (cx, cy + 15), (255, 212, 0), 1)
        _, buf = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), 65])
        return buf.tobytes()
    except Exception:
        pass

    # 1x1 valid fallback JPEG
    return b"\xff\xd8\xff\xe0\x00\x10JFIF\x00\x01\x01\x01\x00H\x00H\x00\x00\xff\xdb\x00C\x00\x08\x06\x06\x07\x06\x05\x08\x07\x07\x07\t\t\x08\n\x0c\x14\r\x0c\x0b\x0b\x0c\x19\x12\x13\x0f\x14\x1d\x1a\x1f\x1e\x1d\x1a\x1c\x1c $.\' \",#\x1c\x1c(7),01444\x1f\'9=82<.342\xff\xc0\x00\x0b\x08\x00\x01\x00\x01\x01\x01\x11\x00\xff\xc4\x00\x1f\x00\x00\x01\x05\x01\x01\x01\x01\x01\x01\x00\x00\x00\x00\x00\x00\x00\x00\x01\x02\x03\x04\x05\x06\x07\x08\t\n\x0b\xff\xda\x00\x08\x01\x01\x00\x00?\x00\xbf\x00\xff\xd9"


class MjpegStreamer:
    """Manages low-latency MJPEG streaming and frame buffering."""

    def __init__(self):
        self._current_frame: bytes = create_default_frame()
        self.fps: float = 15.0

    def update_frame(self, frame_bytes: Optional[bytes]):
        """Update buffer with new camera JPEG frame."""
        if frame_bytes and len(frame_bytes) > 50:
            self._current_frame = frame_bytes

    def get_frame(self) -> bytes:
        """Get latest camera frame bytes."""
        return self._current_frame

    def get_snapshot(self) -> Tuple[bytes, str]:
        """Return frame and MIME content type."""
        mime = "image/bmp" if self._current_frame.startswith(b"BM") else "image/jpeg"
        return self._current_frame, mime

    async def frame_generator(self) -> AsyncGenerator[bytes, None]:
        """Async generator yielding multipart MJPEG stream."""
        interval = 1.0 / self.fps
        while True:
            try:
                frame = self._current_frame
                mime = "image/bmp" if frame.startswith(b"BM") else "image/jpeg"
                header = f"--frame\r\nContent-Type: {mime}\r\n\r\n".encode("ascii")
                yield (header + frame + b"\r\n")
                await asyncio.sleep(interval)
            except asyncio.CancelledError:
                break
            except Exception:
                await asyncio.sleep(0.1)
