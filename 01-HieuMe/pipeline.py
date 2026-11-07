#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
End-to-End Computer Vision Pipeline (Perception & Floor Obstacle Compensation)
=============================================================================
CS532 - Advanced AI Robotics (Perception Engine - Hieu Me)
Chức năng chính:
  1. Đọc luồng camera độ trễ cực thấp (Zero-Buffer Grabber).
  2. Phát hiện đối tượng YOLO (người, ghế, đồ đạc) kèm góc phương vị Azimuth CCW & Social Bubble.
  3. Bù điểm mù LiDAR sát sàn (<16cm) bằng Floor IPM thích ứng (Adaptive Contrast + Morphology).
  4. Trực quan hóa toàn diện HUD thời gian thực:
      - Bounding box & nhãn tọa độ mét của vật cản tầm thấp sát sàn (Low-lying Obstacles).
      - Ranh giới vùng quan sát mặt sàn Floor IPM Zone.
      - Widget Mini BEV Radar 2D hiển thị không gian xung quanh JetBot.
      - Cảnh báo khẩn cấp E-Stop khi vật cản tiếp cận nguy hiểm (<0.35m).
  5. Đóng gói payload JSON theo chuẩn giao tiếp chung của đồ án (agents-doc/INTERFACE.md).
"""

import os
import sys
import time
import math
import json
import argparse
from typing import Dict, List, Tuple, Optional, Any, Union
import numpy as np
import cv2

# Đảm bảo import được các module core
_MODULE_DIR = os.path.dirname(os.path.abspath(__file__))
if _MODULE_DIR not in sys.path:
    sys.path.insert(0, _MODULE_DIR)

from core.frame_grabber import FrameGrabber
from core.floor_ipm import FloorIPMTransformer, draw_bev_radar
from core.yolo_detector import YoloDetector
from core.proxemics import ProxemicsEngine, pixel_to_azimuth_deg, compute_dynamic_proxemics


class CVPipeline:
    """Pipeline thị giác máy tính toàn diện cho JetBot."""

    def __init__(
        self,
        source: Optional[Union[str, int]] = None,
        model_path: Optional[str] = None,
        config_path: Optional[str] = None,
        conf_threshold: float = 0.40,
        enable_grabber: bool = True,
    ):
        self.config_path = config_path or os.path.join(_MODULE_DIR, "config", "camera_config.json")
        self.ipm = FloorIPMTransformer(config_path=self.config_path)
        self.detector = YoloDetector(
            model_path=model_path,
            config_path=self.config_path,
            conf_threshold=conf_threshold
        )
        self.proxemics = ProxemicsEngine(
            safety_margins=self.ipm.config.get("safety_margins_m"),
            fov_deg=self.ipm.fov_deg
        )

        self.grabber: Optional[FrameGrabber] = None
        if enable_grabber:
            self.grabber = FrameGrabber(
                source=source or "auto",
                width=self.ipm.img_w,
                height=self.ipm.img_h,
                fps=20
            )

        self.frame_count = 0
        self.latest_mask: Optional[np.ndarray] = None
        self.latest_detailed_lows: List[Dict[str, Any]] = []

    def process_frame(self, frame: np.ndarray) -> Dict[str, Any]:
        """
        Xử lý 1 khung hình BGR (640x480):
          - Chạy YOLO Detection (TensorRT / ONNX).
          - Trích xuất Low-lying Obstacles (<16cm) qua Floor IPM Homography thích ứng.
          - Tạo danh sách Social Bubbles cho người đi bộ.
          - Đóng gói JSON dictionary chuẩn hóa.
        """
        t0 = time.perf_counter()
        h, w = frame.shape[:2]
        self.frame_count += 1

        # 1. Nhận diện đối tượng (YOLO Detection)
        detections = self.detector.detect(frame)

        # 2. Phân đoạn thích ứng vật cản sàn (<16cm) bù điểm mù LiDAR
        full_obs_mask, detailed_low_obstacles = self.ipm.segment_floor_obstacles(frame, detections)
        self.latest_mask = full_obs_mask
        self.latest_detailed_lows = detailed_low_obstacles
        low_obstacles = [d["metric"] for d in detailed_low_obstacles]

        # 3. Trích xuất Social Bubbles (dành cho người để nạp vào Costmap)
        social_bubbles = []
        for det in detections:
            cname = det.get("class_name", "")
            if cname == "person":
                azimuth_deg = det.get("azimuth_deg", 0.0)
                dist_m = det.get("distance", det.get("estimated_dist", 1.0))
                margin_m = det.get("safety_margin_m", 0.80)

                # Chuyển từ (azimuth, dist) sang tọa độ tương đối robot (x_forward, y_left)
                az_rad = math.radians(azimuth_deg)
                xr = round(dist_m * math.cos(az_rad), 3)
                yr = round(dist_m * math.sin(az_rad), 3)

                social_bubbles.append({
                    "x": xr,
                    "y": yr,
                    "radius_m": margin_m,
                    "azimuth_deg": azimuth_deg,
                    "distance_m": dist_m,
                })

        latency_ms = round((time.perf_counter() - t0) * 1000.0, 2)

        payload = {
            "timestamp": round(time.time(), 3),
            "frame_id": self.frame_count,
            "detections": detections,
            "low_obstacles": low_obstacles[:8],
            "low_obstacles_detail": detailed_low_obstacles[:8],
            "social_bubbles": social_bubbles,
            "latency_ms": latency_ms,
        }
        return payload

    def run_once(self) -> Optional[Dict[str, Any]]:
        """Đọc 1 khung hình từ grabber và thực thi pipeline."""
        if self.grabber is None:
            return None
        ret, frame = self.grabber.read()
        if not ret or frame is None:
            return None
        return self.process_frame(frame)

    def close(self):
        if self.grabber is not None:
            self.grabber.release()


def render_pipeline_hud(
    frame: np.ndarray,
    payload: Dict[str, Any],
    detailed_lows: List[Dict[str, Any]],
    mask: Optional[np.ndarray] = None,
    show_radar: bool = True,
    show_mask: bool = False
) -> np.ndarray:
    """
    Vẽ giao diện hiển thị thông số HUD toàn diện trên khung hình camera:
      - Vùng quan sát mặt sàn Floor IPM Zone (z < 16cm).
      - Bounding box và nhãn tọa độ mét của các vật cản tầm thấp sát sàn (màu cam/đỏ).
      - Bounding box của các đối tượng YOLO (màu xanh lá).
      - Widget Mini BEV Radar hiển thị không gian xung quanh xe.
      - Thanh trạng thái cảnh báo khi có vật cản tiếp cận nguy hiểm.
    """
    disp = frame.copy()
    h, w = disp.shape[:2]

    # 1. Overlay Mask nhị phân bán trong suốt nếu bật chế độ xem mask
    if show_mask and mask is not None:
        mask_color = np.zeros_like(disp)
        mask_color[mask > 0] = (0, 0, 255)  # Màu đỏ cho mask vật cản
        disp = cv2.addWeighted(disp, 0.75, mask_color, 0.25, 0)

    # 2. Vẽ ranh giới vùng quan sát mặt sàn (Floor IPM Zone)
    roi_y = int(h * 0.65)
    cv2.line(disp, (0, roi_y), (w, roi_y), (255, 160, 0), 1, cv2.LINE_AA)
    cv2.putText(
        disp,
        "▼ VUNG QUAN SAT MAT SAN FLOOR IPM (LiDAR Blind Spot: z < 16cm)",
        (10, roi_y - 6),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.40,
        (255, 160, 0),
        1,
        cv2.LINE_AA
    )

    # 3. Vẽ các đối tượng nhận diện YOLO (Xanh lá)
    for d in payload.get("detections", []):
        b = d["bbox"]
        cname = d["class_name"]
        conf = d["confidence"]
        dist = d["distance"]
        az = d["azimuth_deg"]

        cv2.rectangle(disp, (b[0], b[1]), (b[2], b[3]), (0, 255, 80), 2)
        label = f"{cname} {conf:.2f} (Dist:{dist}m, Az:{az:+.1f}°)"
        cv2.putText(disp, label, (b[0], max(20, b[1] - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 80), 1, cv2.LINE_AA)

        if cname == "person":
            prox = d.get("safety_margin_m", 0.80)
            cv2.putText(disp, f"[Social Bubble: {prox}m]", (b[0], b[3] + 16),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 220, 255), 1, cv2.LINE_AA)

    # 4. Vẽ các vật cản tầm thấp sát sàn LOW-OBSTACLES (Màu Cam / Đỏ nổi bật)
    has_emergency = False
    for idx, obs in enumerate(detailed_lows):
        bx1, by1, bx2, by2 = obs["bbox"]
        xm, ym = obs["metric"]
        dist = obs["distance_m"]
        u_con, v_con = obs["contact_px"]

        is_close = dist < 0.35
        if is_close:
            has_emergency = True
            box_color = (0, 0, 255)      # Đỏ khẩn cấp
            txt_color = (0, 0, 255)
        else:
            box_color = (0, 140, 255)    # Cam cảnh báo
            txt_color = (0, 180, 255)

        # Vẽ viền bao quanh vật cản sàn
        cv2.rectangle(disp, (bx1, by1), (bx2, by2), box_color, 2)

        # Đánh dấu điểm tiếp xúc sàn (ground contact point)
        cv2.circle(disp, (u_con, v_con), 5, (0, 255, 255), -1, cv2.LINE_AA)
        cv2.circle(disp, (u_con, v_con), 7, box_color, 1, cv2.LINE_AA)

        # Nhãn thông tin chi tiết: Thứ tự + Tọa độ mét + Cự ly
        tag = f"LOW-OBS #{idx+1} [X:{xm:+.2f}m, Y:{ym:+.2f}m] Dist:{dist:.2f}m"
        # Nền đen phía sau chữ để dễ đọc trên nền sáng
        (tw, th), _ = cv2.getTextSize(tag, cv2.FONT_HERSHEY_SIMPLEX, 0.42, 1)
        ty = max(24, by1 - 6)
        cv2.rectangle(disp, (bx1, ty - th - 3), (bx1 + tw + 6, ty + 3), (20, 20, 20), -1)
        cv2.putText(disp, tag, (bx1 + 3, ty), cv2.FONT_HERSHEY_SIMPLEX, 0.42, txt_color, 1, cv2.LINE_AA)

    # 5. Thanh cảnh báo đỏ trên cùng nếu có vật cản sát mũi xe (< 0.35m)
    if has_emergency:
        cv2.rectangle(disp, (0, 0), (w, 30), (0, 0, 220), -1)
        cv2.putText(
            disp,
            "⚠️ CANH BAO: VAT CAN SAN SAT MUI XE (<0.35m) — KICH HOAT PHAN XA DUNG KHAN CAP!",
            (14, 21),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.46,
            (255, 255, 255),
            2,
            cv2.LINE_AA
        )

    # 6. Widget Mini BEV Radar ở góc trên bên phải
    if show_radar:
        radar_widget = draw_bev_radar(
            payload.get("detections", []),
            detailed_lows,
            size=170,
            max_range_m=2.0
        )
        rw, rh = radar_widget.shape[1], radar_widget.shape[0]
        rx = w - rw - 10
        ry = 38 if has_emergency else 10
        if ry + rh < h:
            disp[ry:ry+rh, rx:rx+rw] = radar_widget

    # 7. Hộp thông số trạng thái ở góc trên bên trái
    status_y = 48 if has_emergency else 16
    cv2.putText(
        disp,
        f"Frame: {payload['frame_id']} | Latency: {payload['latency_ms']:.1f}ms",
        (10, status_y),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.44,
        (0, 255, 200),
        1,
        cv2.LINE_AA
    )
    cv2.putText(
        disp,
        f"YOLO: {len(payload.get('detections', []))} | Low Obs (<16cm): {len(detailed_lows)}",
        (10, status_y + 18),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.44,
        (0, 255, 200),
        1,
        cv2.LINE_AA
    )
    cv2.putText(
        disp,
        "[M] Bat/Tat Mask  |  [B] Bat/Tat Radar  |  [Q] Thoat",
        (10, status_y + 36),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.36,
        (170, 180, 190),
        1,
        cv2.LINE_AA
    )

    return disp


def main():
    parser = argparse.ArgumentParser(description="CS532 Computer Vision Pipeline")
    parser.add_argument("-s", "--source", default="auto", help="Nguon video: auto, webcam, http://.../api/stream/mjpeg, hoac file anh/video")
    parser.add_argument("-c", "--conf", type=float, default=0.40, help="Nguong tin cay YOLO (mac dinh: 0.40)")
    parser.add_argument("--infer-interval", type=int, default=1, help="Khoang cach chay suy luan AI (1: moi frame, 2: moi 2 frame giup giam tai CPU va tang do muot)")
    parser.add_argument("--no-display", action="store_true", help="Khong mo cua so GUI")
    args = parser.parse_args()

    print("=" * 70)
    print("🤖 CS532 COMPUTER VISION PIPELINE — MODULAR RUNNER")
    print(f"   Nguon du lieu: {args.source}")
    print(f"   Nguong YOLO:   {args.conf}")
    print("   Phim tat GUI:  [M] Toggle Mask  |  [B] Toggle Radar  |  [Q] Thoat")
    print("=" * 70)

    pipeline = CVPipeline(source=args.source, conf_threshold=args.conf, enable_grabber=True)

    show_radar = True
    show_mask = False
    cached_payload = None
    cached_detailed_lows = []
    frame_idx = 0

    try:
        while True:
            ret, frame = pipeline.grabber.read()
            if not ret or frame is None:
                time.sleep(0.005)
                continue

            frame_idx += 1
            if cached_payload is None or (frame_idx % max(1, args.infer_interval) == 0):
                payload = pipeline.process_frame(frame)
                cached_payload = payload
                cached_detailed_lows = payload.get("low_obstacles_detail", [])
            else:
                payload = cached_payload.copy()
                payload["frame_id"] = frame_idx

            dets = payload["detections"]
            lows = payload["low_obstacles"]
            detailed_lows = cached_detailed_lows
            lat = payload["latency_ms"]

            status = f"Frame {payload['frame_id']:04d} | Dets: {len(dets)} | Low Obs (<16cm): {len(lows)} | Latency: {lat:.1f}ms"
            if len(lows) > 0:
                obs_summary = ", ".join([f"[{pt[0]:.2f}m, {pt[1]:.2f}m]" for pt in lows[:3]])
                status += f" | Obs: {obs_summary}"
            print(f"\r{status}", end="", flush=True)

            if not args.no_display:
                disp = render_pipeline_hud(
                    frame=frame,
                    payload=payload,
                    detailed_lows=detailed_lows,
                    mask=pipeline.latest_mask,
                    show_radar=show_radar,
                    show_mask=show_mask
                )

                cv2.imshow("CS532 Computer Vision Pipeline HUD", disp)
                key = cv2.waitKey(1) & 0xFF
                if key in (27, ord('q'), ord('Q')):
                    break
                elif key in (ord('m'), ord('M')):
                    show_mask = not show_mask
                    print(f"\n[HUD] Che do xem Mask: {'BAT' if show_mask else 'TAT'}")
                elif key in (ord('b'), ord('B')):
                    show_radar = not show_radar
                    print(f"\n[HUD] Che do xem Mini BEV Radar: {'BAT' if show_radar else 'TAT'}")

    except KeyboardInterrupt:
        print("\nDa nhan tin hieu dung tu nguoi dung.")
    finally:
        pipeline.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
