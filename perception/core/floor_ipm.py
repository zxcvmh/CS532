#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Floor Inverse Perspective Mapping (IPM) & Blind Spot Compensator
=================================================================
CS532 - Advanced AI Robotics (Module Computer Vision - Hieu Me)
Academic Standards & Engineering Details:
  - Biến đổi phối cảnh ngược (IPM) chuyển ảnh camera nghiêng thành Bird's-Eye View (BEV).
  - Giải quyết bài toán điểm mù LiDAR D500 ở độ cao z < 16cm (dây điện, dép, gờ cửa, sách...).
  - Chiếu tọa độ pixel (u, v) sang tọa độ mét thực 2D (X, Y) trong Robot Frame {R}:
      + Trục +X: Hướng tới trước (m)
      + Trục +Y: Sang trái (m)
      + Trục -Y: Sang phải (m)
  - Bảo vệ 100% chống lỗi chia cho 0 (Division-by-Zero Protection khi Z_cam < 1e-6).
  - Bộ lọc làm mượt thời gian Temporal Smoothing Filter (EMA alpha=0.65, min_hits=2, reflex <0.35m).
"""

import os
import sys
import json
import math
from typing import List, Tuple, Dict, Optional, Union, Any
import numpy as np
import cv2


class TemporalObstacleFilter:
    """
    Bộ lọc làm mượt thời gian (Temporal Smoothing & Track Persistence)
    cho các điểm vật cản sàn (low_obstacles):
      - Triệt tiêu nhấp nháy (flickering) do rung lắc khung gầm JetBot.
      - Exponential Moving Average (EMA) 2D: X_t = alpha * X_new + (1 - alpha) * X_old.
      - Xác nhận vật cản chỉ khi xuất hiện liên tục >= min_hits frames.
      - Phản ứng khẩn cấp: nếu cự ly < 0.35m thì kích hoạt né ngay (min_hits = 1).
    """

    def __init__(
        self,
        alpha: float = 0.65,
        min_hits: int = 2,
        max_age: int = 3,
        match_dist_m: float = 0.15
    ):
        self.alpha = float(alpha)
        self.min_hits = int(min_hits)
        self.max_age = int(max_age)
        self.match_dist_m = float(match_dist_m)
        self.tracks: List[Dict[str, Any]] = []

    def update(self, detected_points: List[List[float]]) -> List[List[float]]:
        # 1. Tăng tuổi thọ age cho mọi track hiện có
        for trk in self.tracks:
            trk["age"] += 1

        matched_tracks = set()
        for pt in detected_points:
            px, py = float(pt[0]), float(pt[1])
            best_idx = -1
            best_dist = 999.0

            for idx, trk in enumerate(self.tracks):
                if idx in matched_tracks:
                    continue
                d = math.hypot(px - trk["pos"][0], py - trk["pos"][1])
                if d < self.match_dist_m and d < best_dist:
                    best_dist = d
                    best_idx = idx

            if best_idx >= 0:
                trk = self.tracks[best_idx]
                trk["pos"][0] = round(self.alpha * px + (1.0 - self.alpha) * trk["pos"][0], 3)
                trk["pos"][1] = round(self.alpha * py + (1.0 - self.alpha) * trk["pos"][1], 3)
                trk["hits"] += 1
                trk["age"] = 0
                matched_tracks.add(best_idx)
            else:
                self.tracks.append({
                    "pos": [round(px, 3), round(py, 3)],
                    "hits": 1,
                    "age": 0
                })

        # 2. Xóa các track quá hạn
        self.tracks = [trk for trk in self.tracks if trk["age"] <= self.max_age]

        # 3. Trích xuất các track đã được xác nhận
        confirmed = []
        for trk in self.tracks:
            dist_to_robot = math.hypot(trk["pos"][0], trk["pos"][1])
            if trk["hits"] >= self.min_hits or dist_to_robot < 0.35:
                confirmed.append([trk["pos"][0], trk["pos"][1]])

        confirmed.sort(key=lambda p: math.hypot(p[0], p[1]))
        return confirmed

    def reset(self):
        self.tracks.clear()


class FloorIPMTransformer:
    """Class biến đổi phối cảnh ngược và định vị vật cản sàn sang tọa độ mét robot."""

    def __init__(self, config_path: Optional[str] = None):
        self.config_path = config_path or self._locate_default_config()
        self.config = self._load_config()

        # Thông số camera
        cam_cfg = self.config.get("camera", {})
        self.img_w = int(cam_cfg.get("width", 640))
        self.img_h = int(cam_cfg.get("height", 480))
        self.fov_deg = float(cam_cfg.get("fov_deg", 160.0))
        self.mount_height_m = float(cam_cfg.get("mount_height_m", 0.12))
        self.nominal_pitch_deg = float(cam_cfg.get("pitch_angle_deg", 20.0))
        self.pitch_deg = self.nominal_pitch_deg

        # Kích thước ảnh BEV
        ipm_cfg = self.config.get("ipm", {})
        bev_sz = ipm_cfg.get("bev_size", [300, 300])
        self.bev_w = int(bev_sz[0])
        self.bev_h = int(bev_sz[1])

        # Vùng thực tế mặt sàn hiệu chuẩn
        real_rect = ipm_cfg.get("ground_rect_real_m", {})
        self.rect_width_m = float(real_rect.get("width_m", 0.50))
        self.rect_length_m = float(real_rect.get("length_m", 0.50))
        self.dist_from_base_m = float(real_rect.get("distance_from_base_m", 0.15))

        # Phạm vi không gian BEV (Robot Frame mét)
        self.bev_x_min = 0.0
        self.bev_x_max = 2.00
        self.bev_y_min = -0.75
        self.bev_y_max = 0.75

        # 4 điểm gốc hình thang trên ảnh camera
        self.base_raw_src = ipm_cfg.get("source_points", [
            [128.0, 408.0],
            [512.0, 408.0],
            [608.0, 470.0],
            [32.0, 470.0]
        ])

        # Cấu hình lọc vật cản thấp (< 16cm)
        low_cfg = self.config.get("low_obstacle_filter", {})
        self.min_area_px = int(low_cfg.get("min_contour_area_px", 50))
        self.max_area_px = int(low_cfg.get("max_contour_area_px", 8000))
        self.max_ground_height_cm = float(low_cfg.get("max_ground_height_cm", 16.0))

        # Khởi tạo bộ lọc thời gian Temporal Filter
        temp_cfg = self.config.get("temporal_filter", {})
        if temp_cfg.get("enabled", True):
            self.temporal_filter = TemporalObstacleFilter(
                alpha=float(temp_cfg.get("alpha", 0.65)),
                min_hits=int(temp_cfg.get("min_hits", 2)),
                max_age=int(temp_cfg.get("max_age_frames", 3)),
                match_dist_m=float(temp_cfg.get("match_dist_m", 0.15))
            )
        else:
            self.temporal_filter = None

        self._recompute_homography()

    def _locate_default_config(self) -> str:
        base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        candidates = [
            os.path.join(base_dir, "config", "camera_config.json"),
            os.path.join(base_dir, "camera_config.json"),
        ]
        for c in candidates:
            if os.path.exists(c):
                return c
        return candidates[0]

    def _load_config(self) -> Dict[str, Any]:
        if os.path.exists(self.config_path):
            try:
                with open(self.config_path, "r", encoding="utf-8") as f:
                    return json.load(f)
            except Exception as e:
                print(f"[WARN] Không đọc được config {self.config_path}: {e}")
        return {}

    def _recompute_homography(self):
        """Tính ma trận H và H_metric theo góc pitch camera."""
        delta_pitch_deg = self.pitch_deg - self.nominal_pitch_deg
        fy = (self.img_h / 2.0) / max(math.tan(math.radians(45.0)), 1e-4)
        delta_v = fy * math.tan(math.radians(delta_pitch_deg))

        adjusted_src = []
        for pt in self.base_raw_src:
            adj_u = pt[0]
            adj_v = np.clip(pt[1] - delta_v, 280.0, self.img_h - 1)
            adjusted_src.append([adj_u, adj_v])
        self.src_pts = np.float32(adjusted_src)

        x_far = self.dist_from_base_m + self.rect_length_m
        x_near = self.dist_from_base_m
        y_half = self.rect_width_m / 2.0

        self.ground_metric_pts = np.float32([
            [x_far, y_half],
            [x_far, -y_half],
            [x_near, -y_half],
            [x_near, y_half]
        ])

        # Ma trận Homography trực tiếp: Image Pixel (u, v) -> Robot Metric (X, Y)
        self.H_metric = cv2.getPerspectiveTransform(self.src_pts, self.ground_metric_pts)
        try:
            self.H_metric_inv = np.linalg.inv(self.H_metric)
        except np.linalg.LinAlgError:
            self.H_metric_inv = np.eye(3, dtype=np.float32)

        # Ma trận Homography tạo ảnh BEV
        dst_bev = []
        for pt in self.ground_metric_pts:
            xm, ym = pt[0], pt[1]
            bx = (self.bev_y_max - ym) / (self.bev_y_max - self.bev_y_min) * float(self.bev_w)
            by = (self.bev_x_max - xm) / (self.bev_x_max - self.bev_x_min) * float(self.bev_h)
            dst_bev.append([bx, by])
        self.dst_pts = np.float32(dst_bev)

        self.H = cv2.getPerspectiveTransform(self.src_pts, self.dst_pts)
        try:
            self.H_inv = np.linalg.inv(self.H)
        except np.linalg.LinAlgError:
            self.H_inv = np.eye(3, dtype=np.float32)

    def update_pitch_deg(self, new_pitch_deg: float):
        """Bù trừ góc chúc camera động (IMU pitch compensation) < 0.05ms."""
        self.pitch_deg = float(new_pitch_deg)
        self._recompute_homography()

    def warp_to_bev(self, image: np.ndarray) -> np.ndarray:
        """Biến đổi phối cảnh sang ảnh Bird's-Eye View (BEV)."""
        return cv2.warpPerspective(image, self.H, (self.bev_w, self.bev_h))

    def pixel_to_bev(self, u: float, v: float) -> Tuple[float, float]:
        """Chuyển điểm pixel (u, v) sang pixel BEV có bảo vệ division-by-zero."""
        denom = float(self.H[2, 0] * u + self.H[2, 1] * v + self.H[2, 2])
        if abs(denom) < 1e-7:
            return float(self.bev_w / 2.0), 0.0
        bx = (self.H[0, 0] * u + self.H[0, 1] * v + self.H[0, 2]) / denom
        by = (self.H[1, 0] * u + self.H[1, 1] * v + self.H[1, 2]) / denom
        return float(bx), float(by)

    def bev_to_robot_metric(self, bev_x: float, bev_y: float) -> Tuple[float, float]:
        """Chuyển pixel BEV sang hệ mét robot (+X tiến, +Y trái)."""
        xm = self.bev_x_max - (float(bev_y) / float(self.bev_h)) * (self.bev_x_max - self.bev_x_min)
        ym = self.bev_y_max - (float(bev_x) / float(self.bev_w)) * (self.bev_y_max - self.bev_y_min)
        return round(float(xm), 3), round(float(ym), 3)

    def image_pixel_to_robot_metric(self, u: float, v: float) -> Tuple[float, float]:
        """
        Chuyển trực tiếp pixel camera (u, v) sang tọa độ mét robot (X, Y).
        Có lớp bảo vệ chống chia cho 0: Nếu điểm nằm ở chân trời/trần nhà thì gán cự ly an toàn 999m.
        """
        denom = float(self.H_metric[2, 0] * u + self.H_metric[2, 1] * v + self.H_metric[2, 2])
        if abs(denom) < 1e-6:
            return 999.0, 0.0

        pt = np.array([[[float(u), float(v)]]], dtype=np.float32)
        warped = cv2.perspectiveTransform(pt, self.H_metric)
        xm = float(warped[0][0][0])
        ym = float(warped[0][0][1])

        if xm <= 0.05 or xm > 5.0:
            return 999.0, 0.0

        return round(float(xm), 3), round(float(ym), 3)

    def batch_image_pixels_to_robot_metric(self, points: np.ndarray) -> np.ndarray:
        """Chuyển đổi mảng (N, 2) pixel sang mảng (N, 2) tọa độ mét bằng cv2.perspectiveTransform."""
        if points is None or len(points) == 0:
            return np.empty((0, 2), dtype=np.float32)

        pts = np.asarray(points, dtype=np.float32).reshape(-1, 1, 2)
        warped = cv2.perspectiveTransform(pts, self.H_metric).reshape(-1, 2)

        x_vals = warped[:, 0]
        out_of_bounds = (x_vals <= 0.05) | (x_vals > 5.0)
        warped[out_of_bounds] = [999.0, 0.0]

        return np.round(warped, 3)

    # Alias tương thích
    batch_pixels_to_robot_metric = batch_image_pixels_to_robot_metric

    def extract_low_obstacles_from_mask(
        self,
        obstacle_mask: np.ndarray,
        min_area_px: Optional[int] = None,
        max_area_px: Optional[int] = None,
        apply_filter: bool = True
    ) -> List[List[float]]:
        """
        Trích xuất danh sách tọa độ mét [[x1, y1], [x2, y2], ...] của các vật cản thấp sát sàn.
        Hỗ trợ bộ lọc Temporal Smoothing Filter.
        """
        if min_area_px is None:
            min_area_px = self.min_area_px
        if max_area_px is None:
            max_area_px = self.max_area_px

        if obstacle_mask is None or not np.any(obstacle_mask):
            if apply_filter and self.temporal_filter is not None:
                return self.temporal_filter.update([])
            return []

        contours, _ = cv2.findContours(obstacle_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        detected_points = []

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < min_area_px or area > max_area_px:
                continue

            # Điểm đáy của vật thể (v_bottom) là vị trí tiếp xúc sàn
            pts = cnt.reshape(-1, 2)
            u_center = float(np.mean(pts[:, 0]))
            v_bottom = float(np.max(pts[:, 1]))

            # Chỉ xét vật thể nằm ở nửa dưới ảnh (vùng sàn)
            if v_bottom < float(self.img_h) * 0.60:
                continue

            xm, ym = self.image_pixel_to_robot_metric(u_center, v_bottom)
            if xm < 900.0:
                detected_points.append([xm, ym])

        if apply_filter and self.temporal_filter is not None:
            return self.temporal_filter.update(detected_points)

        detected_points.sort(key=lambda p: math.hypot(p[0], p[1]))
        return detected_points
