#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Proxemics & Azimuth Calculations (ROS REP-103 CCW Compliant)
=============================================================
CS532 - Advanced AI Robotics (Module Computer Vision - Hieu Me)
Academic Standards:
  - Góc phương vị Azimuth theo chuẩn ROS REP-103 / ISO 8855:
      + Trục +X hướng tới trước, trục +Y sang trái (CCW: Counter-Clockwise dương).
      + Vật thể bên TRÁI mang dấu DƯƠNG (+θ), bên PHẢI mang dấu ÂM (-θ).
  - Vùng an toàn người Asymmetric Gaussian Proxemics (Hall's Proxemics):
      + Trực diện: front_m = 0.80 m
      + Bên sườn: side_m = 0.40 m
      + Phía sau: back_m = 0.30 m
      + Công thức elip: R(θ) = (front * side) / sqrt((side*cosθ)^2 + (front*sinθ)^2)
"""

import math
from typing import Dict, Tuple, Optional, Any, Union


def pixel_to_azimuth_deg(cx: float, img_w: int = 640, fov_deg: float = 160.0) -> float:
    """
    Tính góc phương vị (Azimuth) từ tọa độ hoành độ tâm bounding box (cx).
    Quy chuẩn ROS REP-103 (CCW):
      - Tâm ảnh (cx = img_w / 2): θ = 0.0°
      - Bên trái (cx < img_w / 2): θ > 0 (dương, max +fov/2)
      - Bên phải (cx > img_w / 2): θ < 0 (âm, min -fov/2)
    """
    half_w = float(img_w) / 2.0
    if half_w <= 0.0:
        return 0.0
    ratio = max(-1.0, min(1.0, (half_w - float(cx)) / half_w))
    return round(float(ratio * (fov_deg / 2.0)), 1)


def compute_dynamic_proxemics(
    azimuth_deg: float,
    cls_name: str,
    safety_margins_config: Optional[Dict[str, Any]] = None
) -> Tuple[float, Optional[Dict[str, float]]]:
    """
    Tính toán bán kính vùng an toàn động (Social Bubble / Safety Margin) theo góc lệch azimuth.
    Đối với 'person', áp dụng mô hình Asymmetric Gaussian Proxemics.
    Đối với các vật cản khác (chair, backpack, bottle...), trả về bán kính tĩnh cấu hình.

    Trả về:
      (effective_radius_m, proxemics_envelope_dict_or_None)
    """
    cfg = safety_margins_config or {
        "person": {"front_m": 0.80, "side_m": 0.40, "back_m": 0.30},
        "chair": 0.30,
        "backpack": 0.20,
        "bottle": 0.15,
        "default": 0.25
    }

    c_lower = str(cls_name).lower().strip()
    margin_cfg = cfg.get(c_lower, cfg.get("default", 0.25))

    if c_lower == "person":
        if isinstance(margin_cfg, dict):
            front_m = float(margin_cfg.get("front_m", 0.80))
            side_m = float(margin_cfg.get("side_m", 0.40))
            back_m = float(margin_cfg.get("back_m", 0.30))
        elif isinstance(margin_cfg, (int, float)):
            front_m = float(margin_cfg)
            side_m = round(front_m * 0.5, 2)
            back_m = round(front_m * 0.375, 2)
        else:
            front_m, side_m, back_m = 0.80, 0.40, 0.30

        theta_rad = math.radians(abs(float(azimuth_deg)))
        denom = math.sqrt((side_m * math.cos(theta_rad)) ** 2 + (front_m * math.sin(theta_rad)) ** 2)
        if denom < 1e-6:
            effective_radius = front_m
        else:
            effective_radius = (front_m * side_m) / denom

        envelope = {"front_m": front_m, "side_m": side_m, "back_m": back_m}
        return round(float(effective_radius), 2), envelope

    if isinstance(margin_cfg, (int, float)):
        return round(float(margin_cfg), 2), None

    return 0.25, None


class ProxemicsEngine:
    """Wrapper class phục vụ tính toán khoảng cách xã hội và phương vị cho toàn pipeline."""

    def __init__(self, safety_margins: Optional[Dict[str, Any]] = None, fov_deg: float = 160.0):
        self.safety_margins = safety_margins or {
            "person": {"front_m": 0.80, "side_m": 0.40, "back_m": 0.30},
            "chair": 0.30,
            "backpack": 0.20,
            "bottle": 0.15,
            "default": 0.25
        }
        self.fov_deg = float(fov_deg)

    def get_azimuth(self, cx: float, img_w: int = 640) -> float:
        return pixel_to_azimuth_deg(cx, img_w, self.fov_deg)

    def get_margin(self, azimuth_deg: float, cls_name: str) -> Tuple[float, Optional[Dict[str, float]]]:
        return compute_dynamic_proxemics(azimuth_deg, cls_name, self.safety_margins)
