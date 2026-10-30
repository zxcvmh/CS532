#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
CS532 Computer Vision & Floor Obstacle Compensation Package
===========================================================
Authors: Hieu Me & CS532 Robotics Team (2026)
"""

from .core.floor_ipm import FloorIPMTransformer, TemporalObstacleFilter
from .core.frame_grabber import FrameGrabber, CameraLoader
from .core.yolo_detector import YoloDetector, YoloSegRunner, letterbox_blob, decode_yolo_output
from .core.proxemics import ProxemicsEngine, pixel_to_azimuth_deg, compute_dynamic_proxemics
from .pipeline import CVPipeline

__all__ = [
    "CVPipeline",
    "FloorIPMTransformer",
    "TemporalObstacleFilter",
    "FrameGrabber",
    "CameraLoader",
    "YoloDetector",
    "YoloSegRunner",
    "letterbox_blob",
    "decode_yolo_output",
    "ProxemicsEngine",
    "pixel_to_azimuth_deg",
    "compute_dynamic_proxemics",
]
