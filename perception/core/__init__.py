#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Core Computer Vision Modules for CS532 Autonomous JetBot
"""

from .floor_ipm import FloorIPMTransformer, TemporalObstacleFilter
from .frame_grabber import FrameGrabber, CameraLoader
from .yolo_detector import YoloDetector, YoloSegRunner, letterbox_blob, decode_yolo_output
from .proxemics import ProxemicsEngine, pixel_to_azimuth_deg, compute_dynamic_proxemics
from .person_lidar_fusion import (
    CameraIntrinsics,
    LidarCluster,
    FusionResult,
    PersonLidarFuser,
    cluster_lidar_360,
    mask_person_ranges,
)

__all__ = [
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
    "CameraIntrinsics",
    "LidarCluster",
    "FusionResult",
    "PersonLidarFuser",
    "cluster_lidar_360",
    "mask_person_ranges",
]
