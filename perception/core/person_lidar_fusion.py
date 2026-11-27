#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
perception/core/person_lidar_fusion.py — Person-aware LiDAR clustering & camera fusion (TK-CV-02A / TK-CV-02B).

Projects YOLO person bounding boxes onto calibrated ROS REP-103 CCW LiDAR azimuths
(0° = front, +90° = left, 180° = back, -90°/270° = right; index i = i deg CCW),
clusters 360-ray LiDAR scans with circular wraparound, selects foreground person
clusters (rejecting background walls behind the person), and outputs:
  - person_mask_360 : bool[360] for IL observation v2 (113-D) & CSM beam masking
  - person_flags_36 : float32[36] pooled identically to imitation_learning/dataset.py
"""

from dataclasses import dataclass
import json
import math
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple, Union

import cv2
import numpy as np


def _normalize_deg(angle_deg: float) -> float:
    """Normalize angle in degrees to (-180.0, 180.0]."""
    wrapped = (float(angle_deg) + 180.0) % 360.0 - 180.0
    return 180.0 if wrapped == -180.0 else wrapped


def _angle_in_interval(angle_deg: float, az_min_deg: float, az_max_deg: float) -> bool:
    """Check whether angle_deg falls within [az_min_deg, az_max_deg] on the circle."""
    if az_max_deg < az_min_deg:
        az_min_deg, az_max_deg = az_max_deg, az_min_deg
    half_span = 0.5 * (az_max_deg - az_min_deg)
    if half_span >= 180.0:
        return True
    center = 0.5 * (az_min_deg + az_max_deg)
    diff = abs((float(angle_deg) - center + 180.0) % 360.0 - 180.0)
    return diff <= half_span + 1e-9


@dataclass
class CameraIntrinsics:
    """Pinhole + distortion camera intrinsics with optional mounting yaw offset."""

    image_width: int
    image_height: int
    camera_matrix: np.ndarray
    dist_coeffs: np.ndarray
    camera_yaw_offset_deg: float = 0.0

    @classmethod
    def from_json(
        cls,
        path: Union[str, Path],
        camera_yaw_offset_deg: Optional[float] = None,
    ) -> "CameraIntrinsics":
        """Load intrinsics JSON written by setup/calibrate_camera.py."""
        data = json.loads(Path(path).read_text(encoding="utf-8"))
        mtx = np.asarray(data["camera_matrix"], dtype=np.float64).reshape(3, 3)
        dist = np.asarray(data.get("dist_coeffs", [0.0, 0.0, 0.0, 0.0, 0.0]), dtype=np.float64).ravel()
        yaw_off = (
            float(camera_yaw_offset_deg)
            if camera_yaw_offset_deg is not None
            else float(data.get("camera_yaw_offset_deg", 0.0))
        )
        return cls(
            image_width=int(data["image_width"]),
            image_height=int(data["image_height"]),
            camera_matrix=mtx,
            dist_coeffs=dist,
            camera_yaw_offset_deg=yaw_off,
        )

    @classmethod
    def from_fov(
        cls,
        width: int = 640,
        height: int = 480,
        hfov_deg: float = 160.0,
        camera_yaw_offset_deg: float = 0.0,
    ) -> "CameraIntrinsics":
        """Construct zero-distortion pinhole intrinsics from horizontal FOV."""
        half_fov_rad = math.radians(max(1.0, min(178.0, float(hfov_deg))) * 0.5)
        fx = (float(width) * 0.5) / math.tan(half_fov_rad)
        fy = fx
        cx = float(width) * 0.5
        cy = float(height) * 0.5
        mtx = np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]], dtype=np.float64)
        dist = np.zeros(5, dtype=np.float64)
        return cls(
            image_width=int(width),
            image_height=int(height),
            camera_matrix=mtx,
            dist_coeffs=dist,
            camera_yaw_offset_deg=float(camera_yaw_offset_deg),
        )

    def pixel_to_azimuth_deg(self, u: float, v: Optional[float] = None) -> float:
        """Convert pixel (u, v) to ROS REP-103 CCW azimuth in degrees (left +, right -)."""
        v_coord = float(self.image_height) * 0.5 if v is None else float(v)
        pts = np.array([[[float(u), v_coord]]], dtype=np.float64)
        undist = cv2.undistortPoints(pts, self.camera_matrix, self.dist_coeffs)
        x_norm = float(undist[0, 0, 0])
        return math.degrees(math.atan2(-x_norm, 1.0)) + float(self.camera_yaw_offset_deg)

    def bbox_to_azimuth_interval_deg(
        self, bbox: Sequence[float], pad_deg: float = 0.0
    ) -> Tuple[float, float]:
        """Project [x1, y1, x2, y2] bounding box to (az_min_deg, az_max_deg) CCW."""
        x1, y1, x2, y2 = float(bbox[0]), float(bbox[1]), float(bbox[2]), float(bbox[3])
        v_mid = 0.5 * (y1 + y2)
        az_a = self.pixel_to_azimuth_deg(x1, v_mid)
        az_b = self.pixel_to_azimuth_deg(x2, v_mid)
        return min(az_a, az_b) - float(pad_deg), max(az_a, az_b) + float(pad_deg)


@dataclass
class LidarCluster:
    """1D range-jump cluster extracted from a 360-ray LiDAR scan."""

    cluster_id: int
    ray_indices: List[int]
    min_range_m: float
    median_range_m: float
    center_azimuth_deg: float
    angular_span_deg: float
    estimated_width_m: float
    centroid_xy: Tuple[float, float]
    label: str = "STATIC"
    confidence: float = 0.0


@dataclass
class FusionResult:
    """Output of person-aware LiDAR fusion for downstream IL and CSM."""

    clusters: List[LidarCluster]
    person_clusters: List[LidarCluster]
    person_mask_360: np.ndarray
    person_flags_36: np.ndarray


def _build_cluster(cluster_id: int, ray_indices: List[int], ranges: np.ndarray) -> LidarCluster:
    idx_arr = np.asarray(ray_indices, dtype=np.int32)
    r_vals = ranges[idx_arr]
    min_r = float(np.min(r_vals))
    med_r = float(np.median(r_vals))

    rads = np.deg2rad(idx_arr.astype(np.float64))
    mean_sin = float(np.mean(np.sin(rads)))
    mean_cos = float(np.mean(np.cos(rads)))
    center_az = _normalize_deg(math.degrees(math.atan2(mean_sin, mean_cos)))

    rel_angles = ((idx_arr.astype(np.float64) - center_az + 180.0) % 360.0) - 180.0
    span_deg = float(np.max(rel_angles) - np.min(rel_angles))
    chord_width = 2.0 * med_r * math.sin(math.radians(min(span_deg, 179.0) * 0.5))

    center_rad = math.radians(center_az)
    cx = med_r * math.cos(center_rad)
    cy = med_r * math.sin(center_rad)
    return LidarCluster(
        cluster_id=cluster_id,
        ray_indices=list(ray_indices),
        min_range_m=min_r,
        median_range_m=med_r,
        center_azimuth_deg=center_az,
        angular_span_deg=span_deg,
        estimated_width_m=chord_width,
        centroid_xy=(cx, cy),
    )


def cluster_lidar_360(
    ranges_360: Sequence[float],
    min_range_m: float = 0.12,
    max_range_m: float = 6.0,
    range_jump_m: float = 0.25,
    max_angle_gap_deg: int = 3,
    min_cluster_rays: int = 2,
) -> List[LidarCluster]:
    """Segment a 360-ray LiDAR ring into contiguous 1D range-jump clusters (with 359->0 wraparound)."""
    ranges = np.asarray(ranges_360, dtype=np.float64).ravel()
    if ranges.size != 360:
        raise ValueError(f"Expected 360 LiDAR ranges, got {ranges.size}")

    valid = np.isfinite(ranges) & (ranges >= float(min_range_m)) & (ranges <= float(max_range_m))
    valid_idx = np.flatnonzero(valid)
    if valid_idx.size == 0:
        return []

    raw_groups: List[List[int]] = [[int(valid_idx[0])]]
    for idx_np in valid_idx[1:]:
        idx = int(idx_np)
        prev_idx = raw_groups[-1][-1]
        ang_gap = idx - prev_idx
        r_diff = abs(float(ranges[idx]) - float(ranges[prev_idx]))
        if ang_gap <= max_angle_gap_deg and r_diff <= range_jump_m:
            raw_groups[-1].append(idx)
        else:
            raw_groups.append([idx])

    # Merge wraparound cluster across 359 -> 0 if continuous
    if len(raw_groups) >= 2:
        first_idx = raw_groups[0][0]
        last_idx = raw_groups[-1][-1]
        wrap_gap = (first_idx + 360) - last_idx
        wrap_r_diff = abs(float(ranges[first_idx]) - float(ranges[last_idx]))
        if wrap_gap <= max_angle_gap_deg and wrap_r_diff <= range_jump_m:
            merged = raw_groups[-1] + raw_groups[0]
            raw_groups = [merged] + raw_groups[1:-1]

    clusters: List[LidarCluster] = []
    cid = 0
    for grp in raw_groups:
        if len(grp) < min_cluster_rays:
            continue
        clusters.append(_build_cluster(cid, grp, ranges))
        cid += 1
    return clusters


def pool_person_flags_36(person_mask_360: np.ndarray) -> np.ndarray:
    """Pool 360-ray bool mask to 36 sector flags matching imitation_learning/dataset.py."""
    mask = np.asarray(person_mask_360, dtype=bool).ravel()
    if mask.size != 360:
        raise ValueError(f"Expected 360 mask elements, got {mask.size}")
    sectors = np.roll(mask, 5).reshape(36, 10)
    return sectors.any(axis=1).astype(np.float32)


def mask_person_ranges(
    ranges_360: Sequence[float],
    person_mask_360: np.ndarray,
    fill_value: float = 0.0,
) -> np.ndarray:
    """Return a copy of ranges_360 with person-labeled rays masked out for CSMTracker."""
    out = np.asarray(ranges_360, dtype=np.float32).copy()
    mask = np.asarray(person_mask_360, dtype=bool).ravel()
    if out.size != 360 or mask.size != 360:
        raise ValueError(f"Expected 360-ray arrays, got ranges={out.size}, mask={mask.size}")
    out[mask] = float(fill_value)
    return out


class PersonLidarFuser:
    """Labels LiDAR clusters matching YOLO person boxes while rejecting background walls."""

    def __init__(
        self,
        intrinsics: CameraIntrinsics,
        min_confidence: float = 0.40,
        angular_pad_deg: float = 2.0,
        range_jump_m: float = 0.25,
        max_angle_gap_deg: int = 3,
        min_cluster_rays: int = 2,
        min_range_m: float = 0.12,
        max_person_range_m: float = 4.0,
        max_person_width_m: float = 1.2,
        depth_band_m: float = 0.45,
        min_overlap_ratio: float = 0.30,
    ):
        self.intrinsics = intrinsics
        self.min_confidence = float(min_confidence)
        self.angular_pad_deg = float(angular_pad_deg)
        self.range_jump_m = float(range_jump_m)
        self.max_angle_gap_deg = int(max_angle_gap_deg)
        self.min_cluster_rays = int(min_cluster_rays)
        self.min_range_m = float(min_range_m)
        self.max_person_range_m = float(max_person_range_m)
        self.max_person_width_m = float(max_person_width_m)
        self.depth_band_m = float(depth_band_m)
        self.min_overlap_ratio = float(min_overlap_ratio)

    def fuse(
        self,
        ranges_360: Sequence[float],
        detections: Optional[Iterable[Dict[str, Any]]],
    ) -> FusionResult:
        """Cluster scan and label foreground person clusters falling inside YOLO person boxes."""
        clusters = cluster_lidar_360(
            ranges_360,
            min_range_m=self.min_range_m,
            max_range_m=6.0,
            range_jump_m=self.range_jump_m,
            max_angle_gap_deg=self.max_angle_gap_deg,
            min_cluster_rays=self.min_cluster_rays,
        )
        person_mask_360 = np.zeros(360, dtype=bool)

        if detections:
            for det in detections:
                cname = str(det.get("class_name") or det.get("class") or "").strip().lower()
                if cname != "person":
                    continue
                conf = float(det.get("confidence", 0.0))
                if conf < self.min_confidence:
                    continue
                bbox = det.get("bbox")
                if not bbox or len(bbox) != 4:
                    continue
                if float(bbox[2]) <= float(bbox[0]) or float(bbox[3]) <= float(bbox[1]):
                    continue

                az_min, az_max = self.intrinsics.bbox_to_azimuth_interval_deg(
                    bbox, pad_deg=self.angular_pad_deg
                )
                candidates: List[LidarCluster] = []
                for c in clusters:
                    if c.median_range_m > self.max_person_range_m:
                        continue
                    if c.estimated_width_m > self.max_person_width_m:
                        continue
                    in_center = _angle_in_interval(c.center_azimuth_deg, az_min, az_max)
                    overlap_hits = sum(
                        1 for idx in c.ray_indices if _angle_in_interval(_normalize_deg(idx), az_min, az_max)
                    )
                    overlap_ratio = overlap_hits / max(1, len(c.ray_indices))
                    if in_center or overlap_ratio >= self.min_overlap_ratio:
                        candidates.append(c)

                if not candidates:
                    continue

                # Select only foreground cluster(s) within depth_band_m of the closest hit
                closest_r = min(c.min_range_m for c in candidates)
                for c in candidates:
                    if c.min_range_m <= closest_r + self.depth_band_m:
                        c.label = "PERSON"
                        c.confidence = max(c.confidence, conf)
                        person_mask_360[c.ray_indices] = True

        person_clusters = [c for c in clusters if c.label == "PERSON"]
        person_flags_36 = pool_person_flags_36(person_mask_360)
        return FusionResult(
            clusters=clusters,
            person_clusters=person_clusters,
            person_mask_360=person_mask_360,
            person_flags_36=person_flags_36,
        )
