#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Unit tests for perception.core.person_lidar_fusion (TK-CV-02A / TK-CV-02B)."""

import json
import math
import os
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "imitation_learning"))

from perception.core.person_lidar_fusion import (  # noqa: E402
    CameraIntrinsics,
    PersonLidarFuser,
    cluster_lidar_360,
    mask_person_ranges,
)
import dataset as il_dataset  # noqa: E402


class TestPersonLidarFusion(unittest.TestCase):
    def test_01_rep103_projection_and_yaw_offset(self):
        intr = CameraIntrinsics.from_fov(width=640, height=480, hfov_deg=120.0, camera_yaw_offset_deg=0.0)
        az_center = intr.pixel_to_azimuth_deg(320.0, 240.0)
        az_left = intr.pixel_to_azimuth_deg(120.0, 240.0)
        az_right = intr.pixel_to_azimuth_deg(520.0, 240.0)

        self.assertAlmostEqual(az_center, 0.0, places=4)
        self.assertGreater(az_left, 0.0)
        self.assertLess(az_right, 0.0)
        self.assertAlmostEqual(abs(az_left), abs(az_right), places=4)

        intr_shifted = CameraIntrinsics.from_fov(width=640, height=480, hfov_deg=120.0, camera_yaw_offset_deg=4.5)
        self.assertAlmostEqual(intr_shifted.pixel_to_azimuth_deg(320.0, 240.0), 4.5, places=4)

        with tempfile.TemporaryDirectory() as td:
            p = Path(td) / "intrinsics.json"
            p.write_text(
                json.dumps(
                    {
                        "image_width": 640,
                        "image_height": 480,
                        "camera_matrix": intr.camera_matrix.tolist(),
                        "dist_coeffs": [0.0, 0.0, 0.0, 0.0, 0.0],
                        "camera_yaw_offset_deg": -2.5,
                    }
                ),
                encoding="utf-8",
            )
            loaded = CameraIntrinsics.from_json(p)
            self.assertAlmostEqual(loaded.pixel_to_azimuth_deg(320.0), -2.5, places=4)

    def test_02_foreground_person_rejects_background_wall(self):
        intr = CameraIntrinsics.from_fov(width=640, height=480, hfov_deg=120.0)
        fuser = PersonLidarFuser(intrinsics=intr)

        # Wall all around at 3.2 m, except foreground person across 0 deg (-6..+6 deg) at 1.4 m
        ranges = np.full(360, 3.2, dtype=np.float32)
        person_rays = list(range(354, 360)) + list(range(0, 7))
        for r in person_rays:
            ranges[r] = 1.4

        # Wide person bbox covering roughly [-18, +18] deg so wall rays at ±10..±18 deg also fall inside azimuth
        x_left = 320.0 - intr.camera_matrix[0, 0] * math.tan(math.radians(18.0))
        x_right = 320.0 + intr.camera_matrix[0, 0] * math.tan(math.radians(18.0))
        dets = [{"class_name": "person", "confidence": 0.92, "bbox": [x_left, 100.0, x_right, 440.0]}]

        res = fuser.fuse(ranges, dets)
        self.assertEqual(len(res.person_clusters), 1)
        self.assertEqual(sorted(res.person_clusters[0].ray_indices), sorted(person_rays))

        for r in person_rays:
            self.assertTrue(bool(res.person_mask_360[r]))
        for wall_r in (10, 14, 346, 350):
            self.assertFalse(bool(res.person_mask_360[wall_r]))

    def test_03_multi_person_and_static_obstacle(self):
        intr = CameraIntrinsics.from_fov(width=640, height=480, hfov_deg=120.0)
        fuser = PersonLidarFuser(intrinsics=intr)

        ranges = np.full(360, 4.5, dtype=np.float32)
        left_person = list(range(27, 34))       # +30 deg
        center_chair = list(range(358, 360)) + [0, 1, 2]  # 0 deg (chair, NOT person)
        right_person = list(range(322, 329))     # -35 deg
        for r in left_person:
            ranges[r] = 1.5
        for r in center_chair:
            ranges[r] = 1.2
        for r in right_person:
            ranges[r] = 1.8

        def _bbox_for_az(az_deg: float, half_deg: float = 5.0):
            u1 = 320.0 - intr.camera_matrix[0, 0] * math.tan(math.radians(az_deg + half_deg))
            u2 = 320.0 - intr.camera_matrix[0, 0] * math.tan(math.radians(az_deg - half_deg))
            return [min(u1, u2), 120.0, max(u1, u2), 420.0]

        dets = [
            {"class_name": "person", "confidence": 0.88, "bbox": _bbox_for_az(30.0)},
            {"class_name": "chair", "confidence": 0.95, "bbox": _bbox_for_az(0.0)},
            {"class_name": "person", "confidence": 0.85, "bbox": _bbox_for_az(-35.0)},
        ]
        res = fuser.fuse(ranges, dets)
        self.assertEqual(len(res.person_clusters), 2)
        for r in left_person + right_person:
            self.assertTrue(bool(res.person_mask_360[r]))
        for r in center_chair:
            self.assertFalse(bool(res.person_mask_360[r]))

    def test_04_il_v2_113d_and_csm_mask_compatibility(self):
        intr = CameraIntrinsics.from_fov(width=640, height=480, hfov_deg=120.0)
        fuser = PersonLidarFuser(intrinsics=intr)

        ranges = np.full(360, 2.8, dtype=np.float32)
        for r in range(18, 24):  # +20 deg
            ranges[r] = 1.1

        u1 = 320.0 - intr.camera_matrix[0, 0] * math.tan(math.radians(24.0))
        u2 = 320.0 - intr.camera_matrix[0, 0] * math.tan(math.radians(16.0))
        dets = [{"class_name": "person", "confidence": 0.90, "bbox": [u1, 100.0, u2, 420.0]}]

        res = fuser.fuse(ranges, dets)
        obs = il_dataset.extract_observation_vector_v2(
            ranges.tolist(), goal_x=2.0, goal_y=0.0, robot_x=0.0, robot_y=0.0, robot_theta=0.0,
            person_mask_360=res.person_mask_360,
        )
        np.testing.assert_array_equal(obs[41:77], res.person_flags_36)
        self.assertEqual(float(res.person_flags_36[2]), 1.0)
        self.assertEqual(float(res.person_flags_36[0]), 0.0)

        masked = mask_person_ranges(ranges, res.person_mask_360, fill_value=0.0)
        self.assertTrue(np.all(masked[18:24] == 0.0))
        self.assertTrue(np.allclose(masked[50:60], 2.8))

    def test_05_real_lidar_crossing_smoke(self):
        npz_path = ROOT / "docs" / "measurements" / "2026-11-16" / "lidar_crossing.npz"
        if not npz_path.is_file():
            self.skipTest("real lidar_crossing.npz not present")

        intr = CameraIntrinsics.from_fov(width=640, height=480, hfov_deg=160.0)
        fuser = PersonLidarFuser(intrinsics=intr)
        with np.load(npz_path) as z:
            scans = z["ranges"][:5]
        for scan in scans:
            clusters = cluster_lidar_360(scan)
            self.assertIsInstance(clusters, list)
            res = fuser.fuse(scan, [])
            self.assertEqual(res.person_mask_360.shape, (360,))
            self.assertEqual(res.person_flags_36.shape, (36,))
            self.assertFalse(bool(res.person_mask_360.any()))


if __name__ == "__main__":
    unittest.main()
