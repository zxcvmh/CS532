#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Unit tests for CSM person-beam masking (TK-CV-02B)."""

import math
import sys
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from motion.planning.slam.csm_tracker import CSMTracker  # noqa: E402
from motion.planning.lidar_odometry import Lidar2DOdometry  # noqa: E402
from perception.core.person_lidar_fusion import CameraIntrinsics, PersonLidarFuser  # noqa: E402


def _raycast_room_with_circle(
    rx: float, ry: float, rth: float,
    half_box: float = 2.0,
    circle: tuple = None,
) -> np.ndarray:
    """Raycast 360-deg LiDAR inside [-half_box, half_box]^2 with optional circle (cx, cy, r)."""
    ranges = np.full(360, 6.0, dtype=np.float32)
    for deg in range(360):
        ang = rth + math.radians(deg)
        c, s = math.cos(ang), math.sin(ang)
        dmin = 6.0
        if c < -1e-4:
            dmin = min(dmin, (-half_box - rx) / c)
        elif c > 1e-4:
            dmin = min(dmin, (half_box - rx) / c)
        if s < -1e-4:
            dmin = min(dmin, (-half_box - ry) / s)
        elif s > 1e-4:
            dmin = min(dmin, (half_box - ry) / s)
        if circle is not None:
            ox, oy, orad = circle
            dx, dy = rx - ox, ry - oy
            b = 2.0 * (dx * c + dy * s)
            cc = dx * dx + dy * dy - orad * orad
            disc = b * b - 4.0 * cc
            if disc >= 0.0:
                t1 = (-b - math.sqrt(disc)) * 0.5
                if t1 > 0.05:
                    dmin = min(dmin, t1)
        ranges[deg] = float(max(0.05, min(6.0, dmin)))
    return ranges


class TestCSMPersonMasking(unittest.TestCase):
    def test_01_none_mask_bit_exact(self):
        csm_a = CSMTracker()
        csm_b = CSMTracker()
        scans = [_raycast_room_with_circle(0.0, 0.0, 0.05 * i) for i in range(8)]
        for sc in scans:
            ra = csm_a.update(sc)
            rb = csm_b.update(sc, person_mask=None)
            self.assertEqual(ra, rb)
            self.assertEqual(csm_a.last_score, csm_b.last_score)
            self.assertEqual(csm_b.last_masked_rays, 0)

    def test_02_crossing_person_ablation_and_no_submap_ghost(self):
        intr = CameraIntrinsics.from_fov(width=640, height=480, hfov_deg=120.0)
        fuser = PersonLidarFuser(intrinsics=intr)

        csm_off = CSMTracker()
        csm_on = CSMTracker()

        # 2 clean static room sweeps to initialize submap, then 25 sweeps with person crossing at x=0.65m
        scans = [_raycast_room_with_circle(0.0, 0.0, 0.0) for _ in range(2)]
        person_masks = [np.zeros(360, dtype=bool) for _ in range(2)]

        for k in range(25):
            py = -0.60 + k * 0.05
            sc = _raycast_room_with_circle(0.0, 0.0, 0.0, circle=(0.65, py, 0.27))
            # Project person bounding box onto camera intrinsics
            az_c = math.degrees(math.atan2(py, 0.65))
            az_half = math.degrees(math.atan2(0.27, 0.65)) + 3.0
            u1 = 320.0 - intr.camera_matrix[0, 0] * math.tan(math.radians(az_c + az_half))
            u2 = 320.0 - intr.camera_matrix[0, 0] * math.tan(math.radians(az_c - az_half))
            dets = [{"class_name": "person", "confidence": 0.92, "bbox": [min(u1, u2), 100.0, max(u1, u2), 420.0]}]
            res = fuser.fuse(sc, dets)
            scans.append(sc)
            person_masks.append(res.person_mask_360)

        off_poses = []
        on_poses = []
        masked_counts = []
        for sc, pmask in zip(scans, person_masks):
            ox, oy, oth, _, _ = csm_off.update(sc)
            nx, ny, nth, _, _ = csm_on.update(sc, person_mask=pmask)
            off_poses.append((ox, oy, oth))
            on_poses.append((nx, ny, nth))
            masked_counts.append(csm_on.last_masked_rays)

        self.assertGreater(sum(masked_counts), 0)
        # Mask ON keeps stationary robot pose tight
        max_on_pos = max(math.hypot(x, y) for x, y, _ in on_poses)
        max_on_yaw = max(abs(math.degrees(th)) for _, _, th in on_poses)
        self.assertLessEqual(max_on_pos, 0.05)
        self.assertLessEqual(max_on_yaw, 2.0)

        # Verify person corridor around (0.65, 0.0) has NO ghost occupancy in ON submap
        grid_on = csm_on.submaps_mgr.submaps[0].grid
        c_idx = int(math.floor((0.65 - grid_on.origin_x) * grid_on.inv_resolution))
        r_idx = int(math.floor((0.0 - grid_on.origin_y) * grid_on.inv_resolution))
        patch_on = grid_on.odds[r_idx - 2 : r_idx + 3, c_idx - 2 : c_idx + 3]
        prob_on = patch_on / (1.0 + patch_on)
        self.assertFalse(bool((prob_on > 0.55).any()), "Mask ON wrote person ghost cells into CSM submap")

        # Contrast: Mask OFF writes ghost cells into submap along the crossing path
        grid_off = csm_off.submaps_mgr.submaps[0].grid
        c_off = int(math.floor((0.65 - grid_off.origin_x) * grid_off.inv_resolution))
        r_off = int(math.floor((0.0 - grid_off.origin_y) * grid_off.inv_resolution))
        patch_off = grid_off.odds[r_off - 4 : r_off + 5, c_off - 4 : c_off + 5]
        prob_off = patch_off / (1.0 + patch_off)
        self.assertTrue(bool((prob_off > 0.55).any()), "Expected unmasked person to leave ghost trace in submap")

    def test_03_safety_fallback_on_all_true_mask(self):
        csm = CSMTracker()
        sc = _raycast_room_with_circle(0.0, 0.0, 0.0)
        csm.update(sc)
        all_true = np.ones(360, dtype=bool)
        x, y, th, _still, _cw = csm.update(sc, person_mask=all_true)
        self.assertEqual(csm.last_masked_rays, 0)
        self.assertTrue(math.isfinite(x) and math.isfinite(y) and math.isfinite(th))

    def test_04_lidar_odometry_forwarding(self):
        odom = Lidar2DOdometry()
        sc = _raycast_room_with_circle(0.0, 0.0, 0.0, circle=(1.0, 0.0, 0.25))
        odom.update(sc)
        mask = np.zeros(360, dtype=bool)
        mask[0:10] = True
        mask[350:360] = True
        _x, _y, _th, info = odom.update(sc, person_mask=mask)
        self.assertGreater(info["masked_person_rays"], 0)


if __name__ == "__main__":
    unittest.main()
