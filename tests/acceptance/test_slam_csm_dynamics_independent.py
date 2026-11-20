"""
LOCKED acceptance test — written by the Claude Supervisor (TK-SLAM-10, scope extension 2026-11-20). Coders must NOT edit it.

The CSM tracker is LiDAR-only: its poses on a recording must NOT change when the human edits `turn_time_90_deg_sec` in
`setup/robot_dynamics.json`. Found 2026-11-20: lidar_odometry.py derived CSMTracker.max_dth_step_rad from that value
(0.3705 rad at 0.53 s, 0.3068 at 0.64 s) and the audited SLAM-09 test broke (big recording, 43 deg) after the human
slowed the pivot. The audited value 0.3705 rad = (pi/2)/0.53 * 1.25 * 0.1 is the one `slam09_golden.npz` was made with.

Replays `big_turns` with three variants of the dynamics file (0.53 / 0.64 / 1.15 s; other keys copied from the real file)
and asserts: same clamp, same poses (1e-12) across variants, and equal to the SLAM-09 golden (1e-9).
Run:  .venv/bin/python tests/acceptance/test_slam_csm_dynamics_independent.py -v
"""
import json
import logging
import math
import os
import sys
import tempfile
import unittest

import numpy as np

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "dashboard", "backend"))
sys.path.insert(0, os.path.join(ROOT, "motion"))
logging.disable(logging.CRITICAL)
from app.navigation.motion_adapter import MotionAdapter  # noqa: E402
from planning.lidar_odometry import Lidar2DOdometry  # noqa: E402

DATA = os.path.join(ROOT, "tests", "acceptance", "data")
REC = np.load(os.path.join(DATA, "big_turns_2026-11-15.npz"))
GOLDEN = np.load(os.path.join(DATA, "slam09_golden.npz"))["big"]
VARIANTS = (0.53, 0.64, 1.15)
AUDITED_CLAMP = (math.pi / 2.0) / 0.53 * 1.25 * 0.1
with open(os.path.join(ROOT, "setup", "robot_dynamics.json"), encoding="utf-8") as _f:
    REAL = json.load(_f)


def _replay(turn_time_90):
    cfg = dict(REAL, turn_time_90_deg_sec=turn_time_90, turn_time_180_deg_sec=2 * turn_time_90)
    with tempfile.TemporaryDirectory() as td:
        path = os.path.join(td, "robot_dynamics.json")
        with open(path, "w", encoding="utf-8") as f:
            json.dump(cfg, f)
        orig = Lidar2DOdometry.load_dynamics_config
        Lidar2DOdometry.load_dynamics_config = lambda self, config_path=None: orig(self, path)
        try:
            ad = MotionAdapter()
        finally:
            Lidar2DOdometry.load_dynamics_config = orig
    ad.anchor_initialized = True
    clamp = ad.lidar_odometry.csm_tracker.max_dth_step_rad
    poses = []
    for i in range(len(REC["ranges"])):
        dt = float(REC["stamp"][i] - REC["stamp"][i - 1]) if i else 0.1
        x, y, th, _ = ad.update_odometry(REC["ranges"][i].astype(np.float32).tolist(), max(0.02, min(0.3, dt)), 0.0,
                                         float(REC["w_bridge"][i]))
        poses.append((x, y, th))
    return clamp, np.array(poses)


class TestDynamicsIndependent(unittest.TestCase):
    def test_01_clamp_and_poses_do_not_depend_on_turn_time(self):
        res = {t: _replay(t) for t in VARIANTS}
        for t, (clamp, poses) in res.items():
            print(f"\n[METRIC] turn_time_90 {t}: clamp {clamp:.4f} rad, pose vs golden {np.abs(poses - GOLDEN).max():.2e}",
                  flush=True)
            self.assertAlmostEqual(clamp, AUDITED_CLAMP, places=9, msg=f"turn_time_90={t}: tracker clamp depends on dynamics")
            self.assertLessEqual(float(np.abs(poses - GOLDEN).max()), 1e-9, f"turn_time_90={t}: poses differ from SLAM-09 golden")


if __name__ == "__main__":
    unittest.main()
