"""
LOCKED acceptance test — written by the Claude Supervisor (TK-INT-12). Coders must NOT edit it.

The tracker's heading error grows with the rotation inside ONE LiDAR sweep (measured on the two turn recordings:
<= 8 deg/sweep -> max 4.2 deg; <= 17 deg/sweep -> max 9.5 deg and 2.8 % of sweeps above 6 deg).
Human decision 2026-11-20: the demo needs a FAST pivot so the robot reacts to obstacles; 90 deg in 0.64 s is accepted
(~14.3 deg/sweep) although the clean limit would be 8 deg/sweep. The hard ceiling below is therefore 15 deg/sweep
(must beat the 17.3 deg/sweep of the 0.53 s pivot); TK-SLAM-10 flags the sweeps that still go bad.

Reads `setup/robot_dynamics.json` (human field measurements) and the sweep period from the recordings. Nothing typed.
Run:  .venv/bin/python tests/acceptance/test_pivot_rate_vs_sweep.py -v
"""
import json
import os
import unittest

import numpy as np

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
STAMPS = [
    os.path.join(ROOT, "tests", "acceptance", "data", "field_turns_2026-11-15.npz"),
    os.path.join(ROOT, "tests", "acceptance", "data", "big_turns_2026-11-15.npz"),
    os.path.join(ROOT, "docs", "measurements", "2026-11-16", "lidar_static_person.npz"),
    os.path.join(ROOT, "docs", "measurements", "2026-11-16", "lidar_crossing.npz"),
    os.path.join(ROOT, "docs", "measurements", "2026-11-16", "lidar_head_on.npz"),
]
MAX_DEG_PER_SWEEP = 15.0   # accepted ceiling (human, 2026-11-20); 8.0 is the clean target
MAX_90_180_DISAGREE = 0.20  # t180 must be within 20 % of 2 * t90 (same pivot, two timings)

with open(os.path.join(ROOT, "setup", "robot_dynamics.json"), encoding="utf-8") as _f:
    DYN = json.load(_f)
SWEEP_S = float(np.median(np.concatenate([np.diff(np.load(p)["stamp"].astype(float)) for p in STAMPS])))


class TestPivotRate(unittest.TestCase):
    def test_01_rate_per_sweep(self):
        t90 = float(DYN["turn_time_90_deg_sec"])
        per_sweep = 90.0 / t90 * SWEEP_S
        print(f"\n[METRIC] pivot {90.0 / t90:.0f} deg/s x sweep {SWEEP_S * 1000:.0f} ms = {per_sweep:.1f} deg/sweep "
              f"(limit {MAX_DEG_PER_SWEEP})", flush=True)
        self.assertLessEqual(per_sweep, MAX_DEG_PER_SWEEP,
                             f"pivot rotates {per_sweep:.1f} deg per LiDAR sweep: lower turn_speed and re-time")

    def test_02_90_and_180_agree(self):
        t90, t180 = float(DYN["turn_time_90_deg_sec"]), float(DYN["turn_time_180_deg_sec"])
        ratio = t180 / (2.0 * t90)
        print(f"\n[METRIC] t180 / (2 x t90) = {ratio:.2f}", flush=True)
        self.assertLessEqual(abs(ratio - 1.0), MAX_90_180_DISAGREE, "90 and 180 deg timings disagree: re-time both")


if __name__ == "__main__":
    unittest.main()
