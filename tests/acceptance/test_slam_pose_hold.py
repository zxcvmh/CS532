"""
LOCKED acceptance test — written by the Claude Supervisor (TK-SLAM-08).
Antigravity agents must NOT edit this file (see agents-doc/LOCKED_PATHS.txt).

Field symptom (2026-11-15, after TK-SLAM-07 attempt 2): pressing A/D or the joystick makes the map
rotate with the robot, then snap back; black trails and ghost legs/boxes are left behind.

Cause, measured on the real sweeps (tests/acceptance/data/field_turns_2026-11-15.npz):
  while the button is held the robot has NOT moved yet (motor start delay) and it keeps turning for
  0.3-0.6 s after release (coast). The SLAM pose jumps 30-120 deg at the first command sweep
  (5.24 rad/s x command time), stays wrong until the LiDAR says "still", then re-anchors.
  Every consumer that runs on each sweep with that pose — obstacle tracker (legs/boxes), obstacle
  costmap layer, low-obstacle layer — draws the wrong world positions.

Required: a heading the LiDAR has not confirmed is never published or used.
  1. Published heading never claims rotation the LiDAR has not seen: relative to the last truly
     still sweep, it stays inside the envelope of the LiDAR rotation seen so far (+-TOL_DEG).
     (Holding the heading until the LiDAR re-anchors satisfies this; so does LiDAR tracking.)
  2. Nothing pose-dependent is written while the published heading is wrong (> BAD_DEG from truth):
     grid_map.update_scan, obstacle tracker update, obstacle-cluster layer. (The low-obstacle layer is
     left out: MotionAdapter.insert_low_obstacles raises TypeError on the cartographer path — TK-INT-08.)
     The replay calls MotionAdapter exactly as telemetry_parser.py does on every fresh sweep.
  3. TK-SLAM-07 still holds: at still sweeps max heading error <= 4 deg (checked by its own test).

Ground truth: same exhaustive circular alignment as test_slam_field_turns.py (moving sweeps are
smeared by the rotation inside the sweep; TOL_DEG covers it).

Run:  .venv/bin/python tests/acceptance/test_slam_pose_hold.py -v
"""
import math
import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import test_slam_field_turns as F  # noqa: E402  (data, truth estimator, command variants, paths)
from app.navigation.motion_adapter import MotionAdapter  # noqa: E402

# --- Locked evaluation constants -----------------------------------------------------------
TOL_DEG = 10.0       # envelope tolerance: 3 deg still error + 1 deg truth bins + up to ~half of the
                     # in-sweep smear at a MOVING sweep (pivot turns ~30 deg per sweep). Failures are > 100 deg.
BAD_DEG = 5.0        # a sweep whose published heading is further than this from truth is "wrong"

TRUTH_ALL = np.array([F._rotation_deg(F.RANGES[0], F.RANGES[i]) for i in range(len(F.RANGES))])
ANCHORS = sorted(F.SCORED)


def _wrap(a):
    return (a + 180.0) % 360.0 - 180.0


def _replay(variant):
    ad = MotionAdapter()
    ad.anchor_initialized = True  # production path
    cur, sinks = [0], {"map": [], "tracker": [], "clusters": []}

    def spy(name, fn):
        def wrapped(*a, **k):
            sinks[name].append(cur[0])
            return fn(*a, **k)
        return wrapped
    ad.grid_map.update_scan = spy("map", ad.grid_map.update_scan)
    ad.obstacle_tracker.update = spy("tracker", ad.obstacle_tracker.update)
    if hasattr(ad.grid_map, "insert_obstacle_clusters"):
        ad.grid_map.insert_obstacle_clusters = spy("clusters", ad.grid_map.insert_obstacle_clusters)

    cmd = F._command(variant)
    heading = []
    for i in range(len(F.RANGES)):
        cur[0] = i
        dt = float(F.STAMP[i] - F.STAMP[i - 1]) if i else 0.1
        r = F.RANGES[i].tolist()
        ox, oy, oth, _ = ad.update_odometry(r, max(0.02, min(0.3, dt)), 0.0, float(cmd[i]))
        # same call sequence as telemetry_parser.apply_binary_telemetry on a fresh tick
        valid_cls, _ = ad.process_clusters(r, ox, oy, oth)
        ad.insert_obstacle_clusters(valid_cls, ox, oy, oth, defer_costmap=True)
        heading.append(math.degrees(oth))
    return np.array(heading), sinks


def _envelope_violations(heading):
    worst, where = 0.0, None
    for i in range(len(heading)):
        prior = [a for a in ANCHORS if a <= i]
        if not prior:
            continue
        a = prior[-1]
        d_truth = [_wrap(TRUTH_ALL[j] - TRUTH_ALL[a]) for j in range(a, i + 1)]
        lo, hi = min(0.0, min(d_truth)) - TOL_DEG, max(0.0, max(d_truth)) + TOL_DEG
        d = _wrap(heading[i] - TRUTH_ALL[a])
        over = max(0.0, d - hi, lo - d)
        if over > worst:
            worst, where = over, i
    return worst, where


class TestSlamPoseHold(unittest.TestCase):
    def _check(self, variant):
        heading, sinks = _replay(variant)
        err = np.abs(_wrap(heading - TRUTH_ALL))
        bad = set(np.flatnonzero(err > BAD_DEG))
        over, where = _envelope_violations(heading)
        bad_writes = {k: sorted(set(v) & bad) for k, v in sinks.items()}
        n_bad = {k: len(v) for k, v in bad_writes.items()}
        print(f"\n[METRIC] {variant}: heading beyond LiDAR envelope worst={over:.1f} deg (sweep {where}) | "
              f"sweeps with wrong heading={len(bad)} | writes at wrong heading {n_bad} | "
              f"total writes { {k: len(v) for k, v in sinks.items()} }", flush=True)
        self.assertLessEqual(over, 0.0, f"{variant}: published heading ran {over:.1f} deg ahead of "
                                        f"what the LiDAR saw (sweep {where})")
        for k, v in bad_writes.items():
            self.assertEqual(v, [], f"{variant}: {k} written at {len(v)} sweeps with wrong heading: {v[:10]}")
        self.assertGreater(len(sinks["map"]), 100, f"{variant}: map almost never written")
        self.assertGreater(len(sinks["clusters"]), 100, f"{variant}: obstacle layer almost never updated")

    def test_01_recorded(self):
        self._check("recorded")

    def test_02_widened(self):
        self._check("widened")

    def test_03_leads(self):
        self._check("leads")


if __name__ == "__main__":
    unittest.main()
