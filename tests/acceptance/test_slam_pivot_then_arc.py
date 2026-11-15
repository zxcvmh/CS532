"""
LOCKED acceptance test — written by the Claude Supervisor (TK-SLAM-07, attempt 2).
Antigravity agents must NOT edit this file (see agents-doc/LOCKED_PATHS.txt).

Gap found by the quality-auditor on TK-SLAM-07 attempt 1: after an in-place pivot, the driver goes
straight into an ARC (v > 0 and w != 0, as MANUAL mode allows: v <= 0.14 m/s, |w| up to the pivot
rate) with no stationary sweep in between. Attempt 1 kept treating every such sweep as "still
spinning" (inter-sweep rotation > 5 deg), dead-reckoned the 5.24 rad/s pivot rate with dx = dy = 0,
and wrote nothing to the map: heading error 111-180 deg, position frozen.

Scene and sweep model are shared with test_slam_stop_turn_go.py (same room, lever arm, rolling-
shutter sweep). Per case: 6 still sweeps, a commanded 90 deg pivot that physically turns only 0.9x
(bridge-slewed w_cmd, as in the field), then N_ARC arc sweeps with no stop, then STOP sweeps.

Asserted per case (baseline HEAD 3f6870a passes all; attempt 1 fails w >= 1.0):
  during the arc : worst position error <= 5 cm, worst heading error <= 8 deg
  after the stop : position error <= 5 cm, heading error <= 3 deg   (project SLAM threshold)
  the occupancy grid is written during the arc (costmap must stay live while driving)

Run:  .venv/bin/python tests/acceptance/test_slam_pivot_then_arc.py -v
"""
import math
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import test_slam_stop_turn_go as T  # noqa: E402  (scene, sweep model, MotionAdapter import paths)
from app.navigation.motion_adapter import MotionAdapter  # noqa: E402

# --- Locked evaluation constants -----------------------------------------------------------
X0, Y0 = -0.6, 0.0
WARMUP = 6
PIVOT_FACTOR = 0.9            # robot physically turns 0.9x of the commanded 90 deg
N_ARC = 8
STOP = 4
MAX_ARC_POS_M = 0.05
MAX_ARC_HEAD_DEG = 8.0
MAX_STOP_POS_M = 0.05
MAX_STOP_HEAD_DEG = 3.0


def _sweep(x, y, th, rate):
    """360-bin CCW sweep ending at pose (x, y, th) while the robot turns at `rate` during it."""
    out = []
    for i in range(360):
        th_i = th - rate * T.DT * (1.0 - i / 360.0)
        lx, ly = x + T.LEVER_M * math.cos(th_i), y + T.LEVER_M * math.sin(th_i)
        out.append(T._ray(lx, ly, th_i + math.radians(i)))
    return out


def _run(v, w, pivot_sign):
    ad = MotionAdapter()
    ad.anchor_initialized = True  # production path (telemetry_parser.py)
    writes = [0]
    orig = ad.grid_map.update_scan

    def counting(*a, **k):
        writes[0] += 1
        return orig(*a, **k)
    ad.grid_map.update_scan = counting

    x, y, th = X0, Y0, 0.0
    for _ in range(WARMUP):
        ad.update_odometry(_sweep(x, y, th, 0.0), T.DT, 0.0, 0.0)

    rate = pivot_sign * T.PIVOT_RATE * PIVOT_FACTOR
    for j in range(int(round((math.pi / 2.0) / (T.PIVOT_RATE * T.DT)))):
        th += rate * T.DT
        w_rep = pivot_sign * min(T.PIVOT_RATE, T.BRIDGE_W_SLEW * T.DT * (j + 1))
        ad.update_odometry(_sweep(x, y, th, rate), T.DT, 0.0, w_rep)

    def err(out):
        return math.hypot(out[0] - (x - X0), out[1] - (y - Y0)), T._wrap_deg(out[2] - th)

    w0, arc_pos, arc_head = writes[0], 0.0, 0.0
    for _ in range(N_ARC):
        th += w * T.DT
        x += v * T.DT * math.cos(th)
        y += v * T.DT * math.sin(th)
        p, h = err(ad.update_odometry(_sweep(x, y, th, w), T.DT, v, w))
        arc_pos, arc_head = max(arc_pos, p), max(arc_head, h)
    arc_writes = writes[0] - w0
    for _ in range(STOP):
        out = ad.update_odometry(_sweep(x, y, th, 0.0), T.DT, 0.0, 0.0)
    stop_pos, stop_head = err(out)
    return arc_pos, arc_head, arc_writes, stop_pos, stop_head


class TestSlamPivotThenArc(unittest.TestCase):
    def _check(self, v, w, pivot_sign=1):
        arc_pos, arc_head, arc_writes, stop_pos, stop_head = _run(v, w, pivot_sign)
        tag = f"pivot {'L' if pivot_sign > 0 else 'R'} x{PIVOT_FACTOR} -> arc v={v} w={w:+}"
        print(f"\n[METRIC] {tag}: arc worst pos={100 * arc_pos:.1f} cm head={arc_head:.1f} deg "
              f"writes={arc_writes}/{N_ARC} | after stop pos={100 * stop_pos:.1f} cm "
              f"head={stop_head:.1f} deg", flush=True)
        self.assertLessEqual(arc_pos, MAX_ARC_POS_M, f"{tag}: position error during arc")
        self.assertLessEqual(arc_head, MAX_ARC_HEAD_DEG, f"{tag}: heading error during arc")
        self.assertGreater(arc_writes, 0, f"{tag}: map never written while driving the arc")
        self.assertLessEqual(stop_pos, MAX_STOP_POS_M, f"{tag}: position error after stop")
        self.assertLessEqual(stop_head, MAX_STOP_HEAD_DEG, f"{tag}: heading error after stop")

    def test_01_manual_slow_arc(self):
        self._check(0.14, 0.5)

    def test_02_manual_arc_w1(self):
        self._check(0.14, 1.0)

    def test_03_manual_arc_w1_5(self):
        self._check(0.14, 1.5)

    def test_04_manual_arc_w2(self):
        # w = 3.0 (17 deg/sweep) is left out: HEAD itself reaches 10.7 deg mid-arc there (sweep smear
        # beyond Hector's range) — a separate limitation, not this ticket's regression.
        self._check(0.14, 2.0)

    def test_05_fast_arc_w0_5(self):
        self._check(0.294, 0.5)

    def test_06_fast_arc_w1(self):
        self._check(0.294, 1.0)

    def test_07_right_pivot_then_right_arc(self):
        self._check(0.14, -1.5, pivot_sign=-1)


if __name__ == "__main__":
    unittest.main()
