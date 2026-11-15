"""
LOCKED acceptance test — written by the Claude Supervisor (TK-SLAM-03).
Antigravity agents must NOT edit this file (see agents-doc/LOCKED_PATHS.txt).

Drives the RUNTIME SLAM path (dashboard/backend/app/navigation/motion_adapter.py ->
MotionAdapter.update_odometry -> Lidar2DOdometry + OccupancyGridMap) through a full 360 deg
in-place turn inside a simulated 2 x 1 m arena with one 0.2 m box (breaks symmetry), and asserts
real metrics against ground truth:

    pose drift after 360 deg  <= 5 cm  and  <= 3 deg   (project SLAM threshold, CLAUDE.md)
    ghost wall cells (occupied cells > 10 cm from any true wall) <= 10
    median SLAM time per scan (odometry + map + costmap + dashboard encode) <= 15 ms (PC guard)

Scans are IDEAL (no intra-sweep smear): deskew is TK-SLAM-04, not this ticket.
Both anchor modes are run: anchor ON (what tests/test_hector_anchor_slam.py uses) and anchor OFF
(what production does, telemetry_parser.py sets anchor_initialized=True).
The simulated LiDAR sits at `lidar_lever_arm_m` ahead of the axle, read from setup/robot_dynamics.json.

Run:  .venv/bin/python tests/acceptance/test_slam_inplace_turn.py -v
"""
import json
import logging
import math
import os
import sys
import time
import unittest

import numpy as np

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "dashboard", "backend"))
sys.path.insert(0, os.path.join(ROOT, "motion"))
logging.disable(logging.CRITICAL)

from app.navigation.motion_adapter import MotionAdapter  # noqa: E402

with open(os.path.join(ROOT, "setup", "robot_dynamics.json"), encoding="utf-8") as _f:
    DYN = json.load(_f)
LEVER_M = float(DYN["lidar_lever_arm_m"])
PIVOT_RATE = (math.pi / 2.0) / float(DYN["turn_time_90_deg_sec"])  # physical spin rate, rad/s

# --- Locked evaluation constants (the coder cannot tune these) ---------------------------
DT = 0.10                  # D500 sweep period, s
SLOW_RATE = 0.35           # rad/s, a gentle mapping turn (test scenario, not a robot parameter)
WARMUP_SCANS = 6
MAX_DRIFT_M = 0.05
MAX_HEADING_DEG = 3.0
GHOST_DIST_M = 0.10
MAX_GHOST_CELLS = 10
MAX_MEDIAN_MS = 15.0
OFFSET = 0.0137            # keeps walls off cell boundaries (avoids a quantisation artefact)

_BOX = [(.4, .1), (.6, .1), (.6, .3), (.4, .3)]
_RECT = [(-1, -.5), (1, -.5), (1, .5), (-1, .5)]
SEGS = []
for poly in (_RECT, _BOX):
    for i in range(4):
        (x1, y1), (x2, y2) = poly[i], poly[(i + 1) % 4]
        SEGS.append(((x1 + OFFSET, y1 + OFFSET), (x2 + OFFSET, y2 + OFFSET)))


def _ray(ox, oy, a):
    dx, dy = math.cos(a), math.sin(a)
    best = 12.0
    for (x1, y1), (x2, y2) in SEGS:
        ex, ey = x2 - x1, y2 - y1
        den = dx * ey - dy * ex
        if abs(den) < 1e-9:
            continue
        t = ((x1 - ox) * ey - (y1 - oy) * ex) / den
        u = ((x1 - ox) * dy - (y1 - oy) * dx) / den
        if t > 0 and 0 <= u <= 1:
            best = min(best, t)
    return best


def _scan(x, y, th):
    """360 CCW bins in the body frame, LiDAR at LEVER_M ahead of the axle (REP-103)."""
    lx, ly = x + LEVER_M * math.cos(th), y + LEVER_M * math.sin(th)
    return [_ray(lx, ly, th + math.radians(i)) for i in range(360)]


def _ghost_cells(adapter):
    g = adapter.grid_map.grid_core
    rows, cols = np.nonzero(g.log_odds > 0.6)
    if len(rows) == 0:
        return 0
    P = np.stack([g.origin_x + (cols + 0.5) * g.resolution,
                  g.origin_y + (rows + 0.5) * g.resolution], 1)
    best = np.full(len(P), 9.0)
    for (x1, y1), (x2, y2) in SEGS:
        a, b = np.array([x1, y1]), np.array([x2, y2])
        ab = b - a
        t = np.clip(((P - a) @ ab) / (ab @ ab), 0, 1)
        best = np.minimum(best, np.linalg.norm(P - (a + t[:, None] * ab), axis=1))
    return int(np.sum(best > GHOST_DIST_M))


def _run_turn(rate, production_mode):
    ad = MotionAdapter()
    if production_mode:
        ad.anchor_initialized = True  # mirrors telemetry_parser.py production path
    for _ in range(WARMUP_SCANS):
        ad.update_odometry(_scan(0.0, 0.0, 0.0), DT, 0.0, 0.0)
    steps = int(round(2 * math.pi / (rate * DT)))
    th, lat = 0.0, []
    ox = oy = oth = 0.0
    for _ in range(steps):
        th = (th + rate * DT + math.pi) % (2 * math.pi) - math.pi
        r = _scan(0.0, 0.0, th)
        t0 = time.perf_counter()
        ox, oy, oth, _ = ad.update_odometry(r, DT, 0.0, rate)
        ad.update_costmap()
        ad.to_dashboard_b64()
        lat.append((time.perf_counter() - t0) * 1e3)
    return {
        "drift_cm": 100.0 * math.hypot(ox, oy),
        "heading_deg": abs(math.degrees((oth - th + math.pi) % (2 * math.pi) - math.pi)),
        "ghost": _ghost_cells(ad),
        "median_ms": float(np.median(lat)),
    }


class TestSlamInPlaceTurn(unittest.TestCase):
    def _check(self, rate, production_mode):
        m = _run_turn(rate, production_mode)
        tag = f"rate={rate:.2f}rad/s anchor={'OFF(prod)' if production_mode else 'ON'}"
        print(f"\n[METRIC] {tag} drift={m['drift_cm']:.1f}cm heading={m['heading_deg']:.1f}deg "
              f"ghost_cells={m['ghost']} median={m['median_ms']:.1f}ms", flush=True)
        self.assertLessEqual(m["drift_cm"], 100 * MAX_DRIFT_M, f"{tag}: drift {m['drift_cm']:.1f} cm")
        self.assertLessEqual(m["heading_deg"], MAX_HEADING_DEG, f"{tag}: heading {m['heading_deg']:.1f} deg")
        self.assertLessEqual(m["ghost"], MAX_GHOST_CELLS, f"{tag}: {m['ghost']} ghost wall cells")
        self.assertLessEqual(m["median_ms"], MAX_MEDIAN_MS, f"{tag}: median {m['median_ms']:.1f} ms")

    def test_01_slow_turn_anchor_on(self):
        self._check(SLOW_RATE, production_mode=False)

    def test_02_slow_turn_production(self):
        self._check(SLOW_RATE, production_mode=True)

    def test_03_physical_rate_anchor_on(self):
        self._check(PIVOT_RATE, production_mode=False)

    def test_04_physical_rate_production(self):
        self._check(PIVOT_RATE, production_mode=True)


if __name__ == "__main__":
    unittest.main()
