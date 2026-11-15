"""
LOCKED acceptance test — written by the Claude Supervisor (TK-SLAM-04, direction C).
Antigravity agents must NOT edit this file (see agents-doc/LOCKED_PATHS.txt).

Direction C ("stop, then scan"): the robot can only spin at one physical rate (~5.2 rad/s, the
minimum PWM that moves the GA25 motors), so one D500 sweep (10 Hz) is smeared by ~30 deg while
spinning. SLAM must therefore NOT write those smeared sweeps into the map, and must re-anchor the
pose from the first stationary sweeps after each turn.

Scenario (runtime path: MotionAdapter.update_odometry -> odometry + map), 2 x 1 m arena + 0.2 m box:
  warm-up: 6 stationary sweeps
  4 x [ spin ~90 deg at the physical rate (commanded w > 0, smeared sweeps),
        then 4 stationary sweeps (commanded w = 0) ]
  The ACTUAL turn differs from the commanded 90 deg (x1.00, x0.90, x1.10, x0.95) like a real
  open-loop motor, so integrating the command alone cannot pass.
  The w_cmd fed to SLAM does NOT carry the physical spin rate: in production it is the planner's
  command (coordinator.py `self.angular_vel = w`, any magnitude up to MAX_ANGULAR_SPEED), while the
  bridge spins at ONE physical rate whenever |w| >= 0.02 (turn_speed == min_drive_speed, PWM step,
  TK-INT-07). The test feeds a ramp below the physical rate (0.6, 1.2, 1.8 rad/s, the bridge's own
  w slew); only the SIGN of w_cmd may be trusted, the rate comes from turn_time_90_deg_sec.

Asserted after the last stationary sweep of EVERY stop (and at the end):
  heading error <= 3 deg, position drift <= 5 cm      (project SLAM threshold, CLAUDE.md)
  ghost wall cells (occupied > 10 cm from any true wall) <= 10 at the end
  map writes (grid_map.update_scan calls): 0 during every spin, >= 1 during every stop
  (so "freeze the map forever" cannot pass)
Both anchor modes are run (anchor ON as in tests/test_hector_anchor_slam.py, and production OFF).

Run:  .venv/bin/python tests/acceptance/test_slam_stop_turn_go.py -v
"""
import json
import logging
import math
import os
import sys
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
PIVOT_RATE = (math.pi / 2.0) / float(DYN["turn_time_90_deg_sec"])  # rad/s, the only spin speed

# --- Locked evaluation constants (the coder cannot tune these) ---------------------------
DT = 0.10                     # D500 sweep period, s
BRIDGE_W_SLEW = 1.5 / 0.25    # rad/s^2 ramp of w_cmd magnitude (= bridge `max_step_w`)
WARMUP_SCANS = 6
STOP_SCANS = 4                # stationary sweeps after each turn (0.4 s pause)
TURN_FACTORS = [1.00, 0.90, 1.10, 0.95]   # actual / commanded rotation per 90 deg turn
MAX_DRIFT_M = 0.05
MAX_HEADING_DEG = 3.0
GHOST_DIST_M = 0.10
MAX_GHOST_CELLS = 10
OFFSET = 0.0137               # keeps walls off cell boundaries

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


def _sweep(th_end, rate):
    """One CCW 360-bin sweep ending at heading th_end. Bin i is measured at time (i/360)*DT inside
    the sweep, while the robot keeps turning at `rate` — the driver reports it as if static."""
    out = []
    for i in range(360):
        th_i = th_end - rate * DT * (1.0 - i / 360.0)
        lx, ly = LEVER_M * math.cos(th_i), LEVER_M * math.sin(th_i)
        out.append(_ray(lx, ly, th_i + math.radians(i)))
    return out


def _wrap_deg(a):
    return abs(math.degrees((a + math.pi) % (2 * math.pi) - math.pi))


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


def _run(production_mode):
    ad = MotionAdapter()
    writes = [0]
    _orig_update = ad.grid_map.update_scan

    def _counting_update(*a, **k):
        writes[0] += 1
        return _orig_update(*a, **k)
    ad.grid_map.update_scan = _counting_update
    if production_mode:
        ad.anchor_initialized = True  # mirrors telemetry_parser.py production path
    for _ in range(WARMUP_SCANS):
        ad.update_odometry(_sweep(0.0, 0.0), DT, 0.0, 0.0)

    th = 0.0
    stops = []
    spin_writes, stop_writes = [], []
    for factor in TURN_FACTORS:
        actual_rate = PIVOT_RATE * factor
        n_spin = int(round((math.pi / 2.0) / (PIVOT_RATE * DT)))  # commanded 90 deg
        w0 = writes[0]
        for j in range(n_spin):
            th = (th + actual_rate * DT + math.pi) % (2 * math.pi) - math.pi
            w_reported = min(PIVOT_RATE, BRIDGE_W_SLEW * DT * (j + 1))  # magnitude != physical rate
            ad.update_odometry(_sweep(th, actual_rate), DT, 0.0, w_reported)
        w1 = writes[0]
        for _ in range(STOP_SCANS):
            ox, oy, oth, _ = ad.update_odometry(_sweep(th, 0.0), DT, 0.0, 0.0)
        spin_writes.append(w1 - w0)
        stop_writes.append(writes[0] - w1)
        stops.append((100.0 * math.hypot(ox, oy), _wrap_deg(oth - th)))
    return stops, _ghost_cells(ad), spin_writes, stop_writes


class TestSlamStopTurnGo(unittest.TestCase):
    def _check(self, production_mode):
        stops, ghost, spin_writes, stop_writes = _run(production_mode)
        tag = "anchor=OFF(prod)" if production_mode else "anchor=ON"
        for k, (d, h) in enumerate(stops, 1):
            print(f"\n[METRIC] {tag} stop {k}: drift={d:.1f}cm heading_err={h:.1f}deg", end="")
        print(f"\n[METRIC] {tag} ghost_cells={ghost} map_writes spin={spin_writes} stop={stop_writes}",
              flush=True)
        for k, (d, h) in enumerate(stops, 1):
            self.assertLessEqual(h, MAX_HEADING_DEG, f"{tag} stop {k}: heading error {h:.1f} deg")
            self.assertLessEqual(d, 100 * MAX_DRIFT_M, f"{tag} stop {k}: drift {d:.1f} cm")
        self.assertLessEqual(ghost, MAX_GHOST_CELLS, f"{tag}: {ghost} ghost wall cells")
        for k, (ws, wt) in enumerate(zip(spin_writes, stop_writes), 1):
            self.assertEqual(ws, 0, f"{tag} turn {k}: {ws} smeared sweeps written to the map")
            self.assertGreaterEqual(wt, 1, f"{tag} stop {k}: map writes never resumed")

    def test_01_stop_turn_go_anchor_on(self):
        self._check(production_mode=False)

    def test_02_stop_turn_go_production(self):
        self._check(production_mode=True)


if __name__ == "__main__":
    unittest.main()
