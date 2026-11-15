"""
LOCKED acceptance test — written by the Claude Supervisor (TK-SLAM-07).
Antigravity agents must NOT edit this file or its data (see agents-doc/LOCKED_PATHS.txt).

REAL field data (2026-11-15, D500 on the robot, 4 manual left turns from the dashboard):
  tests/acceptance/data/field_turns_2026-11-15.npz — 306 unique sweeps (ranges, 360 CCW bins),
  bridge-reported w, timestamps.

Field failure reproduced by this replay: SLAM dead-reckons heading as 5.24 rad/s x (time the button
was held). The motor needs a few hundred ms to start and coasts after release, so the real turns were
+60, +28, +97, +32 deg while SLAM predicted +110..+152 deg per turn -> heading error up to ~160 deg,
the map rotates with the robot. The heading after a turn must come from the LiDAR, not the button.

Ground truth (independent of the code under test): rotation of each stationary sweep relative to the
first sweep, by exhaustive circular alignment of the 360 range bins (robot pivots in place; lever arm
0.01 m). 1-degree bins -> truth resolution +-0.5 deg.

Three command timings are replayed, because the button never matches the motor:
  recorded : w_cmd as the bridge reported it
  widened  : w_cmd starts 3 sweeps early and ends 3 sweeps late
  leads    : w_cmd ends 3 sweeps BEFORE the motor stops (the robot coasts with w_cmd = 0)
Each in both anchor modes (ON, and production OFF).

Asserted per run, at stationary sweeps (no motor motion in the previous 1.5 s or next 0.3 s):
  max heading error <= 4.0 deg and 90th percentile <= 3.0 deg   (project: 3 deg; +1 for truth bins)
  occupied cells (log_odds > 0.6) <= 1.25 x the oracle map (stationary sweeps written at true heading)
  median MotionAdapter.update_odometry time <= 15 ms on the laptop
Baseline 2026-11-15 (HEAD 869509b): heading error max ~160 deg, median ~60 deg; occupied ~2x oracle.

Run:  .venv/bin/python tests/acceptance/test_slam_field_turns.py -v
"""
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

DATA = os.path.join(os.path.dirname(__file__), "data", "field_turns_2026-11-15.npz")

# --- Locked evaluation constants -----------------------------------------------------------
SHIFT = 3                     # sweeps of command/motor mismatch in the synthetic timings
STILL_BEFORE = 15             # sweeps (~1.5 s) without motor motion before a scored sweep
STILL_AFTER = 3               # sweeps without motor motion after a scored sweep
MAX_HEADING_DEG = 4.0
P90_HEADING_DEG = 3.0
MAX_OCC_RATIO = 1.25
MAX_MEDIAN_MS = 15.0

D = np.load(DATA)
RANGES = D["ranges"].astype(np.float32)
W_BRIDGE = D["w_bridge"].astype(float)
STAMP = D["stamp"].astype(float)
MOTOR = np.abs(W_BRIDGE) > 0.02          # sweeps during which the bridge drove the motors


def _rotation_deg(ref, cur):
    """CCW rotation (deg) of the robot between two stationary sweeps, exhaustive 1-deg search."""
    best_k, best_e = 0, None
    for k in range(-180, 180):
        c = np.roll(cur, -k)
        m = (ref > 0.05) & (c > 0.05) & (ref < 6.0) & (c < 6.0)
        if m.sum() < 200:
            continue
        e = float(np.median(np.abs(ref[m] - c[m])))
        if best_e is None or e < best_e:
            best_k, best_e = k, e
    return float(-best_k)


SCORED = [i for i in range(len(RANGES))
          if not MOTOR[max(0, i - STILL_BEFORE):i + STILL_AFTER + 1].any()]
TRUTH = {i: _rotation_deg(RANGES[0], RANGES[i]) for i in SCORED[::2]}


def _command(variant):
    on = np.flatnonzero(MOTOR)
    c = np.zeros_like(W_BRIDGE)
    for i in on:
        if variant == "recorded":
            c[i] = W_BRIDGE[i]
        elif variant == "widened":
            c[max(0, i - SHIFT):i + SHIFT + 1] = W_BRIDGE[i]
        elif variant == "leads":
            c[max(0, i - SHIFT)] = W_BRIDGE[i]
    return c


def _occupied(adapter):
    return int((adapter.grid_map.grid_core.log_odds > 0.6).sum())


def _oracle_occupied():
    ad = MotionAdapter()
    for i, deg in TRUTH.items():
        ad.grid_map.update_scan(0.0, 0.0, math.radians(deg), ranges=RANGES[i].tolist(),
                                max_valid_range=6.0, defer_costmap=False)
    return _occupied(ad)


ORACLE_OCC = _oracle_occupied()


def _replay(variant, production_mode):
    ad = MotionAdapter()
    if production_mode:
        ad.anchor_initialized = True  # mirrors telemetry_parser.py production path
    cmd = _command(variant)
    heading, times = [], []
    for i in range(len(RANGES)):
        dt = float(STAMP[i] - STAMP[i - 1]) if i else 0.1
        t0 = time.perf_counter()
        _, _, th, _ = ad.update_odometry(RANGES[i].tolist(), max(0.02, min(0.3, dt)), 0.0, float(cmd[i]))
        times.append((time.perf_counter() - t0) * 1000.0)
        heading.append(math.degrees(th))
    err = np.array([abs((heading[i] - deg + 180.0) % 360.0 - 180.0) for i, deg in TRUTH.items()])
    return err, _occupied(ad), float(np.median(times))


class TestSlamFieldTurns(unittest.TestCase):
    def _check(self, variant, production_mode):
        err, occ, med_ms = _replay(variant, production_mode)
        tag = f"{variant} anchor={'OFF(prod)' if production_mode else 'ON'}"
        print(f"\n[METRIC] {tag}: heading_err max={err.max():.1f} p90={np.percentile(err, 90):.1f} "
              f"median={np.median(err):.1f} deg | occupied={occ} oracle={ORACLE_OCC} "
              f"ratio={occ / ORACLE_OCC:.2f} | median {med_ms:.1f} ms", flush=True)
        self.assertLessEqual(err.max(), MAX_HEADING_DEG, f"{tag}: max heading error {err.max():.1f} deg")
        self.assertLessEqual(np.percentile(err, 90), P90_HEADING_DEG, f"{tag}: p90 heading error")
        self.assertLessEqual(occ, MAX_OCC_RATIO * ORACLE_OCC, f"{tag}: {occ} occupied cells (smeared map)")
        self.assertLessEqual(med_ms, MAX_MEDIAN_MS, f"{tag}: median update {med_ms:.1f} ms")

    def test_01_recorded_anchor_on(self):
        self._check("recorded", False)

    def test_02_recorded_production(self):
        self._check("recorded", True)

    def test_03_widened_anchor_on(self):
        self._check("widened", False)

    def test_04_widened_production(self):
        self._check("widened", True)

    def test_05_leads_anchor_on(self):
        self._check("leads", False)

    def test_06_leads_production(self):
        self._check("leads", True)


if __name__ == "__main__":
    print(f"[TRUTH] scored sweeps={len(TRUTH)} cumulative rotation at end={list(TRUTH.values())[-1]:+.0f} deg")
    unittest.main()
