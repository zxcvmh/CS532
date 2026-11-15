"""
LOCKED acceptance test — written by the Claude Supervisor (TK-SLAM-09).
Antigravity agents must NOT edit this file or its data (see agents-doc/LOCKED_PATHS.txt).

Method change (human, 2026-11-15): heading must be TRACKED from the LiDAR with a wide-window correlative
scan matcher (Olson 2009 style), also DURING the turn — no dead-reckoning from the key press, no holding
the pose while the robot spins. The motor command is unreliable (start delay, coast), so the pose must
not depend on it at all.

REAL field data, two recordings (D500 on the robot, manual pivots, v = 0):
  field_turns_2026-11-15.npz  306 sweeps, 4 small left turns (+28..+97 deg)
  big_turns_2026-11-15.npz    184 sweeps, 4 large turns, left AND right (backend /ws recording, 21:07)

Command variants (the pose must not care):
  recorded : w_cmd as the bridge reported it
  none     : w_cmd = 0 for every sweep
  inverted : w_cmd with the wrong sign

Ground truth (independent of the code under test):
  heading  — exhaustive 1-deg circular alignment of each sweep against the first one. At MOVING sweeps the
             sweep is smeared by the rotation inside it (~30 deg at the pivot rate), so that bound is loose.
  position — the robot is NOT fixed during a pivot (wheel slip: 2-4 cm per turn in both recordings). At the
             middle sweep of every still segment: rotate its cloud by the truth heading (+-1 deg), then a 1 cm
             grid search of (dx, dy) in +-20 cm minimising the median nearest-point distance to the first
             still sweep's cloud. Residuals 0.2-1.5 cm.

Asserted per (recording, variant), MotionAdapter in production mode:
  still sweeps (no motor in the previous 1.5 s / next 0.3 s): heading max <= 5.0 deg, p90 <= 4.0 deg
  moving sweeps (motor within -0.3 s .. +0.8 s): heading worst <= 20 deg, p90 <= 10 deg  (= tracked, not held)
  position: |SLAM displacement - truth displacement| <= 3 cm at every still-segment middle sweep
  occupied cells <= 1.25 x oracle map (still sweeps written at the true heading)
  median MotionAdapter.update_odometry <= 15 ms on the laptop

Supervisor feasibility prototype (scratchpad, not in the repo): vectorised coarse-to-fine CSM (45 deg window,
1.5 deg coarse / 0.25 deg fine, +-10 cm), constant-velocity prior from its own previous estimate, submap fed
with LiDAR-still sweeps only and only when the pose changed (> 1.5 cm / 1 deg) or every 5th sweep:
  6/6 here — still max 2.8 / 4.5 deg, moving worst 14.5 / 12.5 deg, pos err 1.6 / 0.7 cm, ~5.7 ms;
  also passes field_turns 6/6, pivot_then_arc 7/7, stop_turn_go 2/2, inplace_turn 2/2.
Baseline (HEAD 7383775 + TK-SLAM-07/08 working tree): 6/6 FAIL — moving worst 62-179 deg, w_cmd=0 gives
24.5 deg still error and 10 cm position error.

Run:  .venv/bin/python tests/acceptance/test_slam_csm_turns.py -v
"""
import logging
import math
import os
import sys
import time
import unittest

import numpy as np
from scipy.spatial import cKDTree

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "dashboard", "backend"))
sys.path.insert(0, os.path.join(ROOT, "motion"))
logging.disable(logging.CRITICAL)

from app.navigation.motion_adapter import MotionAdapter  # noqa: E402

DATA_DIR = os.path.join(os.path.dirname(__file__), "data")
RECORDINGS = {"field": "field_turns_2026-11-15.npz", "big": "big_turns_2026-11-15.npz"}

# --- Locked evaluation constants -----------------------------------------------------------
STILL_BEFORE, STILL_AFTER = 15, 3       # sweeps without motor command around a still sweep
MOVE_BEFORE, MOVE_AFTER = 3, 8          # sweeps around a motor command counted as moving (incl. coast)
MAX_STILL_DEG, P90_STILL_DEG = 5.0, 4.0
MAX_MOVING_DEG, P90_MOVING_DEG = 20.0, 10.0
MAX_POS_ERR_M = 0.03
MAX_OCC_RATIO = 1.25
MAX_MEDIAN_MS = 15.0


def _rotation_deg(ref, cur):
    """CCW rotation (deg) of the robot between two sweeps, exhaustive 1-deg circular search."""
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


_ANG = np.radians(np.arange(360))


def _cloud(r):
    ok = (r > 0.05) & (r < 6.0)
    return np.column_stack([r[ok] * np.cos(_ANG[ok]), r[ok] * np.sin(_ANG[ok])])


def _translation(ref_r, cur_r, heading_deg):
    """LiDAR displacement (m) of cur relative to ref, in ref's frame, given the truth heading."""
    tree, cur = cKDTree(_cloud(ref_r)), _cloud(cur_r)
    best = None
    for dth in np.radians(np.arange(-1.0, 1.01, 0.25)):
        c, s = math.cos(math.radians(heading_deg) + dth), math.sin(math.radians(heading_deg) + dth)
        P = cur @ np.array([[c, s], [-s, c]])
        for dx in np.arange(-0.20, 0.201, 0.01):
            for dy in np.arange(-0.20, 0.201, 0.01):
                e = float(np.median(tree.query(P + (dx, dy))[0]))
                if best is None or e < best[0]:
                    best = (e, dx, dy)
    return np.array(best[1:])


def _load(name):
    d = np.load(os.path.join(DATA_DIR, RECORDINGS[name]))
    ranges = d["ranges"].astype(np.float32)
    w = d["w_bridge"].astype(float)
    motor = np.abs(w) > 0.02
    n = len(ranges)
    still = [i for i in range(n) if not motor[max(0, i - STILL_BEFORE):i + STILL_AFTER + 1].any()]
    moving = [i for i in range(n) if motor[max(0, i - MOVE_BEFORE):i + MOVE_AFTER + 1].any()]
    truth = np.array([_rotation_deg(ranges[0], ranges[i]) for i in range(n)])
    segs = np.split(np.array(still), np.flatnonzero(np.diff(still) > 1) + 1)
    mids = [int(sg[len(sg) // 2]) for sg in segs]
    s0 = still[0]
    trans = {i: _translation(ranges[s0], ranges[i], truth[i] - truth[s0]) for i in mids}
    return {"ranges": ranges, "w": w, "stamp": d["stamp"].astype(float),
            "still": still, "moving": moving, "truth": truth, "trans": trans}


REC = {k: _load(k) for k in RECORDINGS}


def _occupied(adapter):
    return int((adapter.grid_map.grid_core.log_odds > 0.6).sum())


def _oracle(rec):
    ad = MotionAdapter()
    for i in rec["still"][::2]:
        ad.grid_map.update_scan(0.0, 0.0, math.radians(rec["truth"][i]), ranges=rec["ranges"][i].tolist(),
                                max_valid_range=6.0, defer_costmap=False)
    return _occupied(ad)


ORACLE = {k: _oracle(v) for k, v in REC.items()}


def _wrap(a):
    return (a + 180.0) % 360.0 - 180.0


def _replay(rec, variant):
    ad = MotionAdapter()
    ad.anchor_initialized = True  # production path (telemetry_parser.py)
    w = {"recorded": rec["w"], "none": np.zeros_like(rec["w"]), "inverted": -rec["w"]}[variant]
    heading, pos, times = [], [], []
    for i in range(len(rec["ranges"])):
        dt = float(rec["stamp"][i] - rec["stamp"][i - 1]) if i else 0.1
        t0 = time.perf_counter()
        x, y, th, _ = ad.update_odometry(rec["ranges"][i].tolist(), max(0.02, min(0.3, dt)), 0.0, float(w[i]))
        times.append((time.perf_counter() - t0) * 1000.0)
        heading.append(math.degrees(th))
        pos.append((x, y))
    heading = np.array(heading)
    err = np.abs(_wrap(heading - rec["truth"]))
    s0 = rec["still"][0]
    pos_err = max(float(np.linalg.norm(np.subtract(pos[i], pos[s0]) - t)) for i, t in rec["trans"].items())
    return err[rec["still"]], err[rec["moving"]], pos_err, _occupied(ad), float(np.median(times))


class TestSlamCsmTurns(unittest.TestCase):
    def _check(self, name, variant):
        e_still, e_mov, drift, occ, med = _replay(REC[name], variant)
        tag = f"{name}/{variant}"
        print(f"\n[METRIC] {tag}: still max={e_still.max():.1f} p90={np.percentile(e_still, 90):.1f} deg | "
              f"moving worst={e_mov.max():.1f} p90={np.percentile(e_mov, 90):.1f} deg | pos err {100 * drift:.1f} cm | "
              f"occupied {occ}/{ORACLE[name]} = {occ / ORACLE[name]:.2f} | median {med:.1f} ms", flush=True)
        self.assertLessEqual(e_still.max(), MAX_STILL_DEG, f"{tag}: still heading max")
        self.assertLessEqual(np.percentile(e_still, 90), P90_STILL_DEG, f"{tag}: still heading p90")
        self.assertLessEqual(e_mov.max(), MAX_MOVING_DEG, f"{tag}: heading not tracked during the turn")
        self.assertLessEqual(np.percentile(e_mov, 90), P90_MOVING_DEG, f"{tag}: moving heading p90")
        self.assertLessEqual(drift, MAX_POS_ERR_M, f"{tag}: position error {100 * drift:.1f} cm vs LiDAR truth")
        self.assertLessEqual(occ, MAX_OCC_RATIO * ORACLE[name], f"{tag}: smeared map")
        self.assertLessEqual(med, MAX_MEDIAN_MS, f"{tag}: median update {med:.1f} ms")

    def test_01_field_recorded(self):
        self._check("field", "recorded")

    def test_02_field_no_command(self):
        self._check("field", "none")

    def test_03_field_inverted_command(self):
        self._check("field", "inverted")

    def test_04_big_recorded(self):
        self._check("big", "recorded")

    def test_05_big_no_command(self):
        self._check("big", "none")

    def test_06_big_inverted_command(self):
        self._check("big", "inverted")


if __name__ == "__main__":
    unittest.main()
