"""
LOCKED acceptance test — written by the Claude Supervisor (TK-SLAM-10). Coders must NOT edit it or its data.

CSMTracker (via MotionAdapter.update_odometry) must flag weak scan matches WITHOUT touching the pose.
Interface fixed here (info dict, 4th return value of MotionAdapter.update_odometry):
  info["match_status"]      "TRACKING" | "WEAK" | "LOST"   (existing info["status"] "STILL"/"TRACKING" stays as is)
  info["match_score"]       float, per-point best fine score of this sweep
  info["reinit_requested"]  bool, True once LOST, cleared by MotionAdapter.set_pose / reset
  info["can_write_map"]     must be False while match_status is WEAK or LOST
Constants are provisional (10 bad sweeps in 2 recordings): SCORE_WEAK 0.64, LOST_AFTER 5 consecutive WEAK sweeps.

Real data: error vs LiDAR truth reuses test_slam_csm_turns.py (field + big turns). Person recordings
(docs/measurements/2026-11-16, robot still, people nearby) must stay TRACKING. Pose must equal the SLAM-09 golden
(tests/acceptance/data/slam09_golden.npz, made on HEAD e15a8e1 by make_slam09_golden.py) to 1e-9.
Synthetic known case: white-noise scans (no structure) must go WEAK then LOST and request re-init.

Run:  .venv/bin/python tests/acceptance/test_slam_csm_status.py -v
"""
import math
import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.dirname(__file__))
import test_slam_csm_turns as base  # noqa: E402  (locked SLAM-09 helpers: REC, truth, MotionAdapter)

ROOT = base.ROOT
GOLDEN = np.load(os.path.join(base.DATA_DIR, "slam09_golden.npz"))
PERSON = {k: os.path.join(ROOT, "docs", "measurements", "2026-11-16", f"lidar_{k}.npz")
          for k in ("static_person", "crossing", "head_on")}
BAD_DEG, GOOD_DEG = 6.0, 3.0
MIN_RECALL, MAX_FALSE_ALARM, MIN_TRACKING_PERSON = 0.80, 0.10, 0.98
LOST_AFTER = 5


def _replay(ranges, stamp, w):
    ad = base.MotionAdapter()
    ad.anchor_initialized = True
    poses, infos = [], []
    for i in range(len(ranges)):
        dt = float(stamp[i] - stamp[i - 1]) if i else 0.1
        x, y, th, info = ad.update_odometry(ranges[i].astype(np.float32).tolist(), max(0.02, min(0.3, dt)), 0.0,
                                            float(w[i]))
        poses.append((x, y, th))
        infos.append(info)
    return ad, np.array(poses), infos


def _status(infos):
    return [i["match_status"] for i in infos]  # KeyError here = interface not implemented (the expected red)


class TestCsmStatus(unittest.TestCase):
    def test_01_flags_bad_turn_sweeps(self):
        flagged_bad = total_bad = false_alarm = total_good = 0
        for name, rec in base.REC.items():
            _, poses, infos = _replay(rec["ranges"], rec["stamp"], rec["w"])
            st = _status(infos)
            err = np.abs(base._wrap(np.degrees(poses[:, 2]) - rec["truth"]))
            for e, s in zip(err, st):
                if e > BAD_DEG:
                    total_bad += 1
                    flagged_bad += s in ("WEAK", "LOST")
                elif e <= GOOD_DEG:
                    total_good += 1
                    false_alarm += s in ("WEAK", "LOST")
            self.assertNotIn("LOST", st, f"{name}: LOST on a recording that never loses the map")
        recall, fa = flagged_bad / max(1, total_bad), false_alarm / max(1, total_good)
        print(f"\n[METRIC] recall {flagged_bad}/{total_bad} = {recall:.2f}, false alarm {false_alarm}/{total_good} = {fa:.3f}",
              flush=True)
        self.assertGreater(total_bad, 0, "no sweep with error > 6 deg: truth or tracker changed, test is void")
        self.assertGreaterEqual(recall, MIN_RECALL, "bad sweeps not flagged")
        self.assertLessEqual(fa, MAX_FALSE_ALARM, "too many false alarms on good sweeps")

    def test_02_people_do_not_trigger(self):
        for name, path in PERSON.items():
            d = np.load(path)
            _, _, infos = _replay(d["ranges"], d["stamp"].astype(float), d["w_bridge"])
            st = _status(infos)
            frac = st.count("TRACKING") / len(st)
            print(f"\n[METRIC] {name}: TRACKING {100 * frac:.1f} % of {len(st)} sweeps", flush=True)
            self.assertGreaterEqual(frac, MIN_TRACKING_PERSON, f"{name}: person near a still robot flagged weak")
            self.assertNotIn("LOST", st, f"{name}: LOST")

    def test_03_pose_unchanged(self):
        recs = dict(((k, (v["ranges"], v["stamp"], v["w"])) for k, v in base.REC.items()))
        for k, p in PERSON.items():
            d = np.load(p)
            recs[k] = (d["ranges"], d["stamp"].astype(float), d["w_bridge"])
        for name, (r, s, w) in recs.items():
            _, poses, _ = _replay(r, s, w)
            worst = float(np.abs(poses - GOLDEN[name]).max())
            self.assertLessEqual(worst, 1e-9, f"{name}: published pose differs from SLAM-09 by {worst:.2e}")

    def test_04_noise_goes_weak_then_lost(self):
        rec = base.REC["field"]
        ad = base.MotionAdapter()
        ad.anchor_initialized = True
        for i in range(60):
            info = ad.update_odometry(rec["ranges"][i].astype(np.float32).tolist(), 0.1, 0.0, 0.0)[3]
        self.assertEqual(info["match_status"], "TRACKING", "clean still scene must be TRACKING")
        self.assertIsInstance(info["match_score"], float)
        self.assertFalse(info["reinit_requested"])
        rng = np.random.default_rng(0)
        seq = []
        for _ in range(LOST_AFTER + 2):
            info = ad.update_odometry(rng.uniform(0.5, 5.5, 360).astype(np.float32).tolist(), 0.1, 0.0, 0.0)[3]
            seq.append(info["match_status"])
            self.assertFalse(info["can_write_map"], "map must not be written while WEAK/LOST")
        self.assertEqual(seq[0], "WEAK", f"first noise sweep: {seq}")
        self.assertEqual(seq[LOST_AFTER - 2], "WEAK", f"must stay WEAK before {LOST_AFTER} in a row: {seq}")
        self.assertEqual(seq[LOST_AFTER - 1], "LOST", f"LOST after {LOST_AFTER} consecutive WEAK: {seq}")
        self.assertTrue(info["reinit_requested"], "LOST must request re-initialisation")
        ad.set_pose(0.0, 0.0, 0.0)
        info = ad.update_odometry(rec["ranges"][60].astype(np.float32).tolist(), 0.1, 0.0, 0.0)[3]
        self.assertFalse(info["reinit_requested"], "set_pose must clear reinit_requested")


if __name__ == "__main__":
    unittest.main()
