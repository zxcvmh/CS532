"""Generates slam09_golden.npz: the pose MotionAdapter publishes on HEAD e15a8e1 (TK-SLAM-09 tracker, before TK-SLAM-10).
TK-SLAM-10 must keep these poses bit-identical (status is extra output, never a correction).
Run once from the repo root:  .venv/bin/python tests/acceptance/data/make_slam09_golden.py
"""
import logging
import os
import sys

import numpy as np

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "dashboard", "backend"))
sys.path.insert(0, os.path.join(ROOT, "motion"))
logging.disable(logging.CRITICAL)
from app.navigation.motion_adapter import MotionAdapter  # noqa: E402

HERE = os.path.dirname(__file__)
FILES = {
    "field": os.path.join(HERE, "field_turns_2026-11-15.npz"),
    "big": os.path.join(HERE, "big_turns_2026-11-15.npz"),
    "static_person": os.path.join(ROOT, "docs", "measurements", "2026-11-16", "lidar_static_person.npz"),
    "crossing": os.path.join(ROOT, "docs", "measurements", "2026-11-16", "lidar_crossing.npz"),
    "head_on": os.path.join(ROOT, "docs", "measurements", "2026-11-16", "lidar_head_on.npz"),
}


def replay_poses(path):
    d = np.load(path)
    ad = MotionAdapter()
    ad.anchor_initialized = True
    out = []
    for i in range(len(d["ranges"])):
        dt = float(d["stamp"][i] - d["stamp"][i - 1]) if i else 0.1
        x, y, th, _ = ad.update_odometry(d["ranges"][i].astype(np.float32).tolist(), max(0.02, min(0.3, dt)), 0.0,
                                         float(d["w_bridge"][i]))
        out.append((x, y, th))
    return np.array(out)


if __name__ == "__main__":
    np.savez(os.path.join(HERE, "slam09_golden.npz"), **{k: replay_poses(p) for k, p in FILES.items()})
    print("written slam09_golden.npz")
