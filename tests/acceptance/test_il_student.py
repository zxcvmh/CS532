"""
LOCKED acceptance test — written by the Claude Supervisor (TK-IL-06). Coders must NOT edit it.

Why: the old BC/DAgger net regresses (v, w) with MSE, which averages "detour left" and "detour right" into "straight"
(train_bc.py:111), and its DAgger/BC data loops label states WITHOUT the teacher's `privileged` obstacle list (the path that
gets 15 % on head-on people). Decision (human, 2026-11-22): the student is a small MLP CLASSIFIER over the 4 stop-turn-go actions
on observation v2 (113-D, TK-IL-08), trained BC -> DAgger from the PRIVILEGED teacher. IL is the only avoider.

What this test fixes (interface, so a coder cannot grade itself):
  imitation_learning/student.py
      train_student(out_path, seed=0, n_bc_episodes=600, dagger_iters=4, dagger_episodes=200) -> dict
          writes ONE .npz with keys  W0,b0,W1,b1,...,W{L},b{L}  (x @ W + b, ReLU between layers, last layer = 4 logits,
          class order = config.ACTION_STOP, ACTION_FORWARD, ACTION_PIVOT_LEFT, ACTION_PIVOT_RIGHT) and the scalars
          seed, n_samples, dagger_iters. The FIRST layer eats 117 inputs = the raw 113-D observation v2 followed by a 4-D one-hot
          of the PREVIOUS executed action (all zeros at step 0), in the same class order. Why: the teacher keeps a hysteresis
          (`_last_turn_dir`: no reversing while pivoting in place, mock_expert_simulator.py:~690); a memoryless student sees the
          same symmetric state after every 14-deg pivot and chatters left/right forever (measured on the Supervisor reference:
          46-64 % timeouts on static / standing). Fold any input normalisation into W0/b0.
          Every training label comes from GeometricExpertPolicy.get_action(..., privileged=<true obstacles>).
          Episode seeds used for training must be >= 20000 (the evaluation seeds below are < 1000).
  imitation_learning/models/student_v2.npz   the trained artifact (committed with the ticket)
This test loads the .npz and runs the forward pass ITSELF in numpy: what is measured is the real weights, not coder code, and
the student gets ONLY the observation (no privileged list, no override). So "NN-driven steps" is 100 % by construction.

Closed loop: stop-turn-go sim (TK-IL-04), 10 Hz, 450 steps, robot/person radii and goal tolerance of test_il_sim_success,
episodes from the locked generators (IL-01 group A, TK-IL-08 static / standing, TK-IL-05 B1 crossing / B2 head-on),
100 episodes per class. Person mask = the sim's ideal flag (no detector noise yet: TK-IL-07 injects it).
Gate per class: success >= 90 %, collision <= 2 %. Head-on is a FLOOR (success >= 85 %, collision <= 8 %): its teacher
collides 4 % today and TK-IL-09 owns the 2 % target and the 0.30 m clearance.

Run:  .venv/bin/python tests/acceptance/test_il_student.py -v
      (IL_STUDENT_MODEL=<file> overrides the artifact path, for the Supervisor prototype only)
"""
import math
import os
import random
import sys
import tempfile
import time
import unittest

import numpy as np

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, os.path.dirname(__file__))
sys.path.insert(0, os.path.join(ROOT, "imitation_learning"))
import test_il_expert as X  # noqa: E402  (locked generators and constants)
import test_il_observation as O  # noqa: E402
import config as C  # noqa: E402
import dataset as D  # noqa: E402
from mock_expert_simulator import GeometricExpertPolicy, SimulationEnvironment  # noqa: E402

T = X.T
MODEL = os.environ.get("IL_STUDENT_MODEL", os.path.join(ROOT, "imitation_learning", "models", "student_v2.npz"))
N_EP = 100
IN_DIM = C.OBSERVATION_DIM_V2 + 4
MAX_PARAMS = 100_000
MAX_FWD_P95_MS = 2.0
TRAIN_SEED_MIN = 20000
# class -> (evaluation seeds, min success %, max collision %)
CLASSES = {
    "A": (range(0, 100), 90.0, 2.0),
    "static": (range(100, 200), 90.0, 2.0),
    "standing": (range(100, 200), 90.0, 2.0),
    "crossing": (range(300, 400), 90.0, 2.0),
    "headon": (range(300, 400), 85.0, 8.0),
}


def _load(path):
    z = np.load(path, allow_pickle=False)
    n = len([k for k in z.files if k.startswith("W")])
    return [z["W%d" % i].astype(np.float32) for i in range(n)], [z["b%d" % i].astype(np.float32) for i in range(n)], z


def _act(Ws, bs, obs):
    h = obs.astype(np.float32)
    for W, b in zip(Ws[:-1], bs[:-1]):
        h = np.maximum(h @ W + b, 0.0)
    return int(np.argmax(h @ Ws[-1] + bs[-1]))


def _episode(kind, seed):
    """-> (start, goal, ped dict or None, statics)."""
    if kind == "A":
        return X._episode_a(seed)
    if kind in ("static", "standing"):
        start, goal, statics, ped = O._episode(seed, kind)
        return start, goal, ped, statics
    return X._episode_b(seed, "B1" if kind == "crossing" else "B2")


def _drive(ep, choose):
    """Closed loop. choose(x117) -> action int. Returns (outcome, min clearance m)."""
    start, goal, ped, statics = ep
    random.seed(0)
    np.random.seed(0)
    env = SimulationEnvironment(arena_width=T.ARENA_W, arena_height=T.ARENA_H)
    env.reset(custom_start=start, custom_goal=goal, num_obstacles=0)
    half_h = T.ARENA_H / 2
    px, py = (ped["x"], ped["y"]) if ped else (0.0, 0.0)
    gone, prev, minc, last = ped is None, None, 9.0, np.zeros(4, np.float32)
    coop = bool(ped and ped.get("coop"))
    for step in range(T.MAX_STEPS):
        if ped is not None:
            moving = step >= ped["delay"] and not gone
            if moving and coop and math.hypot(px - env.x, py - env.y) <= T.PED_STOP_DIST_M + T.PED_RADIUS_M:
                moving = False
            vx, vy = (ped["vx"], ped["vy"]) if moving else (0.0, 0.0)
            if moving:
                px, py = px + vx * T.DT, py + vy * T.DT
            if coop:
                if moving and abs(py) > half_h - 0.20:
                    gone = step > ped["delay"] + 5
            else:
                gone = abs(py) > half_h + 0.3 or px < -0.3 or px > T.ARENA_W + 0.3
        persons = [] if gone else [(px, py, T.PED_RADIUS_M)]
        env.obstacles = list(statics) + persons
        scan = env.get_lidar_scan()
        pose0 = (env.x, env.y, env.theta)
        obs = D.extract_observation_vector_v2(
            scan, goal[0], goal[1], *pose0, prev_ranges_360=None if prev is None else prev[0],
            ego_dpose=None if prev is None else O._ego(*prev[1], *pose0), person_mask_360=env.get_person_mask360(persons))
        prev = (scan, pose0)
        act = int(choose(np.concatenate([obs, last])))
        last = np.zeros(4, np.float32)
        last[act] = 1.0
        env.step_action(act, dt=T.DT)
        for a, b, r in env.obstacles:
            d = math.hypot(env.x - a, env.y - b) - (T.ROBOT_RADIUS_M + r)
            minc = min(minc, d)
            if d < 0:
                return "collision", minc
        if not (T.ROBOT_RADIUS_M < env.x < T.ARENA_W - T.ROBOT_RADIUS_M
                and -half_h + T.ROBOT_RADIUS_M < env.y < half_h - T.ROBOT_RADIUS_M):
            return "collision", minc
        if math.hypot(goal[0] - env.x, goal[1] - env.y) <= T.GOAL_TOL_M:
            return "success", minc
    return "timeout", minc


class TestStudentArtifact(unittest.TestCase):
    def test_01_artifact_format_and_budget(self):
        self.assertTrue(os.path.isfile(MODEL), "student artifact missing: %s" % MODEL)
        Ws, bs, z = _load(MODEL)
        self.assertGreaterEqual(len(Ws), 2, "need at least one hidden layer")
        self.assertEqual(Ws[0].shape[0], IN_DIM, "first layer must eat obs v2 (113) + previous-action one-hot (4)")
        self.assertEqual(Ws[-1].shape[1], 4, "last layer = 4 logits (stop, forward, pivot left, pivot right)")
        for i in range(len(Ws) - 1):
            self.assertEqual(Ws[i].shape[1], Ws[i + 1].shape[0], "layer %d -> %d shapes do not chain" % (i, i + 1))
        for i, (W, b) in enumerate(zip(Ws, bs)):
            self.assertEqual(b.shape, (W.shape[1],))
            self.assertTrue(np.isfinite(W).all() and np.isfinite(b).all(), "layer %d has NaN/inf" % i)
        params = sum(int(W.size + b.size) for W, b in zip(Ws, bs))
        self.assertLessEqual(params, MAX_PARAMS, "student too large for the Jetson budget")
        for key in ("seed", "n_samples", "dagger_iters"):
            self.assertIn(key, z.files, "metadata key %s missing" % key)
        self.assertGreaterEqual(int(z["n_samples"]), 20000, "too few training samples to be a real BC/DAgger set")
        self.assertGreaterEqual(int(z["dagger_iters"]), 2, "DAgger needs >= 2 aggregation iterations")
        rng = np.random.RandomState(0)
        xs = [rng.uniform(0, 1, IN_DIM).astype(np.float32) for _ in range(200)]
        t = []
        for x in xs:
            t0 = time.perf_counter()
            _act(Ws, bs, x)
            t.append((time.perf_counter() - t0) * 1000.0)
        p95 = float(np.percentile(t, 95))
        print("\n[METRIC] student: %d params, %d layers, numpy forward p95 %.3f ms" % (params, len(Ws), p95), flush=True)
        self.assertLessEqual(p95, MAX_FWD_P95_MS)


class TestStudentClosedLoop(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.Ws, cls.bs, _ = _load(MODEL)

    def _class(self, kind):
        seeds, min_s, max_c = CLASSES[kind]
        res = [_drive(_episode(kind, s), lambda o: _act(self.Ws, self.bs, o)) for s in seeds]
        out = [o for o, _ in res]
        clear = np.array([c for _, c in res])
        s, c = 100.0 * out.count("success") / len(out), 100.0 * out.count("collision") / len(out)
        print("\n[METRIC] student class %s: success %.0f %% collision %.0f %% timeout %.0f %% | min clearance p5 %.3f m, "
              "share >= 0.30 m %.0f %% (NN drives 100 %% of steps)" % (
                  kind, s, c, 100.0 * out.count("timeout") / len(out), np.percentile(clear, 5), 100 * np.mean(clear >= 0.30)),
              flush=True)
        self.assertGreaterEqual(s, min_s, "%s: success" % kind)
        self.assertLessEqual(c, max_c, "%s: collision" % kind)

    def test_02_group_a_il01_episodes(self):
        self._class("A")

    def test_03_static_obstacle_on_path(self):
        self._class("static")

    def test_04_standing_person_on_path(self):
        self._class("standing")

    def test_05_crossing_person(self):
        self._class("crossing")

    def test_06_head_on_person(self):
        self._class("headon")


class TestTrainingPipeline(unittest.TestCase):
    def test_07_train_student_runs_and_labels_come_from_the_privileged_teacher(self):
        try:
            import student
        except ImportError as e:
            self.fail("imitation_learning/student.py (train_student) missing: %s" % e)
        calls = {"priv": 0, "plain": 0}
        orig = GeometricExpertPolicy.get_action

        def counting(self_, *a, **kw):
            calls["priv" if kw.get("privileged") is not None else "plain"] += 1
            return orig(self_, *a, **kw)

        GeometricExpertPolicy.get_action = counting
        try:
            with tempfile.TemporaryDirectory() as td:
                out = os.path.join(td, "tiny.npz")
                t0 = time.time()
                info = student.train_student(out, seed=0, n_bc_episodes=30, dagger_iters=2, dagger_episodes=10)
                dt = time.time() - t0
                self.assertTrue(os.path.isfile(out), "train_student did not write out_path")
                Ws, bs, z = _load(out)
                self.assertEqual(Ws[0].shape[0], IN_DIM)
                self.assertEqual(Ws[-1].shape[1], 4)
                self.assertEqual(int(z["dagger_iters"]), 2)
                self.assertIsInstance(info, dict)
        finally:
            GeometricExpertPolicy.get_action = orig
        print("\n[METRIC] tiny train_student: %.0f s, teacher labels with privileged %d, without %d" % (
            dt, calls["priv"], calls["plain"]), flush=True)
        self.assertGreater(calls["priv"], 1000, "the teacher was barely queried: labels cannot come from it")
        self.assertEqual(calls["plain"], 0, "a teacher label was made WITHOUT `privileged` (15 % head-on path)")
        self.assertLessEqual(dt, 600.0, "tiny training budget exceeded 10 minutes")


if __name__ == "__main__":
    unittest.main()
