"""
LOCKED acceptance test — written by the Claude Supervisor (TK-IL-08). Coders must NOT edit it.

Why: the student only sees ONE LiDAR frame (41-D: 36 beams, goal, v/w forced to 0). A person crossing or walking at the
robot and a wall/static obstacle at the same range give the SAME observation, so no network can learn to yield to one and
not to the other. Human decision 2026-11-16: IL is the only avoider, the person flag comes from the camera (YOLO) and a motion cue
goes into the observation.

Observation v2 (113-D) = the legacy 41-D UNCHANGED (old runtime/ONNX/tests keep working) + 2 new per-beam blocks:
  [0:41]    exactly what `extract_observation_vector` returns today
  [41:77]   person flag per beam (0.0 / 1.0): 1 if ANY ray of the beam's 10-deg sector [10b-5, 10b+4] is flagged as person
  [77:113]  ego-compensated motion per beam in [-1, 1] (a person is judged by its STRONGEST flagged beam; edge beams are diluted): the previous scan is moved into the current robot frame with the
            robot's own motion (dx, dy, dth in the PREVIOUS robot frame), then
            motion = clip(median over the sector of (previous_compensated_range - current_range) / 0.10 m, -1, 1)
            so +0.5 = something got 5 cm closer in one 0.1 s step, -0.5 = receded, ~0 = static scenery even while the robot
            pivots (stop-turn-go pivots 14 deg per step) or drives.
New API (fixed here):
  config.OBSERVATION_DIM (41) unchanged; config.OBSERVATION_DIM_V2 = 113; config.MOTION_NORM_M = 0.10
  dataset.extract_observation_vector_v2(raw_ranges_360, goal_x, goal_y, robot_x, robot_y, robot_theta,
        prev_ranges_360=None, ego_dpose=None, person_mask_360=None, curr_v=0.0, curr_w=0.0) -> float32 (113,)
        (None prev/ego -> motion block zeros; None mask -> person block zeros)
  dataset.mirror_sample_v2(obs113, act) -> (obs113', act'): the legacy mirror on [0:41] AND the same left/right beam swap
        on both new blocks (index i <-> (36 - i) % 36); motion values keep their sign; mirroring twice is the identity
  SimulationEnvironment.get_person_mask360(persons, miss_rate=0.0, false_alarm_rate=0.0, rng=None) -> bool array (360,)
        persons = [(x, y, r), ...] circles that are people; ray d is True iff its FIRST hit is one of them (occlusion respected);
        miss_rate = per call, per person, probability that the detector misses that person completely;
        false_alarm_rate = per call probability that a random 20-deg window of rays is flagged although it is no person.
Supervisor prototype (scratchpad, not in the repo) measured: static scenery max |motion| 0.01 under a 14 deg pivot and a
2.8 cm step; person walking head-on at 0.5 m/s +0.50, receding -0.50, with the robot still or pivoting.

Scenario classes (4 x 50 seeds, the expert is the teacher with `privileged`): static obstacle on the path, standing person
on the path, person crossing (never stops), person head-on (never stops, 0.25-0.6 m/s). Gate: success >= 90 %, collision <= 5 %
per class (head-on is 4 % today; the human target 2 % and the 0.30 m clearance are NOT gated here: TK-IL-09). Clearance is printed.

Run:  .venv/bin/python tests/acceptance/test_il_observation.py -v
"""
import inspect
import math
import os
import random
import sys
import unittest

import numpy as np

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, os.path.dirname(__file__))
sys.path.insert(0, os.path.join(ROOT, "imitation_learning"))
import test_il_expert as X  # noqa: E402  (locked: episode generators, constants)
import config as C  # noqa: E402
import dataset as D  # noqa: E402
from mock_expert_simulator import GeometricExpertPolicy, SimulationEnvironment  # noqa: E402

T = X.T
NORM = 0.10


def _env(x, y, th, obstacles):
    e = SimulationEnvironment(arena_width=3.0, arena_height=2.4)
    e.reset(custom_start=(x, y, th), custom_goal=(2.5, 0.0), num_obstacles=0)
    e.obstacles = list(obstacles)
    return e


def _ego(x0, y0, th0, x1, y1, th1):
    dx, dy, c, s = x1 - x0, y1 - y0, math.cos(th0), math.sin(th0)
    return (c * dx + s * dy, -s * dx + c * dy, (th1 - th0 + math.pi) % (2 * math.pi) - math.pi)


def _v2(prev, cur, ego, mask, goal=(2.5, 0.0), pose=(1.0, 0.0, 0.0)):
    return D.extract_observation_vector_v2(cur, goal[0], goal[1], pose[0], pose[1], pose[2],
                                           prev_ranges_360=prev, ego_dpose=ego, person_mask_360=mask)


def _sector(mask360, b):
    return bool(any(mask360[(10 * b + d) % 360] for d in range(-5, 5)))


def _ray_hits(x, y, th, deg, px, py, r):
    """first-intersection distance of ray `deg` (robot frame, CCW) with the circle, or None."""
    a = th + math.radians(deg)
    dx, dy = x - px, y - py
    b = 2.0 * (dx * math.cos(a) + dy * math.sin(a))
    c = dx * dx + dy * dy - r * r
    disc = b * b - 4.0 * c
    if disc < 0:
        return None
    t = (-b - math.sqrt(disc)) / 2.0
    return t if t > 0 else None


class TestObservationV2(unittest.TestCase):
    def test_01_dims_layout_and_legacy_identical(self):
        self.assertEqual(C.OBSERVATION_DIM, 41, "legacy dimension must stay 41 (runtime / ONNX / old tests)")
        self.assertEqual(getattr(C, "OBSERVATION_DIM_V2", None), 113)
        self.assertAlmostEqual(getattr(C, "MOTION_NORM_M", -1.0), NORM, places=9)
        rng = np.random.RandomState(3)
        scan = (rng.uniform(0.3, 2.9, 360)).tolist()
        legacy = D.extract_observation_vector(scan, 2.0, 0.5, 0.4, 0.1, 0.3)
        v2 = D.extract_observation_vector_v2(scan, 2.0, 0.5, 0.4, 0.1, 0.3)
        self.assertEqual(v2.shape, (113,))
        self.assertEqual(v2.dtype, np.float32)
        np.testing.assert_array_equal(v2[:41], legacy, "first 41 entries must equal the legacy vector bit for bit")
        self.assertEqual(float(np.abs(v2[41:]).max()), 0.0, "no mask / no previous scan -> person and motion blocks are zero")

    def test_02_person_mask_matches_geometry_and_occlusion(self):
        person = (2.0, 0.0, 0.18)
        e = _env(1.0, 0.0, 0.0, [person])
        mask = np.asarray(e.get_person_mask360([person]))
        self.assertEqual(mask.shape, (360,))
        expected = np.array([_ray_hits(1.0, 0.0, 0.0, d, *person) is not None for d in range(360)])
        self.assertLessEqual(int((mask != expected).sum()), 2, "person rays differ from the analytic circle silhouette")
        self.assertTrue(mask[0] and not mask[90] and not mask[180])
        blocker = (1.5, 0.0, 0.10)                                  # static obstacle between robot and person hides the centre
        e2 = _env(1.0, 0.0, 0.0, [blocker, person])
        m2 = np.asarray(e2.get_person_mask360([person]))
        self.assertFalse(m2[0], "ray 0 hits the blocker first: it must NOT be flagged as person")
        self.assertTrue(m2.sum() < mask.sum(), "occlusion must remove person rays")
        self.assertFalse(np.asarray(e2.get_person_mask360([])).any(), "no persons -> no flagged ray")

    def test_03_detector_noise_rates(self):
        person = (2.0, 0.0, 0.18)
        e = _env(1.0, 0.0, 0.0, [person])
        rng = random.Random(5)
        miss = sum(1 for _ in range(2000) if not np.asarray(e.get_person_mask360([person], miss_rate=0.2, rng=rng))[0])
        self.assertAlmostEqual(miss / 2000.0, 0.2, delta=0.03, msg="miss_rate not honoured")
        rng = random.Random(6)
        fa = 0
        for _ in range(2000):
            m = np.asarray(e.get_person_mask360([], false_alarm_rate=0.1, rng=rng))
            if m.any():
                fa += 1
                self.assertTrue(15 <= int(m.sum()) <= 25, "a false alarm is one ~20 deg window of rays")
        self.assertAlmostEqual(fa / 2000.0, 0.1, delta=0.03, msg="false_alarm_rate not honoured")

    def test_04_person_block_from_mask(self):
        scan = [2.0] * 360
        mask = np.zeros(360, bool)
        mask[[44, 130, 131]] = True                                 # rays 44 -> sector of beam 4 ([35,44]); 130,131 -> beam 13
        obs = D.extract_observation_vector_v2(scan, 2.5, 0.0, 1.0, 0.0, 0.0, person_mask_360=mask)
        flags = obs[41:77]
        self.assertEqual([i for i in range(36) if flags[i] == 1.0], [4, 13])
        self.assertTrue(set(np.unique(flags)) <= {0.0, 1.0})
        mask[:] = False
        mask[355] = True                                            # ray 355 is in beam 0's sector [-5, 4]
        obs = D.extract_observation_vector_v2(scan, 2.5, 0.0, 1.0, 0.0, 0.0, person_mask_360=mask)
        self.assertEqual([i for i in range(36) if obs[41 + i] == 1.0], [0])

    def test_05_motion_static_scenery_is_quiet_even_when_the_robot_moves(self):
        rng = random.Random(11)
        for name, move in (("pivot 14 deg", lambda x, y, th: (x, y, th + 0.245)),
                           ("forward 2.8 cm", lambda x, y, th: (x + 0.028 * math.cos(th), y + 0.028 * math.sin(th), th))):
            fracs, maxes = [], []
            for k in range(40):
                x, y, th = rng.uniform(0.5, 2.5), rng.uniform(-0.8, 0.8), rng.uniform(-math.pi, math.pi)
                obs = [] if k % 2 == 0 else [(x + rng.uniform(0.3, 1.0), y + rng.uniform(-0.5, 0.5), 0.1)]
                prev = _env(x, y, th, obs).get_lidar_scan()
                x1, y1, th1 = move(x, y, th)
                cur = _env(x1, y1, th1, obs).get_lidar_scan()
                m = _v2(prev, cur, _ego(x, y, th, x1, y1, th1), None)[77:113]
                fracs.append(float(np.mean(np.abs(m) >= 0.3)))
                maxes.append(float(np.abs(m).max()))
            print(f"\n[METRIC] static scenery, robot {name}: beams |motion|>=0.3 mean {np.mean(fracs):.3f}; "
                  f"max |motion| p95 {np.percentile(maxes, 95):.3f}", flush=True)
            self.assertLessEqual(float(np.mean(fracs)), 0.01, f"{name}: static scenery looks like motion")
            self.assertLessEqual(float(np.percentile(maxes, 95)), 0.10, f"{name}: static scenery looks like motion")

    def test_06_motion_sees_people(self):
        rng = random.Random(12)
        for rname, move in (("still", lambda x, y, th: (x, y, th)), ("pivoting", lambda x, y, th: (x, y, th + 0.245))):
            for pname, vel, lo in (("head-on 0.5 m/s", (-0.5, 0.0), +0.35), ("receding 0.5 m/s", (0.5, 0.0), -0.35)):
                vals = []
                for _ in range(40):
                    x, y, th = rng.uniform(0.5, 1.0), rng.uniform(-0.3, 0.3), rng.uniform(-0.2, 0.2)
                    d = rng.uniform(0.9, 1.4)
                    px, py = x + d * math.cos(th), y + d * math.sin(th)
                    prev = _env(x, y, th, [(px, py, 0.18)]).get_lidar_scan()
                    x1, y1, th1 = move(x, y, th)
                    p1 = (px + vel[0] * 0.1, py + vel[1] * 0.1, 0.18)
                    e1 = _env(x1, y1, th1, [p1])
                    cur, mask = e1.get_lidar_scan(), np.asarray(e1.get_person_mask360([p1]))
                    o = _v2(prev, cur, _ego(x, y, th, x1, y1, th1), mask)
                    on = [b for b in range(36) if o[41 + b] == 1.0]
                    self.assertTrue(on, "the person must be flagged in the person block")
                    blk = o[77:113][on]                      # edge beams cover only part of the person: take the strongest one
                    vals.append(float(blk[np.argmax(np.abs(blk))]))
                med, tail = float(np.median(vals)), float(np.percentile(vals, 10) if lo > 0 else np.percentile(vals, 90))
                print(f"\n[METRIC] person {pname}, robot {rname}: strongest person beam: median {med:+.2f}, tail {tail:+.2f}",
                      flush=True)
                if lo > 0:
                    self.assertGreaterEqual(med, lo, f"{pname}/{rname}: approaching person not seen")
                    self.assertGreaterEqual(tail, 0.25, f"{pname}/{rname}: approaching person not seen (tail)")
                else:
                    self.assertLessEqual(med, lo, f"{pname}/{rname}: receding person not seen")
                    self.assertLessEqual(tail, -0.25, f"{pname}/{rname}: receding person not seen (tail)")

    def test_07_mirror_v2(self):
        rng = np.random.RandomState(7)
        obs = np.concatenate([rng.uniform(0, 1, 41), rng.randint(0, 2, 36), rng.uniform(-1, 1, 36)]).astype(np.float32)
        act = np.array([0.2, 0.5], dtype=np.float32)
        m_obs, m_act = D.mirror_sample_v2(obs, act)
        l_obs, l_act = D.mirror_sample(obs[:41], act)
        np.testing.assert_array_equal(m_obs[:41], l_obs)
        np.testing.assert_array_equal(m_act, l_act)
        for off in (41, 77):
            for i in range(36):
                self.assertEqual(m_obs[off + i], obs[off + (36 - i) % 36], f"block at {off}: beam {i} not mirrored")
        back, back_act = D.mirror_sample_v2(m_obs, m_act)
        np.testing.assert_array_equal(back, obs)
        np.testing.assert_array_equal(back_act, act)


def _episode(seed, kind):
    rng = random.Random(9000 + seed)
    start = (rng.uniform(0.40, 0.60), rng.uniform(-0.30, 0.30), rng.uniform(-0.3, 0.3))
    goal = (rng.uniform(2.30, 2.50), rng.uniform(-0.30, 0.30))
    if kind in ("static", "standing"):
        f = rng.uniform(0.35, 0.70)
        ox = start[0] + f * (goal[0] - start[0])
        oy = start[1] + f * (goal[1] - start[1]) + rng.uniform(-0.04, 0.04)
        return start, goal, ([(ox, oy, rng.uniform(0.08, 0.15))] if kind == "static" else []), \
            ({"x": ox, "y": oy, "vx": 0.0, "vy": 0.0, "delay": 0} if kind == "standing" else None)
    ep = X._episode_b(seed, "B1" if kind == "crossing" else "B2")
    return ep[0], ep[1], [], ep[2]


def _run(seed, kind, expert, with_obs):
    start, goal, statics, ped = _episode(seed, kind)
    random.seed(0)
    np.random.seed(0)
    env = SimulationEnvironment(arena_width=T.ARENA_W, arena_height=T.ARENA_H)
    env.reset(custom_start=start, custom_goal=goal, num_obstacles=0)
    half_h = T.ARENA_H / 2
    expert.reset_state()
    px, py = (ped["x"], ped["y"]) if ped else (0.0, 0.0)
    gone, prev, minc, bad = ped is None, None, 9.0, 0
    for step in range(T.MAX_STEPS):
        vx = vy = 0.0
        if ped and not gone and step >= ped["delay"]:
            vx, vy = ped["vx"], ped["vy"]
            px, py = px + vx * T.DT, py + vy * T.DT
            gone = abs(py) > half_h + 0.3 or px < -0.3 or px > T.ARENA_W + 0.3
        persons = [] if gone else [(px, py, T.PED_RADIUS_M)]
        env.obstacles = list(statics) + persons
        scan = env.get_lidar_scan()
        pose0 = (env.x, env.y, env.theta)
        if with_obs:
            mask = env.get_person_mask360(persons)
            ego = None if prev is None else _ego(*prev[1], *pose0)
            o = D.extract_observation_vector_v2(scan, goal[0], goal[1], *pose0, prev_ranges_360=None if prev is None else prev[0],
                                                ego_dpose=ego, person_mask_360=mask)
            ok = (o.shape == (113,) and bool(np.isfinite(o).all()) and set(np.unique(o[41:77])) <= {0.0, 1.0}
                  and float(np.abs(o[77:113]).max()) <= 1.0 and (not np.asarray(mask).any() or o[41:77].any()))
            bad += 0 if ok else 1
            prev = (scan, pose0)
        priv = [(a, b, r, 0.0, 0.0) for a, b, r in statics] + [(px, py, T.PED_RADIUS_M, vx, vy) for _ in persons]
        v, w = expert.get_action(scan, goal[0], goal[1], env.x, env.y, env.theta, privileged=priv)
        env.step(float(v), float(w), dt=T.DT)
        for a, b, r in env.obstacles:
            d = math.hypot(env.x - a, env.y - b) - (T.ROBOT_RADIUS_M + r)
            minc = min(minc, d)
            if d < 0:
                return "collision", minc, bad
        if not (T.ROBOT_RADIUS_M < env.x < T.ARENA_W - T.ROBOT_RADIUS_M and -half_h + T.ROBOT_RADIUS_M < env.y < half_h - T.ROBOT_RADIUS_M):
            return "collision", minc, bad
        if math.hypot(goal[0] - env.x, goal[1] - env.y) <= T.GOAL_TOL_M:
            return "success", minc, bad
    return "timeout", minc, bad


class TestScenarioClasses(unittest.TestCase):
    def _class(self, kind):
        has_v2 = hasattr(D, "extract_observation_vector_v2") and hasattr(SimulationEnvironment, "get_person_mask360")
        expert = GeometricExpertPolicy()
        res = [_run(s, kind, expert, has_v2) for s in range(50)]
        out = [o for o, _, _ in res]
        clear = np.array([c for _, c, _ in res])
        bad = sum(b for _, _, b in res)
        s, c = 2.0 * out.count("success"), 2.0 * out.count("collision")
        print(f"\n[METRIC] class {kind}: success {s:.0f} % collision {c:.0f} % timeout {2.0 * out.count('timeout'):.0f} % | "
              f"min clearance p5 {np.percentile(clear, 5):.3f} m, share >= 0.30 m {100 * np.mean(clear >= 0.30):.0f} % | "
              f"invalid v2 observations {bad}", flush=True)
        self.assertTrue(has_v2, "observation v2 / get_person_mask360 missing: the loop cannot build the student's input")
        self.assertEqual(bad, 0, f"{kind}: v2 observation invalid (shape, NaN, flag values, motion range, or a visible person not flagged)")
        self.assertGreaterEqual(s, 90.0, f"{kind}: success")
        self.assertLessEqual(c, 5.0, f"{kind}: collision")

    def test_08_static_obstacle_on_path(self):
        self._class("static")

    def test_09_standing_person_on_path(self):
        self._class("standing")

    def test_10_crossing_person(self):
        self._class("crossing")

    def test_11_head_on_person(self):
        self._class("headon")


if __name__ == "__main__":
    unittest.main()
