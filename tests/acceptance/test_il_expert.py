"""
LOCKED acceptance test — written by the Claude Supervisor (TK-IL-05). Coders must NOT edit it.

The teacher (`GeometricExpertPolicy`) must drive the stop-turn-go sim (TK-IL-04) to the goal around people and obstacles.
Root cause found 2026-11-21: the old expert emits ARCS (v > 0 AND w != 0, e.g. `w = -2.8 * lat_err`), the sim executes
only "forward" OR "pivot" and silently drops w while moving, so every evasive steer is lost: 47 % success / 52 %
collision on the locked seeds. A Supervisor prototype (scratchpad, not in the repo: a lookahead over "pivot n steps, then
forward" plans, replanned every step, STOP when nothing is safe) gets, on THESE groups: A 99 % / 0 %, B1 85-89 % /
11-15 %, B2 93-94 % / 6-7 % (margin/horizon sweeps did not beat that); a version WITHOUT velocity gets 2 % on head-on.

Interface fixed by this test (old callers keep working, `privileged` is optional):
  GeometricExpertPolicy().get_action(lidar_ranges, goal_x, goal_y, robot_x, robot_y, robot_theta,
                                     apply_dart_noise=False, privileged=None) -> (v, w)
  privileged = list of true obstacles (x, y, radius, vx, vy) in the world frame (the sim knows them; the STUDENT never
  gets this, it only sees LiDAR, so this is a teacher-only input).
  The returned command must be stop-turn-go: (0, 0) | (v > 0, 0) | (0, w != 0). Never an arc.

Groups (robot radius, ped radius, arena 3.0 x 2.4 m, 10 Hz, 450 steps, goal 0.15 m = same constants as test_il_sim_success):
  A  seeds 0-99, the exact IL-01 episodes (one crossing person who STOPS when the robot is near + 0-2 static obstacles)
       GATE: success >= 95 %, collision <= 1 %
  B1 seeds 100-199, person crosses at 0.25-0.6 m/s and never stops
  B2 seeds 200-299, person walks head-on at 0.25-0.6 m/s and never stops
       B is a FLOOR, not a target: success >= 80 % / collision <= 18 % (B1), >= 88 % / <= 10 % (B2), set just below the
       prototype so a teacher that ignores velocity fails (it gets ~2 % head-on). The real target is 90 % / 2 %
       (human acceptance: no contact). How much of the B collisions is physically unavoidable for a robot that needs
       0.64 s to pivot 90 deg and drives 0.284 m/s is NOT measured yet: TK-IL-08 must measure it and replace this floor.
Run:  .venv/bin/python tests/acceptance/test_il_expert.py -v
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
import test_il_sim_success as T  # noqa: E402  (locked IL-01 constants and episode generator)
from mock_expert_simulator import GeometricExpertPolicy, SimulationEnvironment  # noqa: E402

DT, MAX_STEPS = T.DT, T.MAX_STEPS
B_SPEED = (0.25, 0.60)
LIMITS = {"A": (95.0, 1.0), "B1": (80.0, 18.0), "B2": (88.0, 10.0)}  # B = floor, see docstring


def _episode_b(seed, kind):
    rng = random.Random(7000 + seed)
    start = (rng.uniform(0.40, 0.60), rng.uniform(-0.30, 0.30), rng.uniform(-0.3, 0.3))
    goal = (rng.uniform(2.30, 2.50), rng.uniform(-0.30, 0.30))
    half_h = T.ARENA_H / 2
    speed = rng.uniform(*B_SPEED)
    if kind == "B1":
        side = rng.choice([-1.0, 1.0])
        x = start[0] + rng.uniform(0.35, 0.75) * (goal[0] - start[0]) + rng.uniform(-0.15, 0.15)
        ped = {"x": x, "y": side * (half_h - 0.25), "vx": 0.0, "vy": -side * speed}
    else:
        ped = {"x": goal[0] + 0.1, "y": start[1] + rng.uniform(-0.12, 0.12), "vx": -speed, "vy": 0.0}
    ped.update(delay=rng.randint(0, 40), coop=False)
    return start, goal, ped, []


def _episode_a(seed):
    start, goal, ped, statics = T._make_episode(seed)
    return start, goal, {"x": ped["x"], "y": ped["y"], "vx": 0.0, "vy": ped["vy"], "delay": ped["delay"], "coop": True}, statics


def _call(expert, scan, goal, x, y, th, priv):
    kw = {"privileged": priv} if "privileged" in inspect.signature(expert.get_action).parameters else {}
    return expert.get_action(scan, goal[0], goal[1], x, y, th, **kw)


def _run(ep, expert):
    """-> (outcome, arcs) with outcome in success | collision | timeout; arcs = commands that are not stop-turn-go."""
    start, goal, ped, statics = ep
    random.seed(0)
    np.random.seed(0)
    env = SimulationEnvironment(arena_width=T.ARENA_W, arena_height=T.ARENA_H)
    env.reset(custom_start=start, custom_goal=goal, num_obstacles=0)
    half_h = T.ARENA_H / 2
    px, py, gone, arcs = ped["x"], ped["y"], False, 0
    expert.reset_state()
    for step in range(MAX_STEPS):
        moving = step >= ped["delay"] and not gone
        if moving and ped["coop"] and math.hypot(px - env.x, py - env.y) <= T.PED_STOP_DIST_M + T.PED_RADIUS_M:
            moving = False                      # cooperative person stops instead of walking into the robot
        vx, vy = (ped["vx"], ped["vy"]) if moving else (0.0, 0.0)
        if moving:
            px, py = px + vx * DT, py + vy * DT
        if ped["coop"]:
            if moving and abs(py) > half_h - 0.20:
                gone = step > ped["delay"] + 5
        else:
            gone = abs(py) > half_h + 0.3 or px < -0.3 or px > T.ARENA_W + 0.3
        env.obstacles = list(statics) + ([] if gone else [(px, py, T.PED_RADIUS_M)])
        priv = [(ox, oy, r, 0.0, 0.0) for ox, oy, r in statics] + ([] if gone else [(px, py, T.PED_RADIUS_M, vx, vy)])
        v, w = _call(expert, env.get_lidar_scan(), goal, env.x, env.y, env.theta, priv)
        if abs(v) >= 0.01 and abs(w) >= 0.02:
            arcs += 1
        env.step(float(v), float(w), dt=DT)
        for ox, oy, r in env.obstacles:
            if math.hypot(env.x - ox, env.y - oy) < T.ROBOT_RADIUS_M + r:
                return "collision", arcs
        if not (T.ROBOT_RADIUS_M < env.x < T.ARENA_W - T.ROBOT_RADIUS_M
                and -half_h + T.ROBOT_RADIUS_M < env.y < half_h - T.ROBOT_RADIUS_M):
            return "collision", arcs
        if math.hypot(goal[0] - env.x, goal[1] - env.y) <= T.GOAL_TOL_M:
            return "success", arcs
    return "timeout", arcs


def _group(name):
    if name == "A":
        eps = [_episode_a(s) for s in range(100)]
    elif name == "B1":
        eps = [_episode_b(s, "B1") for s in range(100, 200)]
    else:
        eps = [_episode_b(s, "B2") for s in range(200, 300)]
    expert = GeometricExpertPolicy()
    res = [_run(e, expert) for e in eps]
    outcomes = [o for o, _ in res]
    return T._rates(outcomes), sum(a for _, a in res), [i for i, o in enumerate(outcomes) if o != "success"]


class TestExpert(unittest.TestCase):
    def _check(self, name):
        r, arcs, failed = _group(name)
        need_s, max_c = LIMITS[name]
        print(f"\n[METRIC] group {name}: success {r['success']:.0f} % collision {r['collision']:.0f} % timeout "
              f"{r['timeout']:.0f} % | arcs {arcs} | failed idx {failed[:15]} (need success >= {need_s}, collision <= {max_c})",
              flush=True)
        self.assertEqual(arcs, 0, f"{name}: expert emitted {arcs} arc commands (v and w together); the sim drops w while driving")
        self.assertGreaterEqual(r["success"], need_s, f"{name}: success")
        self.assertLessEqual(r["collision"], max_c, f"{name}: collision")

    def test_01_interface_accepts_privileged(self):
        self.assertIn("privileged", inspect.signature(GeometricExpertPolicy.get_action).parameters,
                      "get_action must take the optional teacher-only `privileged` obstacle list")

    def test_02_group_A_cooperative_person(self):
        self._check("A")

    def test_03_group_B1_crossing_person_never_stops(self):
        self._check("B1")

    def test_04_group_B2_head_on_person_never_stops(self):
        self._check("B2")


if __name__ == "__main__":
    unittest.main()
