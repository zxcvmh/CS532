"""
LOCKED acceptance test — written by the Claude Supervisor (TK-IL-01, TK-IL-02).
Antigravity agents must NOT edit this file (see agents-doc/LOCKED_PATHS.txt).

Evaluates the policy the RUNTIME actually loads (dashboard/backend/app/navigation/il_onnx_bridge.py
-> get_il_policy_runner) on 100 fixed-seed episodes, each with one pedestrian crossing the
robot's path plus 0-2 small static obstacles, and asserts real outcome metrics:

    success >= 90 %, collision <= 2 %   (thresholds decided by the human, 2026-11-15)

Run:  .venv/bin/python tests/acceptance/test_il_sim_success.py -v
"""
import importlib.util
import json
import math
import os
import random
import sys
import time as _real_time
import unittest

import numpy as np

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
IL_DIR = os.path.join(ROOT, "imitation_learning")
BRIDGE_FILE = os.path.join(ROOT, "dashboard", "backend", "app", "navigation", "il_onnx_bridge.py")
DYN_FILE = os.path.join(ROOT, "setup", "robot_dynamics.json")
sys.path.insert(0, IL_DIR)

# --- Locked evaluation constants (the coder cannot tune these) ---------------------------
N_EPISODES = 100
SEEDS = list(range(N_EPISODES))
DT = 0.10                     # control period, s
MAX_STEPS = 450               # 45 s budget for a ~1.9 m route at <= 0.14 m/s
ROBOT_RADIUS_M = 0.09         # JetBot 15x10 cm chassis, circumscribed (backend Config.ROBOT_RADIUS)
PED_RADIUS_M = 0.18           # adult torso footprint
PED_STOP_DIST_M = 0.35        # a human stops instead of walking into the robot
GOAL_TOL_M = 0.15
ARENA_W, ARENA_H = 3.0, 2.4
MIN_SUCCESS_PCT = 90.0
MAX_COLLISION_PCT = 2.0
MIN_NN_SHARE_PCT = 30.0       # hybrid allowed, but the learned net must keep driving (baseline 40.2 %)


class _SimClock:
    """Feeds the runner simulated time so its slew limiter sees DT, not wall-clock speed."""
    def __init__(self):
        self.t = 1000.0

    def time(self):
        return self.t

    perf_counter = staticmethod(_real_time.perf_counter)


def _load_runtime_runner():
    spec = importlib.util.spec_from_file_location("il_onnx_bridge_under_test", BRIDGE_FILE)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    runner = mod.get_il_policy_runner()
    if runner is None:
        raise RuntimeError("Runtime found no ONNX policy (get_il_policy_runner() returned None)")
    return runner


def _make_episode(seed):
    rng = random.Random(seed)
    start = (rng.uniform(0.40, 0.60), rng.uniform(-0.30, 0.30), rng.uniform(-0.3, 0.3))
    goal = (rng.uniform(2.30, 2.50), rng.uniform(-0.30, 0.30))
    # Pedestrian crosses the straight route between 35 % and 75 % of the way.
    frac = rng.uniform(0.35, 0.75)
    cx = start[0] + frac * (goal[0] - start[0])
    side = rng.choice([-1.0, 1.0])
    ped = {"x": cx + rng.uniform(-0.15, 0.15), "y": side * (ARENA_H / 2 - 0.25),
           "vy": -side * rng.uniform(0.25, 0.50), "delay": rng.randint(0, 40)}
    statics = []
    for _ in range(rng.randint(0, 2)):
        f = rng.uniform(0.2, 0.9)
        ox = start[0] + f * (goal[0] - start[0])
        oy = start[1] + f * (goal[1] - start[1]) + rng.choice([-1, 1]) * rng.uniform(0.05, 0.25)
        statics.append((ox, oy, rng.uniform(0.04, 0.08)))
    return start, goal, ped, statics


def _run_episode(seed, act):
    """act(scan, goal, x, y, th, v, w) -> (v, w). Returns 'success' | 'collision' | 'timeout'."""
    from mock_expert_simulator import SimulationEnvironment
    random.seed(seed)
    np.random.seed(seed)
    start, goal, ped, statics = _make_episode(seed)
    env = SimulationEnvironment(arena_width=ARENA_W, arena_height=ARENA_H)
    env.reset(custom_start=start, custom_goal=goal, num_obstacles=0)
    half_h = ARENA_H / 2
    px, py, done_crossing = ped["x"], ped["y"], False

    for step in range(MAX_STEPS):
        # Pedestrian: walks once across, stops near the robot, exits at the far side.
        if step >= ped["delay"] and not done_crossing:
            if math.hypot(px - env.x, py - env.y) > PED_STOP_DIST_M + PED_RADIUS_M:
                py += ped["vy"] * DT
            if abs(py) > half_h - 0.20:
                done_crossing = step > ped["delay"] + 5
        env.obstacles = list(statics) + ([] if done_crossing else [(px, py, PED_RADIUS_M)])

        v, w = act(env.get_lidar_scan(), goal, env.x, env.y, env.theta, env.v, env.w)
        env.step(float(v), float(w), dt=DT)

        for ox, oy, r in env.obstacles:
            if math.hypot(env.x - ox, env.y - oy) < ROBOT_RADIUS_M + r:
                return "collision"
        if not (ROBOT_RADIUS_M < env.x < ARENA_W - ROBOT_RADIUS_M
                and -half_h + ROBOT_RADIUS_M < env.y < half_h - ROBOT_RADIUS_M):
            return "collision"
        if math.hypot(goal[0] - env.x, goal[1] - env.y) <= GOAL_TOL_M:
            return "success"
    return "timeout"


def _rates(outcomes):
    n = len(outcomes)
    return {k: 100.0 * outcomes.count(k) / n for k in ("success", "collision", "timeout")}


class TestILSimAcceptance(unittest.TestCase):

    def test_runtime_policy_avoids_crossing_pedestrians(self):
        """TK-IL-01: runtime policy, 100 seeded episodes, success >= 90 %, collision <= 2 %."""
        import il_policy_runner
        clock = _SimClock()
        il_policy_runner.time = clock          # simulated time for the runner's slew limiter
        runner = _load_runtime_runner()
        counts = {"nn": 0, "steps": 0}
        _forward = runner.runner.forward

        def _counted_forward(obs):
            counts["nn"] += 1
            return _forward(obs)
        runner.runner.forward = _counted_forward

        def act(scan, goal, x, y, th, v, w):
            clock.t += DT
            counts["steps"] += 1
            cv, cw, _ = runner.compute_command(lidar_ranges_360=scan, goal_x=goal[0], goal_y=goal[1],
                                               robot_x=x, robot_y=y, robot_theta=th, curr_v=v, curr_w=w)
            return cv, cw

        outcomes = []
        for s in SEEDS:
            runner.active_v = runner.active_w = 0.0
            runner.last_step_time = 0.0
            outcomes.append(_run_episode(s, act))
        r = _rates(outcomes)
        failed = [s for s, o in zip(SEEDS, outcomes) if o != "success"]
        print(f"\n[ACCEPTANCE TK-IL-01] episodes={N_EPISODES} success={r['success']:.1f}% "
              f"collision={r['collision']:.1f}% timeout={r['timeout']:.1f}% "
              f"(need success>={MIN_SUCCESS_PCT}%, collision<={MAX_COLLISION_PCT}%)")
        nn_share = 100.0 * counts["nn"] / max(1, counts["steps"])
        print(f"[ACCEPTANCE TK-IL-01] failed seeds: {failed[:30]}{' ...' if len(failed) > 30 else ''}")
        print(f"[ACCEPTANCE TK-IL-01] NN-driven steps: {nn_share:.1f}% (need >= {MIN_NN_SHARE_PCT}%)")
        self.assertGreaterEqual(r["success"], MIN_SUCCESS_PCT)
        self.assertLessEqual(r["collision"], MAX_COLLISION_PCT)
        self.assertGreaterEqual(nn_share, MIN_NN_SHARE_PCT)

    def test_expert_baseline_reported(self):
        """Expert on the same seeds. Informational: a policy cannot reliably beat its teacher."""
        from mock_expert_simulator import GeometricExpertPolicy
        expert = GeometricExpertPolicy()

        def act(scan, goal, x, y, th, v, w):
            return expert.get_action(scan, goal[0], goal[1], x, y, th)

        outcomes = []
        for s in SEEDS:
            expert.reset_state()
            outcomes.append(_run_episode(s, act))
        r = _rates(outcomes)
        print(f"\n[ACCEPTANCE EXPERT] success={r['success']:.1f}% collision={r['collision']:.1f}% "
              f"timeout={r['timeout']:.1f}%")

    def test_sim_physics_matches_field_dynamics(self):
        """TK-IL-02: the sim/policy speed limits must come from setup/robot_dynamics.json."""
        import config as il_config
        with open(DYN_FILE, encoding="utf-8") as f:
            dyn = json.load(f)
        self.assertAlmostEqual(il_config.MIN_DRIVE_SPEED, dyn["min_drive_speed"], places=3,
                               msg="IL deadband (config.MIN_DRIVE_SPEED) != field min_drive_speed")
        # TK-IL-04 (2026-11-16): the old `MAX_LINEAR_SPEED <= max_safe_speed_mps` line is gone — the robot
        # physically drives at actual_speed_mps (0.294 > max_safe 0.14); speed/pivot physics are now locked
        # by tests/acceptance/test_il_sim_dynamics.py.


if __name__ == "__main__":
    unittest.main()
