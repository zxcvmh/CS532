"""
LOCKED acceptance test — written by the Claude Supervisor (TK-IL-04).
Antigravity agents must NOT edit this file (see agents-doc/LOCKED_PATHS.txt).

The IL simulator must move the robot the way the REAL robot moves: stop-turn-go with ONE forward speed
(`actual_speed_mps`) and ONE pivot rate ((pi/2) / `turn_time_90_deg_sec`), both read from
setup/robot_dynamics.json — never typed into the code. This mirrors dashboard/jetbot_bridge.py
`set_motor_speeds` (TK-INT-09): any |w| >= 0.02 with v ~ 0 pivots at the single calibrated rate, any
v >= 0.01 drives at the single calibrated speed, and a moving robot has no steering authority.

Contract the coder must satisfy (only imitation_learning/config.py and mock_expert_simulator.py may change):
  * config.py reads the dynamics file from env var ROBOT_DYNAMICS_FILE if set, else <repo>/setup/robot_dynamics.json.
  * config.ACTION_STOP/FORWARD/PIVOT_LEFT/PIVOT_RIGHT == 0/1/2/3.
  * config.MAX_LINEAR_SPEED == actual_speed_mps, config.MAX_ANGULAR_SPEED == (pi/2)/turn_time_90_deg_sec,
    config.MIN_DRIVE_SPEED == min_drive_speed (names stay importable: model.py etc. import them).
  * SimulationEnvironment.step(cmd_v, cmd_w, dt) and the new SimulationEnvironment.step_action(action, dt)
    obey the physics below.

Anti-hardcode: every physics check also runs against a SYNTHETIC dynamics file (different speed, 90-degree
time and min_drive_speed). A simulator that copied 0.294 / 0.3 into the code fails there.

Run:  .venv/bin/python tests/acceptance/test_il_sim_dynamics.py -v
"""
import json
import math
import os
import subprocess
import sys
import tempfile
import unittest

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
IL_DIR = os.path.join(ROOT, "imitation_learning")
DYN_FILE = os.path.join(ROOT, "setup", "robot_dynamics.json")

DT = 0.10                      # control period, s (config.CONTROL_HZ = 10)
FWD_TOL = 0.03                 # +-3 %: the field speed already contains floor slip, so a second 5 % slip fails
PIVOT_TOL = 0.03               # +-3 % on the pivot rate
SPEED_INDEPENDENCE_TOL = 0.01  # any v >= 0.01 gives the same speed
MAX_HEADING_CHANGE_RAD = math.radians(2.0)   # per 1 s of driving: no steering while moving
MAX_PIVOT_TRANSLATION_M = 0.05  # real pivots slip 2-4 cm; ideal 0 is fine
MAX_STOP_CREEP_M = 0.01

# Runs inside a fresh interpreter so config.py is imported under the dynamics file we choose.
PROBE = r"""
import json, math, os, sys
sys.path.insert(0, os.environ["IL_DIR"])
out = {"errors": []}
def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi
try:
    import config
    from mock_expert_simulator import SimulationEnvironment
except Exception as e:
    print("PROBE_JSON " + json.dumps({"errors": ["import: %r" % (e,)]})); sys.exit(0)

for name in ("ACTION_STOP", "ACTION_FORWARD", "ACTION_PIVOT_LEFT", "ACTION_PIVOT_RIGHT",
             "MAX_LINEAR_SPEED", "MAX_ANGULAR_SPEED", "MIN_DRIVE_SPEED"):
    out[name] = getattr(config, name, None)

def fresh():
    env = SimulationEnvironment()
    env.reset(custom_goal=(2.9, 0.0), custom_start=(0.8, 0.0, 0.0), num_obstacles=0)
    env.obstacles = []
    return env

def drive(fn, n):
    env = fresh()
    x0, y0, th = env.x, env.y, env.theta
    dth = 0.0
    for _ in range(n):
        prev = env.theta
        fn(env)
        dth += wrap(env.theta - prev)
    return {"dist": math.hypot(env.x - x0, env.y - y0), "dth": dth, "T": n * __DT__}

def probe(key, fn, n):
    try:
        out[key] = drive(fn, n)
    except Exception as e:
        out["errors"].append("%s: %r" % (key, e))

DT = __DT__
probe("fwd_v016", lambda e: e.step(0.16, 0.0, DT), 30)
for v in (0.02, 0.05, 0.10, 0.30):
    probe("fwd_v%g" % v, lambda e, v=v: e.step(v, 0.0, DT), 10)
probe("fwd_w_pos", lambda e: e.step(0.16, 1.0, DT), 10)
probe("fwd_w_neg", lambda e: e.step(0.16, -1.0, DT), 10)
for w in (0.5, 3.0, -0.5, -3.0):
    probe("pivot_w%g" % w, lambda e, w=w: e.step(0.0, w, DT), 20)
probe("act_forward", lambda e: e.step_action(config.ACTION_FORWARD, DT), 30)
probe("act_left", lambda e: e.step_action(config.ACTION_PIVOT_LEFT, DT), 20)
probe("act_right", lambda e: e.step_action(config.ACTION_PIVOT_RIGHT, DT), 20)

try:
    env = fresh()
    for _ in range(10):
        env.step(0.16, 0.0, DT)
    x0, y0, th0 = env.x, env.y, env.theta
    for _ in range(5):
        env.step(0.0, 0.0, DT)
    out["stop"] = {"dist": math.hypot(env.x - x0, env.y - y0), "dth": wrap(env.theta - th0)}
    env = fresh()
    for _ in range(5):
        env.step_action(config.ACTION_STOP, DT)
    out["act_stop"] = {"dist": math.hypot(env.x - 0.8, env.y), "dth": wrap(env.theta)}
except Exception as e:
    out["errors"].append("stop: %r" % (e,))
print("PROBE_JSON " + json.dumps(out))
""".replace("__DT__", repr(DT))


def _run_probe(dyn_path):
    env = dict(os.environ, ROBOT_DYNAMICS_FILE=dyn_path, IL_DIR=IL_DIR)
    r = subprocess.run([sys.executable, "-I", "-c", PROBE], env=env, cwd=IL_DIR,
                       stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True, timeout=300)
    for line in r.stdout.splitlines():
        if line.startswith("PROBE_JSON "):
            return json.loads(line[len("PROBE_JSON "):])
    raise AssertionError("probe produced no result (rc=%s)\nstderr tail:\n%s" % (r.returncode, r.stderr[-1500:]))


class _Res(object):
    """r[key] returns a probe result or fails with the probe's own error, so one missing API does not mask others."""
    def __init__(self, raw):
        self.raw = raw

    def __getitem__(self, key):
        if key not in self.raw:
            raise AssertionError("no result for %r; probe errors: %s" % (key, self.raw["errors"]))
        return self.raw[key]


def per_dynamics(fn):
    """Run the test body once against the real dynamics file and once against the synthetic one."""
    def run(self):
        for label, dyn, raw in self.cases:
            with self.subTest(dynamics=label):
                fn(self, label, dyn, _Res(raw))
    run.__name__ = fn.__name__
    run.__doc__ = fn.__doc__
    return run


class TestSimMatchesFieldDynamics(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        with open(DYN_FILE, encoding="utf-8") as f:
            real = json.load(f)
        synth = dict(real)
        synth.update({"actual_speed_mps": 0.41, "turn_time_90_deg_sec": 0.54, "turn_time_180_deg_sec": 1.08,
                      "turn_speed": 0.14, "min_drive_speed": 0.23})
        cls.tmp = tempfile.mkdtemp(prefix="il_dyn_")
        cls.synth_path = os.path.join(cls.tmp, "synthetic_dynamics.json")
        with open(cls.synth_path, "w", encoding="utf-8") as f:
            json.dump(synth, f)
        cls.cases = [("real file", real, _run_probe(DYN_FILE)),
                     ("synthetic file", synth, _run_probe(cls.synth_path))]

    @classmethod
    def tearDownClass(cls):
        try:
            os.remove(cls.synth_path)
            os.rmdir(cls.tmp)
        except OSError:
            pass

    @per_dynamics
    def test_probe_imports_and_step_action_exist(self, label, dyn, r):
        self.assertEqual(r.raw["errors"], [], "probe errors (missing step_action / import failure?)")

    @per_dynamics
    def test_config_contract(self, label, dyn, r):
        self.assertEqual([r["ACTION_STOP"], r["ACTION_FORWARD"], r["ACTION_PIVOT_LEFT"],
                          r["ACTION_PIVOT_RIGHT"]], [0, 1, 2, 3], "ACTION_* constants must be 0/1/2/3")
        self.assertAlmostEqual(r["MAX_LINEAR_SPEED"], dyn["actual_speed_mps"], delta=0.01 * dyn["actual_speed_mps"])
        w_pivot = (math.pi / 2.0) / dyn["turn_time_90_deg_sec"]
        self.assertAlmostEqual(r["MAX_ANGULAR_SPEED"], w_pivot, delta=0.01 * w_pivot)
        self.assertAlmostEqual(r["MIN_DRIVE_SPEED"], dyn["min_drive_speed"], places=3)

    @per_dynamics
    def test_forward_speed_is_field_speed(self, label, dyn, r):
        for key in ("fwd_v016", "act_forward"):
            speed = r[key]["dist"] / r[key]["T"]
            print("[METRIC] %s %s: %.4f m/s (field %.4f)" % (label, key, speed, dyn["actual_speed_mps"]))
            self.assertAlmostEqual(speed, dyn["actual_speed_mps"], delta=FWD_TOL * dyn["actual_speed_mps"],
                                   msg="%s: forward speed != actual_speed_mps (deadband / slip / old caps?)" % key)

    @per_dynamics
    def test_forward_has_one_speed_no_deadband(self, label, dyn, r):
        ref = dyn["actual_speed_mps"]
        for key in ("fwd_v0.02", "fwd_v0.05", "fwd_v0.1", "fwd_v0.3"):
            speed = r[key]["dist"] / r[key]["T"]
            self.assertAlmostEqual(speed, ref, delta=max(SPEED_INDEPENDENCE_TOL, FWD_TOL) * ref,
                                   msg="%s: any v >= 0.01 must drive at the single field speed" % key)

    @per_dynamics
    def test_moving_robot_cannot_steer(self, label, dyn, r):
        for key in ("fwd_w_pos", "fwd_w_neg"):
            self.assertLessEqual(abs(r[key]["dth"]) / r[key]["T"], MAX_HEADING_CHANGE_RAD,
                                 "%s: heading changed while driving forward (stop-turn-go has no arcs)" % key)

    @per_dynamics
    def test_pivot_rate_is_single_field_rate(self, label, dyn, r):
        rate = (math.pi / 2.0) / dyn["turn_time_90_deg_sec"]
        for key, sign in (("pivot_w0.5", 1), ("pivot_w3", 1), ("pivot_w-0.5", -1), ("pivot_w-3", -1),
                          ("act_left", 1), ("act_right", -1)):
            got = r[key]["dth"] / r[key]["T"]
            print("[METRIC] %s %s: %.3f rad/s (field %.3f)" % (label, key, got, sign * rate))
            self.assertAlmostEqual(got, sign * rate, delta=PIVOT_TOL * rate,
                                   msg="%s: pivot must turn at +-(pi/2)/turn_time_90_deg_sec for any |w|" % key)
            self.assertLessEqual(r[key]["dist"], MAX_PIVOT_TRANSLATION_M, "%s: pivot translated the robot" % key)

    @per_dynamics
    def test_stop_stops(self, label, dyn, r):
        for key in ("stop", "act_stop"):
            self.assertLessEqual(r[key]["dist"], MAX_STOP_CREEP_M, "%s: robot kept moving after STOP" % key)
            self.assertLessEqual(abs(r[key]["dth"]), math.radians(1.0), "%s: robot kept turning after STOP" % key)


if __name__ == "__main__":
    unittest.main()
