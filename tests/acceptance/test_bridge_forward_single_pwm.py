"""
LOCKED acceptance test — written by the Claude Supervisor (TK-INT-10).
Antigravity agents must NOT edit this file (see agents-doc/LOCKED_PATHS.txt).

Field fact (human, 2026-11-16): the robot rolls from ~0.08 PWM, so `min_drive_speed` is 0.08, not 0.19. The
bridge (dashboard/jetbot_bridge.py `set_motor_speeds`) still turns a forward command v into a PWM anywhere in
[min_drive_speed, cruise_speed] through a window around max_safe_speed_mps. With the new file a command of
0.10 drives at PWM 0.140 (~0.185 m/s), 0.20 at 0.175 and only 0.284 reaches cruise 0.2 — while the IL sim
(TK-IL-04), the SLAM tests and the measured actual_speed_mps (0.284 at cruise 0.2) all assume ONE forward speed.
It also steers a moving robot through wheel-speed differences that stop-turn-go does not have.

Required (stop-turn-go, like TK-INT-09 did for the pivot):
  * |v| >= 0.01 and no pivot: both logical wheel PWMs equal sign(v) * cruise_speed once the soft start has run,
    for ANY |v| and ANY w (w is ignored while driving). Trim is applied later, in set_raw_motor_speeds; do not
    apply it here. min_drive_speed and max_safe_speed_mps must not influence the forward PWM.
  * Soft start kept (setup/calibrate_motors.py measured the speed with `soft_start_drive`, a PWM ramp): the
    magnitude never decreases while the command holds, never exceeds cruise_speed, starts below cruise and
    reaches it within soft_start_ramp_sec (+ one 0.1 s call).
  * Backward is symmetric (-cruise_speed). v ~ 0 and w ~ 0 stops. A pivot followed by forward drive starts a
    clean forward ramp. Every constant is read from the module globals (checked with several dynamics sets).

The logical PWM is read from the bridge's own globals `_current_pwm_l` / `_current_pwm_r`, the values it passes
to `set_raw_motor_speeds`. Time is faked: one call = 0.1 s.

Run:  .venv/bin/python tests/acceptance/test_bridge_forward_single_pwm.py -v
"""
import math
import os
import sys
import types
import unittest
from unittest import mock

sys.modules.setdefault("serial", types.ModuleType("serial"))  # pyserial is Jetson-only

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "dashboard"))
import jetbot_bridge as br  # noqa: E402

DT = 0.1
EPS = 1e-6
FORWARD_COMMANDS = (0.02, 0.05, 0.10, 0.20, 0.284, 0.50)
# (cruise_speed, min_drive_speed, max_safe_speed_mps, soft_start_ramp_sec) — the field file, the old file, two others
DYNAMICS = ((0.20, 0.08, 0.10, 0.40), (0.20, 0.19, 0.14, 0.40), (0.17, 0.12, 0.14, 0.30), (0.23, 0.05, 0.30, 0.40))


class _Clock(object):
    def __init__(self):
        self.t = 5000.0

    def time(self):
        return self.t


class TestBridgeForwardSinglePwm(unittest.TestCase):
    def setUp(self):
        self.clock = _Clock()
        patcher = mock.patch.object(br.time, "time", self.clock.time)
        patcher.start()
        self.addCleanup(patcher.stop)
        br.HAS_MOTOR = False
        self._stop()

    def _call(self, v, w):
        self.clock.t += DT
        br.set_motor_speeds(v, w)
        return br._current_pwm_l, br._current_pwm_r

    def _stop(self):
        self._call(0.0, 0.0)
        self._call(0.0, 0.0)
        br._last_vel_cmd_time = 0.0

    def _dyn(self, cruise, min_drive, max_safe, ramp):
        p = mock.patch.multiple(br, CRUISE_SPEED=cruise, MIN_DRIVE_SPEED=min_drive,
                                MAX_SAFE_SPEED_MPS=max_safe, SOFT_START_RAMP_SEC=ramp)
        p.start()
        self.addCleanup(p.stop)

    def test_steady_forward_pwm_is_cruise_for_any_command(self):
        for cruise, min_drive, max_safe, ramp in DYNAMICS:
            with mock.patch.multiple(br, CRUISE_SPEED=cruise, MIN_DRIVE_SPEED=min_drive,
                                     MAX_SAFE_SPEED_MPS=max_safe, SOFT_START_RAMP_SEC=ramp):
                for v in FORWARD_COMMANDS:
                    with self.subTest(cruise=cruise, min_drive=min_drive, max_safe=max_safe, v=v):
                        self._stop()
                        for _ in range(20):
                            l, r = self._call(v, 0.0)
                        self.assertAlmostEqual(l, cruise, delta=EPS, msg="left PWM != cruise_speed")
                        self.assertAlmostEqual(r, cruise, delta=EPS, msg="right PWM != cruise_speed")

    def test_forward_ignores_angular_command(self):
        for w in (1.0, -1.0, 3.0, -3.0):
            with self.subTest(w=w):
                self._stop()
                for _ in range(20):
                    l, r = self._call(0.10, w)
                self.assertAlmostEqual(l, br.CRUISE_SPEED, delta=EPS, msg="left wheel steered by w")
                self.assertAlmostEqual(r, br.CRUISE_SPEED, delta=EPS, msg="right wheel steered by w")

    def test_backward_is_symmetric(self):
        for v in (-0.05, -0.20):
            with self.subTest(v=v):
                self._stop()
                for _ in range(20):
                    l, r = self._call(v, 0.0)
                self.assertAlmostEqual(l, -br.CRUISE_SPEED, delta=EPS)
                self.assertAlmostEqual(r, -br.CRUISE_SPEED, delta=EPS)

    def test_soft_start_ramp(self):
        for cruise, min_drive, max_safe, ramp in DYNAMICS:
            with mock.patch.multiple(br, CRUISE_SPEED=cruise, MIN_DRIVE_SPEED=min_drive,
                                     MAX_SAFE_SPEED_MPS=max_safe, SOFT_START_RAMP_SEC=ramp):
                with self.subTest(cruise=cruise, ramp=ramp):
                    self._stop()
                    mags = [abs(self._call(0.10, 0.0)[0]) for _ in range(12)]
                    self.assertLess(mags[0], cruise - EPS, "no soft start: the first command already jumps to cruise")
                    self.assertTrue(all(b >= a - EPS for a, b in zip(mags, mags[1:])), "PWM dipped during the ramp: %s" % mags)
                    self.assertLessEqual(max(mags), cruise + EPS, "PWM above cruise_speed")
                    reach = int(math.ceil(ramp / DT)) + 1
                    self.assertAlmostEqual(mags[reach - 1], cruise, delta=EPS,
                                           msg="ramp longer than soft_start_ramp_sec: %s" % mags)

    def test_pivot_then_forward_starts_a_clean_ramp(self):
        for _ in range(5):
            self._call(0.0, br.MAX_ANGULAR_SPEED_RADS)
        for _ in range(20):
            l, r = self._call(0.10, 0.0)
        self.assertAlmostEqual(l, br.CRUISE_SPEED, delta=EPS)
        self.assertAlmostEqual(r, br.CRUISE_SPEED, delta=EPS)
        self.assertEqual(br._current_spin_pwm, 0.0, "spin state leaked into forward drive")

    def test_stop_motors_restart_starts_clean_ramp(self):
        # Real stop path (watchdog, reflex, e-stop) calls stop_motors(), not set_motor_speeds(0, 0).
        # A restart after it must ramp from zero again, forward and backward (TK-INT-10 REJECT, 2026-11-16).
        for v in (0.10, -0.10):
            with self.subTest(v=v):
                self._stop()
                for _ in range(20):
                    self._call(v, 0.0)
                br.stop_motors()
                br._last_vel_cmd_time = 0.0
                clock_before = self.clock.t
                first = abs(self._call(v, 0.0)[0])
                self.assertLess(first, br.CRUISE_SPEED - EPS, "restart after stop_motors() jumped to cruise")
                self.assertLess(self.clock.t, clock_before + 1.0)

    def test_stop_command_stops(self):
        for _ in range(10):
            self._call(0.10, 0.0)
        l, r = self._call(0.0, 0.0)
        self.assertEqual((l, r), (0.0, 0.0))


if __name__ == "__main__":
    unittest.main()
