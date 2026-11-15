"""Supervisor test (TK-INT-07): the bridge must apply the spin PWM as a step, exactly like
setup/calibrate_motors.py measured turn_time_90_deg_sec (drive_raw, no ramp). turn_speed equals
min_drive_speed, so any ramp below it is motor dead time that SLAM cannot model."""
import os
import sys
import types
import unittest

sys.modules.setdefault("serial", types.ModuleType("serial"))  # pyserial is Jetson-only

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(ROOT, "dashboard"))
import jetbot_bridge as br  # noqa: E402


class TestBridgeSpinStep(unittest.TestCase):
    def setUp(self):
        br.HAS_MOTOR = False
        br.set_motor_speeds(0.0, 0.0)
        br._last_vel_cmd_time = 0.0

    def test_first_spin_command_reaches_turn_speed(self):
        br.set_motor_speeds(0.0, br.MAX_ANGULAR_SPEED_RADS)
        self.assertAlmostEqual(abs(br._current_spin_pwm), br.TURN_SPEED, places=6)
        self.assertGreater(br._current_spin_pwm, 0.0)

    def test_cw_spin_sign(self):
        br.set_motor_speeds(0.0, -br.MAX_ANGULAR_SPEED_RADS)
        self.assertAlmostEqual(br._current_spin_pwm, -br.TURN_SPEED, places=6)

    def test_partial_w_spins_at_turn_speed_even_below_min_drive(self):
        """TK-INT-09: only ONE spin PWM was calibrated (turn_speed -> turn_time_90_deg_sec). With
        turn_speed (0.14) < min_drive_speed (0.19) the old interpolation min + (turn - min) * ratio made a
        partial joystick / slider command spin FASTER (0.19) than a full one (0.14)."""
        saved = (br.TURN_SPEED, br.MIN_DRIVE_SPEED)
        try:
            br.TURN_SPEED, br.MIN_DRIVE_SPEED = 0.14, 0.19
            for ratio in (0.1, 0.5, 1.0):
                br.set_motor_speeds(0.0, 0.0)
                br.set_motor_speeds(0.0, ratio * br.MAX_ANGULAR_SPEED_RADS)
                self.assertAlmostEqual(br._current_spin_pwm, 0.14, places=6, msg=f"w ratio {ratio}")
                br.set_motor_speeds(0.0, 0.0)  # stop first: the bridge slews w, it cannot flip sign in 10 ms
                br.set_motor_speeds(0.0, -ratio * br.MAX_ANGULAR_SPEED_RADS)
                self.assertAlmostEqual(br._current_spin_pwm, -0.14, places=6, msg=f"w ratio -{ratio}")
        finally:
            br.TURN_SPEED, br.MIN_DRIVE_SPEED = saved

    def test_stop_zeroes_spin(self):
        br.set_motor_speeds(0.0, br.MAX_ANGULAR_SPEED_RADS)
        br.set_motor_speeds(0.0, 0.0)
        self.assertEqual(br._current_spin_pwm, 0.0)
        self.assertEqual(br._active_cmd_w, 0.0)


if __name__ == "__main__":
    unittest.main()
