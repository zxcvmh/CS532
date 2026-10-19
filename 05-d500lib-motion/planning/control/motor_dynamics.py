"""
MotorDynamics: Differential drive inverse kinematics, continuous deadband lift,
wheel trim balancing, and acceleration slew-rate limiting for JetBot.
"""
import math
import os
import json
from typing import Tuple, Optional


class MotorDynamics:
    """Manages conversion between unicycle velocities (v, w) and physical motor PWM outputs."""

    def __init__(
        self,
        track_width: float = 0.115,
        min_drive_speed: float = 0.12,
        max_linear_speed: float = 0.14,
        max_angular_speed: float = 0.35,
        turn_speed: float = 0.14,
        cruise_speed: float = 0.17,
        trim_factor: float = 1.024,
        soft_start_ramp_sec: float = 0.4,
    ):
        self.track_width = float(track_width)
        self.min_drive_speed = float(min_drive_speed)
        self.max_linear_speed = float(max_linear_speed)
        self.max_angular_speed = float(max_angular_speed)
        self.turn_speed = float(turn_speed)
        self.cruise_speed = float(cruise_speed)
        self.trim_factor = float(trim_factor)
        self.soft_start_ramp_sec = float(soft_start_ramp_sec)

        self.last_pwm_left: float = 0.0
        self.last_pwm_right: float = 0.0
        self.load_dynamics_config()

    def differential_drive_ik(self, v: float, w: float) -> Tuple[float, float]:
        """Unicycle inverse kinematics for two-wheel differential drive."""
        half_l = self.track_width / 2.0
        v_left = v - (w * half_l)
        v_right = v + (w * half_l)
        return round(v_left, 4), round(v_right, 4)

    def compute_motor_pwm(
        self,
        linear_v: float,
        angular_w: float,
        dt: float = 0.1,
        emergency_stop: bool = False,
        max_pwm: Optional[float] = None,
    ) -> Tuple[Tuple[float, float], Tuple[float, float]]:
        """
        Computes calibrated wheel PWM values with deadband lift and differential steering gain,
        strictly synchronized with robot_dynamics.json.
        Returns ((pwm_left, pwm_right), (v_left, v_right)).
        """
        if emergency_stop or (abs(linear_v) < 1e-4 and abs(angular_w) < 1e-4):
            self.last_pwm_left = 0.0
            self.last_pwm_right = 0.0
            return (0.0, 0.0), (0.0, 0.0)

        # 1. Differential Inverse Kinematics
        v_left, v_right = self.differential_drive_ik(linear_v, angular_w)

        # 2. In-place spin decoupling
        if abs(linear_v) < 0.01 and abs(angular_w) >= 0.02:
            w_ratio = min(1.0, abs(angular_w) / max(0.05, self.max_angular_speed))
            spin_mag = self.min_drive_speed + (self.turn_speed - self.min_drive_speed) * w_ratio
            raw_pwm_l = -math.copysign(spin_mag, angular_w)
            raw_pwm_r = math.copysign(spin_mag, angular_w)
        else:
            # Linear drive with differential steering
            half_l = self.track_width / 2.0
            v_base = max(0.05, self.max_linear_speed)
            delta_v_max = self.max_angular_speed * half_l
            span_v = 2.0 * delta_v_max

            def to_pwm(vw: float) -> float:
                if abs(vw) < 0.005:
                    return 0.0
                sign = 1.0 if vw >= 0 else -1.0
                ratio = (abs(vw) - (v_base - delta_v_max)) / span_v
                pwm = self.min_drive_speed + (self.cruise_speed - self.min_drive_speed) * ratio
                return sign * max(self.min_drive_speed, min(self.cruise_speed, pwm))

            raw_pwm_l = to_pwm(v_left)
            raw_pwm_r = to_pwm(v_right)

        # 3. Mechanical Trim Balancing (compensate gear motor variance)
        raw_pwm_l = max(-1.0, min(1.0, raw_pwm_l * self.trim_factor))
        raw_pwm_r = max(-1.0, min(1.0, raw_pwm_r))

        # 4. Slew-Rate Acceleration Limiter
        ramp_time = max(0.1, self.soft_start_ramp_sec)
        max_rate = 1.0 / ramp_time
        max_delta = max_rate * max(0.01, min(0.2, dt))

        def clamp_rate(target: float, previous: float) -> float:
            delta = target - previous
            if abs(delta) > max_delta:
                return previous + math.copysign(max_delta, delta)
            return target

        final_pwm_l = clamp_rate(raw_pwm_l, self.last_pwm_left)
        final_pwm_r = clamp_rate(raw_pwm_r, self.last_pwm_right)

        self.last_pwm_left = final_pwm_l
        self.last_pwm_right = final_pwm_r

        return (round(final_pwm_l, 4), round(final_pwm_r, 4)), (v_left, v_right)

    def load_dynamics_config(self, config_path: Optional[str] = None):
        """Loads physical motor dynamics parameters from robot_dynamics.json."""
        if config_path is None:
            candidates = [
                os.path.join(os.path.dirname(__file__), "..", "..", "..", "03-MinhHieu", "test-hardware", "robot_dynamics.json"),
                os.path.join(os.path.dirname(__file__), "..", "..", "..", "setup", "robot_dynamics.json"),
                os.path.join(os.path.dirname(__file__), "..", "..", "setup", "robot_dynamics.json"),
                "/home/zxcvmh/Projects/CS532/03-MinhHieu/test-hardware/robot_dynamics.json",
                "/home/zxcvmh/Projects/CS532/setup/robot_dynamics.json",
            ]
            for c in candidates:
                if os.path.isfile(c):
                    config_path = c
                    break

        if config_path and os.path.isfile(config_path):
            try:
                with open(config_path, "r", encoding="utf-8") as f:
                    cfg = json.load(f)
                self.min_drive_speed = float(cfg.get("min_drive_speed", self.min_drive_speed))
                self.cruise_speed = float(cfg.get("cruise_speed", self.cruise_speed))
                self.turn_speed = float(cfg.get("turn_speed", self.turn_speed))
                self.trim_factor = float(cfg.get("trim_factor", self.trim_factor))
                self.soft_start_ramp_sec = float(cfg.get("soft_start_ramp_sec", self.soft_start_ramp_sec))
                if "max_safe_speed_mps" in cfg:
                    self.max_linear_speed = float(cfg["max_safe_speed_mps"])
                if "max_angular_speed_rads" in cfg:
                    self.max_angular_speed = float(cfg["max_angular_speed_rads"])
            except Exception:
                pass
