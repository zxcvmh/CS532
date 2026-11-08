"""
VelocityRegulator: Curvature-aware speed scaling, S-curve deceleration ramp,
and multi-tier obstacle braking for JetBot unicycle control.
"""
import math
from typing import List, Optional, Any


class VelocityRegulator:
    """Regulates forward linear velocity based on path curvature, goal distance, and obstacle clearance."""

    def __init__(
        self,
        min_drive_speed: float = 0.12,
        max_linear_speed: float = 0.14,
        goal_tolerance: float = 0.18,
        curvature_scale_k: float = 1.2,
        soft_start_ramp_sec: float = 0.4,
    ):
        self.min_drive_speed = float(min_drive_speed)
        self.max_linear_speed = float(max_linear_speed)
        self.goal_tolerance = float(goal_tolerance)
        self.curvature_scale_k = float(curvature_scale_k)
        self.soft_start_ramp_sec = float(soft_start_ramp_sec)

    def compute_goal_ramp_velocity(self, dist_to_goal: float) -> float:
        """Computes deceleration ramp as the robot approaches the terminal goal."""
        ramp_dist = max(0.35, self.goal_tolerance + 0.15)
        if dist_to_goal <= ramp_dist:
            ramp = max(0.0, min(1.0, (dist_to_goal - self.goal_tolerance) / max(0.01, ramp_dist - self.goal_tolerance)))
            return self.min_drive_speed + (self.max_linear_speed - self.min_drive_speed) * ramp
        return self.max_linear_speed

    def scale_curvature_velocity(self, base_v: float, alpha: float) -> float:
        """
        Regulated Pure Pursuit (Steve Macenski 2023):
        v = base_v * cos^2(alpha / 2).
        Allows continuous smooth turning for diagonal headings without halting forward progress.
        """
        cos_half = math.cos(alpha / 2.0)
        return max(0.04, base_v * (cos_half * cos_half))

    def modulate_velocity(
        self,
        v_target: float,
        last_v: float,
        dt: float = 0.1,
        lidar_ranges: Optional[List[float]] = None,
        clusters: Optional[List[Any]] = None,
        kappa: float = 0.0,
        front_min: Optional[float] = None,
    ) -> float:
        """
        Applies obstacle proximity braking and S-curve acceleration limiting.
        """
        v = v_target

        # 1. Frontal LiDAR obstacle slowdown
        if front_min is not None and front_min < 0.60:
            slowdown_ratio = max(0.35, min(1.0, (front_min - 0.20) / 0.40))
            v = v * slowdown_ratio

        # 2. Dynamic cluster TTC slowdown
        if clusters:
            for cl in clusters:
                ttc = getattr(cl, "ttc", 99.0)
                if 0.0 < ttc < 1.5:
                    v = min(v, self.min_drive_speed)
                    break

        # 3. Curvature limit: v <= v_max / (1 + k * |kappa|)
        if abs(kappa) > 1e-3:
            v_curv_cap = self.max_linear_speed / (1.0 + self.curvature_scale_k * abs(kappa))
            v = min(v, v_curv_cap)

        # 4. Acceleration slew-rate ramp
        ramp_time = max(0.1, self.soft_start_ramp_sec)
        max_accel = max(0.2, self.max_linear_speed / ramp_time)
        max_delta = max_accel * max(0.01, min(0.2, dt))

        delta_v = v - last_v
        if abs(delta_v) > max_delta:
            v = last_v + math.copysign(max_delta, delta_v)

        return max(0.0, min(self.max_linear_speed, v))
