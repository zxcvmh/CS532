"""
Mock 2D JetBot Simulator & Analytical Expert (Omnidirectional S-Curve & Pivot Oracle)
====================================================================================
CS532 Autonomous JetBot — Module 07-imitation-learning
Based on UIT ITCourses RL-101 (Bài 10: Học bắt chước - Imitation Learning)

Purpose:
  1. High-fidelity 2D Kinematic Simulation:
     - Arena walls (3m x 2m / 3m x 3m testbed) + diverse obstacles (ca nuoc to slalom boxes)
     - Fast 2D Vectorized Raycaster generating 360-degree LiDAR scans
     - Wheel slip & motor deadband dynamics on ceramic tile
  2. Optimal Omnidirectional Oracle Expert (pi*):
     - In-place pivot alignment for wide goal angles (|alpha| > 50 deg)
     - Smooth corridor bypass with Safe Contour margin (0.16m) for small obstacles (ca nuoc)
     - Counter-steering S-curve recovery back to goal
     - Hysteresis latching to eliminate steering chattering
     - Provides ground-truth optimal action a* = pi*(s) for any visited state s
  3. DART Perturbation Mode:
     - Injects Gaussian kicks to record expert recovery trajectories
"""

import math
import random
import numpy as np
from typing import List, Tuple, Dict, Any, Optional

from config import (
    MAX_LINEAR_SPEED,
    MAX_ANGULAR_SPEED,
    MIN_DRIVE_SPEED,
    LIDAR_MAX_RANGE_M,
    LIDAR_MIN_RANGE_M,
    GOAL_TOLERANCE_M,
    CONTROL_HZ,
    DART_ENABLED,
    DART_PROBABILITY,
    DART_NOISE_STD_V,
    DART_NOISE_STD_W
)
from dataset import extract_observation_vector


class SimulationEnvironment:
    """Simulates JetBot kinematics and LiDAR raycasting in an arena."""
    def __init__(
        self,
        arena_width: float = 3.0,
        arena_height: float = 2.4,
        slip_ratio: float = 0.05
    ):
        self.arena_w = arena_width
        self.arena_h = arena_height
        self.slip_ratio = slip_ratio  # 5% wheel slip on glossy tile
        self.lidar_range_max = LIDAR_MAX_RANGE_M

        # Robot state
        self.x = 0.50
        self.y = 0.00
        self.theta = 0.00
        self.v = 0.00
        self.w = 0.00

        # Goal
        self.goal_x = 2.40
        self.goal_y = 0.00

        # Obstacles: list of (x, y, radius)
        self.obstacles: List[Tuple[float, float, float]] = []
        self.reset()

    def reset(
        self,
        custom_goal: Optional[Tuple[float, float]] = None,
        custom_start: Optional[Tuple[float, float, float]] = None,
        num_obstacles: int = 2
    ):
        """Resets robot, goal, and generates realistic obstacles across full 360-degree orientations."""
        half_h = self.arena_h / 2.0

        if custom_start:
            self.x, self.y, self.theta = custom_start
        else:
            self.x = random.uniform(0.50, self.arena_w - 0.50)
            self.y = random.uniform(-half_h + 0.40, half_h - 0.40)
            self.theta = random.uniform(-math.pi, math.pi)

        if custom_goal:
            self.goal_x, self.goal_y = custom_goal
        else:
            # Generate goal at least 0.8m away from robot
            attempts = 0
            while attempts < 20:
                gx = random.uniform(0.40, self.arena_w - 0.40)
                gy = random.uniform(-half_h + 0.40, half_h - 0.40)
                if math.hypot(gx - self.x, gy - self.y) >= 0.80:
                    self.goal_x, self.goal_y = gx, gy
                    break
                attempts += 1
            if attempts >= 20:
                self.goal_x = self.arena_w - 0.50
                self.goal_y = 0.0

        self.v = 0.0
        self.w = 0.0

        # Spawn obstacles
        self.obstacles.clear()
        scenario_type = random.random()

        if scenario_type < 0.35:
            # Scenario A: Small frontal obstacle (Ca nuoc / chai nuoc r ~ 0.04m) directly on path
            dx_g = self.goal_x - self.x
            dy_g = self.goal_y - self.y
            dist_g = max(0.1, math.hypot(dx_g, dy_g))
            if dist_g > 0.60:
                fwd_d = random.uniform(0.35, min(0.70, dist_g - 0.20))
                ux, uy = dx_g / dist_g, dy_g / dist_g
                ox = self.x + fwd_d * ux + random.uniform(-0.02, 0.02)
                oy = self.y + fwd_d * uy + random.uniform(-0.03, 0.03)
                rad = random.uniform(0.035, 0.045)
                # Keep obstacle within arena bounds
                if 0.25 <= ox <= self.arena_w - 0.25 and (-half_h + 0.25) <= oy <= (half_h - 0.25):
                    self.obstacles.append((ox, oy, rad))

        elif scenario_type < 0.65:
            # Scenario B: Navigable slalom corridors (obstacles radius ~ 0.07m - 0.09m)
            dx_g = self.goal_x - self.x
            dy_g = self.goal_y - self.y
            dist_g = max(0.1, math.hypot(dx_g, dy_g))
            if dist_g > 1.0:
                ux, uy = dx_g / dist_g, dy_g / dist_g
                nx, ny = -uy, ux  # perpendicular normal
                num_obs = min(2, num_obstacles)
                for i in range(num_obs):
                    d_step = (dist_g * 0.35) * (i + 1)
                    side = 1.0 if (i % 2 == 0) else -1.0
                    lat = side * random.uniform(0.15, 0.22)
                    ox = self.x + d_step * ux + lat * nx
                    oy = self.y + d_step * uy + lat * ny
                    rad = random.uniform(0.06, 0.08)
                    if 0.25 <= ox <= self.arena_w - 0.25 and (-half_h + 0.25) <= oy <= (half_h - 0.25):
                        self.obstacles.append((ox, oy, rad))

        else:
            # Scenario C: Clean open field (practice pure heading alignment and pivot turning)
            pass

    def step(self, cmd_v: float, cmd_w: float, dt: float = 1.0 / CONTROL_HZ) -> Tuple[bool, bool]:
        """
        Steps forward using differential drive kinematics with simulated wheel slip.
        Returns: (reached_goal, collision)
        """
        # Deadband stiction: if commanded PWM/velocity is below deadband, motor does not overcome friction
        if 0.0 < cmd_v < MIN_DRIVE_SPEED:
            act_v = 0.0
        else:
            # Apply simulated wheel slip on glossy ceramic tile
            act_v = cmd_v * (1.0 - self.slip_ratio)
        act_w = cmd_w

        self.theta = (self.theta + act_w * dt + math.pi) % (2.0 * math.pi) - math.pi
        self.x += act_v * math.cos(self.theta) * dt
        self.y += act_v * math.sin(self.theta) * dt
        self.v = act_v
        self.w = act_w

        # Check collision with walls or obstacles (robot radius ~ 0.08m)
        r_bot = 0.08
        half_h = self.arena_h / 2.0
        collided = (
            self.x - r_bot <= 0.0
            or self.x + r_bot >= self.arena_w
            or self.y - r_bot <= -half_h
            or self.y + r_bot >= half_h
        )

        for ox, oy, orad in self.obstacles:
            if math.hypot(self.x - ox, self.y - oy) < (r_bot + orad):
                collided = True
                break

        dist_goal = math.hypot(self.goal_x - self.x, self.goal_y - self.y)
        reached = dist_goal <= GOAL_TOLERANCE_M

        return reached, collided

    def get_lidar_scan(self) -> List[float]:
        """
        Simulates 360-degree 2D LiDAR raycaster.
        Returns ranges in meters [0..359] CCW.
        """
        ranges = [float(self.lidar_range_max)] * 360
        r_max = self.lidar_range_max

        # Raycast for each integer degree
        for deg in range(360):
            th_ray = (self.theta + math.radians(deg) + math.pi) % (2.0 * math.pi) - math.pi
            cos_th = math.cos(th_ray)
            sin_th = math.sin(th_ray)

            d_min = r_max

            # Wall 1: x = 0 (Left boundary)
            if cos_th < -1e-4:
                d = (0.0 - self.x) / cos_th
                if d > 0:
                    d_min = min(d_min, d)
            # Wall 2: x = arena_w (Right boundary)
            elif cos_th > 1e-4:
                d = (self.arena_w - self.x) / cos_th
                if d > 0:
                    d_min = min(d_min, d)

            # Wall 3: y = -arena_h/2 (Bottom boundary)
            half_h = self.arena_h / 2.0
            if sin_th < -1e-4:
                d = (-half_h - self.y) / sin_th
                if d > 0:
                    d_min = min(d_min, d)
            # Wall 4: y = +arena_h/2 (Top boundary)
            elif sin_th > 1e-4:
                d = (half_h - self.y) / sin_th
                if d > 0:
                    d_min = min(d_min, d)

            # Circle obstacles intersection
            for ox, oy, orad in self.obstacles:
                dx = self.x - ox
                dy = self.y - oy
                b = 2.0 * (dx * cos_th + dy * sin_th)
                c = dx * dx + dy * dy - orad * orad
                disc = b * b - 4.0 * c
                if disc >= 0:
                    sq = math.sqrt(disc)
                    t1 = (-b - sq) / 2.0
                    if t1 > 0:
                        d_min = min(d_min, t1)

            # Clamp & add small sensor noise
            d_meas = min(r_max, max(LIDAR_MIN_RANGE_M, d_min))
            ranges[deg] = round(d_meas, 3)

        return ranges


class GeometricExpertPolicy:
    """
    Analytical Oracle Policy (pi*) combining:
      - Omnidirectional in-place pivot alignment for large heading deviations (|alpha| > 50 deg)
      - Tangent Bug Safe Contour Bypass for small obstacles (ca nuoc) and large obstacles
      - Continuous proportional goal tracking in open space (w = 1.8 * alpha, v = 0.16)
      - Anti-chattering hysteresis latching
    """
    def __init__(self):
        self.max_v = MAX_LINEAR_SPEED
        self.max_w = MAX_ANGULAR_SPEED
        self.min_v = MIN_DRIVE_SPEED
        self.latched_evade = 0
        self.latch_timer = 0

    def reset_state(self):
        """Reset hysteresis state between episodes."""
        self.latched_evade = 0
        self.latch_timer = 0

    def get_action(
        self,
        lidar_ranges: List[float],
        goal_x: float,
        goal_y: float,
        robot_x: float,
        robot_y: float,
        robot_theta: float,
        apply_dart_noise: bool = False
    ) -> Tuple[float, float]:
        """Computes optimal (v*, w*) action across 360-degree orientations."""
        dx = goal_x - robot_x
        dy = goal_y - robot_y
        dist_goal = math.hypot(dx, dy)

        if dist_goal <= GOAL_TOLERANCE_M:
            self.reset_state()
            return 0.0, 0.0

        angle_world = math.atan2(dy, dx)
        alpha = (angle_world - robot_theta + math.pi) % (2.0 * math.pi) - math.pi

        # 1. PIVOT TURN IN PLACE: For large heading deviations (> 50 deg)
        # Guarantees the robot halts linear motion and rotates in place until facing the goal!
        if abs(alpha) > math.radians(50.0):
            self.reset_state()
            w_pivot = math.copysign(self.max_w, alpha)
            return 0.0, float(round(w_pivot, 3))

        # 2. Obstacle detection in forward semicircle (-80 to +80 deg, up to 0.70m ahead)
        closest_xr = 999.0
        closest_yr = 0.0
        has_front_obs = False

        for d in range(-80, 81):
            idx = (d + 360) % 360
            r = lidar_ranges[idx]
            if 0.05 <= r < 0.72:
                ang_r = math.radians(d)
                xr = r * math.cos(ang_r)
                yr = r * math.sin(ang_r)
                # Frontal corridor zone: 0.65m ahead, |yr| < 0.22m
                if -0.05 <= xr <= 0.65 and abs(yr) < 0.22:
                    has_front_obs = True
                    if xr < closest_xr:
                        closest_xr = xr
                        closest_yr = yr

        left_clear = sum(lidar_ranges[d] for d in range(20, 80)) / 60.0
        right_clear = sum(lidar_ranges[(360 - d) % 360] for d in range(20, 80)) / 60.0

        if has_front_obs:
            if self.latched_evade == 0:
                if alpha > 0.08:
                    self.latched_evade = +1
                elif alpha < -0.08:
                    self.latched_evade = -1
                else:
                    self.latched_evade = +1 if left_clear >= right_clear else -1

            # Desired lateral clearance: 0.24m (leaves safe 15cm margin from ca nuoc)
            if self.latched_evade > 0:
                lat_err = (-0.24) - closest_yr
            else:
                lat_err = (+0.24) - closest_yr

            w_cmd = -2.8 * lat_err
            # Gentle cruise deceleration during bypass to tighten turn radius (R = v / w)
            v_cmd = max(self.min_v + 0.01, self.max_v * min(0.75, max(0.40, closest_xr / 0.65)))
        else:
            self.latched_evade = 0
            w_cmd = 1.8 * alpha
            v_cmd = self.max_v * max(0.65, math.cos(alpha / 2.0) ** 2)


        # Clamp actions safely above deadband
        v_cmd = max(self.min_v + 0.01, min(self.max_v, v_cmd))
        w_cmd = max(-self.max_w, min(self.max_w, w_cmd))

        # DART Perturbation (Data Gathering with Asserted Random Trajectories)
        if apply_dart_noise and DART_ENABLED and not has_front_obs and random.random() < DART_PROBABILITY:
            noise_v = random.gauss(0.0, DART_NOISE_STD_V)
            noise_w = random.gauss(0.0, DART_NOISE_STD_W)
            v_cmd = max(self.min_v + 0.01, min(self.max_v, v_cmd + noise_v))
            w_cmd = max(-self.max_w, min(self.max_w, w_cmd + noise_w))

        return float(round(v_cmd, 3)), float(round(w_cmd, 3))
