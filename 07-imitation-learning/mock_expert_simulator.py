"""
Mock 2D JetBot Simulator & Analytical Expert (Dynamic Obstacles & Omni Oracle v2)
================================================================================
CS532 Autonomous JetBot — Module 07-imitation-learning
Based on UIT ITCourses RL-101 (Bài 10: Học bắt chước - Imitation Learning)

Major Enhancements (v2):
  1. Dynamic Obstacle Simulation:
     - Moving obstacles with velocity (vx, vy) simulating crossing pedestrians/objects.
     - Sudden pop-up obstacles appearing unexpectedly directly in front of the robot.
  2. Terminal Approach Zone Protection:
     - Clamps perception horizon when approaching goal (dist_goal <= 0.40m) so walls/objects
       behind the goal do not trigger false bypass swerves.
  3. Decoupled Hardware Invariance:
     - Zero reliance on physical IMU or wheel encoders (pure LiDAR + Relative Goal vector).
  4. Dynamic Yield & Emergency Escape Oracle:
     - Oracle yields briefly for dynamic cross-cutting objects, or bypasses with safe margins.
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


class SimulationObstacle:
    """Represents a static, dynamic, or sudden pop-up circular obstacle."""
    def __init__(
        self,
        x: float,
        y: float,
        radius: float,
        vx: float = 0.0,
        vy: float = 0.0,
        popup_step: int = 0
    ):
        self.x = x
        self.y = y
        self.radius = radius
        self.vx = vx
        self.vy = vy
        self.popup_step = popup_step
        self.active = (popup_step == 0)

    def update(self, current_step: int, dt: float, arena_w: float, arena_h: float):
        if current_step >= self.popup_step:
            self.active = True
        if not self.active:
            return

        self.x += self.vx * dt
        self.y += self.vy * dt

        # Bounce off arena boundaries with margin
        half_h = arena_h / 2.0
        if self.x - self.radius < 0.20:
            self.x = 0.20 + self.radius
            self.vx = abs(self.vx)
        elif self.x + self.radius > arena_w - 0.20:
            self.x = arena_w - 0.20 - self.radius
            self.vx = -abs(self.vx)

        if self.y - self.radius < -half_h + 0.20:
            self.y = -half_h + 0.20 + self.radius
            self.vy = abs(self.vy)
        elif self.y + self.radius > half_h - 0.20:
            self.y = half_h - 0.20 - self.radius
            self.vy = -abs(self.vy)


class SimulationEnvironment:
    """Simulates JetBot kinematics, LiDAR raycasting, and dynamic obstacles."""
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
        self.step_count = 0

        # Goal
        self.goal_x = 2.40
        self.goal_y = 0.00

        # Obstacles
        self.obstacles: List[SimulationObstacle] = []
        self.reset()

    def reset(
        self,
        custom_goal: Optional[Tuple[float, float]] = None,
        custom_start: Optional[Tuple[float, float, float]] = None,
        num_obstacles: int = 2
    ):
        """Resets robot, goal, and generates realistic static & dynamic obstacles."""
        half_h = self.arena_h / 2.0
        self.step_count = 0

        if custom_start:
            self.x, self.y, self.theta = custom_start
        else:
            self.x = random.uniform(0.50, self.arena_w - 0.50)
            self.y = random.uniform(-half_h + 0.40, half_h - 0.40)
            self.theta = random.uniform(-math.pi, math.pi)

        if custom_goal:
            self.goal_x, self.goal_y = custom_goal
        else:
            attempts = 0
            while attempts < 25:
                gx = random.uniform(0.35, self.arena_w - 0.35)
                gy = random.uniform(-half_h + 0.35, half_h - 0.35)
                if math.hypot(gx - self.x, gy - self.y) >= 0.75:
                    self.goal_x, self.goal_y = gx, gy
                    break
                attempts += 1
            if attempts >= 25:
                self.goal_x = self.arena_w - 0.50
                self.goal_y = 0.0

        self.v = 0.0
        self.w = 0.0

        # Spawn obstacles across 6 distinct realistic operational scenarios
        self.obstacles.clear()
        scenario_type = random.random()

        dx_g = self.goal_x - self.x
        dy_g = self.goal_y - self.y
        dist_g = max(0.1, math.hypot(dx_g, dy_g))
        ux, uy = dx_g / dist_g, dy_g / dist_g
        nx, ny = -uy, ux

        if scenario_type < 0.25:
            # Scenario A: Static Ca Nuoc / Cup directly on path (r ~ 0.04m)
            if dist_g > 0.55:
                fwd_d = random.uniform(0.30, min(0.65, dist_g - 0.20))
                ox = self.x + fwd_d * ux + random.uniform(-0.03, 0.03)
                oy = self.y + fwd_d * uy + random.uniform(-0.03, 0.03)
                rad = random.uniform(0.035, 0.045)
                if 0.20 <= ox <= self.arena_w - 0.20 and (-half_h + 0.20) <= oy <= (half_h - 0.20):
                    self.obstacles.append(SimulationObstacle(ox, oy, rad))

        elif scenario_type < 0.45:
            # Scenario B: Dynamic Moving Obstacle (cross-cutting obstacle moving across path)
            if dist_g > 0.80:
                cross_d = random.uniform(0.40, min(0.85, dist_g - 0.25))
                side = 1.0 if random.random() > 0.5 else -1.0
                ox = self.x + cross_d * ux + side * 0.40 * nx
                oy = self.y + cross_d * uy + side * 0.40 * ny
                # Velocity towards opposite side
                cross_speed = random.uniform(0.08, 0.15)
                vx = -side * cross_speed * nx
                vy = -side * cross_speed * ny
                rad = random.uniform(0.05, 0.07)
                self.obstacles.append(SimulationObstacle(ox, oy, rad, vx=vx, vy=vy))

        elif scenario_type < 0.60:
            # Scenario C: Sudden Pop-Up Obstacle (object placed directly in front at step 10..25)
            if dist_g > 0.60:
                fwd_d = random.uniform(0.35, min(0.55, dist_g - 0.15))
                ox = self.x + fwd_d * ux + random.uniform(-0.02, 0.02)
                oy = self.y + fwd_d * uy + random.uniform(-0.02, 0.02)
                rad = random.uniform(0.045, 0.065)
                popup_step = random.randint(10, 25)
                self.obstacles.append(SimulationObstacle(ox, oy, rad, popup_step=popup_step))

        elif scenario_type < 0.80:
            # Scenario D: Multi-obstacle slalom corridor
            if dist_g > 1.0:
                num_obs = min(3, num_obstacles + 1)
                for i in range(num_obs):
                    d_step = (dist_g * 0.28) * (i + 1)
                    s_mult = 1.0 if (i % 2 == 0) else -1.0
                    lat = s_mult * random.uniform(0.16, 0.24)
                    ox = self.x + d_step * ux + lat * nx
                    oy = self.y + d_step * uy + lat * ny
                    rad = random.uniform(0.05, 0.08)
                    if 0.20 <= ox <= self.arena_w - 0.20 and (-half_h + 0.20) <= oy <= (half_h - 0.20):
                        self.obstacles.append(SimulationObstacle(ox, oy, rad))

        elif scenario_type < 0.90:
            # Scenario E: Goal near wall / corner (Terminal Approach Zone)
            # Robot must approach smoothly and not swerve away from goal due to wall behind
            wall_dist = random.uniform(0.25, 0.38)
            side = random.choice(["left", "right", "top", "bottom"])
            if side == "left":
                self.goal_x = wall_dist
            elif side == "right":
                self.goal_x = self.arena_w - wall_dist
            elif side == "top":
                self.goal_y = half_h - wall_dist
            else:
                self.goal_y = -half_h + wall_dist

        else:
            # Scenario F: Clean open space (omni target reaching & in-place pivot)
            pass

    def step(self, cmd_v: float, cmd_w: float, dt: float = 1.0 / CONTROL_HZ) -> Tuple[bool, bool]:
        """Steps forward kinematics with simulated slip and updates dynamic obstacles."""
        self.step_count += 1

        # Deadband handling
        if 0.0 < cmd_v < MIN_DRIVE_SPEED:
            act_v = 0.0
        else:
            act_v = cmd_v * (1.0 - self.slip_ratio)
        act_w = cmd_w

        self.theta = (self.theta + act_w * dt + math.pi) % (2.0 * math.pi) - math.pi
        self.x += act_v * math.cos(self.theta) * dt
        self.y += act_v * math.sin(self.theta) * dt
        self.v = act_v
        self.w = act_w

        # Update dynamic obstacles
        for obs in self.obstacles:
            if hasattr(obs, "update"):
                obs.update(self.step_count, dt, self.arena_w, self.arena_h)

        # Check collision with walls or active obstacles
        r_bot = 0.08
        half_h = self.arena_h / 2.0
        collided = (
            self.x - r_bot <= 0.0
            or self.x + r_bot >= self.arena_w
            or self.y - r_bot <= -half_h
            or self.y + r_bot >= half_h
        )

        for obs in self.obstacles:
            if isinstance(obs, tuple):
                ox, oy, orad = obs[0], obs[1], obs[2]
                active = True
            else:
                ox, oy, orad = obs.x, obs.y, obs.radius
                active = obs.active

            if active and math.hypot(self.x - ox, self.y - oy) < (r_bot + orad):
                collided = True
                break

        dist_goal = math.hypot(self.goal_x - self.x, self.goal_y - self.y)
        reached = dist_goal <= GOAL_TOLERANCE_M

        return reached, collided

    def get_lidar_scan(self) -> List[float]:
        """Raycasts 360-degree LiDAR against arena walls and active obstacles."""
        ranges = [float(self.lidar_range_max)] * 360
        r_max = self.lidar_range_max
        half_h = self.arena_h / 2.0

        for deg in range(360):
            th_ray = (self.theta + math.radians(deg) + math.pi) % (2.0 * math.pi) - math.pi
            cos_th = math.cos(th_ray)
            sin_th = math.sin(th_ray)

            d_min = r_max

            # Wall boundaries
            if cos_th < -1e-4:
                d = (0.0 - self.x) / cos_th
                if d > 0:
                    d_min = min(d_min, d)
            elif cos_th > 1e-4:
                d = (self.arena_w - self.x) / cos_th
                if d > 0:
                    d_min = min(d_min, d)

            if sin_th < -1e-4:
                d = (-half_h - self.y) / sin_th
                if d > 0:
                    d_min = min(d_min, d)
            elif sin_th > 1e-4:
                d = (half_h - self.y) / sin_th
                if d > 0:
                    d_min = min(d_min, d)

            # Circle obstacles intersection (active only)
            for obs in self.obstacles:
                if isinstance(obs, tuple):
                    ox, oy, orad = obs[0], obs[1], obs[2]
                    active = True
                else:
                    ox, oy, orad = obs.x, obs.y, obs.radius
                    active = obs.active

                if not active:
                    continue

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

            # Add sensor noise and clamp
            d_meas = min(r_max, max(LIDAR_MIN_RANGE_M, d_min))
            ranges[deg] = round(d_meas, 3)

        return ranges


class GeometricExpertPolicy:
    """
    Enhanced Oracle Policy (pi* v2):
      - Handles Static & Dynamic Cross-Cutting Obstacles
      - Dedicated Terminal Approach Mode (prevents false evasions near destination)
      - Dynamic Yielding for sudden moving objects
      - In-place pivot for wide orientation errors
    """
    def __init__(self):
        self.max_v = MAX_LINEAR_SPEED
        self.max_w = MAX_ANGULAR_SPEED
        self.min_v = MIN_DRIVE_SPEED
        self.latched_evade = 0
        self.yield_counter = 0

    def reset_state(self):
        self.latched_evade = 0
        self.yield_counter = 0

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
        dx = goal_x - robot_x
        dy = goal_y - robot_y
        dist_goal = math.hypot(dx, dy)

        # 1. Goal Reached
        if dist_goal <= GOAL_TOLERANCE_M:
            self.reset_state()
            return 0.0, 0.0

        angle_world = math.atan2(dy, dx)
        alpha = (angle_world - robot_theta + math.pi) % (2.0 * math.pi) - math.pi

        # 2. Terminal Approach Zone (dist_goal <= 0.40m):
        # Smoothly slow down and align straight to goal without triggering false obstacle bypass
        if dist_goal <= 0.40:
            self.latched_evade = 0
            # Check immediate collision bubble (< 0.14m directly ahead)
            front_crit = [lidar_ranges[d % 360] for d in range(-25, 26) if 0.05 <= lidar_ranges[d % 360] < 0.14]
            if front_crit:
                # Critical halt
                return 0.0, 0.0
            
            # Smooth terminal deceleration
            v_term = max(self.min_v, min(self.max_v, 0.13 * (dist_goal / 0.40)))
            w_term = max(-self.max_w, min(self.max_w, 1.8 * alpha))
            return float(round(v_term, 3)), float(round(w_term, 3))

        # 3. PIVOT TURN IN PLACE: For large heading deviations (> 50 deg)
        if abs(alpha) > math.radians(50.0):
            self.reset_state()
            w_pivot = math.copysign(self.max_w, alpha)
            return 0.0, float(round(w_pivot, 3))

        # 4. Forward Corridor Perception
        # Only consider obstacles BETWEEN robot and goal (xr < dist_goal - 0.06)
        closest_xr = 999.0
        closest_yr = 0.0
        has_front_obs = False
        max_fwd_check = min(0.68, dist_goal - 0.06)

        for d in range(-80, 81):
            idx = (d + 360) % 360
            r = lidar_ranges[idx]
            if 0.05 <= r < max_fwd_check:
                ang_r = math.radians(d)
                xr = r * math.cos(ang_r)
                yr = r * math.sin(ang_r)
                # Frontal corridor: xr in [0.0, max_fwd_check], |yr| < 0.22m
                if -0.04 <= xr <= max_fwd_check and abs(yr) < 0.22:
                    has_front_obs = True
                    if xr < closest_xr:
                        closest_xr = xr
                        closest_yr = yr

        left_clear = sum(lidar_ranges[d] for d in range(20, 80)) / 60.0
        right_clear = sum(lidar_ranges[(360 - d) % 360] for d in range(20, 80)) / 60.0

        if has_front_obs:
            # A. Sudden Impending Collision (< 0.20m): Dynamic Yield
            if closest_xr < 0.20:
                if self.yield_counter < 6:  # Yield for up to ~0.6s
                    self.yield_counter += 1
                    return 0.0, 0.0
                # If still blocked after yield, proceed to active evasive steering

            self.yield_counter = 0

            # B. Safe Contour Bypass
            if self.latched_evade == 0:
                if alpha > 0.08:
                    self.latched_evade = +1
                elif alpha < -0.08:
                    self.latched_evade = -1
                else:
                    self.latched_evade = +1 if left_clear >= right_clear else -1

            # Desired lateral clearance: 0.24m
            target_lat = -0.24 if self.latched_evade > 0 else +0.24
            lat_err = target_lat - closest_yr

            w_cmd = -2.8 * lat_err
            v_cmd = max(self.min_v + 0.01, self.max_v * min(0.75, max(0.40, closest_xr / 0.65)))
        else:
            self.latched_evade = 0
            self.yield_counter = 0
            w_cmd = 1.8 * alpha
            v_cmd = self.max_v * max(0.65, math.cos(alpha / 2.0) ** 2)

        # Enforce bounds
        v_cmd = max(self.min_v + 0.01, min(self.max_v, v_cmd))
        w_cmd = max(-self.max_w, min(self.max_w, w_cmd))

        # DART Perturbation
        if apply_dart_noise and DART_ENABLED and not has_front_obs and random.random() < DART_PROBABILITY:
            noise_v = random.gauss(0.0, DART_NOISE_STD_V)
            noise_w = random.gauss(0.0, DART_NOISE_STD_W)
            v_cmd = max(self.min_v + 0.01, min(self.max_v, v_cmd + noise_v))
            w_cmd = max(-self.max_w, min(self.max_w, w_cmd + noise_w))

        return float(round(v_cmd, 3)), float(round(w_cmd, 3))
