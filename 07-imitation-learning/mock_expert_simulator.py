"""
Mock 2D JetBot Simulator & Analytical Expert (Generalized Dynamic Obstacle Geometry)
===================================================================================
CS532 Autonomous JetBot — Module 07-imitation-learning
Based on UIT ITCourses RL-101 (Bài 10: Học bắt chước - Imitation Learning)

Features:
  1. Generalized Multi-Geometry Simulation Environment:
     - Arbitrary 2D Obstacles: Small Cylinders (ca nuoc), Wide Boxes (thung carton),
       Large Partitions (vach ngan), and Slalom Corridors.
     - Vectorized 2D Raycaster supporting Circles & Oriented Bounding Boxes (Slab Method).
     - Wheel slip & motor deadband dynamics on glossy tile.
  2. Dynamic Geometry-Aware Expert Policy (pi*):
     - Dynamically extracts obstacle length, width, distance, and lateral bounds from LiDAR.
     - Computes dynamic clearance y_target and curvature kappa without hardcoded offsets.
     - Longitudinal completion tracking: maintains bypass until obstacle is cleared (x <= 0),
       completely eliminating back-and-forth limit-cycle oscillations ("xoay vong qua lai").
     - Omnidirectional pivot alignment for large heading deviations (|alpha| > 50 deg).
  3. DART Perturbation Injection:
     - Injects Gaussian kicks to record robust expert recovery demonstrations.
"""

import math
import random
import numpy as np
from typing import List, Tuple, Dict, Any, Optional, Union

from config import (
    MAX_LINEAR_SPEED,
    MAX_ANGULAR_SPEED,
    MIN_DRIVE_SPEED,
    LIDAR_MAX_RANGE_M,
    LIDAR_MIN_RANGE_M,
    GOAL_TOLERANCE_M,
    ROBOT_RADIUS_M,
    CONTROL_HZ,
    DART_ENABLED,
    DART_PROBABILITY,
    DART_NOISE_STD_V,
    DART_NOISE_STD_W
)
from dataset import extract_observation_vector


class SimulationEnvironment:
    """Simulates JetBot kinematics and LiDAR raycasting with generalized obstacles."""
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

        # Generalized Obstacles:
        # Tuples can be:
        #   ('circle', ox, oy, radius) OR (ox, oy, radius) [legacy]
        #   ('box', cx, cy, half_l, half_w, yaw_rad)
        self.obstacles: List[Any] = []
        self.reset()

    def reset(
        self,
        custom_goal: Optional[Tuple[float, float]] = None,
        custom_start: Optional[Tuple[float, float, float]] = None,
        num_obstacles: int = 2
    ):
        """Resets robot, goal, and generates diverse generalized obstacles."""
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
            attempts = 0
            while attempts < 25:
                gx = random.uniform(0.40, self.arena_w - 0.40)
                gy = random.uniform(-half_h + 0.40, half_h - 0.40)
                if math.hypot(gx - self.x, gy - self.y) >= 0.85:
                    self.goal_x, self.goal_y = gx, gy
                    break
                attempts += 1
            if attempts >= 25:
                self.goal_x = self.arena_w - 0.50
                self.goal_y = 0.0

        self.v = 0.0
        self.w = 0.0
        self.obstacles.clear()

        # Vector pointing towards goal
        dx_g = self.goal_x - self.x
        dy_g = self.goal_y - self.y
        dist_g = max(0.1, math.hypot(dx_g, dy_g))
        ux, uy = dx_g / dist_g, dy_g / dist_g
        nx, ny = -uy, ux

        scenario_type = random.random()

        if scenario_type < 0.35 and dist_g > 0.70:
            # Scenario A: Wide Rectangular Box (Thung carton, width 0.25m - 0.50m) directly on path
            fwd_d = random.uniform(0.35, min(0.75, dist_g - 0.25))
            cx = self.x + fwd_d * ux + random.uniform(-0.03, 0.03)
            cy = self.y + fwd_d * uy + random.uniform(-0.03, 0.03)
            box_w = random.uniform(0.24, 0.48)
            box_l = random.uniform(0.16, 0.30)
            box_yaw = math.atan2(dy_g, dx_g) + random.uniform(-0.3, 0.3)
            if 0.30 <= cx <= self.arena_w - 0.30 and (-half_h + 0.30) <= cy <= (half_h - 0.30):
                self.obstacles.append(("box", cx, cy, box_l / 2.0, box_w / 2.0, box_yaw))

        elif scenario_type < 0.60 and dist_g > 0.60:
            # Scenario B: Small Cylinder (Ca nuoc / chai nuoc r ~ 0.035m - 0.05m)
            fwd_d = random.uniform(0.35, min(0.70, dist_g - 0.20))
            ox = self.x + fwd_d * ux + random.uniform(-0.02, 0.02)
            oy = self.y + fwd_d * uy + random.uniform(-0.02, 0.02)
            rad = random.uniform(0.035, 0.05)
            if 0.25 <= ox <= self.arena_w - 0.25 and (-half_h + 0.25) <= oy <= (half_h - 0.25):
                self.obstacles.append(("circle", ox, oy, rad))

        elif scenario_type < 0.85 and dist_g > 1.0:
            # Scenario C: Multi-obstacle Slalom (Combination of Box & Cylinder)
            num_obs = min(2, num_obstacles)
            for i in range(num_obs):
                d_step = (dist_g * 0.35) * (i + 1)
                side = 1.0 if (i % 2 == 0) else -1.0
                lat = side * random.uniform(0.16, 0.24)
                ox = self.x + d_step * ux + lat * nx
                oy = self.y + d_step * uy + lat * ny
                if 0.25 <= ox <= self.arena_w - 0.25 and (-half_h + 0.25) <= oy <= (half_h - 0.25):
                    if i % 2 == 0:
                        self.obstacles.append(("box", ox, oy, 0.10, 0.15, 0.0))
                    else:
                        self.obstacles.append(("circle", ox, oy, random.uniform(0.06, 0.09)))

        else:
            # Scenario D: Open field navigation
            pass

    def step(self, cmd_v: float, cmd_w: float, dt: float = 1.0 / CONTROL_HZ) -> Tuple[bool, bool]:
        """Steps forward differential drive kinematics with simulated tile slip."""
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

        # Collision check
        r_bot = ROBOT_RADIUS_M
        half_h = self.arena_h / 2.0
        collided = (
            self.x - r_bot <= 0.0
            or self.x + r_bot >= self.arena_w
            or self.y - r_bot <= -half_h
            or self.y + r_bot >= half_h
        )

        for obs in self.obstacles:
            if isinstance(obs, tuple) and len(obs) == 3 and not isinstance(obs[0], str):
                ox, oy, orad = obs
                if math.hypot(self.x - ox, self.y - oy) < (r_bot + orad):
                    collided = True
                    break
            elif obs[0] == "circle":
                _, ox, oy, orad = obs
                if math.hypot(self.x - ox, self.y - oy) < (r_bot + orad):
                    collided = True
                    break
            elif obs[0] == "box":
                _, cx, cy, hl, hw, psi = obs
                c_psi, s_psi = math.cos(psi), math.sin(psi)
                dx, dy = self.x - cx, self.y - cy
                x_rel = dx * c_psi + dy * s_psi
                y_rel = -dx * s_psi + dy * c_psi
                closest_x = max(-hl, min(hl, x_rel))
                closest_y = max(-hw, min(hw, y_rel))
                if math.hypot(x_rel - closest_x, y_rel - closest_y) < r_bot:
                    collided = True
                    break

        dist_goal = math.hypot(self.goal_x - self.x, self.goal_y - self.y)
        reached = dist_goal <= GOAL_TOLERANCE_M

        return reached, collided

    def get_lidar_scan(self) -> List[float]:
        """Simulates 360-degree LiDAR raycasting supporting circles and boxes."""
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

            # Obstacles
            for obs in self.obstacles:
                if isinstance(obs, tuple) and len(obs) == 3 and not isinstance(obs[0], str):
                    ox, oy, orad = obs
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

                elif obs[0] == "circle":
                    _, ox, oy, orad = obs
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

                elif obs[0] == "box":
                    _, cx, cy, hl, hw, psi = obs
                    c_psi, s_psi = math.cos(psi), math.sin(psi)
                    dx = self.x - cx
                    dy = self.y - cy
                    x_rel = dx * c_psi + dy * s_psi
                    y_rel = -dx * s_psi + dy * c_psi
                    th_rel = th_ray - psi
                    dx_ray = math.cos(th_rel)
                    dy_ray = math.sin(th_rel)

                    if abs(dx_ray) > 1e-5:
                        tx1 = (-hl - x_rel) / dx_ray
                        tx2 = (hl - x_rel) / dx_ray
                        tmin_x = min(tx1, tx2)
                        tmax_x = max(tx1, tx2)
                    else:
                        if abs(x_rel) > hl:
                            continue
                        tmin_x, tmax_x = -float("inf"), float("inf")

                    if abs(dy_ray) > 1e-5:
                        ty1 = (-hw - y_rel) / dy_ray
                        ty2 = (hw - y_rel) / dy_ray
                        tmin_y = min(ty1, ty2)
                        tmax_y = max(ty1, ty2)
                    else:
                        if abs(y_rel) > hw:
                            continue
                        tmin_y, tmax_y = -float("inf"), float("inf")

                    t_enter = max(tmin_x, tmin_y)
                    t_exit = min(tmax_x, tmax_y)
                    if t_enter <= t_exit and t_exit > 0:
                        t_hit = t_enter if t_enter > 0 else t_exit
                        if t_hit > 0:
                            d_min = min(d_min, t_hit)

            d_meas = min(r_max, max(LIDAR_MIN_RANGE_M, d_min))
            ranges[deg] = round(d_meas, 3)

        return ranges


class GeometricExpertPolicy:
    """
    Generalized Dynamic Geometry-Aware Expert Policy (pi*).
    Calculates dynamic obstacle clearance and bypass curvature from real LiDAR clusters.
    Maintains bypass until obstacle is cleared longitudinally, eliminating limit-cycle oscillations.
    """
    def __init__(self):
        self.max_v = MAX_LINEAR_SPEED
        self.max_w = MAX_ANGULAR_SPEED
        self.min_v = MIN_DRIVE_SPEED

        # Dynamic bypass tracking state
        self.bypass_side = 0       # +1: Left, -1: Right, 0: None
        self.bypass_ticks = 0

    def reset_state(self):
        """Resets bypass hysteresis between episodes."""
        self.bypass_side = 0
        self.bypass_ticks = 0

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
        """Computes optimal (v*, w*) action across arbitrary obstacle dimensions."""
        dx = goal_x - robot_x
        dy = goal_y - robot_y
        dist_goal = math.hypot(dx, dy)

        if dist_goal <= GOAL_TOLERANCE_M:
            self.reset_state()
            return 0.0, 0.0

        angle_world = math.atan2(dy, dx)
        alpha = (angle_world - robot_theta + math.pi) % (2.0 * math.pi) - math.pi

        # 1. Frontal threat detection (-60 to +60 deg)
        front_pts = []
        for d in range(-60, 61):
            idx = (d + 360) % 360
            r = lidar_ranges[idx]
            if 0.08 <= r <= min(0.70, dist_goal - 0.05):
                ang = math.radians(d)
                x = r * math.cos(ang)
                y = r * math.sin(ang)
                if -0.05 <= x <= 0.70 and abs(y) <= 0.28:
                    front_pts.append((x, y, r))

        # 2. Flank points check (is obstacle still alongside robot?)
        flank_active = False
        if self.bypass_side != 0:
            flank_deg = range(35, 95) if self.bypass_side == -1 else range(265, 325)
            for d in flank_deg:
                r = lidar_ranges[d % 360]
                if 0.08 <= r <= 0.45:
                    ang = math.radians(d)
                    x = r * math.cos(ang)
                    if x > -0.05:
                        flank_active = True
                        break

        left_c = sum(lidar_ranges[20:70]) / 50.0
        right_c = sum(lidar_ranges[290:340]) / 50.0

        if front_pts:
            xs = [p[0] for p in front_pts]
            ys = [p[1] for p in front_pts]
            min_x = min(xs)
            min_y = min(ys)
            max_y = max(ys)

            if self.bypass_side == 0:
                if alpha > 0.05 and left_c >= right_c:
                    self.bypass_side = +1
                elif alpha < -0.05 and right_c >= left_c:
                    self.bypass_side = -1
                else:
                    self.bypass_side = +1 if left_c >= right_c else -1
                self.bypass_ticks = 0

            self.bypass_ticks += 1
            w_safe = ROBOT_RADIUS_M + 0.13  # ~0.22m safe buffer
            y_tgt = (max_y + w_safe) if self.bypass_side == +1 else (min_y - w_safe)

            if min_x < 0.20:
                # Danger close: halt linear velocity and pivot away
                v_cmd = 0.0
                w_cmd = float(self.bypass_side) * self.max_w
            else:
                kappa = (2.0 * y_tgt) / (min_x * min_x + y_tgt * y_tgt)
                v_cmd = max(self.min_v, 0.15 / (1.0 + 1.2 * abs(kappa)))
                w_cmd = max(-self.max_w, min(self.max_w, v_cmd * kappa))

        elif flank_active and self.bypass_ticks < 35:
            # Maintain longitudinal corridor past the obstacle
            self.bypass_ticks += 1
            v_cmd = 0.13
            w_cmd = 0.0

        else:
            # Clear corridor: goal orientation & proportional navigation
            self.reset_state()
            if abs(alpha) > math.radians(50.0):
                v_cmd = 0.0
                w_cmd = math.copysign(self.max_w, alpha)
            else:
                w_cmd = max(-self.max_w, min(self.max_w, 1.8 * alpha))
                v_cmd = max(self.min_v + 0.01, self.max_v * max(0.65, math.cos(alpha / 2.0) ** 2))

        # DART Perturbation Injection
        if apply_dart_noise and DART_ENABLED and not front_pts and random.random() < DART_PROBABILITY:
            noise_v = random.gauss(0.0, DART_NOISE_STD_V)
            noise_w = random.gauss(0.0, DART_NOISE_STD_W)
            v_cmd = max(self.min_v, min(self.max_v, v_cmd + noise_v))
            w_cmd = max(-self.max_w, min(self.max_w, w_cmd + noise_w))

        v_cmd = max(self.min_v, min(self.max_v, v_cmd)) if v_cmd > 0.0 else 0.0
        w_cmd = max(-self.max_w, min(self.max_w, w_cmd))

        return float(round(v_cmd, 3)), float(round(w_cmd, 3))
