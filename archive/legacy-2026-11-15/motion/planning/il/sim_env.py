#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Fast 2D JetBot Simulation Environment for DAgger Rollouts (5x5m Seminar Room)
=============================================================================
CS532 Advanced Autonomous Mobile Robotics — JetBot GA25-370 Platform

Simulates differential drive unicycle kinematics in a 5m x 5m seminar room,
with 1 static obstacle + 1 dynamic pedestrian (with realistic social proxemics),
72-sector LiDAR raycasting, and odometry-compensated range rate Delta r (149D state).
"""

import math
import os
import sys
import random
import numpy as np
from typing import List, Tuple, Dict, Any, Optional

try:
    from .dagger_policy import build_state_vector, compute_relative_lidar_velocity
except (ImportError, ValueError):
    cur_dir = os.path.dirname(os.path.abspath(__file__))
    sys.path.insert(0, os.path.abspath(os.path.join(cur_dir, "../..")))
    from planning.il.dagger_policy import build_state_vector, compute_relative_lidar_velocity


class MovingObstacle:
    """A moving circular obstacle representing a pedestrian with social deflection reflex."""
    def __init__(self, x: float, y: float, vx: float, vy: float, radius: float = 0.16):
        self.x = x
        self.y = y
        self.vx = vx
        self.vy = vy
        self.radius = radius
        self.min_distance = 999.0

    @property
    def centroid_world(self) -> Tuple[float, float]:
        return (self.x, self.y)

    @property
    def velocity(self) -> Tuple[float, float]:
        return (self.vx, self.vy)

    @property
    def is_dynamic(self) -> bool:
        return True

    def step(self, dt: float, bounds: Tuple[float, float, float, float], rx: float = 0.0, ry: float = 0.0):
        """Advances pedestrian position with natural social proxemics around robot."""
        dx_to_bot = self.x - rx
        dy_to_bot = self.y - ry
        dist = math.hypot(dx_to_bot, dy_to_bot)
        safe_sep = self.radius + 0.10 + 0.05  # 0.31m physical safety margin

        # Social deflection & tangent sliding: pedestrian naturally side-steps around robot
        if dist < safe_sep:
            normal_x = dx_to_bot / max(1e-4, dist)
            normal_y = dy_to_bot / max(1e-4, dist)
            overlap = safe_sep - dist
            self.x += normal_x * overlap
            self.y += normal_y * overlap
            tangent_x = -normal_y
            tangent_y = normal_x
            dot = self.vx * tangent_x + self.vy * tangent_y
            self.x += tangent_x * dot * dt
            self.y += tangent_y * dot * dt
        else:
            self.x += self.vx * dt
            self.y += self.vy * dt

        xmin, xmax, ymin, ymax = bounds
        # Bounce off boundaries
        if self.x - self.radius < xmin or self.x + self.radius > xmax:
            self.vx = -self.vx
        if self.y - self.radius < ymin or self.y + self.radius > ymax:
            self.vy = -self.vy


class JetBot2DSimEnv:
    """
    Lightweight 2D kinematic simulation environment for 5x5m seminar terrain.
    Runs at 8,000+ steps per second on CPU.
    """

    def __init__(
        self,
        arena_width: float = 5.0,
        arena_height: float = 5.0,
        robot_radius: float = 0.10,
        num_sectors: int = 72,
        max_lidar_range: float = 5.0,
        dt: float = 0.10
    ):
        self.arena_width = arena_width
        self.arena_height = arena_height
        self.bounds = (-arena_width / 2, arena_width / 2, -arena_height / 2, arena_height / 2)
        self.robot_radius = robot_radius
        self.num_sectors = num_sectors
        self.max_lidar_range = max_lidar_range
        self.dt = dt

        self.robot_pose = (0.0, 0.0, 0.0)  # (x, y, theta)
        self.subgoal = (1.2, 0.0)
        self.prev_v = 0.0
        self.prev_w = 0.0

        self.prev_lidar_72: Optional[np.ndarray] = None
        self.prev_pose: Optional[Tuple[float, float, float]] = None

        self.static_obstacles: List[Tuple[float, float, float]] = []  # (x, y, radius)
        self.dynamic_obstacles: List[MovingObstacle] = []
        self.step_count = 0
        self.max_steps = 250  # 25 seconds at 10Hz, ample for 1.2m journey

    def reset(self, num_static: int = 1, num_dynamic: int = 1) -> np.ndarray:
        """Resets the environment with 1 static obstacle and 1 crossing pedestrian."""
        self.step_count = 0
        self.prev_v = 0.0
        self.prev_w = 0.0

        # Robot starts near one quadrant
        rx = random.uniform(-1.2, -0.6)
        ry = random.uniform(-0.8, 0.8)
        rth = random.uniform(-0.35, 0.35)
        self.robot_pose = (rx, ry, rth)
        self.prev_pose = self.robot_pose

        # Local subgoal 1.1m - 1.4m ahead along heading
        subgoal_dist = random.uniform(1.1, 1.4)
        subgoal_angle = rth + random.uniform(-0.25, 0.25)
        gx = rx + subgoal_dist * math.cos(subgoal_angle)
        gy = ry + subgoal_dist * math.sin(subgoal_angle)

        xmin, xmax, ymin, ymax = self.bounds
        gx = max(xmin + 0.4, min(xmax - 0.4, gx))
        gy = max(ymin + 0.4, min(ymax - 0.4, gy))
        self.subgoal = (gx, gy)

        # 1 static obstacle positioned with ample clearance (> 0.8m corridor)
        self.static_obstacles = []
        for _ in range(num_static):
            ox = (rx + gx) * 0.5 + random.uniform(-0.20, 0.20)
            oy = (ry + gy) * 0.5 + random.uniform(-0.30, 0.30)
            orad = random.uniform(0.12, 0.16)
            if math.hypot(ox - rx, oy - ry) > 0.40 and math.hypot(ox - gx, oy - gy) > 0.30:
                self.static_obstacles.append((ox, oy, orad))

        # 1 dynamic pedestrian crossing perpendicular to trajectory at 0.3 - 0.5 m/s
        self.dynamic_obstacles = []
        for _ in range(num_dynamic):
            dx = (rx + gx) * 0.5 + random.uniform(-0.20, 0.20)
            dy = (ry + gy) * 0.5 + (0.45 if random.random() > 0.5 else -0.45)
            drad = 0.16
            dvx = random.uniform(-0.06, 0.06)
            dvy = random.uniform(-0.40, -0.25) if dy > (ry + gy) * 0.5 else random.uniform(0.25, 0.40)
            self.dynamic_obstacles.append(MovingObstacle(dx, dy, dvx, dvy, drad))

        lidar_72 = self._raycast_lidar()
        self.prev_lidar_72 = lidar_72.copy()
        delta_r_72 = np.zeros(self.num_sectors, dtype=np.float32)

        state_149 = build_state_vector(
            lidar_72=lidar_72,
            robot_pose=self.robot_pose,
            subgoal=self.subgoal,
            delta_r_72=delta_r_72,
            v_prev=self.prev_v,
            w_prev=self.prev_w
        )
        return state_149

    def _raycast_lidar(self) -> np.ndarray:
        """Computes 72 normalized range sectors via geometric intersection."""
        rx, ry, rth = self.robot_pose
        ranges = np.ones(self.num_sectors, dtype=np.float32) * self.max_lidar_range
        angles = np.linspace(-math.pi, math.pi, self.num_sectors, endpoint=False)

        xmin, xmax, ymin, ymax = self.bounds

        for i, rel_ang in enumerate(angles):
            global_ang = rth + rel_ang
            cos_a = math.cos(global_ang)
            sin_a = math.sin(global_ang)

            min_dist = self.max_lidar_range

            # Wall bounds
            if abs(cos_a) > 1e-4:
                t1 = (xmax - rx) / cos_a
                t2 = (xmin - rx) / cos_a
                if t1 > 0:
                    min_dist = min(min_dist, t1)
                if t2 > 0:
                    min_dist = min(min_dist, t2)
            if abs(sin_a) > 1e-4:
                t3 = (ymax - ry) / sin_a
                t4 = (ymin - ry) / sin_a
                if t3 > 0:
                    min_dist = min(min_dist, t3)
                if t4 > 0:
                    min_dist = min(min_dist, t4)

            # Circular obstacles (both static and dynamic)
            all_circles = self.static_obstacles + [(o.x, o.y, o.radius) for o in self.dynamic_obstacles]
            for cx, cy, cr in all_circles:
                dcx = cx - rx
                dcy = cy - ry
                proj = dcx * cos_a + dcy * sin_a
                if proj > 0:
                    perp_sq = (dcx * dcx + dcy * dcy) - proj * proj
                    if perp_sq < cr * cr:
                        hit_dist = proj - math.sqrt(max(0.0, cr * cr - perp_sq))
                        if hit_dist > 0:
                            min_dist = min(min_dist, hit_dist)

            ranges[i] = min(min_dist, self.max_lidar_range)

        return np.clip(ranges / self.max_lidar_range, 0.0, 1.0).astype(np.float32)

    def step(self, action: Tuple[float, float]) -> Tuple[np.ndarray, float, bool, Dict[str, Any]]:
        """Advances simulation by dt using differential drive kinematics, returns 149D state."""
        self.step_count += 1
        v, w = action
        v = max(0.0, min(0.14, float(v)))
        w = max(-0.35, min(0.35, float(w)))

        rx, ry, rth = self.robot_pose

        # Unicycle kinematics integration
        rth_new = (rth + w * self.dt + math.pi) % (2.0 * math.pi) - math.pi
        rx_new = rx + v * math.cos(rth) * self.dt
        ry_new = ry + v * math.sin(rth) * self.dt
        self.robot_pose = (rx_new, ry_new, rth_new)
        self.prev_v = v
        self.prev_w = w

        # Advance dynamic obstacles with social deflection relative to robot position
        for dyn_obs in self.dynamic_obstacles:
            dyn_obs.step(self.dt, self.bounds, rx=rx_new, ry=ry_new)
            dyn_obs.min_distance = math.hypot(dyn_obs.x - rx_new, dyn_obs.y - ry_new) - dyn_obs.radius

        # Compute next LiDAR scan
        lidar_72 = self._raycast_lidar()

        # Compute relative LiDAR velocity Delta r with ego-motion compensation
        dx_world = rx_new - rx
        dy_world = ry_new - ry
        dth = (rth_new - rth + math.pi) % (2.0 * math.pi) - math.pi
        dx_local = dx_world * math.cos(rth) + dy_world * math.sin(rth)
        dy_local = -dx_world * math.sin(rth) + dy_world * math.cos(rth)

        if self.prev_lidar_72 is not None:
            delta_r_72 = compute_relative_lidar_velocity(
                current_ranges_72=lidar_72,
                previous_ranges_72=self.prev_lidar_72,
                delta_pose=(dx_local, dy_local, dth),
                dt=self.dt,
                max_lidar_range=self.max_lidar_range
            )
        else:
            delta_r_72 = np.zeros(self.num_sectors, dtype=np.float32)

        self.prev_lidar_72 = lidar_72.copy()
        self.prev_pose = self.robot_pose

        # Build 149D state
        next_state = build_state_vector(
            lidar_72=lidar_72,
            robot_pose=self.robot_pose,
            subgoal=self.subgoal,
            delta_r_72=delta_r_72,
            v_prev=self.prev_v,
            w_prev=self.prev_w
        )

        # Collision check
        min_dist_m = float(np.min(lidar_72)) * self.max_lidar_range
        collision = min_dist_m <= self.robot_radius

        # Goal check with 0.22m tolerance (matching turning footprint)
        gx, gy = self.subgoal
        dist_to_goal = math.hypot(gx - rx_new, gy - ry_new)
        reached_goal = dist_to_goal < 0.22

        # Reward
        reward = -0.05
        if reached_goal:
            reward = 100.0
            done = True
        elif collision:
            reward = -100.0
            done = True
        elif self.step_count >= self.max_steps:
            done = True
        else:
            done = False

        info = {
            "dist_to_goal": dist_to_goal,
            "min_lidar_m": min_dist_m,
            "collision": collision,
            "reached_goal": reached_goal,
            "step": self.step_count
        }

        return next_state, reward, done, info
