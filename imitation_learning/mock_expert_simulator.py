"""
Mock 2D JetBot Simulator & Analytical Expert (Dynamic Obstacles & Omni Oracle v2)
================================================================================
CS532 Autonomous JetBot — Module imitation_learning
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
    ROBOT_RADIUS_M,
    ACTION_STOP,
    ACTION_FORWARD,
    ACTION_PIVOT_LEFT,
    ACTION_PIVOT_RIGHT,
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
        """Steps forward kinematics with stop-turn-go dynamics matching the field robot."""
        self.step_count += 1

        # Stop-turn-go kinematics (mirrors dashboard/jetbot_bridge.py set_motor_speeds)
        if abs(cmd_v) < 0.01 and abs(cmd_w) < 0.02:
            act_v = 0.0
            act_w = 0.0
        elif abs(cmd_v) < 0.01:
            act_v = 0.0
            act_w = math.copysign(MAX_ANGULAR_SPEED, cmd_w)
        else:
            act_v = math.copysign(MAX_LINEAR_SPEED, cmd_v)
            act_w = 0.0

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

    def step_action(self, action: int, dt: float = 1.0 / CONTROL_HZ) -> Tuple[bool, bool]:
        """Maps discrete actions onto step() kinematics:
        0: ACTION_STOP -> (0.0, 0.0)
        1: ACTION_FORWARD -> (MAX_LINEAR_SPEED, 0.0)
        2: ACTION_PIVOT_LEFT -> (0.0, 1.0)
        3: ACTION_PIVOT_RIGHT -> (0.0, -1.0)
        """
        if action == ACTION_STOP:
            return self.step(0.0, 0.0, dt)
        elif action == ACTION_FORWARD:
            return self.step(MAX_LINEAR_SPEED, 0.0, dt)
        elif action == ACTION_PIVOT_LEFT:
            return self.step(0.0, 1.0, dt)
        elif action == ACTION_PIVOT_RIGHT:
            return self.step(0.0, -1.0, dt)
        else:
            return self.step(0.0, 0.0, dt)

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
    Stop-Turn-Go Lookahead Expert Policy (teacher oracle).
    Evaluates discrete stop-turn-go motion plans (pivot n steps then forward, or
    wait k steps then forward) against predicted constant-velocity obstacles and
    LiDAR-sensed walls/static obstacles. Never emits arc commands.
    """
    def __init__(self, forward_steps: int = 18, static_steps: int = 10, clearance_margin: float = 0.10):
        self.max_v = MAX_LINEAR_SPEED
        self.max_w = MAX_ANGULAR_SPEED
        self.min_v = MIN_DRIVE_SPEED
        self.dt = 1.0 / CONTROL_HZ
        self.robot_radius = ROBOT_RADIUS_M
        self.forward_steps = forward_steps
        self.static_steps = static_steps
        self.clearance_margin = clearance_margin
        self._last_turn_dir = 0
        self._pivot_origin: Optional[Tuple[float, float]] = None

    def reset_state(self):
        self._last_turn_dir = 0
        self._pivot_origin = None

    def _extract_static_points(
        self,
        lidar_ranges: List[float],
        robot_x: float,
        robot_y: float,
        robot_theta: float,
        max_reach: float,
        privileged: Optional[List[Tuple[float, float, float, float, float]]]
    ) -> List[Tuple[float, float]]:
        """Extracts nearby static/wall surface points from 360-deg LiDAR (5-deg bins)."""
        pts: List[Tuple[float, float]] = []
        r_ceil = min(LIDAR_MAX_RANGE_M - 0.02, max_reach)
        n_rays = len(lidar_ranges)
        if n_rays == 0:
            return pts

        bin_step = 2
        for base_deg in range(0, 360, bin_step):
            best_r = r_ceil
            best_deg = -1
            for offset in range(bin_step):
                deg = (base_deg + offset) % n_rays
                r = lidar_ranges[deg]
                if LIDAR_MIN_RANGE_M <= r < best_r:
                    best_r = r
                    best_deg = deg
            if best_deg < 0:
                continue
            ang = robot_theta + math.radians(best_deg)
            wx = robot_x + best_r * math.cos(ang)
            wy = robot_y + best_r * math.sin(ang)
            if privileged is not None:
                # Skip LiDAR hits belonging to privileged obstacles (handled analytically with velocity)
                on_priv = False
                for ox, oy, orad, _, _ in privileged:
                    if math.hypot(wx - ox, wy - oy) <= orad + 0.06:
                        on_priv = True
                        break
                if on_priv:
                    continue
            pts.append((wx, wy))
        return pts

    def _evaluate_plan(
        self,
        n_pivot: int,
        k_wait: int,
        robot_x: float,
        robot_y: float,
        robot_theta: float,
        goal_x: float,
        goal_y: float,
        dyn_obs: List[Tuple[float, float, float, float, float]],
        static_obs: List[Tuple[float, float, float]],
        static_pts: List[Tuple[float, float]],
        margin: float
    ) -> Tuple[bool, int, float, float, bool]:
        """Simulates (pivot n_pivot or wait k_wait) then forward up to horizon.
        Returns (is_safe, first_fail_step, min_clearance, end_dist_to_goal, reached_goal).
        """
        dt = self.dt
        ds = self.max_v * dt
        dth = self.max_w * dt
        r_bot = self.robot_radius
        idle_steps = abs(n_pivot) + k_wait
        # Fixed forward rollout length keeps static feasibility identical across adjacent headings (prevents pivot chatter).
        # Use shorter static horizon so narrow corridors near walls stay navigable while keeping full horizon for moving obstacles.
        fwd_len = self.forward_steps if dyn_obs else self.static_steps
        total_steps = idle_steps + (0 if k_wait >= self.forward_steps else fwd_len)
        static_end_step = idle_steps + self.static_steps
        pt_lim = r_bot + margin
        pt_lim_sq = pt_lim * pt_lim
        goal_tol_sq = GOAL_TOLERANCE_M * GOAL_TOLERANCE_M

        th = robot_theta + n_pivot * dth
        cth = math.cos(th)
        sth = math.sin(th)

        min_clear = 1e9
        first_fail = total_steps + 1

        # 1. Check moving obstacles during stationary pivot/wait steps
        if dyn_obs and idle_steps > 0:
            for t in range(1, idle_steps + 1):
                tt = t * dt
                for ox, oy, orad, vx, vy in dyn_obs:
                    dx = robot_x - (ox + vx * tt)
                    dy = robot_y - (oy + vy * tt)
                    d_sq = dx * dx + dy * dy
                    lim = r_bot + orad + margin
                    if d_sq < lim * lim:
                        c = math.sqrt(d_sq) - (r_bot + orad)
                        if c < min_clear:
                            min_clear = c
                        if t < first_fail:
                            first_fail = t

        # 2. Simulate forward motion after idle_steps
        px, py = robot_x, robot_y
        reached = False
        reach_step = total_steps
        for t in range(idle_steps + 1, total_steps + 1):
            if not reached:
                px += ds * cth
                py += ds * sth
                gdx = goal_x - px
                gdy = goal_y - py
                if gdx * gdx + gdy * gdy <= goal_tol_sq:
                    reached = True
                    reach_step = t

            tt = t * dt
            # Dynamic obstacles (checked every step)
            for ox, oy, orad, vx, vy in dyn_obs:
                dx = px - (ox + vx * tt)
                dy = py - (oy + vy * tt)
                d_sq = dx * dx + dy * dy
                lim = r_bot + orad + margin
                if d_sq < lim * lim:
                    c = math.sqrt(d_sq) - (r_bot + orad)
                    if c < min_clear:
                        min_clear = c
                    if t < first_fail:
                        first_fail = t

            # Static obstacles & LiDAR wall/surface points (within static_steps while moving, or at arrival step)
            if t <= static_end_step and (not reached or t == reach_step):
                for ox, oy, orad in static_obs:
                    dx = px - ox
                    dy = py - oy
                    d_sq = dx * dx + dy * dy
                    lim = r_bot + orad + margin
                    if d_sq < lim * lim:
                        c = math.sqrt(d_sq) - (r_bot + orad)
                        if c < min_clear:
                            min_clear = c
                        if t < first_fail:
                            first_fail = t
                for wx, wy in static_pts:
                    dx = px - wx
                    dy = py - wy
                    d_sq = dx * dx + dy * dy
                    if d_sq < pt_lim_sq:
                        c = math.sqrt(d_sq) - r_bot
                        if c < min_clear:
                            min_clear = c
                        if t < first_fail:
                            first_fail = t

            if first_fail <= total_steps and reached:
                break

        end_dist = math.hypot(goal_x - px, goal_y - py)
        if reached:
            end_dist = -1.0 + 0.01 * reach_step
        return (first_fail > total_steps), first_fail, min_clear, end_dist, reached

    def get_action(
        self,
        lidar_ranges: List[float],
        goal_x: float,
        goal_y: float,
        robot_x: float,
        robot_y: float,
        robot_theta: float,
        apply_dart_noise: bool = False,
        privileged: Optional[List[Tuple[float, float, float, float, float]]] = None
    ) -> Tuple[float, float]:
        dx = goal_x - robot_x
        dy = goal_y - robot_y
        dist_goal = math.hypot(dx, dy)

        if dist_goal <= GOAL_TOLERANCE_M:
            self.reset_state()
            return 0.0, 0.0

        # Separate privileged obstacles into dynamic vs static
        dyn_obs: List[Tuple[float, float, float, float, float]] = []
        static_obs: List[Tuple[float, float, float]] = []
        if privileged is not None:
            for ox, oy, orad, vx, vy in privileged:
                if abs(vx) > 1e-4 or abs(vy) > 1e-4:
                    dyn_obs.append((ox, oy, orad, vx, vy))
                else:
                    static_obs.append((ox, oy, orad))

        max_reach = self.static_steps * self.max_v * self.dt + self.robot_radius + self.clearance_margin + 0.08
        static_pts = self._extract_static_points(
            lidar_ranges, robot_x, robot_y, robot_theta, max_reach, privileged
        )

        # If still pivoting in place at the same position, block reversing turn direction to prevent 2-cycle chatter
        in_place_pivot = (
            self._pivot_origin is not None
            and self._last_turn_dir != 0
            and math.hypot(robot_x - self._pivot_origin[0], robot_y - self._pivot_origin[1]) < 1e-4
        )
        if not in_place_pivot:
            self._pivot_origin = None

        # Candidate active stop-turn-go plans: (n_pivot, k_wait)
        # Pure STOP is a safety fallback only, never competing against active navigation.
        stop_plan = (0, self.forward_steps)
        candidates: List[Tuple[int, int]] = [(0, 0)]
        for n in range(1, 13):
            if not (in_place_pivot and self._last_turn_dir < 0):
                candidates.append((n, 0))
            if not (in_place_pivot and self._last_turn_dir > 0):
                candidates.append((-n, 0))
        if dyn_obs:
            for k in (2, 4, 8):
                candidates.append((0, k))

        def _pick_best(margin: float):
            best_plan = None
            best_score = 1e18
            fallback_plan = stop_plan
            fallback_key = (-1, -1e9, -1e9)
            for n_piv, k_wait in candidates:
                safe, fail_step, min_c, end_d, _ = self._evaluate_plan(
                    n_piv, k_wait, robot_x, robot_y, robot_theta,
                    goal_x, goal_y, dyn_obs, static_obs, static_pts, margin
                )
                if safe:
                    # Slight tie-break for smaller turn, consistency with last turn dir, and not waiting needlessly
                    turn_dir = 1 if n_piv > 0 else (-1 if n_piv < 0 else 0)
                    dir_pen = 0.0 if (turn_dir == 0 or turn_dir == self._last_turn_dir) else 0.005
                    score = end_d + 0.002 * abs(n_piv) + 0.003 * k_wait + dir_pen
                    if score < best_score:
                        best_score = score
                        best_plan = (n_piv, k_wait)
                else:
                    # Prefer plans that survive longer and maintain larger clearance
                    key = (fail_step, round(min_c, 3), -end_d)
                    if key > fallback_key:
                        fallback_key = key
                        fallback_plan = (n_piv, k_wait)
            return best_plan, fallback_plan

        chosen, _ = _pick_best(self.clearance_margin)
        if chosen is None:
            # Relaxed margin pass before giving up
            chosen, fallback = _pick_best(0.02)
            if chosen is None:
                # Check if pure STOP is safe; otherwise pick best survival escape against moving obstacle
                if dyn_obs:
                    stop_safe, _, _, _, _ = self._evaluate_plan(
                        0, self.forward_steps, robot_x, robot_y, robot_theta,
                        goal_x, goal_y, dyn_obs, static_obs, static_pts, 0.02
                    )
                    chosen = stop_plan if stop_safe else fallback
                else:
                    chosen = stop_plan

        n_piv, k_wait = chosen
        if k_wait > 0:
            self._pivot_origin = None
            return 0.0, 0.0
        if n_piv > 0:
            self._last_turn_dir = 1
            self._pivot_origin = (robot_x, robot_y)
            w_cmd = self.max_w
            if apply_dart_noise and DART_ENABLED and random.random() < DART_PROBABILITY:
                w_cmd = max(0.05, min(self.max_w, w_cmd + random.gauss(0.0, DART_NOISE_STD_W)))
            return 0.0, float(round(w_cmd, 3))
        if n_piv < 0:
            self._last_turn_dir = -1
            self._pivot_origin = (robot_x, robot_y)
            w_cmd = -self.max_w
            if apply_dart_noise and DART_ENABLED and random.random() < DART_PROBABILITY:
                w_cmd = min(-0.05, max(-self.max_w, w_cmd + random.gauss(0.0, DART_NOISE_STD_W)))
            return 0.0, float(round(w_cmd, 3))

        # Forward
        self._pivot_origin = None
        v_cmd = self.max_v
        if apply_dart_noise and DART_ENABLED and random.random() < DART_PROBABILITY:
            v_cmd = max(self.min_v + 0.01, min(self.max_v, v_cmd + random.gauss(0.0, DART_NOISE_STD_V)))
        return float(round(v_cmd, 3)), 0.0
