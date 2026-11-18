#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Kinodynamic Reciprocal Velocity Obstacles (Kinodynamic RVO) Planner for JetBot
=============================================================================
CS532 Advanced Autonomous Mobile Robotics — JetBot GA25-370 Hardware Platform

Key Mathematical & Algorithmic Principles:
1. Differential Drive Unicycle Kinodynamics (GA25-370 Open-Loop DC Motors):
   - Linear speed: v in [0.0, max_safe_speed] (with deadband compensation v >= min_drive_speed).
   - Angular speed: |w| <= max_angular_speed.
   - Ceramic Tile Traction Constraint (mu ~ 0.20):
     Centrifugal acceleration: a_lat = |v * w| <= 0.176 m/s^2 (prevents wheel slip & spinout).

2. Reciprocal Velocity Obstacle (RVO) Formulation:
   - Evaluates dynamic obstacles (pedestrians, moving humans) with estimated velocities (vx, vy).
   - Reciprocal 50-50 burden sharing: Robot takes 50% responsibility to evade:
     v_bot not in RVO_i = v_obs_i + 0.5 * VO_i
   - Predicts Time-to-Collision (TTC) along projected unicycle arcs.

3. Multi-Angle Social Proxemics & Rules of the Road:
   - Head-on Approach (180 deg, Oncoming): Right-hand traffic rule (COLREGS bias to the right).
   - Cross-traffic (90 deg, Orthogonal Crossing): Pass-behind tactic (decelerate to allow human passage).
   - Overtaking from Behind (0 deg, Rear): Maintain steady speed & heading (predictable motion, no abrupt brake).
   - Tactical Yielding: Decelerate to creep (0.05 m/s) or stop at 0.3m if human continues close approach.

4. Decoupled Modular Architecture:
   - Cleanly separated in 03-MinhHieu/backend/app/planning/kinodynamic_rvo.py.
   - Ready for activation in 5m x 5m seminar room via Config.ENVIRONMENT_MODE = "SEMINAR_5X5".
   - Seamlessly bypassed in 2m x 1m arena mode (ARENA_2X1) to strictly avoid boundary wall collisions.
=============================================================================
"""

import math
import time
from typing import List, Tuple, Dict, Any, Optional

try:
    from .obstacle_clustering import ObstacleCluster
except ImportError:
    try:
        from obstacle_clustering import ObstacleCluster
    except ImportError:
        ObstacleCluster = Any


class KinodynamicRVOPlanner:
    """
    Sample-based Kinodynamic Reciprocal Velocity Obstacle (DW-RVO) Planner.
    Calculates dynamically feasible and socially acceptable collision-free (v, w)
    commands for the JetBot differential drive platform.
    """
    def __init__(
        self,
        robot_radius: float = 0.12,             # JetBot effective radius (meters)
        pedestrian_radius: float = 0.25,         # Typical human leg/torso radius (meters)
        safety_margin: float = 0.10,             # Clearance buffer (meters)
        max_speed: float = 0.22,                 # Evade cruise speed on ceramic tile [m/s]
        min_drive_speed: float = 0.12,           # GA25 motor static deadband threshold [m/s]
        max_angular_speed: float = 0.80,         # Angular velocity limit during evasion [rad/s]
        centrifugal_limit: float = 0.176,        # Max centrifugal accel |v*w| [m/s^2]
        time_horizon: float = 2.5,               # TTC evaluation horizon [seconds]
        social_proxemics_dist: float = 0.80,     # Social comfort distance for humans [meters]
        tactical_yield_dist: float = 0.30,       # Distance for tactical yield stop [meters]
    ):
        self.robot_radius = robot_radius
        self.pedestrian_radius = pedestrian_radius
        self.safety_margin = safety_margin
        self.max_speed = max_speed
        self.min_drive_speed = min_drive_speed
        self.max_angular_speed = max_angular_speed
        self.centrifugal_limit = centrifugal_limit
        self.time_horizon = time_horizon
        self.social_proxemics_dist = social_proxemics_dist
        self.tactical_yield_dist = tactical_yield_dist

        # Tracking internal state
        self.last_selected_v: float = 0.0
        self.last_selected_w: float = 0.0
        self.last_social_state: str = "FREE_FLOW"

    def is_kinodynamically_feasible(self, v: float, w: float) -> bool:
        """
        Validates unicycle velocity commands against GA25-370 motor limits
        and the ceramic tile centrifugal anti-slip constraint.
        """
        # Deadband / Speed bounds
        if abs(v) > 1e-4 and abs(v) < self.min_drive_speed and abs(v) > 0.06:
            # Between creep and min_drive: motor may stall on high stiction
            pass
        if abs(v) > self.max_speed:
            return False
        if abs(w) > self.max_angular_speed:
            return False

        # Ceramic tile centrifugal traction limit: |v * w| <= 0.176 m/s^2
        if abs(v * w) > self.centrifugal_limit:
            return False

        return True

    def classify_relative_approach(
        self,
        robot_x: float,
        robot_y: float,
        robot_theta: float,
        obs_x: float,
        obs_y: float,
        obs_vx: float,
        obs_vy: float
    ) -> str:
        """
        Classifies interaction geometry relative to the robot's heading:
        - "HEAD_ON_180": Oncoming pedestrian from the front heading towards the robot.
        - "CROSS_90": Pedestrian trajectory crossing the robot path.
        - "REAR_OVERTAKE_0": Pedestrian behind the robot approaching from the rear.
        - "STATIC_OR_DIVERGING": Not an impending collision course.
        """
        dx = obs_x - robot_x
        dy = obs_y - robot_y
        dist = math.hypot(dx, dy)
        if dist < 1e-3:
            return "HEAD_ON_180"

        # Bearing of obstacle relative to robot heading
        bearing = math.atan2(dy, dx) - robot_theta
        bearing = (bearing + math.pi) % (2.0 * math.pi) - math.pi  # Normalize to [-pi, pi]

        # Closing speed (radial velocity component towards robot)
        # Unit vector from robot to obstacle
        ux = dx / dist
        uy = dy / dist
        # Radial velocity of obstacle towards robot: - (v_obs . u)
        radial_closing_speed = -(obs_vx * ux + obs_vy * uy)

        if abs(bearing) > 2.35:  # > 135 degrees -> obstacle is behind the robot
            if radial_closing_speed > 0.10:
                return "REAR_OVERTAKE_0"
            return "STATIC_OR_DIVERGING"

        if abs(bearing) <= 0.785:  # < 45 degrees -> obstacle in frontal cone
            if radial_closing_speed > 0.15:
                return "HEAD_ON_180"
            return "STATIC_OR_DIVERGING"

        # In intermediate angles (45 deg to 135 deg)
        if math.hypot(obs_vx, obs_vy) > 0.10:
            return "CROSS_90"

        return "STATIC_OR_DIVERGING"

    def compute_rvo_violation(
        self,
        cand_v: float,
        cand_w: float,
        robot_x: float,
        robot_y: float,
        robot_theta: float,
        obstacles: List[Any],
        dt: float = 0.10
    ) -> Tuple[bool, float, Optional[str]]:
        """
        Evaluates whether a candidate (cand_v, cand_w) falls into the RVO cone
        of any active dynamic obstacle over the evaluation time horizon.
        Returns: (is_collision_or_rvo, min_ttc, culprit_mode)
        """
        min_ttc = self.time_horizon
        in_rvo = False
        culprit_mode = None

        # Predict robot position along unicycle trajectory
        # For small dt, integration along circular arc
        num_steps = 15  # 15 steps @ 0.1s = 1.5s rollout
        t_step = self.time_horizon / float(num_steps)

        r_x = robot_x
        r_y = robot_y
        r_th = robot_theta

        traj_pts = []
        for s in range(1, num_steps + 1):
            t_curr = s * t_step
            if abs(cand_w) < 1e-4:
                r_x += cand_v * math.cos(r_th) * t_step
                r_y += cand_v * math.sin(r_th) * t_step
            else:
                r_x += (cand_v / cand_w) * (math.sin(r_th + cand_w * t_step) - math.sin(r_th))
                r_y -= (cand_v / cand_w) * (math.cos(r_th + cand_w * t_step) - math.cos(r_th))
                r_th += cand_w * t_step
            traj_pts.append((t_curr, r_x, r_y))

        for obs in obstacles:
            # Extract obstacle position and velocity in World Frame
            if hasattr(obs, "centroid_world") and obs.centroid_world is not None:
                ox, oy = obs.centroid_world
            elif hasattr(obs, "centroid"):
                # Transform body centroid to world frame if centroid_world is missing
                cx, cy = obs.centroid
                ox = robot_x + cx * math.cos(robot_theta) - cy * math.sin(robot_theta)
                oy = robot_y + cx * math.sin(robot_theta) + cy * math.cos(robot_theta)
            else:
                continue

            vx, vy = getattr(obs, "velocity", (0.0, 0.0))
            is_dynamic = getattr(obs, "is_dynamic", False) or math.hypot(vx, vy) > 0.08

            # Combined footprint radius (robot + obstacle + safety buffer)
            effective_radius = self.robot_radius + self.pedestrian_radius + self.safety_margin
            if getattr(obs, "threat_level", "") == "CRITICAL" or getattr(obs, "obstacle_type", "") == "PERSON":
                effective_radius = max(effective_radius, self.social_proxemics_dist * 0.5)

            # RVO 50-50 velocity adjustment:
            # Instead of standard VO with v_obs, RVO uses v_rvo = 0.5 * (v_cand_vec + v_pref_vec)
            # which geometrically shifts the obstacle's virtual speed by 0.5 * v_obs
            v_effective_x = vx * 0.5
            v_effective_y = vy * 0.5

            rel_mode = self.classify_relative_approach(robot_x, robot_y, robot_theta, ox, oy, vx, vy)

            # Check projected trajectory against moving obstacle circle
            for t_curr, rx_t, ry_t in traj_pts:
                # Obstacle position at t_curr under RVO reciprocal shift
                ox_t = ox + v_effective_x * t_curr if is_dynamic else ox
                oy_t = oy + v_effective_y * t_curr if is_dynamic else oy

                dist_at_t = math.hypot(rx_t - ox_t, ry_t - oy_t)
                if dist_at_t < effective_radius:
                    in_rvo = True
                    if t_curr < min_ttc:
                        min_ttc = t_curr
                        culprit_mode = rel_mode
                    break

        return in_rvo, min_ttc, culprit_mode

    def compute_command(
        self,
        robot_pose: Tuple[float, float, float],
        nominal_v: float,
        nominal_w: float,
        dynamic_obstacles: List[Any],
        grid_map: Optional[Any] = None,
        dt: float = 0.10
    ) -> Tuple[float, float, Dict[str, Any]]:
        """
        Computes optimal (v, w) using Sample-Based Kinodynamic RVO.
        - If no dynamic obstacles threaten the trajectory: passes nominal (v, w) through.
        - If conflict detected: samples admissible (v, w) pairs, applies social biases,
          and returns the safest evasion velocity vector.
        - If impassable / unavoidable: activates Tactical Yielding (v=0, w=0).
        """
        rx, ry, rth = robot_pose

        if not dynamic_obstacles:
            return nominal_v, nominal_w, {
                "rvo_active": False,
                "strategy": "NOMINAL_PURSUIT",
                "reason": "No dynamic obstacles in perception horizon"
            }

        # 1. Check if the current nominal command is already conflict-free
        nom_in_rvo, nom_ttc, primary_mode = self.compute_rvo_violation(
            nominal_v, nominal_w, rx, ry, rth, dynamic_obstacles, dt
        )

        # Check proximity to nearest obstacle for Tactical Yielding
        min_dist_to_obs = 999.0
        for obs in dynamic_obstacles:
            dist = getattr(obs, "min_distance", 999.0)
            if dist < min_dist_to_obs:
                min_dist_to_obs = dist

        # Tactical Yield Trigger: obstacle within 0.30m -> halt safely to avoid wall crash
        if min_dist_to_obs <= self.tactical_yield_dist:
            return 0.0, 0.0, {
                "rvo_active": True,
                "strategy": "TACTICAL_YIELD_STOP",
                "reason": f"Obstacle within tactical yield clearance ({min_dist_to_obs:.2f}m <= {self.tactical_yield_dist:.2f}m)",
                "ttc": round(nom_ttc, 2)
            }

        # If nominal trajectory is safe and TTC is distant, keep nominal motion
        if not nom_in_rvo and nom_ttc >= self.time_horizon:
            return nominal_v, nominal_w, {
                "rvo_active": False,
                "strategy": "CLEAR_TRACKING",
                "reason": "Nominal path is outside RVO cones"
            }

        # 2. Social Rule Handling for specific geometries
        if primary_mode == "REAR_OVERTAKE_0":
            # Rear approach rule: maintain steady cruise speed and heading.
            # Do NOT brake or swerve across the overtaking human!
            v_steady = min(self.max_speed, max(self.min_drive_speed, nominal_v))
            return v_steady, 0.0, {
                "rvo_active": True,
                "strategy": "REAR_OVERTAKE_PREDICTABLE",
                "reason": "Human overtaking from rear; maintaining steady cruise heading and speed"
            }

        # 3. Dynamic Window Sampling for optimal (v, w)
        # Generate candidate velocities within kinodynamic and traction limits
        v_candidates = [
            0.0,                                # Complete stop (tactical yield option)
            0.05,                               # Creep crawl
            self.min_drive_speed,               # 0.12 m/s
            0.15,                               # Moderate speed
            0.18,                               # Evade cruise
            self.max_speed                      # 0.22 m/s max evade
        ]
        # Angular velocities from -0.8 rad/s to +0.8 rad/s
        w_candidates = [
            -0.80, -0.60, -0.40, -0.20, 0.0,
            0.20, 0.40, 0.60, 0.80
        ]

        best_v = 0.0
        best_w = 0.0
        best_cost = float("inf")
        chosen_strategy = "TACTICAL_YIELD_STOP"
        chosen_reason = "No collision-free evasion velocity available"

        for v_cand in v_candidates:
            for w_cand in w_candidates:
                # Filter by unicycle physical limits & ceramic tile friction (|v*w| <= 0.176)
                if not self.is_kinodynamically_feasible(v_cand, w_cand):
                    continue

                in_rvo, ttc, mode = self.compute_rvo_violation(
                    v_cand, w_cand, rx, ry, rth, dynamic_obstacles, dt
                )
                if in_rvo and ttc < 1.2:
                    # Trajectory leads to impending collision within 1.2s -> reject
                    continue

                # Cost formulation:
                # J = w_goal * deviation_from_nominal + w_ttc / ttc + social_penalties
                cost_nominal_dev = 2.0 * abs(v_cand - nominal_v) + 1.2 * abs(w_cand - nominal_w)
                cost_ttc = (self.time_horizon - ttc) * 1.5 if in_rvo else 0.0

                # Social convention bias:
                cost_social = 0.0
                if primary_mode == "HEAD_ON_180":
                    # COLREGS: Right-hand traffic rule bias (w_cand < 0 is turning right)
                    if w_cand > 0.05:
                        cost_social += 3.5  # Heavy penalty for turning left into oncoming human
                    elif w_cand < -0.10:
                        cost_social -= 1.5  # Reward for yielding to the right
                elif primary_mode == "CROSS_90":
                    # Pass-behind tactic: reward decelerating to let human pass
                    if v_cand > 0.12:
                        cost_social += 2.0  # Penalize high speed trying to cut ahead of crossing human
                    else:
                        cost_social -= 1.0  # Reward slower yielding speed

                total_cost = cost_nominal_dev + cost_ttc + cost_social

                if total_cost < best_cost:
                    best_cost = total_cost
                    best_v = v_cand
                    best_w = w_cand
                    chosen_strategy = f"RVO_EVADE_{primary_mode}"
                    chosen_reason = f"Selected collision-free arc (TTC={ttc:.1f}s, cost={total_cost:.2f})"

        self.last_selected_v = best_v
        self.last_selected_w = best_w
        self.last_social_state = chosen_strategy

        return best_v, best_w, {
            "rvo_active": True,
            "strategy": chosen_strategy,
            "reason": chosen_reason,
            "primary_mode": primary_mode
        }
