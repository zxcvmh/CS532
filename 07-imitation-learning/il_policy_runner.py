"""
Real-Time Policy Runner for JetBot Edge Deployment (v2)
=======================================================
CS532 Autonomous JetBot — Module 07-imitation-learning
Based on UIT ITCourses RL-101 (Bài 10: Học bắt chước - Imitation Learning)

Features (v2):
  1. Terminal Approach Zone Protection:
     - Prevents erratic swerving when approaching destination near walls/objects.
  2. Dynamic Yield for Moving Obstacles:
     - Smoothly pauses when a moving object cuts directly across, resuming when clear.
  3. High-Torque Emergency Reflex:
     - Uses 1.4 rad/s pivot to overcome physical motor deadband stiction without stalling.
  4. Decoupled Hardware Invariance:
     - 100% independent of IMU or wheel encoders (pure LiDAR + Relative Goal).
  5. Slew-rate smooth acceleration ramp (anti-wheel-slip on ceramic tile).
"""

import os
import math
import time
from typing import List, Tuple, Dict, Any, Optional
import numpy as np

from config import (
    MAX_LINEAR_SPEED,
    MAX_ANGULAR_SPEED,
    MIN_DRIVE_SPEED,
    GOAL_TOLERANCE_M,
    LIDAR_MAX_RANGE_M
)
from dataset import extract_observation_vector
from model import NumpyPolicyRunner, ONNXPolicyRunner, HAS_ORT


class ILPolicyRunner:
    """
    Real-time reactive navigation controller powered by trained Imitation Learning Policy.
    Supports high-performance ONNX Runtime (.onnx) with seamless fallback to pure NumPy.
    """
    def __init__(
        self,
        model_path: Optional[str] = None,
        weights_path: Optional[str] = None,
        max_v: float = MAX_LINEAR_SPEED,
        max_w: float = MAX_ANGULAR_SPEED,
        goal_tolerance: float = GOAL_TOLERANCE_M
    ):
        target_path = model_path or weights_path or "models/bc_policy.onnx"
        # Resolve candidates: prioritize ONNX over JSON
        candidate_paths = [
            target_path,
            target_path.replace(".json", ".onnx"),
            target_path.replace(".onnx", ".json"),
            "models/bc_policy.onnx",
            "models/dagger_policy.onnx",
            "models/bc_policy_numpy.json",
            "models/dagger_policy_numpy.json",
            "07-imitation-learning/test_artifacts/models/bc_policy.onnx",
            "07-imitation-learning/test_artifacts/models/dagger_policy.onnx",
        ]

        resolved_path = None
        for cp in candidate_paths:
            if os.path.exists(cp):
                resolved_path = cp
                break

        if resolved_path is None:
            resolved_path = target_path

        # Load appropriate runner
        if HAS_ORT and resolved_path.endswith(".onnx") and os.path.exists(resolved_path):
            print(f"[RUNNER] Initializing Standard ONNX Runtime Engine: {resolved_path}")
            self.runner = ONNXPolicyRunner(resolved_path)
            self.engine_type = "ONNX"
        else:
            # Fallback to NumPy JSON
            json_path = resolved_path.replace(".onnx", "_numpy.json")
            if not os.path.exists(json_path):
                json_path = resolved_path
            print(f"[RUNNER] Initializing Pure NumPy Engine: {json_path}")
            self.runner = NumpyPolicyRunner(json_path)
            self.engine_type = "NUMPY"

        self.max_v = max_v
        self.max_w = max_w
        self.goal_tolerance = goal_tolerance

        # Slew-rate smooth acceleration
        self.active_v = 0.0
        self.active_w = 0.0
        self.last_step_time = 0.0

        # Diagnostics & State Latches
        self.last_latency_ms = 0.0
        self.last_status = "INITIALIZED"
        self.bypass_latch = 0     # Hysteresis latch for smooth obstacle bypass (+1: Left, -1: Right)
        self.yield_steps = 0      # Dynamic yield counter for sudden moving obstacles
        self.max_yield_steps = 8  # ~0.8s max pause before switching to active evasive pivot
        self._yield_armed = True  # Only yield once per sudden obstacle encounter
        self._last_steer_angle = 0.0

        # Precompute candidate angles [-90 deg, +90 deg] for vectorized C-Space directional field
        self._cand_deg = np.linspace(-90.0, 90.0, 91, dtype=np.float32)
        self._cand_rad = np.deg2rad(self._cand_deg)

    def reset(self):
        """Resets internal rate-limiters and obstacle avoidance latches between episodes/goals."""
        self.active_v = 0.0
        self.active_w = 0.0
        self.last_step_time = 0.0
        self.bypass_latch = 0
        self.yield_steps = 0
        self._yield_armed = True
        self._last_steer_angle = 0.0

    @staticmethod
    def fuse_low_obstacles_into_scan(
        lidar_ranges_360: List[float],
        low_obstacles: Optional[List[Any]] = None,
        default_obs_radius: float = 0.045,
    ) -> List[float]:
        """
        Projects camera Floor-IPM low-lying obstacles (<16cm height, invisible to 16cm LiDAR)
        into the 360-degree virtual range array in the robot frame.
        Each item in low_obstacles can be [x_r, y_r] or [x_r, y_r, radius] in meters (robot frame).
        """
        if not low_obstacles:
            return lidar_ranges_360
        fused = list(lidar_ranges_360)
        for pt in low_obstacles:
            if not isinstance(pt, (list, tuple, np.ndarray)) or len(pt) < 2:
                continue
            xr, yr = float(pt[0]), float(pt[1])
            r_obs = float(pt[2]) if len(pt) >= 3 else default_obs_radius
            dist_center = math.hypot(xr, yr)
            if dist_center < 0.05 or dist_center > LIDAR_MAX_RANGE_M:
                continue
            surf_dist = max(0.05, dist_center - r_obs)
            bearing_deg = math.degrees(math.atan2(yr, xr))
            half_cone_deg = max(3, int(math.ceil(math.degrees(math.asin(min(0.95, (r_obs + 0.02) / dist_center))))))
            center_idx = int(round(bearing_deg)) % 360
            for d_off in range(-half_cone_deg, half_cone_deg + 1):
                idx = (center_idx + d_off) % 360
                if fused[idx] <= 0.0 or surf_dist < fused[idx]:
                    fused[idx] = surf_dist
        return fused

    def _compute_cspace_steering(
        self,
        scan_360: List[float],
        alpha_goal: float,
        dist_goal: float,
        nn_v: float,
        nn_w: float,
        blocking_goal: bool = False,
    ) -> Tuple[float, float, str]:
        """
        Vectorized 2D C-Space Free-Path & Gap-Aware Local Planner blended with Neural Policy.
        Handles single small obstacles (Ca Nuoc), multi-obstacle slaloms, narrow gates, and corridors.
        """
        # Gather valid obstacle hits in front/flank hemisphere [-105 deg, +105 deg]
        horizon = min(0.90, max(0.35, dist_goal + 0.10))
        obs_angles = []
        obs_ranges = []
        for deg in range(-105, 106):
            r = scan_360[deg % 360]
            if 0.05 <= r <= horizon:
                # Ignore obstacles strictly behind the goal along the goal heading cone
                ang_rad = math.radians(deg)
                if abs(ang_rad - alpha_goal) < math.radians(18.0) and r > (dist_goal + 0.04):
                    continue
                obs_angles.append(ang_rad)
                obs_ranges.append(r)

        if not obs_ranges:
            self.bypass_latch = 0
            self._last_steer_angle = 0.0
            if abs(alpha_goal) > math.radians(55.0):
                return 0.0, math.copysign(self.max_w, alpha_goal), "PIVOT_ALIGN"
            return nn_v, nn_w, "NAVIGATING"

        obs_ang_arr = np.asarray(obs_angles, dtype=np.float32)  # (M,)
        obs_rng_arr = np.asarray(obs_ranges, dtype=np.float32)  # (M,)

        # Check minimum frontal range in [-32 deg, +32 deg]
        front_mask = np.abs(obs_ang_arr) <= math.radians(32.0)
        min_front = float(np.min(obs_rng_arr[front_mask])) if np.any(front_mask) else 1.5

        # Check flank proximity in [-85 deg, +85 deg]
        near_mask = obs_rng_arr <= 0.55
        if not np.any(front_mask) and not np.any(near_mask):
            self.bypass_latch = 0
            self._last_steer_angle = 0.0
            if abs(alpha_goal) > math.radians(55.0):
                return 0.0, math.copysign(self.max_w, alpha_goal), "PIVOT_ALIGN"
            return nn_v, nn_w, "NAVIGATING"

        # Vectorized C-Space ray clearance evaluation across 91 candidate directions [-90, +90] deg
        diff = self._cand_rad[:, None] - obs_ang_arr[None, :]
        abs_diff = np.abs(diff)
        forward_cone = abs_diff < (math.pi * 0.5)

        d_perp = obs_rng_arr[None, :] * np.sin(abs_diff)
        d_para = obs_rng_arr[None, :] * np.cos(abs_diff)

        # If target waypoint itself sits on an obstacle (dist_goal < 0.45m and blocking_goal),
        # use tight safe C-space clearance so the robot skirts within < 0.25m of the waypoint center
        if dist_goal < 0.45 and blocking_goal:
            r_hard = 0.130
            r_soft = 0.165
        else:
            r_hard = 0.145
            r_soft = 0.210

        # Free distance along each candidate ray before touching r_hard disk
        free_dist_mat = np.full(diff.shape, horizon, dtype=np.float32)
        hit_hard = forward_cone & (d_perp < r_hard)
        if np.any(hit_hard):
            pen = np.sqrt(np.maximum(0.0, r_hard * r_hard - d_perp[hit_hard] * d_perp[hit_hard]))
            free_dist_mat[hit_hard] = np.maximum(0.0, d_para[hit_hard] - pen)

        min_free_dist = np.min(free_dist_mat, axis=1)  # (91,)

        # Soft clearance penalty for candidate rays grazing between r_hard and r_soft
        soft_zone = forward_cone & (d_perp >= r_hard) & (d_perp < r_soft) & (d_para < 0.65)
        soft_pen_mat = np.zeros(diff.shape, dtype=np.float32)
        if np.any(soft_zone):
            prox = (r_soft - d_perp[soft_zone]) / (r_soft - r_hard)
            dist_w = np.clip((0.65 - d_para[soft_zone]) / 0.65, 0.1, 1.0)
            soft_pen_mat[soft_zone] = prox * dist_w
        soft_penalty = np.max(soft_pen_mat, axis=1)  # (91,)

        # Goal alignment error for each candidate direction
        clamped_goal = max(-math.pi * 0.65, min(math.pi * 0.65, alpha_goal))
        goal_err = np.abs(self._cand_rad - clamped_goal)

        # Check if direct goal ray is obstructed and measure left/right tangent angular costs
        goal_idx = int(np.argmin(goal_err))
        pass_thresh = min(0.58, max(0.32, dist_goal - 0.02))
        if min_free_dist[goal_idx] < pass_thresh:
            # Find nearest open candidate on left (+angle, idx > goal_idx) and right (-angle, idx < goal_idx)
            open_left = [i for i in range(goal_idx, len(self._cand_rad)) if min_free_dist[i] >= pass_thresh]
            open_right = [i for i in range(goal_idx, -1, -1) if min_free_dist[i] >= pass_thresh]
            err_left = abs(float(self._cand_rad[open_left[0]]) - clamped_goal) if open_left else math.pi
            err_right = abs(float(self._cand_rad[open_right[0]]) - clamped_goal) if open_right else math.pi

            if self.bypass_latch == 0:
                self.bypass_latch = +1 if err_left <= err_right else -1
            elif self.bypass_latch == +1 and (err_right + math.radians(22.0) < err_left):
                # Staggered slalom obstacle on left -> switch to much closer right gap
                self.bypass_latch = -1
            elif self.bypass_latch == -1 and (err_left + math.radians(22.0) < err_right):
                # Staggered slalom obstacle on right -> switch to much closer left gap
                self.bypass_latch = +1

        # Smooth hysteresis with previous steering angle to prevent left-right chatter across symmetric gates
        smooth_err = np.abs(self._cand_rad - self._last_steer_angle)
        latch_bonus = np.zeros_like(self._cand_rad)
        if self.bypass_latch != 0:
            latch_strength = 0.70 if min_free_dist[goal_idx] < pass_thresh else 0.15
            latch_bonus = np.where((self._cand_rad - clamped_goal) * self.bypass_latch >= -0.05, latch_strength, 0.0)

        # Score candidate directions: maximize free distance, minimize goal error and soft grazing penalty
        scores = (
            2.4 * (min_free_dist / max(0.25, horizon))
            - 1.15 * (goal_err / (math.pi * 0.5))
            - 0.85 * soft_penalty
            - 0.30 * (smooth_err / (math.pi * 0.5))
            + latch_bonus
        )

        # Penalize blocked directions heavily
        blocked_cands = min_free_dist < min(0.32, dist_goal)
        scores = np.where(blocked_cands, scores - 3.0, scores)

        best_idx = int(np.argmax(scores))
        best_angle = float(self._cand_rad[best_idx])
        best_free = float(min_free_dist[best_idx])
        self._last_steer_angle = 0.65 * best_angle + 0.35 * self._last_steer_angle

        if self.bypass_latch == 0 and abs(best_angle) > math.radians(8.0):
            self.bypass_latch = +1 if best_angle > 0 else -1

        # In-place pivot alignment when:
        # 1) Best C-space gap angle is very sharp (> 55 deg), OR
        # 2) Goal is far off-axis (> 55 deg) AND the direct C-space path toward goal is unobstructed
        goal_path_clear = float(min_free_dist[goal_idx]) >= pass_thresh
        if (
            abs(best_angle) > math.radians(55.0)
            or (abs(alpha_goal) > math.radians(55.0) and goal_path_clear and min_front > 0.45)
        ):
            steer_dir = best_angle if not goal_path_clear else alpha_goal
            return 0.0, math.copysign(self.max_w, steer_dir), "PIVOT_ALIGN"

        # Compute angular velocity proportional to best C-space gap angle, blended with neural policy when nearly clear
        kp_w = 1.25 if min_front < 0.48 else 0.95
        cspace_w = max(-self.max_w, min(self.max_w, kp_w * best_angle))

        if min_front < 0.55 or soft_penalty[45] > 0.15 or not goal_path_clear:
            # Active obstacle evasion (or flank obstacle still blocking direct goal line): prioritize C-Space gap steering
            blend_nn = 0.15 if (min_front >= 0.35 and goal_path_clear) else 0.0
            raw_w = (1.0 - blend_nn) * cspace_w + blend_nn * nn_w
            raw_w = max(-self.max_w, min(self.max_w, raw_w))
            speed_scale = min(1.0, max(0.75, best_free / 0.55)) * (1.0 - 0.25 * (abs(raw_w) / self.max_w))
            raw_v = max(MIN_DRIVE_SPEED, min(self.max_v, 0.135 * speed_scale))
            return raw_v, raw_w, "BYPASS_CURVE"

        # Clear frontal corridor: blend neural policy with C-space guidance
        raw_w = 0.70 * nn_w + 0.30 * cspace_w
        raw_v = nn_v
        return raw_v, raw_w, "NAVIGATING"

    def compute_command(
        self,
        lidar_ranges_360: List[float],
        goal_x: float,
        goal_y: float,
        robot_x: float,
        robot_y: float,
        robot_theta: float,
        curr_v: float = 0.0,
        curr_w: float = 0.0,
        low_obstacles: Optional[List[Any]] = None,
    ) -> Tuple[float, float, Dict[str, Any]]:
        """
        Computes (v, w) command from current state.
        Hardware Decoupled: curr_v and curr_w are ignored (no IMU/encoder needed).
        Fuses optional camera Floor-IPM low_obstacles (<16cm) directly into the 360 scan.
        Returns: (v_cmd, w_cmd, info_dict)
        """
        t0 = time.perf_counter()
        now = time.time()
        dt = 0.10
        if self.last_step_time > 0:
            dt = max(0.01, min(0.3, now - self.last_step_time))
        self.last_step_time = now

        # 0. Fuse camera Floor-IPM low-lying obstacles (<16cm) into 360-degree scan if present
        effective_ranges = (
            self.fuse_low_obstacles_into_scan(lidar_ranges_360, low_obstacles)
            if low_obstacles
            else lidar_ranges_360
        )

        # 1. Goal Arrival Check
        dx = goal_x - robot_x
        dy = goal_y - robot_y
        dist_goal = math.hypot(dx, dy)
        if dist_goal <= self.goal_tolerance:
            self.active_v = 0.0
            self.active_w = 0.0
            self.bypass_latch = 0
            self._last_steer_angle = 0.0
            self.last_status = "GOAL_REACHED"
            self.last_latency_ms = (time.perf_counter() - t0) * 1000.0
            return 0.0, 0.0, {
                "status": "GOAL_REACHED",
                "dist_goal": round(dist_goal, 3),
                "latency_ms": round(self.last_latency_ms, 3)
            }

        angle_world = math.atan2(dy, dx)
        alpha_goal = (angle_world - robot_theta + math.pi) % (2.0 * math.pi) - math.pi

        # 2. Extract 41D Observation (curr_v & curr_w are neutralized to 0.0 internally)
        obs = extract_observation_vector(
            raw_ranges_360=effective_ranges,
            goal_x=goal_x,
            goal_y=goal_y,
            robot_x=robot_x,
            robot_y=robot_y,
            robot_theta=robot_theta,
            curr_v=0.0,
            curr_w=0.0
        )

        # Check if straight line to goal is unobstructed
        goal_cone_deg = int(round(math.degrees(alpha_goal)))
        blocking_goal = [
            effective_ranges[(goal_cone_deg + d) % 360]
            for d in range(-18, 19)
            if 0.05 <= effective_ranges[(goal_cone_deg + d) % 360] < (dist_goal - 0.03)
        ]

        # 3. Terminal Approach Zone (dist_goal <= 0.35m and no obstacle between robot and goal):
        # Prevents aggressive swerving away from destinations located near walls/tables behind the goal
        if dist_goal <= 0.35 and not blocking_goal:
            self.bypass_latch = 0
            self.yield_steps = 0
            immed_front = [
                effective_ranges[d % 360]
                for d in range(-22, 23)
                if 0.05 <= effective_ranges[d % 360] < min(0.14, max(0.09, dist_goal - 0.04))
            ]
            if immed_front:
                raw_v = 0.0
                raw_w = 0.0
                self.last_status = "TERMINAL_HALT"
            elif abs(alpha_goal) > math.radians(45.0):
                raw_v = 0.0
                raw_w = math.copysign(self.max_w, alpha_goal)
                self.last_status = "PIVOT_ALIGN"
            else:
                raw_v = max(MIN_DRIVE_SPEED, min(self.max_v, 0.13 * (dist_goal / 0.35)))
                raw_w = max(-self.max_w, min(self.max_w, 1.5 * alpha_goal))
                self.last_status = "TERMINAL_APPROACH"

        else:
            # 4. Standard Navigation & C-Space Obstacle Hierarchy
            critical_ranges = [
                effective_ranges[d % 360]
                for d in range(-25, 26)
                if 0.05 <= effective_ranges[d % 360] <= 0.15
            ]

            if critical_ranges:
                # Imminent collision bubble (< 0.15m in [-25, +25] deg):
                if self._yield_armed and self.yield_steps < self.max_yield_steps:
                    # Dynamic yield: pause smoothly to allow sudden moving obstacle to cross
                    self.yield_steps += 1
                    raw_v = 0.0
                    raw_w = 0.0
                    self.last_status = "DYNAMIC_YIELD"
                else:
                    self._yield_armed = False
                    # Persistent obstruction: active evasive pivot with sufficient torque to overcome stiction
                    left_clear = float(np.mean([effective_ranges[d] for d in range(20, 75)]))
                    right_clear = float(np.mean([effective_ranges[(360 - d) % 360] for d in range(20, 75)]))
                    raw_v = 0.0
                    escape_dir = self.bypass_latch if self.bypass_latch != 0 else (+1 if left_clear >= right_clear else -1)
                    # 1.4 rad/s yields ~0.14 PWM on JetBot hardware, guaranteeing motor spin without stalling
                    raw_w = float(escape_dir) * 1.40
                    self.last_status = "REFLEX_EVADE"

            else:
                # Re-arm dynamic yield once frontal 0.28m zone is clear
                front_clear_check = [
                    effective_ranges[d % 360]
                    for d in range(-30, 31)
                    if 0.05 <= effective_ranges[d % 360] <= 0.28
                ]
                if not front_clear_check:
                    self.yield_steps = 0
                    self._yield_armed = True

                nn_v, nn_w = self.runner.forward(obs)
                raw_v, raw_w, self.last_status = self._compute_cspace_steering(
                    scan_360=effective_ranges,
                    alpha_goal=alpha_goal,
                    dist_goal=dist_goal,
                    nn_v=nn_v,
                    nn_w=nn_w,
                    blocking_goal=bool(blocking_goal),
                )

        # 5. Slew-rate smooth acceleration limiting
        if self.last_status in ("REFLEX_EVADE", "DYNAMIC_YIELD", "TERMINAL_HALT", "GOAL_REACHED", "PIVOT_ALIGN"):
            # Instant linear decel for safety / in-place pivot alignment
            self.active_v = 0.0
            if self.last_status in ("REFLEX_EVADE", "PIVOT_ALIGN"):
                self.active_w = raw_w  # Apply evasive turn torque immediately without lag
            else:
                self.active_w = 0.0
        else:
            max_step_v = (MAX_LINEAR_SPEED / 0.40) * dt  # 0.4s soft-start ramp
            diff_v = raw_v - self.active_v
            if abs(diff_v) <= max_step_v:
                self.active_v = raw_v
            else:
                self.active_v += math.copysign(max_step_v, diff_v)

            max_step_w = (MAX_ANGULAR_SPEED / 0.25) * dt
            diff_w = raw_w - self.active_w
            if abs(diff_w) <= max_step_w:
                self.active_w = raw_w
            else:
                self.active_w += math.copysign(max_step_w, diff_w)

        # Enforce deadband stiction on final command
        cmd_v = self.active_v
        cmd_w = self.active_w
        if 0.0 < cmd_v < MIN_DRIVE_SPEED:
            cmd_v = MIN_DRIVE_SPEED if cmd_v >= 0.05 else 0.0

        self.last_latency_ms = (time.perf_counter() - t0) * 1000.0

        return float(round(cmd_v, 3)), float(round(cmd_w, 3)), {
            "status": self.last_status,
            "dist_goal": round(dist_goal, 3),
            "latency_ms": round(self.last_latency_ms, 3),
            "v_cmd": round(cmd_v, 3),
            "w_cmd": round(cmd_w, 3)
        }


# Convenience alias for coordinator and benchmark integration
ILPolicyCoordinatorRunner = ILPolicyRunner
