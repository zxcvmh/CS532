"""
Real-Time Policy Runner for JetBot Edge Deployment
==================================================
CS532 Autonomous JetBot — Module 07-imitation-learning
Based on UIT ITCourses RL-101 (Bài 10: Học bắt chước - Imitation Learning)

Features:
  - Ultra-fast zero-dependency NumPy inference (< 0.08ms per cycle)
  - Emergency safety bumper & watchdog guard
  - Smooth slew-rate acceleration ramp (anti-wheel-slip on tile)
  - Deadband stiction override
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
        target_path = model_path or weights_path or "models/dagger_policy.onnx"
        # Resolve candidates: prioritize ONNX over JSON
        candidate_paths = [
            target_path,
            target_path.replace(".json", ".onnx"),
            target_path.replace(".onnx", ".json"),
            "models/dagger_policy.onnx",
            "models/bc_policy.onnx",
            "models/dagger_policy_numpy.json",
            "models/bc_policy_numpy.json",
            "07-imitation-learning/test_artifacts/models/dagger_policy.onnx",
            "07-imitation-learning/test_artifacts/models/bc_policy.onnx",
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

        # Diagnostics
        self.last_latency_ms = 0.0
        self.last_status = "INITIALIZED"
        self.bypass_latch = 0  # Hysteresis latch for smooth obstacle bypass (+1: Left, -1: Right)


    def compute_command(
        self,
        lidar_ranges_360: List[float],
        goal_x: float,
        goal_y: float,
        robot_x: float,
        robot_y: float,
        robot_theta: float,
        curr_v: float = 0.0,
        curr_w: float = 0.0
    ) -> Tuple[float, float, Dict[str, Any]]:
        """
        Computes (v, w) command from current state.
        Returns: (v_cmd, w_cmd, info_dict)
        """
        t0 = time.perf_counter()
        now = time.time()
        dt = 0.10
        if self.last_step_time > 0:
            dt = max(0.01, min(0.3, now - self.last_step_time))
        self.last_step_time = now

        # 1. Goal Arrival Check
        dx = goal_x - robot_x
        dy = goal_y - robot_y
        dist_goal = math.hypot(dx, dy)
        if dist_goal <= self.goal_tolerance:
            self.active_v = 0.0
            self.active_w = 0.0
            self.last_status = "GOAL_REACHED"
            self.last_latency_ms = (time.perf_counter() - t0) * 1000.0
            return 0.0, 0.0, {
                "status": "GOAL_REACHED",
                "dist_goal": round(dist_goal, 3),
                "latency_ms": round(self.last_latency_ms, 3)
            }

        angle_world = math.atan2(dy, dx)
        alpha_goal = (angle_world - robot_theta + math.pi) % (2.0 * math.pi) - math.pi

        # 2. Extract 41D Observation
        obs = extract_observation_vector(
            raw_ranges_360=lidar_ranges_360,
            goal_x=goal_x,
            goal_y=goal_y,
            robot_x=robot_x,
            robot_y=robot_y,
            robot_theta=robot_theta,
            curr_v=curr_v,
            curr_w=curr_w
        )

        # 3. Perception & Reactive Navigation Hierarchy
        obs_horizon = min(0.65, max(0.25, dist_goal + 0.08))
        front_ranges = [
            lidar_ranges_360[d % 360]
            for d in range(-35, 36)
            if 0.05 <= lidar_ranges_360[d % 360] <= obs_horizon
        ]

        # Flank checking (obstacle alongside robot):
        left_flank_ranges = [
            lidar_ranges_360[d]
            for d in range(20, 86)
            if 0.05 <= lidar_ranges_360[d] <= 0.45
        ]
        right_flank_ranges = [
            lidar_ranges_360[(360 - d) % 360]
            for d in range(20, 86)
            if 0.05 <= lidar_ranges_360[(360 - d) % 360] <= 0.45
        ]

        if front_ranges:
            min_front = min(front_ranges)
            if self.bypass_latch == 0:
                left_space = float(np.mean([lidar_ranges_360[d] for d in range(25, 75)]))
                right_space = float(np.mean([lidar_ranges_360[(360 - d) % 360] for d in range(25, 75)]))
                if alpha_goal > 0.05:
                    self.bypass_latch = +1
                elif alpha_goal < -0.05:
                    self.bypass_latch = -1
                else:
                    self.bypass_latch = +1 if left_space >= right_space else -1

            # Maintain continuous S-curve bypass without oscillating
            raw_v = max(MIN_DRIVE_SPEED, 0.12 * min(1.0, min_front / 0.55))
            raw_w = float(self.bypass_latch) * 0.35
            self.last_status = "BYPASS_CURVE"

        elif self.bypass_latch == +1 and right_flank_ranges:
            # Evading left; obstacle is on right flank: cruise forward past it, forbidden to turn right!
            raw_v = 0.13
            raw_w = 0.0
            self.last_status = "BYPASS_RIGHT_FLANK"

        elif self.bypass_latch == -1 and left_flank_ranges:
            # Evading right; obstacle is on left flank: cruise forward past it, forbidden to turn left!
            raw_v = 0.13
            raw_w = 0.0
            self.last_status = "BYPASS_LEFT_FLANK"

        else:
            self.bypass_latch = 0
            if abs(alpha_goal) > math.radians(55.0):
                # Target is far to the side or behind: execute safe in-place pivot alignment
                raw_v = 0.0
                raw_w = math.copysign(0.32, alpha_goal)
                self.last_status = "PIVOT_ALIGN"
            else:
                raw_v, raw_w = self.runner.forward(obs)
                self.last_status = "NAVIGATING"

        # 4. Critical Emergency Shield (< 0.20m directly ahead)
        # Prevents triggering physical hardware dynamic brake (cmd_v > 0.01)
        critical_ranges = [
            lidar_ranges_360[d % 360]
            for d in range(-35, 36)
            if 0.05 <= lidar_ranges_360[d % 360] <= 0.20
        ]
        if critical_ranges:
            left_clear = float(np.mean([lidar_ranges_360[d] for d in range(25, 75)]))
            right_clear = float(np.mean([lidar_ranges_360[(360 - d) % 360] for d in range(25, 75)]))
            # PIVOT ON THE SPOT - strictly zero linear velocity to avoid DDE dynamic brake
            raw_v = 0.0
            escape_dir = self.bypass_latch if self.bypass_latch != 0 else (+1 if left_clear >= right_clear else -1)
            raw_w = float(escape_dir) * 0.38
            self.last_status = "REFLEX_EVADE"

        # 5. Slew-rate smooth acceleration limiting
        if self.last_status in ("REFLEX_EVADE", "GOAL_REACHED"):
            # Instant decel for critical safety reflex - do not inertia-drift forward
            self.active_v = 0.0
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

