#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Expert Oracle for DAgger Imitation Learning (149D Perception & Disparity Extender SOTA)
========================================================================================
CS532 Advanced Autonomous Mobile Robotics — JetBot GA25-370 Platform

Acts as the Teacher in the DAgger loop:
Uses Active Disparity Extender (F1TENTH / Pfeiffer et al. ICRA 2017) with geometric margin inflation:
  Delta_theta = arcsin((R_bot + margin) / d_near)
Selects optimal clearance gap and outputs smooth conflict-free expert actions u* = (v*, w*).
"""

import math
import os
import sys
import numpy as np
from typing import List, Tuple, Optional, Any

try:
    from ..disparity_extender import DisparityExtender
except (ImportError, ValueError):
    try:
        from planning.disparity_extender import DisparityExtender
    except ImportError:
        try:
            from disparity_extender import DisparityExtender
        except ImportError:
            DisparityExtender = Any


class ExpertOracle:
    """
    Expert Oracle for DAgger Dataset Aggregation.
    Provides expert supervision u* = (v*, w*) given the current state and surroundings.
    Employs Active Disparity Extender (DDE) with obstacle inflation and gap pursuit.
    """

    def __init__(
        self,
        max_v: float = 0.12,
        max_w: float = 0.35,
        lookahead_dist: float = 0.30,
        safe_clearance_m: float = 0.30,
        robot_radius: float = 0.10,
        safety_margin: float = 0.08
    ):
        self.max_v = max_v
        self.max_w = max_w
        self.lookahead_dist = lookahead_dist
        self.safe_clearance_m = safe_clearance_m

        if DisparityExtender is not Any:
            self.dde = DisparityExtender(
                robot_radius=robot_radius,
                safety_margin=safety_margin,
                max_lidar_range=5.0,
                warning_horizon_m=0.60,
                safe_clearance_m=safe_clearance_m,
                critical_stop_m=0.15
            )
        else:
            self.dde = None

    def compute_expert_action(
        self,
        robot_pose: Tuple[float, float, float],
        subgoal: Tuple[float, float],
        lidar_ranges: np.ndarray,
        dynamic_obstacles: Optional[List[Any]] = None
    ) -> Tuple[float, float]:
        """
        Computes the optimal expert action u* = (v*, w*) toward the subgoal.
        Supports raw 72 sectors range, or full 149D state (with Delta r).
        """
        rx, ry, rth = robot_pose
        gx, gy = subgoal

        # Separate range sectors from 149D vector
        raw = np.asarray(lidar_ranges, dtype=np.float32)
        if len(raw) >= 72:
            ranges_72 = raw[:72]
        else:
            ranges_72 = raw

        dx = gx - rx
        dy = gy - ry
        goal_angle = math.atan2(dy, dx)
        alpha = (goal_angle - rth + math.pi) % (2.0 * math.pi) - math.pi

        if self.dde is not None and len(ranges_72) >= 72:
            # Convert 72 sectors [-pi, pi) to 360 array where 0 deg is front.
            # Sector 36 corresponds to 0 deg (front). Roll by -36 so Sector 36 becomes index 0.
            rolled_72 = np.roll(ranges_72, -36)
            scan_360 = np.repeat(rolled_72 * 5.0, 5).tolist()

            eval_res = self.dde.evaluate(scan_360, goal_heading_rad=alpha)
            target_heading = eval_res['target_heading_rad']
            clearance = eval_res.get('front_obstacle_dist', 5.0)

            if eval_res.get('status') == 'EMERGENCY_STOP':
                return 0.0, 0.0

            if abs(target_heading) > math.radians(35):
                v = 0.04
                w = math.copysign(self.max_w, target_heading)
            else:
                speed_factor = max(0.4, math.cos(target_heading) ** 2)
                dist_factor = min(1.0, max(0.4, clearance / 0.5))
                v = self.max_v * speed_factor * dist_factor
                w = max(-self.max_w, min(self.max_w, 2.2 * math.sin(target_heading)))

            return round(float(v), 3), round(float(w), 3)

        # Fallback pure pursuit if DDE is not available
        speed_factor = max(0.3, math.cos(alpha) ** 2)
        v_nom = self.max_v * speed_factor
        w_nom = max(-self.max_w, min(self.max_w, 2.0 * v_nom * math.sin(alpha) / max(0.15, self.lookahead_dist)))
        return round(float(v_nom), 3), round(float(w_nom), 3)
