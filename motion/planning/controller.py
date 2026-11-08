"""
Pure Pursuit Path Tracking, Curvature Velocity Scaling & Plateau Detour Controller
=============================================================================
CS532 Advanced AI Robotics (Autonomous Planning & Control Agent)
Academic Standard (2024-2026 IEEE Robotics & Autonomous Systems):
  - Pure Pursuit Path Tracking with Curvature Steering
  - Curvature-Aware Velocity Scaling v = v_base / (1 + k*|w|)
  - 3-Segment Trapezoidal Plateau Detour (clearance margin >= 0.22m, lateral_offset = 0.52m)
  - 4D Space-Time Collision Inspector along dynamic path horizon (1.8m)
  - 10Hz Hardware Reflex Emergency Brake (< 0.20m frontal cone)
  - Closed-Loop Auto-Evade on Critical Threats and Dynamic Oncoming Obstacles
=============================================================================
"""

import os
import time
import json
import math
from typing import List, Tuple, Optional, Dict, Any

try:
    from .obstacle_clustering import ObstacleCluster
except ImportError:
    try:
        from obstacle_clustering import ObstacleCluster
    except ImportError:
        ObstacleCluster = Any

try:
    from .kinodynamic_rvo import KinodynamicRVOPlanner
except ImportError:
    try:
        from kinodynamic_rvo import KinodynamicRVOPlanner
    except ImportError:
        KinodynamicRVOPlanner = None

try:
    from .disparity_extender import DisparityExtender
except ImportError:
    try:
        from disparity_extender import DisparityExtender
    except ImportError:
        DisparityExtender = None

try:
    from .cbf_safety_filter import CBFSafetyFilter
except ImportError:
    try:
        from cbf_safety_filter import CBFSafetyFilter
    except ImportError:
        CBFSafetyFilter = None

try:
    from .control import PurePursuitTracker, VelocityRegulator, MotorDynamics
except ImportError:
    try:
        from planning.control import PurePursuitTracker, VelocityRegulator, MotorDynamics
    except ImportError:
        PurePursuitTracker = None
        VelocityRegulator = None
        MotorDynamics = None


class PurePursuitController:
    def __init__(
        self,
        lookahead_dist: float = 0.28,
        max_linear_speed: float = 0.14,
        max_angular_speed: float = 0.40,
        track_width: float = 0.115,
        goal_tolerance: float = 0.18,
        reflex_dist_threshold: float = 0.30,
        warning_dist_threshold: float = 0.55,
        curvature_scale_k: float = 1.2,
        lateral_detour_offset: float = 0.22,
        min_drive_speed: float = 0.12,
        trim_factor: float = 1.015,
        soft_start_ramp_sec: float = 0.4,
        env_mode: str = "ARENA_2X1",
        tactical_yield_dist: float = 0.30,
        passage_clearance_min: float = 0.18,
        min_lidar_range: float = 0.05,
    ):
        self.lookahead_dist = lookahead_dist
        self.max_linear_speed = max_linear_speed
        self.max_angular_speed = max_angular_speed
        self.track_width = track_width
        self.goal_tolerance = goal_tolerance
        self.reflex_dist_threshold = reflex_dist_threshold
        self.warning_dist_threshold = warning_dist_threshold
        self.curvature_scale_k = curvature_scale_k
        self.lateral_detour_offset = lateral_detour_offset
        self.min_drive_speed = min_drive_speed
        self.turn_speed = 0.14
        self.trim_factor = trim_factor
        self.soft_start_ramp_sec = soft_start_ramp_sec

        # Multi-scale Environment & Tactical Yielding Parameters
        self.env_mode = env_mode  # "ARENA_2X1" (Sa ban test) or "SEMINAR_5X5" (Phong seminar)
        self.tactical_yield_dist = tactical_yield_dist  # 0.30m stop distance
        self.passage_clearance_min = passage_clearance_min  # 0.18m corridor clearance (allows passing 18cm-25cm corridors)
        # Phase 2.5: the "ignore internal chassis reflection" guard was a local 0.12m
        # literal, inconsistent with the driver/parser floor. One value, from Config.
        self.min_lidar_range = min_lidar_range
        if KinodynamicRVOPlanner is not None:
            self.rvo_planner = KinodynamicRVOPlanner(
                robot_radius=track_width,
                max_speed=max_linear_speed,
                min_drive_speed=min_drive_speed,
                tactical_yield_dist=tactical_yield_dist
            )
        else:
            self.rvo_planner = None

        # SOTA Meta 2026: Active Disparity Extender (DDE) - Polar Sensor-Space Reactive Obstacle Evasion
        if DisparityExtender is not None:
            self.disparity_extender = DisparityExtender(
                robot_radius=0.075,
                safety_margin=0.045,
                disparity_threshold=0.20,
                safe_clearance_m=0.28,
                warning_horizon_m=self.warning_dist_threshold,
                critical_stop_m=self.reflex_dist_threshold
            )
        else:
            self.disparity_extender = None

        # SOTA Meta 2026: Control Barrier Function (CBF-QP) Real-Time Invariance Safety Shield
        if CBFSafetyFilter is not None:
            self.cbf_filter = CBFSafetyFilter(
                lookahead_L=0.10,
                d_safe=0.12,
                gamma=2.8,
                v_max=self.max_linear_speed,
                w_max=self.max_angular_speed
            )
        else:
            self.cbf_filter = None

        self.last_pwm_left: float = 0.0
        self.last_pwm_right: float = 0.0

        self.global_reference_path: List[Tuple[float, float]] = []
        self.global_goal: Optional[Tuple[float, float]] = None
        self.current_path: List[Tuple[float, float]] = []
        self.path_index: int = 0
        self.is_goal_reached: bool = False
        self.is_reflex_stopped: bool = False
        self.is_path_blocked: bool = False
        self.is_evading: bool = False
        self.evasion_end_index: int = 0
        self.last_linear_v: float = 0.0

        # Pudu/Keenon Industrial Standard: Dynamic Wait-and-See FSM before Detour
        self.obstacle_state: str = "CLEAR"  # "CLEAR", "WAITING", "EVADING", "BLOCKED"
        self.obstacle_first_seen_time: float = 0.0
        self.obstacle_wait_duration: float = 1.5  # seconds
        self.blocked_pt: Optional[Tuple[float, float]] = None
        self._last_ctrl_time: float = 0.0
        self._blocked_persistence_count: int = 0
        self.planner: Optional[Any] = None
        self._last_internal_replan_time: float = 0.0

        if PurePursuitTracker is not None:
            self.tracker = PurePursuitTracker(
                lookahead_dist=lookahead_dist,
                goal_tolerance=goal_tolerance,
                pivot_entry_deg=60.0,
                align_exit_deg=30.0,
            )
        else:
            self.tracker = None

        if VelocityRegulator is not None:
            self.velocity_regulator = VelocityRegulator(
                min_drive_speed=min_drive_speed,
                max_linear_speed=max_linear_speed,
                goal_tolerance=goal_tolerance,
                curvature_scale_k=curvature_scale_k,
                soft_start_ramp_sec=soft_start_ramp_sec,
            )
        else:
            self.velocity_regulator = None

        if MotorDynamics is not None:
            self.motor_dynamics = MotorDynamics(
                track_width=track_width,
                min_drive_speed=min_drive_speed,
                max_linear_speed=max_linear_speed,
                max_angular_speed=max_angular_speed,
                trim_factor=trim_factor,
                soft_start_ramp_sec=soft_start_ramp_sec,
            )
        else:
            self.motor_dynamics = None

        self.load_dynamics_config()

    def set_path(self, path: List[Tuple[float, float]]):
        self.global_reference_path = list(path)
        self.global_goal = path[-1] if path else None
        self.current_path = list(path)
        self.path_index = 0
        self.is_goal_reached = False
        self.is_reflex_stopped = False
        self.is_path_blocked = False
        self.is_evading = False
        self.evasion_end_index = 0
        self.obstacle_state = "CLEAR"
        self.obstacle_first_seen_time = 0.0
        self.blocked_pt = None
        self._in_pivot_turn = False
        self._blocked_persistence_count = 0

    def clear_path(self):
        self.current_path = []
        self.global_reference_path = []
        self.global_goal = None
        self.path_index = 0
        self.is_goal_reached = False
        self.is_path_blocked = False
        self.is_evading = False
        self.evasion_end_index = 0
        self.obstacle_state = "CLEAR"
        self.obstacle_first_seen_time = 0.0
        self.blocked_pt = None
        self._blocked_persistence_count = 0

    def check_safety_reflex(self, ranges: List[float], dist_to_goal: Optional[float] = None) -> bool:
        """10Hz Hardware Reflex Emergency Brake on immediate frontal cone.

        The cone edge is ``Config.REFLEX_STOP`` (0.30m, the D500's spec'd accuracy floor),
        not a local 0.16m literal.
        """
        if not ranges or len(ranges) < 360:
            return False

        frontal_indices = list(range(0, 32)) + list(range(328, 360))
        for idx in frontal_indices:
            r = ranges[idx]
            # Ignore internal chassis reflection below the LiDAR near-field floor
            # (Config.LIDAR_MIN_RANGE), then brake inside Config.REFLEX_STOP.
            if self.min_lidar_range <= r < self.reflex_dist_threshold:
                # Terminal Goal Gating: If the obstacle is at or behind the goal, do not trigger reflex stop
                if dist_to_goal is not None and r >= (dist_to_goal + 0.05):
                    continue
                self.is_reflex_stopped = True
                return True

        self.is_reflex_stopped = False
        return False

    def _project_to_path(self, robot_x: float, robot_y: float) -> Tuple[int, Tuple[float, float], float]:
        """Orthogonal projection of robot coordinates onto piecewise path."""
        if not self.current_path or len(self.current_path) < 2:
            return 0, (robot_x, robot_y), 0.0

        best_dist = float("inf")
        best_seg = self.path_index
        best_proj = self.current_path[self.path_index]

        for i in range(self.path_index, len(self.current_path) - 1):
            p1 = self.current_path[i]
            p2 = self.current_path[i + 1]
            dx_seg = p2[0] - p1[0]
            dy_seg = p2[1] - p1[1]
            seg_len_sq = dx_seg * dx_seg + dy_seg * dy_seg

            if seg_len_sq < 1e-6:
                continue

            t = ((robot_x - p1[0]) * dx_seg + (robot_y - p1[1]) * dy_seg) / seg_len_sq
            t_clamped = max(0.0, min(1.0, t))
            proj_x = p1[0] + t_clamped * dx_seg
            proj_y = p1[1] + t_clamped * dy_seg
            d = math.hypot(robot_x - proj_x, robot_y - proj_y)

            if d < best_dist:
                best_dist = d
                best_seg = i
                best_proj = (proj_x, proj_y)

        return best_seg, best_proj, best_dist

    def check_path_collision(
        self,
        robot_x: float,
        robot_y: float,
        robot_theta: float,
        grid_map: Any = None,
        clusters: Optional[List[Any]] = None,
        horizon_m: float = 0.85,
        dist_to_final_goal: Optional[float] = None,
        front_min: Optional[float] = None
    ) -> Tuple[bool, Optional[Tuple[float, float]], str]:
        """
        4D Space-Time Collision Inspector along lookahead horizon (0.85m for indoor arena).
        Detects costmap cell blockages and oncoming dynamic obstacle intersections.
        """
        if not self.current_path or len(self.current_path) < 2:
            self.is_path_blocked = False
            return False, None, "NO_PATH"

        # Terminal Goal Bypass: If robot is within 0.50m of destination and FrontMin > 0.35m,
        # bypass collision check so costmap inflation or ghost cells near the goal do not stop the robot!
        if dist_to_final_goal is not None and front_min is not None:
            if dist_to_final_goal <= 0.50 and front_min > 0.35:
                self.is_path_blocked = False
                return False, None, "TERMINAL_GOAL_BYPASS"

        best_seg, best_proj, _ = self._project_to_path(robot_x, robot_y)
        self.path_index = best_seg

        cos_t = math.cos(robot_theta)
        sin_t = math.sin(robot_theta)

        horizon_m = min(1.0, horizon_m)
        accum_dist = 0.0
        curr_pt = best_proj
        v_est = max(0.12, self.last_linear_v)

        # 0. Direct LiDAR Frontal Obstacle Check (independent of costmap)
        if front_min is not None and front_min < min(horizon_m, 0.45):
            blk_x = robot_x + front_min * cos_t
            blk_y = robot_y + front_min * sin_t
            self.is_path_blocked = True
            return True, (round(blk_x, 3), round(blk_y, 3)), f"LIDAR_FRONTAL_OBSTACLE_{front_min:.2f}m"

        for seg_idx in range(best_seg, len(self.current_path) - 1):
            next_pt = self.current_path[seg_idx + 1]
            seg_dx = next_pt[0] - curr_pt[0]
            seg_dy = next_pt[1] - curr_pt[1]
            seg_len = math.hypot(seg_dx, seg_dy)

            if seg_len < 1e-4:
                curr_pt = next_pt
                continue

            step_size = 0.05
            n_samples = max(1, int(math.ceil(seg_len / step_size)))
            step_len = seg_len / n_samples

            for s in range(1, n_samples + 1):
                interp_x = curr_pt[0] + seg_dx * (s / n_samples)
                interp_y = curr_pt[1] + seg_dy * (s / n_samples)
                accum_dist += step_len

                if accum_dist > horizon_m:
                    self.is_path_blocked = False
                    return False, None, "CLEAR"

                # 1. Costmap Check (Lethal Obstacles Only, Cost >= 254)
                if grid_map is not None:
                    # Ignore lethal cost check if candidate point is within terminal goal tolerance
                    goal_pt = self.current_path[-1]
                    dist_to_goal = math.hypot(interp_x - goal_pt[0], interp_y - goal_pt[1])
                    if dist_to_goal > max(0.12, self.goal_tolerance):
                        c, r = grid_map.world_to_grid(interp_x, interp_y)
                        if not grid_map.is_inside(c, r) or grid_map.costmap[r, c] >= 254:
                            self.is_path_blocked = True
                            return True, (round(interp_x, 3), round(interp_y, 3)), "COSTMAP_OBSTACLE"

                # 2. Dynamic & Static Cluster Collision Check (World Frame Space-Time Horizon)
                if clusters:
                    t_arrival = accum_dist / v_est
                    for cl in clusters:
                        # Obtain obstacle position in World Frame {W}
                        if getattr(cl, "centroid_world", None) is not None:
                            cl_wx, cl_wy = cl.centroid_world
                        else:
                            cx, cy = getattr(cl, "centroid", (0.0, 0.0))
                            cl_wx = robot_x + cx * cos_t - cy * sin_t
                            cl_wy = robot_y + cx * sin_t + cy * cos_t

                        is_dyn = getattr(cl, "is_dynamic", False)
                        vx_w, vy_w = getattr(cl, "velocity", (0.0, 0.0))
                        half_dim = max(getattr(cl, "length_m", 0.15), getattr(cl, "width_m", 0.15)) / 2.0

                        if is_dyn and (abs(vx_w) > 0.05 or abs(vy_w) > 0.05):
                            # Predict obstacle world position at arrival time (Velocity Obstacles)
                            pred_wx = cl_wx + vx_w * t_arrival
                            pred_wy = cl_wy + vy_w * t_arrival
                            # Dynamic safety radius includes social buffer (0.18m) + obstacle half-size
                            r_safe = max(0.28, min(0.48, half_dim + 0.18))
                            dist_dyn = math.hypot(interp_x - pred_wx, interp_y - pred_wy)
                            if dist_dyn <= r_safe:
                                self.is_path_blocked = True
                                return True, (round(interp_x, 3), round(interp_y, 3)), f"DYNAMIC_ONCOMING_#{cl.id}_TTC_{cl.ttc:.1f}s"
                        elif grid_map is None:
                            # Static obstacle proximity check (only active when NO grid_map is present)
                            # When grid_map is loaded, Costmap already handles static wall/obstacle safety margins
                            r_safe_static = min(0.18, half_dim + 0.06)
                            dist_to_cl = math.hypot(interp_x - cl_wx, interp_y - cl_wy)
                            if dist_to_cl <= r_safe_static:
                                self.is_path_blocked = True
                                return True, (round(interp_x, 3), round(interp_y, 3)), f"CLUSTER_OBSTACLE_#{cl.id}_{getattr(cl, 'obstacle_type', 'BOX')}"

            curr_pt = next_pt

        self.is_path_blocked = False
        return False, None, "CLEAR"

    def measure_wall_distance(
        self,
        grid_map: Any,
        origin_pt: Tuple[float, float],
        direction: Tuple[float, float],
        start_offset: float = 0.0,
        max_dist: float = 2.0,
        step: float = 0.02
    ) -> float:
        """
        Raycasts along `direction` starting from `start_offset` outwards
        to find distance to true physical boundary/wall (cost >= 252).
        """
        if grid_map is None:
            return max_dist
        ox, oy = origin_pt
        dx, dy = direction
        res = getattr(grid_map, "resolution", 0.05)
        step = min(step, res * 0.5)
        dist = max(0.0, start_offset)
        # 1. Step past initial obstacle cells if start_offset is still inside obstacle footprint
        while dist <= max_dist:
            px = ox + dist * dx
            py = oy + dist * dy
            c, r = grid_map.world_to_grid(px, py)
            if not grid_map.is_inside(c, r):
                return dist
            if grid_map.costmap[r, c] < 254:
                # Entered the drivable corridor gap!
                break
            dist += step

        # 2. Advance through free corridor until hitting true physical wall or arena boundary
        while dist <= max_dist:
            px = ox + dist * dx
            py = oy + dist * dy
            c, r = grid_map.world_to_grid(px, py)
            if not grid_map.is_inside(c, r) or grid_map.costmap[r, c] >= 254:
                return dist
            dist += step
        return max_dist

    def generate_detour_splice(
        self,
        robot_x: float,
        robot_y: float,
        robot_theta: float,
        blocked_pt: Tuple[float, float],
        obstacle_cluster: Optional[Any] = None,
        grid_map: Optional[Any] = None,
        lateral_offset: Optional[float] = None
    ) -> Optional[List[Tuple[float, float]]]:
        """
        General Dynamic Drivable Corridor & Frenet-Splice Avoidance Planner (FCP-Lite).
        Academic Standard (2024-2026 IEEE RAS / IV):
          1. Continuous spatial corridor clearance raycasting (left & right free space).
          2. Adaptive lateral offset: centers the robot in the available gap, preventing wall hits.
          3. General obstacle geometry handling (small items, medium boxes, large barriers, oncoming persons).
          4. Full blockage protection: gracefully returns None if corridor < 0.28m passability width.
          5. Smooth 3-segment cosine trapezoidal plateau trajectory with Costmap safety verification.
          6. Global Goal & Reference Path Persistence: guarantee the real goal is never lost or truncated.
        """
        if not self.current_path or len(self.current_path) < 2:
            return None

        bx, by = blocked_pt
        dx_fwd = bx - robot_x
        dy_fwd = by - robot_y
        dist_fwd = math.hypot(dx_fwd, dy_fwd)

        if dist_fwd < 1e-3:
            u_fwd = (math.cos(robot_theta), math.sin(robot_theta))
        else:
            u_fwd = (dx_fwd / dist_fwd, dy_fwd / dist_fwd)

        n_left = (-u_fwd[1], u_fwd[0])
        n_right = (u_fwd[1], -u_fwd[0])

        # 1. Obstacle geometry estimation in Frenet frame
        d_obs_left = 0.15
        d_obs_right = -0.15
        obs_length = 0.25
        is_oncoming = False

        if obstacle_cluster is not None:
            is_oncoming = getattr(obstacle_cluster, "is_dynamic", False) and getattr(obstacle_cluster, "closing_speed", 0.0) > 0.05
            cx = getattr(obstacle_cluster, "centroid", (bx, by))[0]
            cy = getattr(obstacle_cluster, "centroid", (bx, by))[1]
            length_m = getattr(obstacle_cluster, "length_m", 0.25)
            width_m = getattr(obstacle_cluster, "width_m", 0.25)
            obs_length = max(0.10, length_m)
            half_w = max(0.04, width_m / 2.0)

            cos_t = math.cos(robot_theta)
            sin_t = math.sin(robot_theta)
            # Support both World and Robot coordinates for cluster centroid
            d_world = math.hypot(cx - bx, cy - by)
            cx_w_trans = robot_x + cx * cos_t - cy * sin_t
            cy_w_trans = robot_y + cx * sin_t + cy * cos_t
            d_robot = math.hypot(cx_w_trans - bx, cy_w_trans - by)
            if d_robot < d_world:
                cx_w, cy_w = cx_w_trans, cy_w_trans
            else:
                cx_w, cy_w = cx, cy

            lat_centroid = (cx_w - bx) * n_left[0] + (cy_w - by) * n_left[1]
            d_obs_left = lat_centroid + half_w
            d_obs_right = lat_centroid - half_w
        elif grid_map is not None:
            # Measure obstacle boundary directly from costmap if no cluster is provided
            d_l = 0.0
            while d_l < 1.0:
                d_l += 0.02
                px = bx + d_l * n_left[0]
                py = by + d_l * n_left[1]
                c, r = grid_map.world_to_grid(px, py)
                if not grid_map.is_inside(c, r) or grid_map.costmap[r, c] < 252:
                    break
            d_obs_left = max(0.08, d_l)

            d_r = 0.0
            while d_r < 1.0:
                d_r += 0.02
                px = bx + d_r * n_right[0]
                py = by + d_r * n_right[1]
                c, r = grid_map.world_to_grid(px, py)
                if not grid_map.is_inside(c, r) or grid_map.costmap[r, c] < 252:
                    break
            d_obs_right = -max(0.08, d_r)

        # 2. Continuous spatial corridor clearance raycasting
        wall_clearance_left = 1.5
        wall_clearance_right = 1.5
        if grid_map is not None:
            wall_clearance_left = self.measure_wall_distance(
                grid_map, (bx, by), n_left, start_offset=max(0.0, d_obs_left) + 0.02
            )
            wall_clearance_right = self.measure_wall_distance(
                grid_map, (bx, by), n_right, start_offset=max(0.0, -d_obs_right) + 0.02
            )

        # 3. Drivable gaps calculation in C-space
        # Note: In C-space, costmap >= 252 already embeds inscribed robot radius (0.09m)
        # on both the obstacle and the wall. A C-space gap >= 0.10m guarantees a true physical
        # corridor >= 0.28m, which easily accommodates JetBot (physical width 0.12m).
        gap_left = max(0.0, wall_clearance_left - max(0.0, d_obs_left))
        gap_right = max(0.0, wall_clearance_right - max(0.0, -d_obs_right))

        MIN_REQUIRED_PASS_GAP = 0.04  # C-space gap = 0.04m (corresponds to physical corridor > 20cm)
        d_min_safe_l = max(0.0, d_obs_left) + 0.015
        d_max_safe_l = wall_clearance_left - 0.015
        left_viable = (gap_left >= MIN_REQUIRED_PASS_GAP) and (d_max_safe_l - d_min_safe_l >= 0.02)

        d_min_safe_r = max(0.0, -d_obs_right) + 0.015
        d_max_safe_r = wall_clearance_right - 0.015
        right_viable = (gap_right >= MIN_REQUIRED_PASS_GAP) and (d_max_safe_r - d_min_safe_r >= 0.02)

        # Arena 2x1m Wall Safety Guardrail: Relaxed boundary limit (0.10m instead of 0.16m to permit detours near wall)
        if self.env_mode == "ARENA_2X1":
            if wall_clearance_left < 0.10:
                left_viable = False
            if wall_clearance_right < 0.10:
                right_viable = False

        # 4. Impassable Corridor Protection (Full Blockage)
        if grid_map is not None and not left_viable and not right_viable:
            # Neither side offers sufficient gap: abort detour safely to avoid wall crash
            return None

        # 5. Side selection logic (Tangent Bug Shortest-Path Heuristic + Fallback)
        candidate_sides = []
        if is_oncoming:
            # COLREGS maritime/road standard: prefer right deflection if viable
            if right_viable:
                candidate_sides.append(("RIGHT", n_right, d_min_safe_r, d_max_safe_r))
            if left_viable:
                candidate_sides.append(("LEFT", n_left, d_min_safe_l, d_max_safe_l))
        else:
            if left_viable and right_viable:
                # Both viable: Tangent Bug Shortest Path Heuristic (Kamon et al., 1998)
                # Select the tangent side that minimizes: J(T) = dist(Robot, T) + dist(T, Goal)
                if self.global_goal is not None:
                    gx, gy = self.global_goal
                    t_left = (bx + d_obs_left * n_left[0], by + d_obs_left * n_left[1])
                    t_right = (bx + (-d_obs_right) * n_right[0], by + (-d_obs_right) * n_right[1])
                    cost_l = math.hypot(t_left[0] - robot_x, t_left[1] - robot_y) + math.hypot(t_left[0] - gx, t_left[1] - gy)
                    cost_r = math.hypot(t_right[0] - robot_x, t_right[1] - robot_y) + math.hypot(t_right[0] - gx, t_right[1] - gy)
                    if cost_l <= cost_r:
                        candidate_sides = [("LEFT", n_left, d_min_safe_l, d_max_safe_l), ("RIGHT", n_right, d_min_safe_r, d_max_safe_r)]
                    else:
                        candidate_sides = [("RIGHT", n_right, d_min_safe_r, d_max_safe_r), ("LEFT", n_left, d_min_safe_l, d_max_safe_l)]
                else:
                    if gap_left >= gap_right:
                        candidate_sides = [("LEFT", n_left, d_min_safe_l, d_max_safe_l), ("RIGHT", n_right, d_min_safe_r, d_max_safe_r)]
                    else:
                        candidate_sides = [("RIGHT", n_right, d_min_safe_r, d_max_safe_r), ("LEFT", n_left, d_min_safe_l, d_max_safe_l)]
            elif left_viable:
                candidate_sides = [("LEFT", n_left, d_min_safe_l, d_max_safe_l)]
            elif right_viable:
                candidate_sides = [("RIGHT", n_right, d_min_safe_r, d_max_safe_r)]
            else:
                # Open space fallback (grid_map is None)
                candidate_sides = [("LEFT", n_left, 0.17, 0.28)]

        # 6. Adaptive Corridor Centering & C1 Profile Generation (Disparity Extender)
        # Evaluate candidate side(s) in priority order; fallback to alternate side if lethal cells hit
        chosen_detour_pts = None
        for side_name, chosen_lat_dir, d_min_s, d_max_s in candidate_sides:
            if grid_map is not None:
                # 6.1 Distance-Transform Voronoi Ridge Centering:
                # If Euclidean Distance Transform is available, pick the offset that maximizes clearance
                if getattr(grid_map, "dist_m", None) is not None and d_max_s > d_min_s:
                    best_clearance = -1.0
                    best_d = 0.5 * (d_min_s + d_max_s)
                    n_samples = 11
                    for step_i in range(n_samples):
                        d_cand = d_min_s + (d_max_s - d_min_s) * (step_i / float(n_samples - 1))
                        pt_cx = bx + d_cand * chosen_lat_dir[0]
                        pt_cy = by + d_cand * chosen_lat_dir[1]
                        c_chk, r_chk = grid_map.world_to_grid(pt_cx, pt_cy)
                        if grid_map.is_inside(c_chk, r_chk):
                            cl_val = float(grid_map.dist_m[r_chk, c_chk])
                            if cl_val > best_clearance:
                                best_clearance = cl_val
                                best_d = d_cand
                    d_center = best_d
                else:
                    d_center = 0.5 * (d_min_s + d_max_s)

                # Nominal compact detour offset: 0.18m - 0.22m (Disparity Extender).
                # Center the robot if corridor is tight, or use compact offset if corridor is wide.
                d_offset = max(d_min_s, min(d_max_s, min(0.22, d_center)))
                if d_offset < d_min_s or d_offset > d_max_s:
                    continue
            else:
                d_offset = lateral_offset if lateral_offset is not None else self.lateral_detour_offset

            # 7. Obstacle-Anchored Longitudinal Profile (Disparity Extender Local Detour)
            half_obs = obs_length / 2.0
            if obstacle_cluster is not None and "cx_w" in locals():
                dist_center = (cx_w - robot_x) * u_fwd[0] + (cy_w - robot_y) * u_fwd[1]
                dist_center = max(dist_fwd + half_obs, dist_center)
            else:
                dist_center = dist_fwd + half_obs

            s_ramp = max(0.14, min(0.22, d_offset * 1.0))
            s_1 = max(0.06, dist_center - half_obs - 0.04)
            s_2 = dist_center + half_obs + 0.06
            s_3 = s_2 + s_ramp

            # Clamp s_3 to prevent overshooting past global goal into arena walls
            if self.global_goal is not None:
                d_to_goal = math.hypot(self.global_goal[0] - robot_x, self.global_goal[1] - robot_y)
                if d_to_goal < s_3:
                    s_3 = max(s_2 + 0.04, d_to_goal)

            # Multi-scale lateral clearance search (nominal, compact, ultra-compact)
            cand_offsets = [d_offset]
            if d_offset * 0.80 >= d_min_s:
                cand_offsets.append(d_offset * 0.80)
            if d_offset * 0.65 >= d_min_s:
                cand_offsets.append(d_offset * 0.65)

            for cur_offset in cand_offsets:
                candidate_pts = []
                collision = False
                n_steps = 12
                for s_idx in range(1, n_steps + 1):
                    s_val = (s_idx / float(n_steps)) * s_3
                    if s_val <= s_1:
                        u_ramp = s_val / max(0.01, s_1)
                        lat_disp = cur_offset * 0.5 * (1.0 - math.cos(math.pi * u_ramp))
                    elif s_val <= s_2:
                        lat_disp = cur_offset
                    else:
                        u_down = (s_val - s_2) / max(0.01, s_3 - s_2)
                        lat_disp = cur_offset * 0.5 * (1.0 + math.cos(math.pi * u_down))

                    px = robot_x + s_val * u_fwd[0] + lat_disp * chosen_lat_dir[0]
                    py = robot_y + s_val * u_fwd[1] + lat_disp * chosen_lat_dir[1]

                    if grid_map is not None:
                        c, r = grid_map.world_to_grid(px, py)
                        if not grid_map.is_inside(c, r) or grid_map.costmap[r, c] >= 254:
                            collision = True
                            break

                    candidate_pts.append((round(px, 3), round(py, 3)))

                if not collision:
                    chosen_detour_pts = candidate_pts
                    break

            if chosen_detour_pts:
                break

        if not chosen_detour_pts:
            return None

        detour_pts = chosen_detour_pts
        rejoin_pt = detour_pts[-1]

        # Splicing into global reference path to prevent goal loss
        target_ref = self.global_reference_path if self.global_reference_path else self.current_path
        splice_idx = -1
        for i in range(len(target_ref)):
            pt = target_ref[i]
            if (pt[0] - rejoin_pt[0]) * u_fwd[0] + (pt[1] - rejoin_pt[1]) * u_fwd[1] > 0.10:
                splice_idx = i
                break

        if splice_idx != -1 and splice_idx < len(target_ref):
            spliced_path = detour_pts + target_ref[splice_idx:]
        else:
            if self.global_goal is not None:
                spliced_path = detour_pts + [self.global_goal]
            else:
                spliced_path = detour_pts

        if self.global_goal is not None and spliced_path[-1] != self.global_goal:
            spliced_path.append(self.global_goal)

        return spliced_path

    def modulate_velocity(
        self,
        base_v: float,
        clusters: Optional[List[Any]] = None,
        lidar_ranges: Optional[List[float]] = None,
        kappa: float = 0.0,
        omega: float = 0.0,
        dt: float = 0.1
    ) -> float:
        """
        Regulated Pure Pursuit (Steve Macenski 2023) Curvature Slowdown
        & S-Curve Soft Deceleration Cushion (Pudu/Keenon industrial standard):
          1. Curvature scaling per RPP paper: v = base_v / (1 + k * |kappa|)
          2. Two-Stage Soft Deceleration Cushion:
             - Zone 1 (Warning Cushion, d_stop < d <= d_warn):
               S-curve deceleration scaling S(u) = 0.5 * (1 - cos(pi * u)), where u in [0, 1].
             - Zone 2 (Safe Stop Margin, d <= d_stop):
               Smoothly drops speed to 0 m/s with zero overshoot.
          3. S-curve rate limiting: decel <= 0.35 m/s^2 on glossy ceramic tile to prevent wheel slip.
        """
        # Steve Macenski (Autonomous Robots 2023): Curvature-based velocity regulation
        curv_val = abs(kappa) if abs(kappa) > 1e-4 else abs(omega)
        scaled_v = base_v / (1.0 + self.curvature_scale_k * curv_val)
        target_v = max(self.min_drive_speed, scaled_v)

        min_frontal_dist = 99.0
        min_ttc = 99.0
        has_oncoming = False

        # 1. Direct frontal corridor LiDAR inspection (-25 deg to +25 deg)
        if lidar_ranges and len(lidar_ranges) >= 360:
            frontal_indices = list(range(0, 26)) + list(range(335, 360))
            valid_f = [lidar_ranges[i] for i in frontal_indices if self.min_lidar_range <= lidar_ranges[i] < 3.0]
            if valid_f:
                min_frontal_dist = min(valid_f)

        # 2. Dynamic cluster inspection & oncoming closing speeds
        if clusters:
            for cl in clusters:
                cx, cy = cl.centroid
                is_dyn = getattr(cl, "is_dynamic", False)
                closing_spd = getattr(cl, "closing_speed", 0.0)

                # Focus on forward path corridor |cy| <= 0.22m
                if cx > 0.04 and abs(cy) <= 0.22:
                    if is_dyn and closing_spd > 0.05:
                        has_oncoming = True
                        ttc = getattr(cl, "ttc", 99.0)
                        if ttc < min_ttc:
                            min_ttc = ttc
                    else:
                        if cl.min_distance < min_frontal_dist:
                            min_frontal_dist = cl.min_distance

        # Oncoming dynamic obstacle protection (TTC braking)
        if has_oncoming and min_ttc < 3.0:
            scale_ttc = max(0.35, min(1.0, (min_ttc - 0.8) / 1.5))
            target_v = target_v * scale_ttc

        # Phase 4.3: the old `max(0.16, reflex_dist_threshold)` was a SECOND, smaller stop
        # distance that silently contradicted the configured reflex threshold. The reflex
        # threshold now comes from Config.REFLEX_STOP (0.30m = the D500's accuracy floor),
        # so it IS the floor; there is no independent literal left.
        d_stop = self.reflex_dist_threshold
        d_warn = max(0.45, self.warning_dist_threshold)

        if min_frontal_dist <= d_stop:
            desired_v = 0.0
        elif min_frontal_dist <= d_warn:
            u = (min_frontal_dist - d_stop) / max(0.01, d_warn - d_stop)
            u = max(0.0, min(1.0, u))
            s_curve = 0.5 * (1.0 - math.cos(math.pi * u))
            # Decelerate smoothly down to MIN_DRIVE_SPEED (0.12) to keep traction and prevent motor stall
            v_floor = self.min_drive_speed
            desired_v = v_floor + max(0.0, target_v - v_floor) * s_curve
        else:
            desired_v = target_v

        # S-curve Deceleration Rate Limiting (0.35 m/s^2 max decel to prevent slip on tile)
        if desired_v < self.last_linear_v:
            max_drop = 0.35 * max(0.02, dt)
            v_out = max(desired_v, self.last_linear_v - max_drop)
            # When stopping intentionally, once speed drops below min_drive_speed, cleanly complete stop
            if desired_v <= 0.02 and v_out < self.min_drive_speed:
                v_out = 0.0
        else:
            ramp_time = max(0.1, self.soft_start_ramp_sec)
            max_rise = (self.max_linear_speed / ramp_time) * max(0.02, dt)
            if self.last_linear_v < 0.01 and desired_v >= self.min_drive_speed:
                v_out = self.min_drive_speed
            else:
                v_out = min(desired_v, self.last_linear_v + max_rise)

        return v_out

    def compute_command(
        self,
        robot_x: float,
        robot_y: float,
        robot_theta: float,
        lidar_ranges: Optional[List[float]] = None,
        clusters: Optional[List[Any]] = None,
        grid_map: Optional[Any] = None,
        auto_evade: bool = True,
        planner: Optional[Any] = None,
        goal: Optional[Tuple[float, float]] = None
    ) -> Tuple[float, float, Dict[str, Any]]:
        """
        Main 10Hz Control Loop: Pure Pursuit + Curvature Scaling + Reflex + Pudu/Keenon FSM Detour.
        Returns: (v, w, telemetry_info)
        """
        now_t = time.time()
        dt_ctrl = max(0.02, min(0.2, now_t - getattr(self, "_last_ctrl_time", now_t - 0.1)))
        self._last_ctrl_time = now_t

        # 1. Idle check
        if not self.current_path:
            self.last_linear_v = 0.0
            return 0.0, 0.0, {"status": "IDLE", "distance_remaining": 0.0, "target": None}

        # Check distance to TRUE GLOBAL GOAL
        if self.global_goal is not None:
            dist_to_final_goal = math.hypot(self.global_goal[0] - robot_x, self.global_goal[1] - robot_y)
        else:
            goal_x, goal_y = self.current_path[-1]
            dist_to_final_goal = math.hypot(goal_x - robot_x, goal_y - robot_y)

        # Frontal minimum obstacle distance from hardware LiDAR (forward cone ±30 deg)
        front_min = 99.0
        if lidar_ranges and len(lidar_ranges) >= 360:
            frontal_indices = list(range(0, 31)) + list(range(330, 360))
            valid_f = [lidar_ranges[i] for i in frontal_indices if self.min_lidar_range <= lidar_ranges[i] < 6.0]
            if valid_f:
                front_min = min(valid_f)

        # Baseline Pure Pursuit & Tangent Bug Lookahead Target Search ("Chấm xanh đi trước")
        best_seg, best_proj, _ = self._project_to_path(robot_x, robot_y)
        self.path_index = best_seg

        # Adaptive Lookahead Distance (Regulated Pure Pursuit - Steve Macenski 2023):
        # Dynamically shortens lookahead near obstacle edges to strictly eliminate corner cutting!
        obs_clearance = 1.0
        if grid_map is not None and getattr(grid_map, "dist_m", None) is not None:
            c_r, r_r = grid_map.world_to_grid(robot_x, robot_y)
            if grid_map.is_inside(c_r, r_r):
                obs_clearance = float(grid_map.dist_m[r_r, c_r])
        elif lidar_ranges:
            valid_r = [r for r in lidar_ranges if self.min_lidar_range <= r < 1.5]
            if valid_r:
                obs_clearance = min(valid_r)

        effective_lookahead = max(0.14, min(self.lookahead_dist, obs_clearance * 0.95))
        rem_L = effective_lookahead
        curr_pt = best_proj
        target_pt = self.current_path[-1]

        for j in range(best_seg, len(self.current_path) - 1):
            next_pt = self.current_path[j + 1]
            seg_dx = next_pt[0] - curr_pt[0]
            seg_dy = next_pt[1] - curr_pt[1]
            seg_len = math.hypot(seg_dx, seg_dy)

            if seg_len >= rem_L:
                target_pt = (
                    curr_pt[0] + (seg_dx / seg_len) * rem_L,
                    curr_pt[1] + (seg_dy / seg_len) * rem_L
                )
                break
            else:
                rem_L -= seg_len
                curr_pt = next_pt

        # 2. 10Hz Hardware Reflex Emergency Brake (Virtual Bumper)
        if lidar_ranges and self.check_safety_reflex(lidar_ranges, dist_to_goal=dist_to_final_goal):
            self.last_linear_v = 0.0
            dx_t = target_pt[0] - robot_x
            dy_t = target_pt[1] - robot_y
            target_angle_ref = math.atan2(dy_t, dx_t)
            alpha_ref = (target_angle_ref - robot_theta + math.pi) % (2.0 * math.pi) - math.pi
            if abs(alpha_ref) > 0.08:
                w_pivot = min(0.35, getattr(self, "pivot_angular_speed", 0.35))
                w_spin = math.copysign(w_pivot, alpha_ref)
                return 0.0, round(w_spin, 3), {
                    "status": "REFLEX_PIVOT_EVADE",
                    "reason": f"Frontal obstacle < {self.reflex_dist_threshold}m! In-place pivot turn ({math.degrees(alpha_ref):.1f}°) to evade obstacle.",
                    "distance_remaining": round(dist_to_final_goal, 3),
                    "target": (round(target_pt[0], 3), round(target_pt[1], 3)),
                    "evasion_active": True,
                    "evasion_reason": "REFLEX_PIVOT_EVADE"
                }
            # Pivot Clearance Gating: when facing frontal obstacle head-on, allow pivot turn towards clearest side
            # instead of freezing in place with (v=0, w=0)!
            left_clearance = min([lidar_ranges[i] for i in range(20, 70) if lidar_ranges[i] > 0.05] or [0.0])
            right_clearance = min([lidar_ranges[i] for i in range(290, 340) if lidar_ranges[i] > 0.05] or [0.0])
            escape_dir = 1.0 if left_clearance >= right_clearance else -1.0
            w_pivot = min(0.35, getattr(self, "pivot_angular_speed", 0.35))
            return 0.0, round(escape_dir * w_pivot, 3), {
                "status": "REFLEX_PIVOT_EVADE",
                "reason": f"Frontal obstacle head-on < {self.reflex_dist_threshold}m! Pivot turn towards clear side to escape deadlock.",
                "distance_remaining": round(dist_to_final_goal, 3),
                "target": (round(target_pt[0], 3), round(target_pt[1], 3)),
                "evasion_active": True,
                "evasion_reason": "REFLEX_PIVOT_EVADE"
            }

        # 3. Goal Reached check (only triggers when near the real global goal!)
        if dist_to_final_goal <= self.goal_tolerance:
            self.is_goal_reached = True
            self.clear_path()
            self.last_linear_v = 0.0
            return 0.0, 0.0, {
                "status": "GOAL_REACHED",
                "distance_remaining": 0.0,
                "target": (round(target_pt[0], 3), round(target_pt[1], 3))
            }

        # 4. Proactive Horizon Collision Inspector & Fast A* Replan
        evasion_active = self.is_evading
        evasion_reason = ""
        is_path_blocked = False

        if auto_evade and self.current_path:
            # Proactive Horizon check: 0.25m to 0.70m ahead on active path
            eff_horizon = max(0.25, min(0.70, dist_to_final_goal - 0.04))
            is_blk, blk_pt, reason = self.check_path_collision(
                robot_x, robot_y, robot_theta,
                grid_map=grid_map,
                clusters=clusters,
                horizon_m=eff_horizon,
                dist_to_final_goal=dist_to_final_goal,
                front_min=front_min
            )

            if is_blk and blk_pt is not None:
                self._blocked_persistence_count += 1
                if self._blocked_persistence_count < 3:
                    is_blk = False
                else:
                    self._blocked_persistence_count = 0
            else:
                self._blocked_persistence_count = 0

            if is_blk and blk_pt is not None:
                # Terminal Goal Bypass: If robot is within 0.40m of goal and front is clear > 0.28m, creep to goal
                if dist_to_final_goal <= 0.40 and front_min > 0.28:
                    pass
                else:
                    self.is_path_blocked = True
                    is_path_blocked = True
                    dist_to_blk = math.hypot(blk_pt[0] - robot_x, blk_pt[1] - robot_y)

                    # Trigger inline A* Replan if planner and goal are available
                    active_planner = planner or getattr(self, "planner", None)
                    target_goal = goal or self.global_goal or (self.current_path[-1] if self.current_path else None)

                    replan_success = False
                    replan_cooldown = now_t - getattr(self, "_last_internal_replan_time", 0.0)

                    if active_planner is not None and target_goal is not None and replan_cooldown >= 0.35:
                        if grid_map is not None:
                            # Phase 3.2/3.3: the SAME mutation order as the telemetry path
                            # (clear the real-radius footprint, then insert the cluster set).
                            # The old hardcoded 0.15m radius evicted near clusters, and the
                            # `if clusters:` gate meant an empty set never cleared anything.
                            grid_map.clear_footprint(
                                robot_x,
                                robot_y,
                                radius=getattr(grid_map, "robot_radius", 0.09),
                                evict_clusters=False,
                            )
                            grid_map.insert_obstacle_clusters(
                                clusters, robot_x, robot_y, robot_theta
                            )
                            grid_map.update_costmap()
                        try:
                            # This inline replan runs inside the 10Hz control loop, so it
                            # must be tightly bounded (Phase 2.4 / BUG-10).
                            new_path = active_planner.plan(
                                (robot_x, robot_y),
                                target_goal,
                                smooth=True,
                                max_time_sec=0.35,
                                max_expansions=15000,
                            )
                        except Exception:
                            new_path = None
                        self._last_internal_replan_time = now_t
                        if new_path and len(new_path) >= 2:
                            self.set_path(new_path)
                            self.is_path_blocked = False
                            is_path_blocked = False
                            self.is_evading = True
                            evasion_active = True
                            evasion_reason = "PROACTIVE_ASTAR_REPLAN"
                            replan_success = True

                    if not replan_success and is_path_blocked:
                        # 1. First attempt: Frenet-Splice Corridor Detour
                        detour_pts = self.generate_detour_splice(
                            robot_x, robot_y, robot_theta,
                            blocked_pt=blk_pt,
                            obstacle_cluster=None,
                            grid_map=grid_map
                        )
                        if detour_pts and len(detour_pts) >= 2:
                            self.set_path(detour_pts)
                            self.is_path_blocked = False
                            is_path_blocked = False
                            self.is_evading = True
                            evasion_active = True
                            evasion_reason = "FRENET_SPLICE_DETOUR"
                            replan_success = True

                    if not replan_success and is_path_blocked:
                        # 2. Second attempt: Active Disparity Extender (DDE) Polar Sensor-Space Reactive Evasion
                        if self.disparity_extender is not None and lidar_ranges and len(lidar_ranges) >= 180:
                            dx_tgt = target_pt[0] - robot_x
                            dy_tgt = target_pt[1] - robot_y
                            tgt_angle = math.atan2(dy_tgt, dx_tgt)
                            tgt_alpha = (tgt_angle - robot_theta + math.pi) % (2.0 * math.pi) - math.pi
                            dde_eval = self.disparity_extender.evaluate(lidar_ranges, goal_heading_rad=tgt_alpha)

                            dde_status = dde_eval.get("status", "CLEAR")
                            if dde_status in ("EVASION_ACTIVE", "EVASION_FALLBACK"):
                                steer_rad = dde_eval.get("target_heading_rad", 0.0)
                                v_evade = max(self.min_drive_speed, min(0.09, self.max_linear_speed * 0.5))
                                L_evade = max(0.12, min(0.35, effective_lookahead))
                                kappa_evade = (2.0 * math.sin(steer_rad)) / L_evade
                                w_evade = max(-self.max_angular_speed, min(self.max_angular_speed, v_evade * kappa_evade))

                                self.is_evading = True
                                self.last_linear_v = v_evade
                                return v_evade, round(w_evade, 3), {
                                    "status": "EVASION_ACTIVE",
                                    "reason": f"DDE Evasion: steer {math.degrees(steer_rad):.1f}° (front={dde_eval.get('front_obstacle_dist', 0):.2f}m)",
                                    "distance_remaining": round(dist_to_final_goal, 3),
                                    "target": (round(target_pt[0], 3), round(target_pt[1], 3)),
                                    "is_blocked": False,
                                    "evasion_active": True,
                                    "evasion_reason": "DDE_ACTIVE_GAP"
                                }

                        # 3. Third fallback: Boxed in / No gap available -> Hold position safely at >= 0.25m
                        if dist_to_blk <= 0.25 or (front_min is not None and front_min <= 0.25):
                            self.last_linear_v = 0.0
                            return 0.0, 0.0, {
                                "status": "BLOCKED_WAIT",
                                "reason": f"Corridor blocked at {blk_pt} (dist={dist_to_blk:.2f}m <= 0.25m). Holding safe clearance >= 0.25m.",
                                "distance_remaining": round(dist_to_final_goal, 3),
                                "target": (round(target_pt[0], 3), round(target_pt[1], 3)),
                                "is_blocked": True,
                                "evasion_active": False,
                                "evasion_reason": reason
                            }
            else:
                self.is_path_blocked = False

        # 5. Pure Pursuit Lookahead Target Search
        best_seg, best_proj, _ = self._project_to_path(robot_x, robot_y)
        self.path_index = best_seg

        if self.is_evading and self.path_index >= self.evasion_end_index:
            self.is_evading = False
            self.obstacle_state = "CLEAR"
            evasion_active = False

        rem_L = effective_lookahead
        curr_pt = best_proj
        target_pt = self.current_path[-1]

        for j in range(best_seg, len(self.current_path) - 1):
            next_pt = self.current_path[j + 1]
            seg_dx = next_pt[0] - curr_pt[0]
            seg_dy = next_pt[1] - curr_pt[1]
            seg_len = math.hypot(seg_dx, seg_dy)

            if seg_len >= rem_L:
                target_pt = (
                    curr_pt[0] + (seg_dx / seg_len) * rem_L,
                    curr_pt[1] + (seg_dy / seg_len) * rem_L
                )
                break
            else:
                rem_L -= seg_len
                curr_pt = next_pt

        # 6. Steering Curvature Computation
        dx = target_pt[0] - robot_x
        dy = target_pt[1] - robot_y
        dist_to_target = math.hypot(dx, dy)
        target_angle = math.atan2(dy, dx)
        alpha = (target_angle - robot_theta + math.pi) % (2.0 * math.pi) - math.pi

        # RPP Chord Length Adaptation
        L_chord = max(0.08, min(effective_lookahead, dist_to_target))
        kappa = (2.0 * math.sin(alpha)) / L_chord

        # Behind-the-Goal trap protection
        if dist_to_final_goal <= (self.goal_tolerance * 1.25) and abs(alpha) > (math.pi / 2.0):
            self.is_goal_reached = True
            self.clear_path()
            self.last_linear_v = 0.0
            return 0.0, 0.0, {"status": "GOAL_REACHED", "distance_remaining": 0.0, "target": (round(target_pt[0], 3), round(target_pt[1], 3))}

        # 7. Goal Approach Deceleration Ramp
        ramp_dist = max(0.35, self.goal_tolerance + 0.15)
        if dist_to_final_goal <= ramp_dist:
            ramp = max(0.0, min(1.0, (dist_to_final_goal - self.goal_tolerance) / max(0.01, ramp_dist - self.goal_tolerance)))
            base_v = self.min_drive_speed + (self.max_linear_speed - self.min_drive_speed) * ramp
        else:
            base_v = self.max_linear_speed

        # 8. Regulated Pure Pursuit (Steve Macenski 2023): Pivot Clearance Gating & Curvature Slowdown
        abs_alpha = abs(alpha)
        pivot_entry_thresh = math.radians(60.0)  # Continuous curvature tracking for diagonal paths <= 60 deg
        align_exit_thresh = math.radians(30.0)   # Resume forward tracking when |alpha| <= 30 deg

        in_pivot = getattr(self, "_in_pivot_turn", False)
        if getattr(self, "tracker", None) is not None:
            in_pivot = self.tracker.update_pivot_state(alpha, in_pivot)
            self._in_pivot_turn = in_pivot
        else:
            if in_pivot:
                if abs_alpha <= align_exit_thresh:
                    self._in_pivot_turn = False
                    in_pivot = False
            else:
                if abs_alpha > pivot_entry_thresh:
                    self._in_pivot_turn = True
                    in_pivot = True

        if in_pivot:
            # Pivot turn: Decouple w from v, eliminating Actuation Deadlock v=0 => w=0
            w_pivot_min = 0.15
            w_pivot_max = min(0.35, getattr(self, "pivot_angular_speed", 0.35))
            w_calc = max(w_pivot_min, min(w_pivot_max, 1.2 * abs_alpha))
            v = 0.0
            w = math.copysign(w_calc, alpha)
            ctrl_status = "PIVOT_ALIGNING"
        else:
            # Curvature-regulated velocity scaling: v = v_cruise * cos^2(alpha / 2)
            cos_half_alpha = math.cos(alpha / 2.0)
            v_curv = max(0.04, base_v * (cos_half_alpha * cos_half_alpha))

            # Pass through velocity modulator for S-curve deceleration and acceleration limiting
            v = self.modulate_velocity(
                v_curv,
                clusters=clusters,
                lidar_ranges=lidar_ranges,
                kappa=kappa,
                omega=v_curv * kappa,
                dt=dt_ctrl
            )
            if v > 0.01:
                v = max(self.min_drive_speed, min(self.max_linear_speed, v))
                w = max(-self.max_angular_speed, min(self.max_angular_speed, v * kappa))
                ctrl_status = "NAVIGATING" if not evasion_active else "EVADING"
            else:
                v = 0.0
                if abs_alpha > math.radians(3.0):
                    w_pivot_min = 0.12
                    w_pivot_max = min(0.35, getattr(self, "pivot_angular_speed", 0.35))
                    w_calc = max(w_pivot_min, min(w_pivot_max, 1.2 * abs_alpha))
                    w = math.copysign(w_calc, alpha)
                    ctrl_status = "PIVOT_ALIGNING"
                else:
                    w = 0.0
                    ctrl_status = "NAVIGATING"

        self.last_linear_v = v
        if v == 0.0 and abs(w) < 0.01:
            if dist_to_final_goal <= self.goal_tolerance:
                ctrl_status = "GOAL_REACHED"
                self.is_path_blocked = False
            elif is_path_blocked:
                ctrl_status = "BLOCKED_WAIT"
            else:
                ctrl_status = "NAVIGATING"

        out_info = {
            "status": ctrl_status,
            "distance_remaining": round(dist_to_final_goal, 3),
            "target": (round(target_pt[0], 3), round(target_pt[1], 3)),
            "is_blocked": self.is_path_blocked,
            "evasion_active": evasion_active,
            "evasion_reason": evasion_reason
        }

        return round(v, 3), round(w, 3), out_info

    def differential_drive_ik(self, v: float, w: float) -> Tuple[float, float]:
        """Inverse Kinematics for Differential Drive robot."""
        if getattr(self, "motor_dynamics", None) is not None:
            return self.motor_dynamics.differential_drive_ik(v, w)
        v_left = v - (w * self.track_width / 2.0)
        v_right = v + (w * self.track_width / 2.0)
        return round(v_left, 3), round(v_right, 3)

    def load_dynamics_config(self, config_path: Optional[str] = None):
        """Loads physical motor dynamics, trim factor and deadband from JSON config."""
        if config_path is None:
            search_paths = [
                os.path.join(os.path.dirname(__file__), "..", "..", "..", "..", "setup", "robot_dynamics.json"),
                os.path.join(os.path.dirname(__file__), "..", "..", "setup", "robot_dynamics.json"),
                os.path.join(os.path.dirname(__file__), "..", "setup", "robot_dynamics.json"),
                os.path.join(os.path.dirname(__file__), "robot_dynamics.json"),
                os.path.join(os.path.dirname(__file__), "..", "..", "03-MinhHieu", "test-hardware", "robot_dynamics.json"),
                os.path.join(os.path.dirname(__file__), "..", "test-hardware", "robot_dynamics.json"),
                os.path.expanduser("~/setup/robot_dynamics.json"),
                os.path.expanduser("~/03-MinhHieu/test-hardware/robot_dynamics.json"),
                os.path.expanduser("~/CS532-main/setup/robot_dynamics.json"),
            ]
            for p in search_paths:
                if os.path.isfile(p):
                    config_path = p
                    break
        if config_path and os.path.isfile(config_path):
            try:
                with open(config_path, "r", encoding="utf-8") as f:
                    cfg = json.load(f)
                self.min_drive_speed = float(cfg.get("min_drive_speed", self.min_drive_speed))
                self.trim_factor = float(cfg.get("trim_factor", self.trim_factor))
                self.soft_start_ramp_sec = float(cfg.get("soft_start_ramp_sec", self.soft_start_ramp_sec))
                if "actual_speed_mps" in cfg:
                    self.max_linear_speed = float(cfg["actual_speed_mps"])
                elif "max_safe_speed_mps" in cfg:
                    self.max_linear_speed = float(cfg["max_safe_speed_mps"])
                self.pivot_angular_speed = 3.1416  # 90 deg in 0.50s
                self.max_angular_speed = 3.1416
                if "turn_speed" in cfg:
                    self.turn_speed = float(cfg["turn_speed"])
                if "environment_mode" in cfg:
                    self.env_mode = str(cfg["environment_mode"])
                if "tactical_yield_dist" in cfg:
                    self.tactical_yield_dist = float(cfg["tactical_yield_dist"])
                if "passage_clearance_min" in cfg:
                    self.passage_clearance_min = float(cfg["passage_clearance_min"])
            except Exception:
                pass

    def compute_motor_pwm(
        self,
        linear_v: float,
        angular_w: float,
        dt: float = 0.1,
        emergency_stop: bool = False
    ) -> Tuple[Tuple[float, float], Tuple[float, float]]:
        """
        Piecewise Kinematic Velocity-to-PWM Remapping with Continuous Deadband Lift,
        Slew-Rate Acceleration Limiter, and Trim Balancing (IEEE 2024-2026 Standard).

        Returns:
            (pwm_left, pwm_right), (v_left, v_right)
            where pwm in [-1.0, 1.0] ready for PCA9685 / Adafruit MotorHAT.
        """
        if emergency_stop or (abs(linear_v) < 1e-4 and abs(angular_w) < 1e-4):
            self.last_pwm_left = 0.0
            self.last_pwm_right = 0.0
            return (0.0, 0.0), (0.0, 0.0)

        # 1. Differential Drive Inverse Kinematics
        half_l = self.track_width / 2.0
        v_left = linear_v - (angular_w * half_l)
        v_right = linear_v + (angular_w * half_l)

        # 2. Speed Normalization by max linear speed
        v_max = max(0.05, self.max_linear_speed)
        u_left = max(-1.0, min(1.0, v_left / v_max))
        u_right = max(-1.0, min(1.0, v_right / v_max))

        # 3. Piecewise Continuous Deadband Lift
        min_pwm = self.min_drive_speed

        def lift_deadband(u: float) -> float:
            if abs(u) < 1e-3:
                return 0.0
            sign = 1.0 if u > 0 else -1.0
            return sign * (min_pwm + (1.0 - min_pwm) * abs(u))

        raw_pwm_l = lift_deadband(u_left)
        raw_pwm_r = lift_deadband(u_right)

        # 4. Mechanical Trim Balancing (Wheel Asymmetry Compensation)
        raw_pwm_l = max(-1.0, min(1.0, raw_pwm_l * self.trim_factor))
        raw_pwm_r = max(-1.0, min(1.0, raw_pwm_r))

        # 5. Slew-Rate Acceleration Limiter (Chống sụt áp pin & trượt bánh)
        ramp_time = max(0.1, self.soft_start_ramp_sec)
        max_rate = 1.0 / ramp_time
        max_delta = max_rate * max(0.01, min(0.2, dt))

        def clamp_rate(target: float, previous: float) -> float:
            delta = target - previous
            if abs(delta) > max_delta:
                return previous + math.copysign(max_delta, delta)
            return target

        final_pwm_l = clamp_rate(raw_pwm_l, self.last_pwm_left)
        final_pwm_r = clamp_rate(raw_pwm_r, self.last_pwm_right)

        self.last_pwm_left = final_pwm_l
        self.last_pwm_right = final_pwm_r

        return (round(final_pwm_l, 3), round(final_pwm_r, 3)), (round(v_left, 3), round(v_right, 3))
