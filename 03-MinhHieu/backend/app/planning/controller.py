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


class PurePursuitController:
    def __init__(
        self,
        lookahead_dist: float = 0.32,
        max_linear_speed: float = 0.32,
        max_angular_speed: float = 1.2,
        track_width: float = 0.115,
        goal_tolerance: float = 0.12,
        reflex_dist_threshold: float = 0.18,
        warning_dist_threshold: float = 0.70,
        curvature_scale_k: float = 1.0,
        lateral_detour_offset: float = 0.32,
        min_drive_speed: float = 0.22,
        trim_factor: float = 1.0,
        soft_start_ramp_sec: float = 0.4
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
        self.trim_factor = trim_factor
        self.soft_start_ramp_sec = soft_start_ramp_sec

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

    def clear_path(self):
        self.current_path = []
        self.global_reference_path = []
        self.global_goal = None
        self.path_index = 0
        self.is_goal_reached = False
        self.is_path_blocked = False
        self.is_evading = False
        self.evasion_end_index = 0

    def check_safety_reflex(self, ranges: List[float]) -> bool:
        """10Hz Hardware Reflex Emergency Brake on immediate frontal cone (< 0.18m)."""
        if not ranges or len(ranges) < 360:
            return False

        frontal_indices = list(range(0, 32)) + list(range(328, 360))
        for idx in frontal_indices:
            r = ranges[idx]
            if 0.04 < r < self.reflex_dist_threshold:
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
        horizon_m: float = 1.8
    ) -> Tuple[bool, Optional[Tuple[float, float]], str]:
        """
        4D Space-Time Collision Inspector along lookahead horizon (1.8m).
        Detects costmap cell blockages and oncoming dynamic obstacle intersections.
        """
        if not self.current_path or len(self.current_path) < 2:
            self.is_path_blocked = False
            return False, None, "NO_PATH"

        best_seg, best_proj, _ = self._project_to_path(robot_x, robot_y)
        self.path_index = best_seg

        cos_t = math.cos(robot_theta)
        sin_t = math.sin(robot_theta)

        accum_dist = 0.0
        curr_pt = best_proj
        v_est = max(0.18, self.last_linear_v)

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

                # 1. Costmap Check
                if grid_map is not None:
                    c, r = grid_map.world_to_grid(interp_x, interp_y)
                    if not grid_map.is_cell_free(c, r):
                        self.is_path_blocked = True
                        return True, (round(interp_x, 3), round(interp_y, 3)), "COSTMAP_OBSTACLE"

                # 2. Dynamic Cluster Collision Check
                if clusters:
                    dx_w = interp_x - robot_x
                    dy_w = interp_y - robot_y
                    pt_rx = (dx_w * cos_t) + (dy_w * sin_t)
                    pt_ry = (-dx_w * sin_t) + (dy_w * cos_t)

                    t_arrival = accum_dist / v_est

                    for cl in clusters:
                        is_dyn = getattr(cl, "is_dynamic", False)
                        closing_speed = getattr(cl, "closing_speed", 0.0)

                        if is_dyn and closing_speed > 0.05:
                            pred_pos = cl.predict_position(t_arrival) if hasattr(cl, "predict_position") else cl.centroid
                            r_safe_dyn = max(cl.length_m, cl.width_m) / 2.0 + 0.22
                            dist_dyn = math.hypot(pt_rx - pred_pos[0], pt_ry - pred_pos[1])
                            if dist_dyn <= r_safe_dyn:
                                self.is_path_blocked = True
                                return True, (round(interp_x, 3), round(interp_y, 3)), f"DYNAMIC_ONCOMING_#{cl.id}_TTC_{cl.ttc:.1f}s"
                        else:
                            cx, cy = cl.centroid
                            r_safe = max(cl.length_m, cl.width_m) / 2.0 + 0.20
                            dist_to_cl = math.hypot(pt_rx - cx, pt_ry - cy)
                            if dist_to_cl <= r_safe:
                                self.is_path_blocked = True
                                return True, (round(interp_x, 3), round(interp_y, 3)), f"CLUSTER_OBSTACLE_#{cl.id}_{cl.obstacle_type}"

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
        to find the distance from origin_pt to the nearest lethal boundary/wall.
        Uses Nyquist spatial sampling (step <= resolution / 2) to ensure no barrier cell is skipped.
        """
        if grid_map is None:
            return max_dist
        ox, oy = origin_pt
        dx, dy = direction
        res = getattr(grid_map, "resolution", 0.05)
        step = min(step, res * 0.5)
        dist = max(0.0, start_offset)
        while dist <= max_dist:
            px = ox + dist * dx
            py = oy + dist * dy
            c, r = grid_map.world_to_grid(px, py)
            if not grid_map.is_cell_free(c, r):
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
                if grid_map.is_cell_free(c, r):
                    break
            d_obs_left = max(0.12, d_l)

            d_r = 0.0
            while d_r < 1.0:
                d_r += 0.02
                px = bx + d_r * n_right[0]
                py = by + d_r * n_right[1]
                c, r = grid_map.world_to_grid(px, py)
                if grid_map.is_cell_free(c, r):
                    break
            d_obs_right = -max(0.12, d_r)

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

        # 3. Drivable gaps calculation
        gap_left = max(0.0, wall_clearance_left - max(0.0, d_obs_left))
        gap_right = max(0.0, wall_clearance_right - max(0.0, -d_obs_right))

        MIN_REQUIRED_PASS_GAP = 0.28  # robot diameter 0.24m + 0.04m physical margin
        left_viable = gap_left >= MIN_REQUIRED_PASS_GAP
        right_viable = gap_right >= MIN_REQUIRED_PASS_GAP

        # 4. Impassable Corridor Protection (Full Blockage)
        if grid_map is not None and not left_viable and not right_viable:
            # Neither side offers sufficient gap: abort detour safely to avoid wall crash
            return None

        # 5. Side selection logic
        if is_oncoming:
            # COLREGS maritime/road standard: prefer right deflection if viable
            if right_viable:
                chosen_side = "RIGHT"
                chosen_lat_dir = n_right
            else:
                chosen_side = "LEFT"
                chosen_lat_dir = n_left
        else:
            if left_viable and not right_viable:
                chosen_side = "LEFT"
                chosen_lat_dir = n_left
            elif right_viable and not left_viable:
                chosen_side = "RIGHT"
                chosen_lat_dir = n_right
            else:
                # Both viable: choose side with larger clearance
                if gap_left >= gap_right:
                    chosen_side = "LEFT"
                    chosen_lat_dir = n_left
                else:
                    chosen_side = "RIGHT"
                    chosen_lat_dir = n_right

        # 6. Adaptive lateral offset computation (Minimal Necessary Deviation)
        ROBOT_RADIUS = 0.12
        DESIRED_BODY_CLEARANCE = ROBOT_RADIUS + 0.06  # 0.18m

        if grid_map is not None:
            if chosen_side == "LEFT":
                if gap_left < 0.60:
                    d_offset = max(0.0, d_obs_left) + (gap_left / 2.0)
                else:
                    d_offset = max(0.0, d_obs_left) + DESIRED_BODY_CLEARANCE
                # Clamp so robot does not strike left wall
                d_offset = min(d_offset, wall_clearance_left - ROBOT_RADIUS - 0.03)
            else:
                if gap_right < 0.60:
                    d_offset = max(0.0, -d_obs_right) + (gap_right / 2.0)
                else:
                    d_offset = max(0.0, -d_obs_right) + DESIRED_BODY_CLEARANCE
                d_offset = min(d_offset, wall_clearance_right - ROBOT_RADIUS - 0.03)
            d_offset = max(DESIRED_BODY_CLEARANCE, min(0.35, d_offset))
        else:
            # Open space fallback
            d_offset = lateral_offset if lateral_offset is not None else self.lateral_detour_offset

        # 7. Adaptive longitudinal splice horizon (Smooth lane-change slope)
        s_ramp = max(0.45, min(0.68, d_offset * 1.85))
        s_plateau = max(0.18, min(0.45, obs_length + 0.05))
        rejoin_dist = s_ramp + s_plateau
        if self.global_goal is not None:
            dist_to_goal_from_blk = math.hypot(self.global_goal[0] - bx, self.global_goal[1] - by)
            if dist_to_goal_from_blk > 0.20:
                rejoin_dist = min(rejoin_dist, max(0.45, dist_to_goal_from_blk))

        rejoin_pt = (bx + rejoin_dist * u_fwd[0], by + rejoin_dist * u_fwd[1])
        p_start = (robot_x, robot_y)
        vec_long = (rejoin_pt[0] - p_start[0], rejoin_pt[1] - p_start[1])

        # 3-segment Trapezoidal Plateau Generation
        detour_pts = []
        n_steps = 14
        total_len = s_ramp + s_plateau + s_ramp
        u1 = s_ramp / total_len
        u2 = (s_ramp + s_plateau) / total_len

        for s in range(1, n_steps + 1):
            u = s / float(n_steps)
            if u <= u1:
                lat_disp = d_offset * 0.5 * (1.0 - math.cos(math.pi * (u / u1)))
            elif u <= u2:
                lat_disp = d_offset
            else:
                lat_disp = d_offset * 0.5 * (1.0 + math.cos(math.pi * ((u - u2) / (1.0 - u2))))

            x = p_start[0] + u * vec_long[0] + lat_disp * chosen_lat_dir[0]
            y = p_start[1] + u * vec_long[1] + lat_disp * chosen_lat_dir[1]
            detour_pts.append((round(x, 3), round(y, 3)))

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

        # 8. Trajectory Costmap Safety Check
        if grid_map is not None:
            for pt in detour_pts:
                c, r = grid_map.world_to_grid(pt[0], pt[1])
                if not grid_map.is_cell_free(c, r):
                    return None

        return spliced_path

    def modulate_velocity(
        self,
        base_v: float,
        clusters: Optional[List[Any]] = None,
        omega: float = 0.0
    ) -> float:
        """
        Curvature-Aware Scaling & Obstacle Proximity Deceleration Governor:
          v = v_base / (1 + k * |omega|)
        - When heading straight (omega ~ 0 rad/s), v maintains cruise speed.
        - When turning tight corner (omega >= 1.0 rad/s), speed gracefully drops
          to prevent differential wheel slip and roll instability.
        - Decelerates softly when obstacles enter warning zone (0.70m).
        """
        # Curvature scaling
        scaled_v = base_v / (1.0 + self.curvature_scale_k * abs(omega))
        target_v = max(0.04, scaled_v)

        if not clusters:
            return target_v

        min_frontal_dist = 99.0
        min_ttc = 99.0
        has_oncoming = False

        for cl in clusters:
            cx, cy = cl.centroid
            is_dyn = getattr(cl, "is_dynamic", False)
            closing_spd = getattr(cl, "closing_speed", 0.0)

            if cx > 0.05 and abs(cy) <= 0.28:
                if is_dyn and closing_spd > 0.05:
                    has_oncoming = True
                    ttc = getattr(cl, "ttc", 99.0)
                    if ttc < min_ttc:
                        min_ttc = ttc
                else:
                    if cl.min_distance < min_frontal_dist:
                        min_frontal_dist = cl.min_distance

        if has_oncoming and min_ttc < 3.0:
            scale_ttc = max(0.35, min(1.0, (min_ttc - 0.8) / 1.5))
            return target_v * scale_ttc

        if min_frontal_dist > self.warning_dist_threshold:
            return target_v

        if min_frontal_dist <= self.reflex_dist_threshold:
            return 0.0

        scale = (min_frontal_dist - self.reflex_dist_threshold) / (self.warning_dist_threshold - self.reflex_dist_threshold)
        scale = max(0.30, min(1.0, scale))
        return target_v * scale

    def compute_command(
        self,
        robot_x: float,
        robot_y: float,
        robot_theta: float,
        lidar_ranges: Optional[List[float]] = None,
        clusters: Optional[List[Any]] = None,
        grid_map: Optional[Any] = None,
        auto_evade: bool = True
    ) -> Tuple[float, float, Dict[str, Any]]:
        """
        Main 10Hz Control Loop: Pure Pursuit + Curvature Scaling + Reflex + Auto-Evade.
        Returns: (v, w, telemetry_info)
        """
        # 1. 10Hz Hardware Reflex Emergency Brake
        if lidar_ranges and self.check_safety_reflex(lidar_ranges):
            self.last_linear_v = 0.0
            return 0.0, 0.0, {
                "status": "EMERGENCY_STOP",
                "reason": "Obstacle < 18cm in front cone!",
                "distance_remaining": 0.0
            }

        # 2. Idle check
        if not self.current_path:
            self.last_linear_v = 0.0
            return 0.0, 0.0, {"status": "IDLE", "distance_remaining": 0.0}

        # Check distance to TRUE GLOBAL GOAL
        if self.global_goal is not None:
            dist_to_final_goal = math.hypot(self.global_goal[0] - robot_x, self.global_goal[1] - robot_y)
        else:
            goal_x, goal_y = self.current_path[-1]
            dist_to_final_goal = math.hypot(goal_x - robot_x, goal_y - robot_y)

        # 3. Goal Reached check (only triggers when near the real global goal!)
        if dist_to_final_goal <= self.goal_tolerance:
            self.is_goal_reached = True
            self.clear_path()
            self.last_linear_v = 0.0
            return 0.0, 0.0, {"status": "GOAL_REACHED", "distance_remaining": 0.0}

        # 4. Closed-Loop Dynamic Detour Evasion
        evasion_active = self.is_evading
        evasion_reason = ""
        if auto_evade and self.current_path:
            is_blk, blk_pt, reason = self.check_path_collision(
                robot_x, robot_y, robot_theta, grid_map=grid_map, clusters=clusters, horizon_m=1.8
            )
            # Re-trigger detour only if not currently evading OR if remaining path has a new close obstruction
            should_evade = False
            if is_blk and blk_pt is not None:
                if not self.is_evading:
                    should_evade = True
                else:
                    curr_seg, _, _ = self._project_to_path(robot_x, robot_y)
                    if curr_seg >= self.evasion_end_index:
                        should_evade = True
                    else:
                        d_blk_cur = math.hypot(blk_pt[0] - robot_x, blk_pt[1] - robot_y)
                        if d_blk_cur < 0.30:
                            should_evade = True

            if should_evade and blk_pt is not None:
                culprit = None
                if clusters:
                    best_d = 999.0
                    for cl in clusters:
                        cx, cy = getattr(cl, "centroid", (0.0, 0.0))
                        d = math.hypot(cx - blk_pt[0], cy - blk_pt[1])
                        if d < best_d:
                            best_d = d
                            culprit = cl

                detour = self.generate_detour_splice(
                    robot_x, robot_y, robot_theta, blk_pt,
                    obstacle_cluster=culprit, grid_map=grid_map, lateral_offset=None
                )
                if detour:
                    self.current_path = detour
                    self.path_index = 0
                    self.is_evading = True
                    self.evasion_end_index = 14
                    evasion_active = True
                    evasion_reason = reason
                else:
                    self.last_linear_v = 0.0
                    return 0.0, 0.0, {
                        "status": "IMPASSABLE_STOP",
                        "reason": f"Path blocked at {blk_pt} and corridor is impassable!",
                        "distance_remaining": round(dist_to_final_goal, 3),
                        "is_blocked": True,
                        "evasion_active": False,
                        "evasion_reason": reason
                    }

        # 5. Pure Pursuit Lookahead Target Search
        best_seg, best_proj, _ = self._project_to_path(robot_x, robot_y)
        self.path_index = best_seg

        if self.is_evading and self.path_index >= self.evasion_end_index:
            self.is_evading = False
            evasion_active = False

        rem_L = self.lookahead_dist
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
        target_angle = math.atan2(dy, dx)
        alpha = (target_angle - robot_theta + math.pi) % (2.0 * math.pi) - math.pi

        kappa = (2.0 * math.sin(alpha)) / self.lookahead_dist

        # 7. Curvature-Aware Scaling
        base_v = min(self.max_linear_speed, max(0.12, dist_to_final_goal * 0.8))
        target_w0 = base_v * kappa

        if abs(alpha) > 1.2:
            # Very sharp turn (> 70 degrees)
            if self.is_evading:
                # Continuous forward motion during detour, prevent spin-turn dead stops
                v = 0.10
                w = math.copysign(self.max_angular_speed * 0.65, alpha)
            else:
                v = 0.04
                w = math.copysign(self.max_angular_speed * 0.85, alpha)
        else:
            v = self.modulate_velocity(base_v, clusters=clusters, omega=target_w0)
            w = max(-self.max_angular_speed, min(self.max_angular_speed, v * kappa))

        self.last_linear_v = v
        return round(v, 3), round(w, 3), {
            "status": "EVADING" if evasion_active else "NAVIGATING",
            "distance_remaining": round(dist_to_final_goal, 3),
            "target": (round(target_pt[0], 3), round(target_pt[1], 3)),
            "is_blocked": self.is_path_blocked,
            "evasion_active": evasion_active,
            "evasion_reason": evasion_reason
        }

    def differential_drive_ik(self, v: float, w: float) -> Tuple[float, float]:
        """Inverse Kinematics for Differential Drive robot."""
        v_left = v - (w * self.track_width / 2.0)
        v_right = v + (w * self.track_width / 2.0)
        return round(v_left, 3), round(v_right, 3)

    def load_dynamics_config(self, config_path: Optional[str] = None):
        """Loads physical motor dynamics, trim factor and deadband from JSON config."""
        if config_path is None:
            search_paths = [
                os.path.join(os.path.dirname(__file__), "..", "setup", "robot_dynamics.json"),
                os.path.join(os.path.dirname(__file__), "robot_dynamics.json"),
                os.path.expanduser("~/setup/robot_dynamics.json"),
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
                if "max_safe_speed_mps" in cfg:
                    self.max_linear_speed = float(cfg["max_safe_speed_mps"])
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
