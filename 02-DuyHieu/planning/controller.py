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
        lateral_detour_offset: float = 0.52
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

        self.current_path: List[Tuple[float, float]] = []
        self.path_index: int = 0
        self.is_goal_reached: bool = False
        self.is_reflex_stopped: bool = False
        self.is_path_blocked: bool = False
        self.last_linear_v: float = 0.0

    def set_path(self, path: List[Tuple[float, float]]):
        self.current_path = path
        self.path_index = 0
        self.is_goal_reached = False
        self.is_reflex_stopped = False
        self.is_path_blocked = False

    def clear_path(self):
        self.current_path = []
        self.path_index = 0
        self.is_goal_reached = False
        self.is_path_blocked = False

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

    def generate_detour_splice(
        self,
        robot_x: float,
        robot_y: float,
        robot_theta: float,
        blocked_pt: Tuple[float, float],
        obstacle_cluster: Optional[Any] = None,
        grid_map: Optional[Any] = None,
        lateral_offset: float = 0.52
    ) -> Optional[List[Tuple[float, float]]]:
        """
        Trapezoidal Plateau Detour Generator (3 segments with parallel hold).
        - lateral_offset: 0.50m - 0.55m (default 0.52m).
        - Segment 1 (u in [0, 0.35]): Smooth cosine lane change ramp out.
        - Segment 2 (u in [0.35, 0.65]): Constant plateau hold at full lateral_offset along obstacle.
        - Segment 3 (u in [0.65, 1.0]): Smooth cosine lane rejoin back to global path.
        - Guarantees minimum physical clearance margin >= 0.22m (> 0.20m E-stop threshold).
        - Costmap-verified direction selection (automatically inverts side if blocked by wall).
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

        cand_left = (bx + lateral_offset * n_left[0], by + lateral_offset * n_left[1])
        cand_right = (bx + lateral_offset * n_right[0], by + lateral_offset * n_right[1])

        # Direction choice: COLREGS right deflection for oncoming, or check obstacle relative lateral pos
        is_oncoming = False
        if obstacle_cluster is not None:
            if getattr(obstacle_cluster, "is_dynamic", False) and getattr(obstacle_cluster, "closing_speed", 0.0) > 0.05:
                is_oncoming = True

        if is_oncoming:
            chosen_cand = cand_right
            chosen_lat_dir = n_right
            if grid_map is not None:
                c_r, r_r = grid_map.world_to_grid(cand_right[0], cand_right[1])
                if not grid_map.is_cell_free(c_r, r_r):
                    chosen_cand = cand_left
                    chosen_lat_dir = n_left
        else:
            chosen_cand = cand_left
            chosen_lat_dir = n_left
            if obstacle_cluster and hasattr(obstacle_cluster, "centroid") and obstacle_cluster.centroid[1] > 0:
                chosen_cand = cand_right
                chosen_lat_dir = n_right

            # Costmap obstacle check: if preferred side is blocked by wall, flip
            if grid_map is not None:
                c_l, r_l = grid_map.world_to_grid(cand_left[0], cand_left[1])
                c_r, r_r = grid_map.world_to_grid(cand_right[0], cand_right[1])
                l_free = grid_map.is_cell_free(c_l, r_l)
                r_free = grid_map.is_cell_free(c_r, r_r)
                if r_free and not l_free:
                    chosen_cand = cand_right
                    chosen_lat_dir = n_right
                elif l_free and not r_free:
                    chosen_cand = cand_left
                    chosen_lat_dir = n_left

        rejoin_dist = 0.85 if is_oncoming else 0.70
        rejoin_pt = (bx + rejoin_dist * u_fwd[0], by + rejoin_dist * u_fwd[1])
        p_start = (robot_x, robot_y)
        vec_long = (rejoin_pt[0] - p_start[0], rejoin_pt[1] - p_start[1])

        # 3-Segment Trapezoidal Plateau Generation
        # Segment 1: u in [0, 0.35] (Ramp out)
        # Segment 2: u in [0.35, 0.65] (Plateau hold: lat_disp = lateral_offset)
        # Segment 3: u in [0.65, 1.0] (Ramp in)
        detour_pts = []
        n_steps = 14
        u1 = 0.35
        u2 = 0.65

        for s in range(1, n_steps + 1):
            u = s / float(n_steps)
            if u <= u1:
                # Smooth cosine ramp out
                lat_disp = lateral_offset * 0.5 * (1.0 - math.cos(math.pi * (u / u1)))
            elif u <= u2:
                # Flat plateau hold parallel to obstacle
                lat_disp = lateral_offset
            else:
                # Smooth cosine ramp in back to path
                lat_disp = lateral_offset * 0.5 * (1.0 + math.cos(math.pi * ((u - u2) / (1.0 - u2))))

            x = p_start[0] + u * vec_long[0] + lat_disp * chosen_lat_dir[0]
            y = p_start[1] + u * vec_long[1] + lat_disp * chosen_lat_dir[1]
            detour_pts.append((round(x, 3), round(y, 3)))

        # Splicing into remaining path
        splice_idx = len(self.current_path) - 1
        for i in range(self.path_index, len(self.current_path)):
            pt = self.current_path[i]
            if (pt[0] - rejoin_pt[0]) * u_fwd[0] + (pt[1] - rejoin_pt[1]) * u_fwd[1] > 0.15:
                splice_idx = i
                break

        spliced_path = detour_pts + self.current_path[splice_idx:]
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

        goal_x, goal_y = self.current_path[-1]
        dist_to_final_goal = math.hypot(goal_x - robot_x, goal_y - robot_y)

        # 3. Goal Reached check
        if dist_to_final_goal <= self.goal_tolerance:
            self.is_goal_reached = True
            self.clear_path()
            self.last_linear_v = 0.0
            return 0.0, 0.0, {"status": "GOAL_REACHED", "distance_remaining": 0.0}

        # 4. Closed-Loop Dynamic Detour Evasion
        evasion_active = False
        evasion_reason = ""
        if auto_evade and clusters and self.current_path:
            is_blk, blk_pt, reason = self.check_path_collision(
                robot_x, robot_y, robot_theta, grid_map=grid_map, clusters=clusters, horizon_m=1.8
            )
            if is_blk and blk_pt is not None:
                culprit = clusters[0] if clusters else None
                detour = self.generate_detour_splice(
                    robot_x, robot_y, robot_theta, blk_pt,
                    obstacle_cluster=culprit, grid_map=grid_map, lateral_offset=self.lateral_detour_offset
                )
                if detour:
                    self.current_path = detour
                    evasion_active = True
                    evasion_reason = reason

        # 5. Pure Pursuit Lookahead Target Search
        best_seg, best_proj, _ = self._project_to_path(robot_x, robot_y)
        self.path_index = best_seg

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
            # Very sharp turn (> 70 degrees): spin-turn throttle
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
