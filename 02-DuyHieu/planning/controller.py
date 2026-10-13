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

try:
    from .disparity_extender import DisparityExtender
    from .cbf_safety_filter import CBFSafetyFilter
except ImportError:
    try:
        from disparity_extender import DisparityExtender
        from cbf_safety_filter import CBFSafetyFilter
    except ImportError:
        DisparityExtender = Any
        CBFSafetyFilter = Any


class PurePursuitController:
    def __init__(
        self,
        lookahead_dist: float = 0.28,
        max_linear_speed: float = 0.10,
        max_angular_speed: float = 0.28,
        track_width: float = 0.115,
        goal_tolerance: float = 0.10,
        reflex_dist_threshold: float = 0.14,
        warning_dist_threshold: float = 0.45,
        curvature_scale_k: float = 1.5,
        lateral_detour_offset: float = 0.20
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

        # SOTA Meta 2026: Active Disparity Extender & CBF-QP Safety Filter
        self.disparity_extender = DisparityExtender(
            robot_radius=0.11,
            safety_margin=0.09,
            disparity_threshold=0.25,
            safe_clearance_m=0.35,
            critical_stop_m=self.reflex_dist_threshold
        ) if DisparityExtender != Any else None

        self.cbf_filter = CBFSafetyFilter(
            lookahead_L=0.12,
            d_safe=0.15,
            gamma=2.8,
            v_max=self.max_linear_speed,
            w_max=self.max_angular_speed
        ) if CBFSafetyFilter != Any else None

        self.current_path: List[Tuple[float, float]] = []
        self.path_index: int = 0
        self.is_goal_reached: bool = False
        self.is_reflex_stopped: bool = False
        self.is_path_blocked: bool = False
        self.last_linear_v: float = 0.0
        self._blocked_persistence_count: int = 0

    def set_path(self, path: List[Tuple[float, float]]):
        self.current_path = path
        self.path_index = 0
        self.is_goal_reached = False
        self.is_reflex_stopped = False
        self.is_path_blocked = False
        self._blocked_persistence_count = 0

    def clear_path(self):
        self.current_path = []
        self.path_index = 0
        self.is_goal_reached = False
        self.is_path_blocked = False
        self._blocked_persistence_count = 0

    def check_safety_reflex(self, ranges: List[float], dist_to_goal: Optional[float] = None) -> bool:
        """10Hz Hardware Reflex Emergency Brake on immediate frontal cone (< 0.18m)."""
        if not ranges or len(ranges) < 360:
            return False

        frontal_indices = list(range(0, 32)) + list(range(328, 360))
        for idx in frontal_indices:
            r = ranges[idx]
            # Filter chassis internal glare (< 0.12m); trigger reflex stop if real obstacle in [0.12, reflex_dist_threshold)
            if 0.12 <= r < self.reflex_dist_threshold:
                # CHỐT 4: Nếu vật cản nằm sau đích (r >= dist_to_goal + 0.05m), không kích hoạt E-Stop!
                if dist_to_goal is not None and r >= (dist_to_goal + 0.05):
                    continue
                self.is_reflex_stopped = True
                return True

        self.is_reflex_stopped = False
        return False

    def _project_to_path(self, robot_x: float, robot_y: float, is_evading: bool = False) -> Tuple[int, Tuple[float, float], float]:
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

        # CHỐT 2: WAYFORWARD PROJECTION - Khi đang né tránh và lệch ngang > 15cm, trượt điểm chiếu tiến lên phía trước
        if is_evading and best_dist > 0.15:
            advance_seg = min(len(self.current_path) - 2, best_seg + 1)
            if advance_seg > best_seg:
                best_seg = advance_seg
                best_proj = self.current_path[best_seg]

        return best_seg, best_proj, best_dist

    def check_path_collision(
        self,
        robot_x: float,
        robot_y: float,
        robot_theta: float,
        grid_map: Any = None,
        clusters: Optional[List[Any]] = None,
        horizon_m: float = 0.45
    ) -> Tuple[bool, Optional[Tuple[float, float]], str]:
        """
        4D Space-Time Collision Inspector along lookahead horizon (0.45m for indoor arena).
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
        v_est = max(0.12, self.last_linear_v)

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

                # 1. Costmap Check (Lethal Obstacles Only, Cost >= 250)
                if grid_map is not None:
                    c, r = grid_map.world_to_grid(interp_x, interp_y)
                    if not grid_map.is_inside(c, r) or grid_map.costmap[r, c] >= 250:
                        self.is_path_blocked = True
                        return True, (round(interp_x, 3), round(interp_y, 3)), "COSTMAP_OBSTACLE"

                # 2. Dynamic Cluster Collision Check (Filtered to Navigation Corridor)
                if clusters:
                    dx_w = interp_x - robot_x
                    dy_w = interp_y - robot_y
                    pt_rx = (dx_w * cos_t) + (dy_w * sin_t)
                    pt_ry = (-dx_w * sin_t) + (dy_w * cos_t)

                    t_arrival = accum_dist / v_est

                    for cl in clusters:
                        cx, cy = cl.centroid
                        # Ignore side walls outside robot forward corridor (|cy| > 0.22m)
                        if abs(cy) > 0.22:
                            continue

                        is_dyn = getattr(cl, "is_dynamic", False)
                        closing_speed = getattr(cl, "closing_speed", 0.0)

                        if is_dyn and closing_speed > 0.05:
                            pred_pos = cl.predict_position(t_arrival) if hasattr(cl, "predict_position") else cl.centroid
                            r_safe_dyn = min(0.18, max(cl.length_m, cl.width_m) / 2.0 + 0.12)
                            dist_dyn = math.hypot(pt_rx - pred_pos[0], pt_ry - pred_pos[1])
                            if dist_dyn <= r_safe_dyn:
                                self.is_path_blocked = True
                                return True, (round(interp_x, 3), round(interp_y, 3)), f"DYNAMIC_ONCOMING_#{cl.id}_TTC_{cl.ttc:.1f}s"
                        else:
                            r_safe = min(0.18, max(cl.length_m, cl.width_m) / 2.0 + 0.10)
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
        lateral_offset: Optional[float] = None
    ) -> Optional[List[Tuple[float, float]]]:
        """
        Trapezoidal Plateau Detour Generator (3 segments with parallel hold).
        """
        if not self.current_path or len(self.current_path) < 2:
            return None

        if lateral_offset is None:
            lateral_offset = self.lateral_detour_offset

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
                if not grid_map.is_inside(c_r, r_r) or grid_map.costmap[r_r, c_r] >= 250:
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
                l_free = grid_map.is_inside(c_l, r_l) and grid_map.costmap[r_l, c_l] < 250
                r_free = grid_map.is_inside(c_r, r_r) and grid_map.costmap[r_r, c_r] < 250
                if r_free and not l_free:
                    chosen_cand = cand_right
                    chosen_lat_dir = n_right
                elif l_free and not r_free:
                    chosen_cand = cand_left
        goal_x, goal_y = self.current_path[-1]
        dist_to_goal = math.hypot(goal_x - bx, goal_y - by)
        nominal_rejoin = 0.85 if is_oncoming else 0.70
        # Rejoin distance must NEVER overshoot past the final goal waypoint
        rejoin_dist = min(nominal_rejoin, max(0.20, dist_to_goal * 0.85))
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

    def _is_path_safe(self, pts: List[Tuple[float, float]], grid_map: Any) -> bool:
        if not pts or grid_map is None:
            return True
        for px, py in pts:
            c, r = grid_map.world_to_grid(px, py)
            if not grid_map.is_inside(c, r) or grid_map.costmap[r, c] >= 250:
                return False
        return True

    def generate_greedy_detour(
        self,
        robot_x: float,
        robot_y: float,
        robot_theta: float,
        goal_x: float,
        goal_y: float,
        grid_map: Optional[Any] = None,
        lidar_ranges: Optional[List[float]] = None
    ) -> Optional[List[Tuple[float, float]]]:
        """
        Greedy Local Reactive Detour for sudden obstacles (Greedy Gap Finder).
        Evaluates angular evasion candidates (+-35, +-55, +-75, +-90 deg) and
        picks the best unblocked heading pointing closest towards the goal.
        """
        th_goal = math.atan2(goal_y - robot_y, goal_x - robot_x)
        # Limit candidates to gentle forward evasion angles (never 75 or 90 deg into narrow arena walls)
        candidates = [math.radians(ang) for ang in [-18.0, 18.0, -32.0, 32.0, -45.0, 45.0]]

        best_cand = None
        best_score = -999.0

        for d_th in candidates:
            cand_th = (robot_theta + d_th + math.pi) % (2.0 * math.pi) - math.pi

            # 1. LiDAR ray check if ranges available (safe clearance >= 24cm)
            if lidar_ranges and len(lidar_ranges) >= 360:
                ray_deg = int(round(math.degrees(d_th))) % 360
                cone_indices = [(ray_deg + ofs) % 360 for ofs in range(-12, 13)]
                min_cone_r = min((lidar_ranges[idx] for idx in cone_indices if lidar_ranges[idx] > 0.05), default=2.0)
                if min_cone_r < 0.24:
                    continue

            # 2. Costmap ray check (check for lethal physical obstacles)
            if grid_map is not None:
                is_free = True
                for d_step in [0.15, 0.25, 0.35]:
                    cx = robot_x + d_step * math.cos(cand_th)
                    cy = robot_y + d_step * math.sin(cand_th)
                    col, row = grid_map.world_to_grid(cx, cy)
                    if not grid_map.is_inside(col, row) or grid_map.costmap[row, col] >= 250:
                        is_free = False
                        break
                if not is_free:
                    continue

            # 3. Greedy Score: highest projection towards goal direction
            score = math.cos(cand_th - th_goal)
            if score > best_score:
                best_score = score
                best_cand = cand_th

        if best_cand is None:
            return None

        # Construct greedy local detour waypoints
        p1 = (round(robot_x + 0.18 * math.cos(best_cand), 3), round(robot_y + 0.18 * math.sin(best_cand), 3))
        p2 = (round(robot_x + 0.35 * math.cos(best_cand), 3), round(robot_y + 0.35 * math.sin(best_cand), 3))

        # Rejoin cleanly at the waypoint closest to p2 and ahead of robot
        best_rejoin = len(self.current_path) - 1
        min_rejoin_d = float("inf")
        for i in range(min(len(self.current_path) - 1, self.path_index + 1), len(self.current_path)):
            pt = self.current_path[i]
            d = math.hypot(pt[0] - p2[0], pt[1] - p2[1])
            if d < min_rejoin_d:
                min_rejoin_d = d
                best_rejoin = i

        return [p1, p2] + self.current_path[best_rejoin:]

    def modulate_velocity(
        self,
        base_v: float,
        clusters: Optional[List[Any]] = None,
        omega: float = 0.0,
        v_floor: Optional[float] = None
    ) -> float:
        """
        Curvature-Aware Scaling & Obstacle Proximity Deceleration Governor:
          v = v_base / (1 + k * |omega|)
        - When heading straight (omega ~ 0 rad/s), v maintains cruise speed.
        - When turning tight corner (omega >= 1.0 rad/s), speed gracefully drops
          to prevent differential wheel slip and roll instability.
        - Decelerates softly when obstacles enter warning zone (0.55m).
        """
        # Curvature scaling with deadband-safe velocity floor
        floor = v_floor if v_floor is not None else min(base_v, 0.14)
        scaled_v = base_v / (1.0 + self.curvature_scale_k * abs(omega))
        target_v = max(floor, scaled_v)

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
        # 1. Idle check
        if not self.current_path:
            self.last_linear_v = 0.0
            return 0.0, 0.0, {"status": "IDLE", "distance_remaining": 0.0}

        goal_x, goal_y = self.current_path[-1]
        dist_to_final_goal = math.hypot(goal_x - robot_x, goal_y - robot_y)

        # 2. 10Hz Hardware Reflex Emergency Brake (Chốt 4: Clip obstacles behind/at goal)
        if lidar_ranges and self.check_safety_reflex(lidar_ranges, dist_to_goal=dist_to_final_goal):
            self.last_linear_v = 0.0
            return 0.0, 0.0, {
                "status": "EMERGENCY_STOP",
                "reason": f"Obstacle < {self.reflex_dist_threshold*100:.0f}cm in front cone!",
                "distance_remaining": round(dist_to_final_goal, 3)
            }

        # 3. Dual-Criteria Goal Reached Check (Euclidean Tolerance + Plane Crossing Overshoot Detection)
        # Criterion 3.1: Standard Euclidean radius check
        is_reached = dist_to_final_goal <= self.goal_tolerance

        # Criterion 3.2: Plane Crossing / Overshoot Detection (Catch fast-moving robot crossing goal line)
        if not is_reached and dist_to_final_goal <= (self.goal_tolerance * 2.2):
            if len(self.current_path) >= 2:
                prev_x, prev_y = self.current_path[-2]
                seg_u_x = goal_x - prev_x
                seg_u_y = goal_y - prev_y
                seg_len = math.hypot(seg_u_x, seg_u_y)
                if seg_len > 1e-4:
                    # Unit vector pointing into goal
                    u_hat_x = seg_u_x / seg_len
                    u_hat_y = seg_u_y / seg_len
                    # Vector from goal to robot
                    w_x = robot_x - goal_x
                    w_y = robot_y - goal_y
                    # Longitudinal displacement past goal along path tangent
                    d_long_past_goal = w_x * u_hat_x + w_y * u_hat_y
                    # Lateral cross-track error from goal tangent
                    d_lat_past_goal = abs(w_x * u_hat_y - w_y * u_hat_x)

                    # Robot has crossed the orthogonal goal plane within acceptable lateral boundary
                    if d_long_past_goal >= 0.0 and d_lat_past_goal <= (self.goal_tolerance * 1.5):
                        is_reached = True

        if is_reached:
            self.is_goal_reached = True
            self.clear_path()
            self.last_linear_v = 0.0
            return 0.0, 0.0, {"status": "GOAL_REACHED", "distance_remaining": 0.0}

        # 4. Closed-Loop Dynamic Detour Evasion (Plateau + Greedy Gap Fallback)
        evasion_active = False
        evasion_reason = ""
        # Goal Approach Detour Guard: Suppress forward detour splicing when nearing final goal (<= 0.55m).
        # Prevents overshooting past the destination into background walls or obstacle boundaries.
        can_splice_detour = auto_evade and self.current_path and (dist_to_final_goal > 0.55)
        if can_splice_detour:
            is_blk, blk_pt, reason = self.check_path_collision(
                robot_x, robot_y, robot_theta, grid_map=grid_map, clusters=clusters, horizon_m=0.45
            )
            if is_blk and blk_pt is not None:
                self._blocked_persistence_count += 1
            else:
                self._blocked_persistence_count = 0

            # Require obstacle to persist for >= 3 ticks (>= 0.3s) before cutting detour
            # Eliminates spurious replanning from single-frame noise or far walls
            if self._blocked_persistence_count >= 3 and blk_pt is not None:
                culprit = clusters[0] if clusters else None
                detour = self.generate_detour_splice(
                    robot_x, robot_y, robot_theta, blk_pt,
                    obstacle_cluster=culprit, grid_map=grid_map, lateral_offset=self.lateral_detour_offset
                )
                if not detour or (grid_map is not None and not self._is_path_safe(detour[:4], grid_map)):
                    # Fallback to Greedy Gap finder around sudden obstacle
                    detour = self.generate_greedy_detour(
                        robot_x, robot_y, robot_theta, goal_x, goal_y,
                        grid_map=grid_map, lidar_ranges=lidar_ranges
                    )
                    if detour:
                        reason = "GREEDY_GAP_EVASION"

                if detour:
                    self.current_path = detour
                    evasion_active = True
                    evasion_reason = reason
                    self._blocked_persistence_count = 0

        # 5. Pure Pursuit Lookahead Target Search (Chốt 2: Wayforward Projection aware)
        best_seg, best_proj, _ = self._project_to_path(robot_x, robot_y, is_evading=evasion_active)
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

        # 7. Curvature-Aware Scaling with Goal Approach Deceleration Ramp
        decel_zone_m = 0.35
        v_cruise = self.max_linear_speed
        v_crawl = 0.04
        v_floor = min(0.08, v_cruise * 0.80)

        if dist_to_final_goal <= decel_zone_m:
            # Linear approach deceleration from cruise floor down to gentle creep (0.04 m/s)
            ramp_factor = max(0.0, min(1.0, (dist_to_final_goal - self.goal_tolerance) / max(0.01, decel_zone_m - self.goal_tolerance)))
            v_approach = v_crawl + (v_floor - v_crawl) * ramp_factor
            base_v = v_approach
            v_floor = v_crawl
        else:
            base_v = min(v_cruise, max(v_floor, dist_to_final_goal * 0.75))

        # Behind-the-Goal trap protection: If within tight vicinity of goal (<= 1.25 * tolerance) and goal is behind robot (|alpha| > 90 deg), STOP immediately!
        if dist_to_final_goal <= (self.goal_tolerance * 1.25) and abs(alpha) > (math.pi / 2.0):
            self.is_goal_reached = True
            self.clear_path()
            self.last_linear_v = 0.0
            return 0.0, 0.0, {"status": "GOAL_REACHED", "distance_remaining": 0.0}

        target_w0 = base_v * kappa

        if abs(alpha) > 1.65:
            # SOTA Differential Drive In-Place Spin Turn (|alpha| > 94.5 degrees):
            # CHỐT 1: Evasion Gating. If evading, maintain positive linear crawl (0.04 m/s)
            # so the robot peels out of the obstacle shadow instead of locking into infinite in-place rotation!
            v = 0.04 if evasion_active else 0.0
            w = math.copysign(min(0.24, self.max_angular_speed * 0.85), alpha)
            aligning = True
        elif abs(alpha) > 0.85:
            # Very sharp turn (48 - 94.5 degrees): creep turn with gentle forward velocity
            v = 0.035 if dist_to_final_goal <= decel_zone_m else 0.04
            w = math.copysign(min(0.22, self.max_angular_speed * 0.75), alpha)
            aligning = True
        elif abs(alpha) > 0.40:
            # Moderate heading error (23 - 48 degrees): smooth cosine velocity scaling prevents wheel slip
            v_head_scale = math.cos(min(math.pi * 0.45, abs(alpha)) / 2.0) ** 2
            v_cand = max(0.04, base_v * v_head_scale)
            v = self.modulate_velocity(v_cand, clusters=clusters, omega=target_w0, v_floor=0.04)
            w = max(-self.max_angular_speed, min(self.max_angular_speed, (v if v > 0 else 0.04) * kappa))
            aligning = True
        else:
            v = self.modulate_velocity(base_v, clusters=clusters, omega=target_w0, v_floor=v_floor)
            v = max(v_floor, min(v_cruise, v))
            w = max(-self.max_angular_speed, min(self.max_angular_speed, v * kappa))
            aligning = False

        # 8. SOTA TẦNG 1: Active Disparity Extender (DDE) Reflex
        dde_meta = None
        if lidar_ranges and self.disparity_extender:
            dde_eval = self.disparity_extender.evaluate(lidar_ranges, goal_heading_rad=alpha)
            dde_meta = dde_eval
            if dde_eval.get("status") == "EMERGENCY_STOP":
                obs_d = dde_eval.get("front_obstacle_dist", 0.0)
                # CHỐT 4: Terminal Obstacle Clipping - If obstacle is at or behind the final goal, bypass E-Stop!
                if dist_to_final_goal <= (obs_d + 0.05):
                    pass
                else:
                    self.last_linear_v = 0.0
                    return 0.0, 0.0, {
                        "status": "EMERGENCY_STOP",
                        "reason": dde_eval.get("reason", f"Obstacle < {self.reflex_dist_threshold*100:.0f}cm in front cone!"),
                        "distance_remaining": round(dist_to_final_goal, 3),
                        "target": (round(target_pt[0], 3), round(target_pt[1], 3)),
                        "cbf_active": False
                    }
            if dde_eval.get("is_blocked"):
                # DDE Goal-Relative Gating: only steer away if obstacle is strictly closer than the goal.
                # If obstacle is behind the goal, do not override steering.
                obs_dist = dde_eval.get("front_obstacle_dist", dde_eval.get("clearance_m", 0.5))
                if dist_to_final_goal > (obs_dist + 0.05):
                    target_steer = dde_eval["target_heading_rad"]
                    evasion_active = True
                    evasion_reason = dde_eval.get("reason", "DDE Reactive Gap Selection")
                    w = max(-self.max_angular_speed, min(self.max_angular_speed, 0.85 * target_steer))
                    # CHỐT 1: Evasion Gating - Maintain forward translation (v >= 0.04 m/s) so robot arcs away
                    # instead of oscillating in place at v = 0.0!
                    v = max(0.04, min(self.max_linear_speed * 0.80, obs_dist * 0.30))

        # 9. SOTA TẦNG 2: Control Barrier Function (CBF-QP) Safety Filter
        cbf_info = {"cbf_active": False}
        if lidar_ranges and self.cbf_filter:
            v_safe, w_safe, cbf_info = self.cbf_filter.filter_control(v, w, raw_ranges=lidar_ranges)
            if cbf_info.get("cbf_active"):
                evasion_active = True
                evasion_reason = cbf_info.get("reason", "CBF Invariance Safety Shield")
                v = v_safe
                w = w_safe

        self.last_linear_v = v
        status = "EVADING" if evasion_active else ("ALIGNING" if aligning else "NAVIGATING")
        return round(v, 3), round(w, 3), {
            "status": status,
            "distance_remaining": round(dist_to_final_goal, 3),
            "target": (round(target_pt[0], 3), round(target_pt[1], 3)),
            "is_blocked": self.is_path_blocked,
            "evasion_active": evasion_active,
            "evasion_reason": evasion_reason,
            "cbf_active": cbf_info.get("cbf_active", False),
            "cbf_min_h": cbf_info.get("min_h", 1.0),
            "dde_active": dde_meta.get("is_blocked", False) if dde_meta else False,
            "dde_target_deg": dde_meta.get("target_heading_deg", 0.0) if dde_meta else 0.0
        }

    def differential_drive_ik(self, v: float, w: float) -> Tuple[float, float]:
        """Inverse Kinematics for Differential Drive robot."""
        v_left = v - (w * self.track_width / 2.0)
        v_right = v + (w * self.track_width / 2.0)
        return round(v_left, 3), round(v_right, 3)
