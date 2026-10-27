"""
Motion Adapter Module.
Encapsulates single-source-of-truth motion planning, odometry, and control from 05-d500lib-motion/planning.
"""
import os
import math
import time
from typing import List, Tuple, Dict, Any, Optional
from app.config import Config
from planning.occupancy_grid import OccupancyGridMap
from planning.astar_planner import AStarPlanner
from planning.controller import PurePursuitController
from planning.mppi_controller import MPPIController
from planning.obstacle_clustering import cluster_lidar_obstacles, DynamicObstacleTracker
from planning.lidar_odometry import Lidar2DOdometry

try:
    from planning.il.dagger_policy import NavPolicyMLP, preprocess_lidar_72, build_state_vector
    from planning.cbf_safety_filter import CBFSafetyFilter
    import torch
    HAS_DAGGER = True
except Exception:
    HAS_DAGGER = False


class MotionAdapter:
    """Interface adapter for A*, Pure Pursuit, MPPI, and LiDAR SLAM Odometry."""

    def __init__(self):
        self.grid_map = OccupancyGridMap(
            width=Config.MAP_WIDTH,
            height=Config.MAP_HEIGHT,
            resolution=Config.MAP_RESOLUTION,
            origin_x=Config.MAP_ORIGIN_X,
            origin_y=Config.MAP_ORIGIN_Y
        )
        self.planner = AStarPlanner(self.grid_map, arena_bounds=Config.ARENA_BOUNDS)
        ctrl_mode = getattr(Config, "NAVIGATION_CONTROLLER", "pure_pursuit").lower()
        if ctrl_mode == "mppi":
            self.controller = MPPIController(
                horizon_steps=15,
                num_samples=250,
                dt=0.10,
                temperature_lambda=5.0,
                max_linear_speed=Config.MAX_SPEED,
                min_drive_speed=0.12,
                max_angular_speed=Config.MAX_ANGULAR_SPEED,
                goal_tolerance=0.18,
                reflex_dist_threshold=Config.REFLEX_STOP,
                min_lidar_range=Config.LIDAR_MIN_RANGE,
            )
        else:
            self.controller = PurePursuitController(
                lookahead_dist=0.28,
                max_linear_speed=Config.MAX_SPEED,
                max_angular_speed=Config.MAX_ANGULAR_SPEED,
                track_width=0.115,
                goal_tolerance=0.18,
                reflex_dist_threshold=Config.REFLEX_STOP,
                warning_dist_threshold=0.55,
                curvature_scale_k=1.2,
                lateral_detour_offset=getattr(Config, "LATERAL_DETOUR_OFFSET", 0.15),
                min_drive_speed=Config.MIN_DRIVE_SPEED,
                trim_factor=1.024,
                soft_start_ramp_sec=0.4,
                env_mode=getattr(Config, "ENVIRONMENT_MODE", "ARENA_2X1"),
                tactical_yield_dist=getattr(Config, "TACTICAL_YIELD_DIST", 0.30),
                passage_clearance_min=getattr(Config, "PASSAGE_CLEARANCE_MIN", 0.18),
                min_lidar_range=Config.LIDAR_MIN_RANGE,
            )
        self.controller.planner = self.planner
        self.obstacle_tracker = DynamicObstacleTracker(assoc_dist_m=0.45)
        self.lidar_odometry = Lidar2DOdometry(
            min_range_m=Config.LIDAR_MIN_RANGE,
            max_range_m=min(6.0, Config.MAX_RANGE),
            decimation_step=2
        )
        self.anchor_initialized: bool = False
        self.static_map_locked: bool = False

        # DAgger Imitation Learning Controller Setup
        self.is_dagger_mode = (ctrl_mode == "dagger_il" and HAS_DAGGER)
        self.dagger_policy = None
        self.cbf_filter = None
        self.prev_v: float = 0.0
        self.prev_w: float = 0.0

        if self.is_dagger_mode:
            try:
                safe_max_w = getattr(Config, "CALIBRATED_MAX_ANGULAR_SPEED", 0.35)
                self.dagger_policy = NavPolicyMLP(max_v=min(0.12, Config.MAX_SPEED), max_w=safe_max_w)
                base_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../.."))
                candidates = [
                    os.path.join(base_dir, "05-d500lib-motion/planning/il/checkpoints/dagger_policy.pth"),
                    "05-d500lib-motion/planning/il/checkpoints/dagger_policy.pth"
                ]
                for pth in candidates:
                    if os.path.exists(pth):
                        self.dagger_policy.load_state_dict(torch.load(pth, map_location="cpu"))
                        break
                self.dagger_policy.eval()
                self.cbf_filter = CBFSafetyFilter(v_max=min(0.12, Config.MAX_SPEED), w_max=safe_max_w)
            except Exception:
                self.is_dagger_mode = False

    def reset(self):
        """Reset SLAM map, odometry, and path state."""
        self.grid_map.reset()
        self.anchor_initialized = False
        self.static_map_locked = False
        self.lidar_odometry.reset_pose(0.0, 0.0, 0.0)
        self.controller.clear_path()

    def set_pose(self, x: float, y: float, theta: float):
        """Sets odometry pose without resetting map or reference keyframes."""
        if hasattr(self.lidar_odometry, "set_pose"):
            self.lidar_odometry.set_pose(x, y, theta)

    def update_odometry(self, ranges: List[float], dt: float,
                        v_cmd: float, w_cmd: float) -> Tuple[float, float, float, Dict[str, Any]]:
        """Update 2D LiDAR scan matching odometry."""
        return self.lidar_odometry.update(
            ranges, dt=dt, v_cmd=v_cmd, w_cmd=w_cmd, grid_map=self.grid_map
        )

    def process_clusters(self, ranges: List[float], pose_x: float, pose_y: float,
                         pose_theta: float) -> Tuple[List[Any], List[Dict[str, Any]]]:
        """Cluster obstacles and filter perimeter walls."""
        if not ranges or len(ranges) < 180:
            return [], []
        raw_clusters = cluster_lidar_obstacles(ranges, min_dist=Config.LIDAR_MIN_RANGE, max_dist=min(6.0, Config.MAX_RANGE))
        tracked = self.obstacle_tracker.update(raw_clusters, robot_pose=(pose_x, pose_y, pose_theta))

        env_mode = getattr(Config, "ENVIRONMENT_MODE", "ROOM")
        c_th, s_th = math.cos(pose_theta), math.sin(pose_theta)
        valid_clusters, out_list = [], []

        for cl in tracked:
            cw_x = round(pose_x + cl.centroid[0] * c_th - cl.centroid[1] * s_th, 3)
            cw_y = round(pose_y + cl.centroid[0] * s_th + cl.centroid[1] * c_th, 3)

            # In arena mode, only discard if far outside the arena boundary
            if env_mode == "ARENA_2X1":
                bounds = getattr(Config, "ARENA_BOUNDS_TUPLE", Config.ARENA_BOUNDS)
                if not (bounds[0] <= cw_x <= bounds[1] and bounds[2] <= cw_y <= bounds[3]):
                    continue

            # Classify clusters: Objects > 1.80m are classified as static boundaries/walls rather than dropped
            is_static_wall = (getattr(cl, "length_m", 0.0) > 1.80 or getattr(cl, "width_m", 0.0) > 1.80)
            if is_static_wall:
                setattr(cl, "is_static_boundary", True)

            valid_clusters.append(cl)
            cd = cl.to_dict()
            cd["is_wall"] = is_static_wall
            cd["centroid_world"] = [cw_x, cw_y]
            cd["bounding_box_world"] = [
                [
                    round(pose_x + px * c_th - py * s_th, 3),
                    round(pose_y + px * s_th + py * c_th, 3)
                ]
                for px, py in cl.bounding_box_2d
            ]
            out_list.append(cd)

        return valid_clusters, out_list

    def plan(self, start: Tuple[float, float], goal: Tuple[float, float],
             start_theta: float = 0.0, timeout: float = 3.0) -> Optional[List[Tuple[float, float]]]:
        """
        Run the A* path planner.

        Passes the real planner argument names (Phase 2.5): ``timeout_sec`` /
        ``max_iterations`` / ``start_theta`` used to be swallowed by ``**kwargs``, so the
        deadline was never applied (BUG-10).
        """
        try:
            return self.planner.plan(
                start,
                goal,
                smooth=True,
                max_time_sec=float(timeout),
                max_expansions=20000,
                robot_theta=start_theta,
            )
        except Exception:
            return None

    @property
    def last_plan_status(self) -> str:
        """Outcome of the most recent plan(): surfaced verbatim in the UI (Phase 2.6)."""
        return getattr(self.planner, "last_plan_status", "OK")

    @property
    def last_plan_reason(self) -> str:
        return getattr(self.planner, "last_plan_reason", "")

    def _extract_subgoal(self, x: float, y: float, lookahead: float = 1.0) -> Optional[Tuple[float, float]]:
        """Extracts a waypoint along current path roughly lookahead distance ahead."""
        path = getattr(self.controller, "current_path", None)
        if not path:
            return getattr(self.controller, "global_goal", None)
        closest_idx = 0
        min_d = 999.0
        for i, pt in enumerate(path):
            d = math.hypot(pt[0] - x, pt[1] - y)
            if d < min_d:
                min_d = d
                closest_idx = i

        target_idx = closest_idx
        for i in range(closest_idx, len(path)):
            d = math.hypot(path[i][0] - x, path[i][1] - y)
            if d >= lookahead:
                target_idx = i
                break
        else:
            target_idx = len(path) - 1

        return path[target_idx]

    def compute_command(self, x: float, y: float, theta: float,
                        ranges: List[float], clusters: List[Any],
                        auto_evade: bool = True) -> Tuple[float, float, Dict[str, Any]]:
        """Compute velocity command (v, w) via DAgger IL, MPPI, or Pure Pursuit."""
        if self.is_dagger_mode:
            subgoal = self._extract_subgoal(x, y)
            if subgoal is not None:
                gx, gy = getattr(self.controller, "global_goal", subgoal)
                dist_to_final = math.hypot(gx - x, gy - y)
                if dist_to_final < getattr(Config, "GOAL_TOLERANCE", 0.18):
                    self.controller.is_goal_reached = True
                    self.prev_v, self.prev_w = 0.0, 0.0
                    return 0.0, 0.0, {"reason": "GOAL_REACHED", "mode": "dagger_il", "is_goal_reached": True}

                lidar_72 = preprocess_lidar_72(ranges)
                state_77 = build_state_vector(
                    lidar_72=lidar_72,
                    robot_pose=(x, y, theta),
                    subgoal=subgoal,
                    v_prev=self.prev_v,
                    w_prev=self.prev_w,
                    max_v=Config.MAX_SPEED,
                    max_w=Config.MAX_ANGULAR_SPEED
                )
                v_nom, w_nom = self.dagger_policy.predict_action(state_77)
                v_safe, w_safe, cbf_info = self.cbf_filter.filter_control(v_nom, w_nom, raw_ranges=ranges)
                self.prev_v, self.prev_w = v_safe, w_safe

                debug_info = {
                    "mode": "dagger_il",
                    "subgoal": list(subgoal),
                    "dist_to_final": round(dist_to_final, 3),
                    "v_nom": v_nom,
                    "w_nom": w_nom,
                    "v_safe": v_safe,
                    "w_safe": w_safe,
                    "cbf": cbf_info
                }
                return v_safe, w_safe, debug_info

        # Default fallback to Pure Pursuit / MPPI
        return self.controller.compute_command(
            x, y, theta,
            lidar_ranges=ranges,
            clusters=clusters,
            grid_map=self.grid_map,
            auto_evade=auto_evade
        )
