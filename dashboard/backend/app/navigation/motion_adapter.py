"""
Motion Adapter Module.
Encapsulates single-source-of-truth motion planning, odometry, and control from motion/planning.
"""
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
from app.navigation.cartographer_adapter import CartographerAdapter


class MotionAdapter:
    """Interface adapter for A*, Pure Pursuit, MPPI, and Cartographer 2D SLAM."""

    def __init__(self):
        self.cartographer = CartographerAdapter(
            resolution=Config.MAP_RESOLUTION,
            width=Config.MAP_WIDTH,
            height=Config.MAP_HEIGHT,
            robot_radius=Config.ROBOT_RADIUS,
            inflation_radius=getattr(Config, "INFLATION_RADIUS", 0.045)
        )

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

    def reset(self):
        """Reset SLAM map, odometry, and path state."""
        self.cartographer.reset()
        self.grid_map.reset()
        self.anchor_initialized = False
        self.static_map_locked = False
        self.lidar_odometry.reset_pose(0.0, 0.0, 0.0)
        self.controller.clear_path()

    def set_pose(self, x: float, y: float, theta: float):
        """Sets odometry pose without resetting map or reference keyframes."""
        self.cartographer.current_pose = (float(x), float(y), float(theta))
        if hasattr(self.lidar_odometry, "set_pose"):
            self.lidar_odometry.set_pose(x, y, theta)

    def update_odometry(self, ranges: List[float], dt: float,
                        v_cmd: float, w_cmd: float) -> Tuple[float, float, float, Dict[str, Any]]:
        """Update 2D LiDAR scan matching odometry via Google Cartographer 2D Core."""
        ox, oy, oth, info = self.cartographer.update_scan(ranges, dt=dt, v_cmd=v_cmd, w_cmd=w_cmd)
        self.cartographer.update_costmap()
        self.grid_map.costmap = self.cartographer.costmap
        self.grid_map.dist_m = self.cartographer.dist_m
        return ox, oy, oth, info

    def to_dashboard_data(self) -> Dict[str, Any]:
        """Produce JSON-compatible int8 occupancy grid map via Cartographer."""
        return self.cartographer.to_dashboard_data()

    def to_dashboard_b64(self) -> str:
        """Produce Base64-encoded int8 occupancy grid stream via Cartographer."""
        return self.cartographer.to_dashboard_b64()


    def process_clusters(self, ranges: List[float], pose_x: float, pose_y: float,
                         pose_theta: float) -> Tuple[List[Any], List[Dict[str, Any]]]:
        """Cluster obstacles and filter perimeter walls."""
        if not ranges or len(ranges) < 180:
            return [], []
        raw_clusters = cluster_lidar_obstacles(ranges, min_dist=Config.LIDAR_MIN_RANGE, max_dist=min(6.0, Config.MAX_RANGE))
        env_mode = getattr(Config, "ENVIRONMENT_MODE", "ROOM")
        c_th, s_th = math.cos(pose_theta), math.sin(pose_theta)
        la = 0.0
        lx = pose_x + la * c_th
        ly = pose_y + la * s_th
        tracked = self.obstacle_tracker.update(raw_clusters, robot_pose=(lx, ly, pose_theta))

        valid_clusters, out_list = [], []

        for cl in tracked:
            cw_x = round(lx + cl.centroid[0] * c_th - cl.centroid[1] * s_th, 3)
            cw_y = round(ly + cl.centroid[0] * s_th + cl.centroid[1] * c_th, 3)

            # In arena mode, only discard if far outside the arena boundary
            if env_mode == "ARENA_2X1":
                bounds = getattr(Config, "ARENA_BOUNDS_TUPLE", Config.ARENA_BOUNDS)
                if not (bounds[0] <= cw_x <= bounds[1] and bounds[2] <= cw_y <= bounds[3]):
                    continue

            # Classify clusters: check if cluster is WALL_SURFACE, falls on/near static map wall, or is large structure
            is_wall_type = (getattr(cl, "obstacle_type", "") == "WALL_SURFACE")

            col, row = self.grid_map.grid_core.world_to_grid(cw_x, cw_y)
            is_wall_cell = False
            for dr in (-1, 0, 1):
                for dc in (-1, 0, 1):
                    nr, nc = row + dr, col + dc
                    if self.grid_map.grid_core.is_inside(nc, nr):
                        if self.grid_map.grid_core.log_odds[nr, nc] > 0.6:
                            is_wall_cell = True
                            break
                if is_wall_cell:
                    break

            is_large_structure = (getattr(cl, "length_m", 0.0) > 0.80 or getattr(cl, "width_m", 0.0) > 0.80)
            is_static_wall = is_wall_type or is_wall_cell or is_large_structure

            if is_static_wall:
                setattr(cl, "is_static_boundary", True)
                setattr(cl, "is_wall", True)
                # Filter out perimeter walls and map boundaries from dynamic obstacle bounding box list
                continue

            valid_clusters.append(cl)
            cd = cl.to_dict()
            cd["is_wall"] = False
            cd["centroid_world"] = [cw_x, cw_y]
            cd["bounding_box_world"] = [
                [
                    round(lx + px * c_th - py * s_th, 3),
                    round(ly + px * s_th + py * c_th, 3)
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

    def compute_command(self, x: float, y: float, theta: float,
                        ranges: List[float], clusters: List[Any],
                        auto_evade: bool = True) -> Tuple[float, float, Dict[str, Any]]:
        """Compute velocity command (v, w) via Pure Pursuit / MPPI."""
        return self.controller.compute_command(
            x, y, theta,
            lidar_ranges=ranges,
            clusters=clusters,
            grid_map=self.grid_map,
            auto_evade=auto_evade
        )
