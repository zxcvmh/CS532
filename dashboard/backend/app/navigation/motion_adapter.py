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
            inflation_radius=getattr(Config, "INFLATION_RADIUS", 0.045),
            arena_bounds=getattr(Config, "ARENA_BOUNDS", None)
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
        self._initial_scan_count: int = 0

    def reset(self):
        """Reset SLAM map, odometry, and path state."""
        self.grid_map.reset()
        self.anchor_initialized = False
        self.static_map_locked = False
        self._initial_scan_count = 0
        self.lidar_odometry.reset_pose(0.0, 0.0, 0.0)
        self.controller.clear_path()
        if hasattr(self, "cartographer") and self.cartographer:
            self.cartographer.reset()

    def set_pose(self, x: float, y: float, theta: float):
        """Sets odometry pose without resetting map or reference keyframes."""
        if hasattr(self.lidar_odometry, "set_pose"):
            self.lidar_odometry.set_pose(x, y, theta)
        if hasattr(self, "cartographer") and self.cartographer:
            self.cartographer.current_pose = (float(x), float(y), float(theta))

    def update_odometry(self, ranges: List[float], dt: float,
                        v_cmd: float, w_cmd: float) -> Tuple[float, float, float, Dict[str, Any]]:
        """Update 2D LiDAR scan matching odometry via Hector Scan-to-Map and Dual-Layer Anchor Locking."""
        # 1. Update pose via Hector SLAM Scan-to-Map
        ox, oy, oth, info = self.lidar_odometry.update(
            ranges, dt=dt, v_cmd=v_cmd, w_cmd=w_cmd, grid_map=self.grid_map
        )

        is_spinning = (abs(w_cmd) > 0.05 and abs(v_cmd) < 0.02)

        # 2. Track scan count for initial pristine anchor locking
        if not is_spinning and not self.anchor_initialized:
            self._initial_scan_count = getattr(self, "_initial_scan_count", 0) + 1
            if self._initial_scan_count >= 5:
                # Lock physical static walls from initial sweeps
                locked = self.grid_map.set_anchor_from_scan(
                    ranges=ranges,
                    robot_x=ox,
                    robot_y=oy,
                    robot_theta=oth,
                    max_range=min(6.0, Config.MAX_RANGE)
                )
                if locked:
                    self.anchor_initialized = True
                    self.static_map_locked = True

        # 3. Update occupancy grid with current scan in stationary World Frame
        if not is_spinning:
            self.grid_map.update_scan(
                ox, oy, oth, ranges=ranges, max_valid_range=min(6.0, Config.MAX_RANGE), defer_costmap=False
            )

        # Sync pose to cartographer adapter for backward compatibility if needed
        if hasattr(self, "cartographer") and self.cartographer:
            self.cartographer.current_pose = (ox, oy, oth)

        return ox, oy, oth, info

    def to_dashboard_data(self) -> Dict[str, Any]:
        """Produce JSON-compatible int8 occupancy grid map via OccupancyGridMap."""
        m_data = self.grid_map.to_dashboard_data()
        return {
            "width": self.grid_map.width,
            "height": self.grid_map.height,
            "resolution": self.grid_map.resolution,
            "origin_x": self.grid_map.origin_x,
            "origin_y": self.grid_map.origin_y,
            "originX": self.grid_map.origin_x,
            "originY": self.grid_map.origin_y,
            "data": m_data
        }

    def to_dashboard_b64(self) -> str:
        """Produce Base64-encoded int8 occupancy grid stream via OccupancyGridMap."""
        return self.grid_map.to_dashboard_b64()

    def to_dashboard_bytes(self) -> bytes:
        """Produce raw int8 bytes via OccupancyGridMap."""
        return self.grid_map.to_dashboard_bytes()

    def clear_footprint(self, x: float, y: float, radius: float = 0.09, **kwargs):
        """Clear footprint around robot center in GridMap."""
        if hasattr(self.grid_map, "clear_footprint"):
            self.grid_map.clear_footprint(x, y, radius=radius, defer_costmap=True)
        if hasattr(self, "cartographer") and self.cartographer:
            self.cartographer.clear_footprint(x, y, radius=radius)

    def insert_obstacle_clusters(self, clusters: List[Any], robot_x: float, robot_y: float, robot_th: float, defer_costmap: bool = False):
        """Ingest dynamic obstacle clusters into decoupled costmap layer."""
        if hasattr(self.grid_map, "insert_obstacle_clusters"):
            self.grid_map.insert_obstacle_clusters(clusters, robot_x=robot_x, robot_y=robot_y, robot_theta=robot_th, defer_costmap=defer_costmap)
        if hasattr(self, "cartographer") and self.cartographer:
            self.cartographer.insert_obstacle_clusters(clusters, robot_x, robot_y, robot_th, defer_costmap=defer_costmap)

    def insert_low_obstacles(self, low_obstacles: List[Any], robot_x: float, robot_y: float, robot_th: float, defer_costmap: bool = False, ttl_sec: Optional[float] = None):
        """Ingest floor IPM low obstacles into short-TTL costmap layer."""
        if hasattr(self.grid_map, "insert_low_obstacles"):
            self.grid_map.insert_low_obstacles(low_obstacles, robot_x=robot_x, robot_y=robot_y, robot_theta=robot_th, defer_costmap=defer_costmap, ttl_sec=ttl_sec)
        if hasattr(self, "cartographer") and self.cartographer:
            self.cartographer.insert_low_obstacles(low_obstacles, robot_x, robot_y, robot_th, defer_costmap=defer_costmap, ttl_sec=ttl_sec)

    def add_social_bubble(self, center_x: float, center_y: float, radius_m: float = 0.40, core_radius_m: float = 0.20):
        """Add Hall's Proxemics Social Safety Bubble into decoupled costmap layer."""
        if hasattr(self.grid_map, "add_social_bubble"):
            self.grid_map.add_social_bubble(center_x, center_y, radius_m=radius_m, core_radius_m=core_radius_m)
        if hasattr(self, "cartographer") and self.cartographer:
            self.cartographer.add_social_bubble(center_x, center_y, radius_m=radius_m, core_radius_m=core_radius_m)

    def update_social_bubbles_from_detections(self, detections: list, robot_x: float, robot_y: float, robot_theta: float, bubble_radius_m: float = 0.40):
        """Update person social safety bubbles from YOLO detections."""
        if hasattr(self.grid_map, "update_social_bubbles_from_detections"):
            self.grid_map.update_social_bubbles_from_detections(detections, robot_x, robot_y, robot_theta, bubble_radius_m=bubble_radius_m)
        self.grid_map.update_costmap()

    def update_costmap(self):
        """Recalculate costmap directly on active OccupancyGridMap."""
        self.grid_map.update_costmap()


    def process_clusters(self, ranges: List[float], pose_x: float, pose_y: float,
                         pose_theta: float) -> Tuple[List[Any], List[Dict[str, Any]]]:
        """Cluster obstacles and filter perimeter walls."""
        if not ranges or len(ranges) < 180:
            return [], []
        raw_clusters = cluster_lidar_obstacles(ranges, min_dist=Config.LIDAR_MIN_RANGE, max_dist=min(6.0, Config.MAX_RANGE))
        env_mode = getattr(Config, "ENVIRONMENT_MODE", "ROOM")
        c_th, s_th = math.cos(pose_theta), math.sin(pose_theta)
        la = getattr(Config, "LIDAR_LEVER_ARM_M", 0.01)
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
