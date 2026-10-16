import os
import asyncio
import math
import numpy as np
import random
import time
import struct
import io
import base64
from typing import List, Tuple
from app.models.schemas import (
    RobotPose, RobotStatus, LidarScan, LidarPoint, SensorHealth,
    MapData, NavigationStatus, PlannedPath, DetectionList, Detection,
    ModeEnum, NavigationStatusEnum, EventLog
)
from app.robot.controller import (
    RobotController, SensorProvider, MapProvider, NavigationManager, DetectionProvider
)
from app.config import Config
from app.planning.occupancy_grid import OccupancyGridMap
from app.planning.astar_planner import AStarPlanner
from app.planning.controller import PurePursuitController
from app.planning.mppi_controller import MPPIController
from app.planning.obstacle_clustering import cluster_lidar_obstacles, DynamicObstacleTracker
from app.planning.lidar_odometry import Lidar2DOdometry
from app.cv.floor_ipm import FloorIPMTransformer


def line_intersection(p1, p2, p3, p4):
    """Find intersection of two line segments."""
    x1, y1 = p1
    x2, y2 = p2
    x3, y3 = p3
    x4, y4 = p4
    den = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
    if abs(den) < 1e-10:
        return None
    t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / den
    u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / den
    if 0 <= t <= 1 and 0 <= u <= 1:
        return (x1 + t * (x2 - x1), y1 + t * (y2 - y1))
    return None


def circle_intersection(p1, p2, cx, cy, r):
    """Find nearest intersection of line segment with circle."""
    x1, y1 = p1
    x2, y2 = p2
    dx = x2 - x1
    dy = y2 - y1
    a = dx**2 + dy**2
    b = 2 * (dx * (x1 - cx) + dy * (y1 - cy))
    c = (x1 - cx)**2 + (y1 - cy)**2 - r**2
    det = b**2 - 4 * a * c
    if det < 0 or a == 0:
        return None
    t1 = (-b + math.sqrt(det)) / (2 * a)
    t2 = (-b - math.sqrt(det)) / (2 * a)
    intersections = []
    if 0 <= t1 <= 1:
        intersections.append((x1 + t1 * dx, y1 + t1 * dy))
    if 0 <= t2 <= 1:
        intersections.append((x1 + t2 * dx, y1 + t2 * dy))
    if not intersections:
        return None
    return min(intersections, key=lambda p: (p[0] - x1)**2 + (p[1] - y1)**2)


def _create_minimal_jpeg(width: int = 640, height: int = 480) -> bytes:
    """Create a simulated camera feed JPEG frame with grid pattern and crosshairs.
    Tier 1: Load bundled default_camera.jpg asset (11 KB 640x480 JPEG)
    Tier 2: Generate via PIL (Pillow)
    Tier 3: Generate via OpenCV (cv2, preinstalled on Jetson Nano)
    """
    # Tier 1: Check bundled asset file
    asset_path = os.path.join(os.path.dirname(__file__), "default_camera.jpg")
    if os.path.exists(asset_path):
        try:
            with open(asset_path, "rb") as f:
                return f.read()
        except Exception:
            pass

    # Tier 2: Try PIL
    try:
        from PIL import Image, ImageDraw
        img = Image.new('RGB', (width, height), color=(15, 18, 25))
        draw = ImageDraw.Draw(img)
        for x in range(0, width, 40):
            draw.line([(x, 0), (x, height)], fill=(25, 30, 42), width=1)
        for y in range(0, height, 40):
            draw.line([(0, y), (width, y)], fill=(25, 30, 42), width=1)
        cx, cy = width // 2, height // 2
        draw.line([(cx - 15, cy), (cx + 15, cy)], fill=(0, 212, 255), width=1)
        draw.line([(cx, cy - 15), (cx, cy + 15)], fill=(0, 212, 255), width=1)
        buf = io.BytesIO()
        img.save(buf, format='JPEG', quality=60)
        return buf.getvalue()
    except Exception:
        pass

    # Tier 3: Try OpenCV (preinstalled on Jetson Nano / JetPack 4.5)
    try:
        import cv2
        import numpy as np
        img = np.zeros((height, width, 3), dtype=np.uint8)
        img[:] = (25, 18, 15)  # Dark BGR
        for x in range(0, width, 40):
            cv2.line(img, (x, 0), (x, height), (42, 30, 25), 1)
        for y in range(0, height, 40):
            cv2.line(img, (0, y), (width, y), (42, 30, 25), 1)
        cx, cy = width // 2, height // 2
        cv2.line(img, (cx - 15, cy), (cx + 15, cy), (255, 212, 0), 1)
        cv2.line(img, (cx, cy - 15), (cx, cy + 15), (255, 212, 0), 1)
        cv2.putText(img, "SIMULATION CAMERA FEED", (cx - 110, cy - 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 212, 255), 1)
        _, buf = cv2.imencode('.jpg', img, [int(cv2.IMWRITE_JPEG_QUALITY), 65])
        return buf.tobytes()
    except Exception:
        pass

    # Fallback to 1x1 valid JPEG if all else fails
    return b'\xff\xd8\xff\xe0\x00\x10JFIF\x00\x01\x01\x01\x00H\x00H\x00\x00\xff\xdb\x00C\x00\x08\x06\x06\x07\x06\x05\x08\x07\x07\x07\t\t\x08\n\x0c\x14\r\x0c\x0b\x0b\x0c\x19\x12\x13\x0f\x14\x1d\x1a\x1f\x1e\x1d\x1a\x1c\x1c $.\' \",#\x1c\x1c(7),01444\x1f\'9=82<.342\xff\xc0\x00\x0b\x08\x00\x01\x00\x01\x01\x01\x11\x00\xff\xc4\x00\x1f\x00\x00\x01\x05\x01\x01\x01\x01\x01\x01\x00\x00\x00\x00\x00\x00\x00\x00\x01\x02\x03\x04\x05\x06\x07\x08\t\n\x0b\xff\xda\x00\x08\x01\x01\x00\x00?\x00\xbf\x00\xff\xd9'


async def _async_run_in_thread(func, *args):
    """Compatible thread runner for Python 3.6 (JetPack 4.5) through Python 3.14"""
    if hasattr(asyncio, "to_thread"):
        return await asyncio.to_thread(func, *args)
    loop = asyncio.get_event_loop()
    return await loop.run_in_executor(None, func, *args)


class MockRobot(RobotController, SensorProvider, MapProvider, NavigationManager, DetectionProvider):
    def __init__(self):
        self.pose = RobotPose(x=0.0, y=0.0, theta=0.0)
        self.battery = 85.0
        self.mode = ModeEnum.MANUAL
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        
        # CS532 Planning & Control Architecture
        self.grid_map = OccupancyGridMap(
            width=Config.MAP_WIDTH,
            height=Config.MAP_HEIGHT,
            resolution=Config.MAP_RESOLUTION,
            origin_x=Config.MAP_ORIGIN_X,
            origin_y=Config.MAP_ORIGIN_Y
        )
        self._anchor_initialized = False
        self._static_map_locked = False
        self.planner = AStarPlanner(self.grid_map)
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
                reflex_dist_threshold=0.16
            )
        else:
            self.controller = PurePursuitController(
                lookahead_dist=0.28,
                max_linear_speed=Config.MAX_SPEED,
                max_angular_speed=Config.MAX_ANGULAR_SPEED,
                track_width=0.115,
                goal_tolerance=0.18,
                reflex_dist_threshold=0.16,
                warning_dist_threshold=0.55,
                curvature_scale_k=1.2,
                lateral_detour_offset=0.22,
                min_drive_speed=Config.MIN_DRIVE_SPEED,
                trim_factor=1.015,
                soft_start_ramp_sec=0.4,
                env_mode=getattr(Config, "ENVIRONMENT_MODE", "ARENA_2X1"),
                tactical_yield_dist=getattr(Config, "TACTICAL_YIELD_DIST", 0.30),
                passage_clearance_min=getattr(Config, "PASSAGE_CLEARANCE_MIN", 0.18)
            )
        self.obstacle_tracker = DynamicObstacleTracker(assoc_dist_m=0.45)
        self.lidar_odometry = Lidar2DOdometry(
            min_range_m=0.15,
            max_range_m=6.0,
            decimation_step=2
        )
        self.floor_ipm = FloorIPMTransformer()
        self.latest_raw_ranges = [0.0] * 360
        self.latest_clusters = []
        self.latest_clusters_dict = []
        self.latest_low_obstacles = []
        self.latest_social_bubbles = []
        self.motion_xai_info = {
            "status": "IDLE",
            "speed_scale": 1.0,
            "curvature_w": 0.0,
            "evasion_active": False,
            "evasion_reason": "",
            "lookahead_target": None
        }
        self._last_evasion_active = False
        self._last_curv_log_time = 0.0
        self._last_map_update_time = 0.0
        
        self.map_data = [-1] * (Config.MAP_WIDTH * Config.MAP_HEIGHT)
        self.trajectory: List[LidarPoint] = []
        self.last_traj_time = time.time()
        
        self.nav_status = NavigationStatus(status=NavigationStatusEnum.IDLE)
        self.planned_path = PlannedPath(points=[])
        self._initial_goal_distance = 0.0
        
        self.current_scan = LidarScan(points=[])
        self.current_detections = DetectionList(objects=[])
        self.events: List[EventLog] = []
        
        # Mock objects for simulation (disabled by default to prevent hallucinating fake people)
        self.mock_objects = []
        
        self._dt = 1.0 / Config.ROBOT_UPDATE_RATE
        self.running = False
        self.start_time = time.time()
        
        # Pre-generate camera frame
        self._camera_frame = _create_minimal_jpeg()
        
        # Real Hardware Bridge
        self.is_real_connected = False
        self.real_ws = None
        self._last_real_data_time = 0.0
        self._last_lidar_time = 0.0
        self._last_camera_time = 0.0
        self._last_yolo_time = 0.0
        self._last_battery_time = 0.0
        self._lidar_connected = False
        self._camera_connected = False
        self._yolo_connected = False
        self._battery_connected = False

        # Add startup events
        self._add_event("INFO", "System initialized")
        self._add_event("SUCCESS", "Camera connected")
        self._add_event("SUCCESS", "LiDAR connected")
        self._add_event("INFO", "SLAM started")

    def _add_event(self, level: str, message: str):
        """Add an event with human-readable timestamp."""
        ts = time.strftime("%H:%M:%S", time.localtime())
        self.events.append(EventLog(timestamp=ts, level=level, message=message))

    async def send_hardware_cmd_vel(self, linear: float, angular: float):
        """Send motor velocities to real JetBot over WebSocket"""
        if self.real_ws and self.is_real_connected:
            try:
                await self.real_ws.send_json({
                    "type": "cmd_vel",
                    "linear": round(linear, 3),
                    "angular": round(angular, 3)
                })
            except Exception:
                pass

    async def send_hardware_stop(self):
        """Send stop to real JetBot over WebSocket"""
        if self.real_ws and self.is_real_connected:
            try:
                await self.real_ws.send_json({"type": "stop"})
            except Exception:
                pass

    def _process_lidar_clusters(self, ranges: List[float]):
        """Runs 1D Range Jump O(N) obstacle clustering & dynamic threat tracking."""
        if not ranges or len(ranges) < 180:
            return
        clusters = cluster_lidar_obstacles(ranges, min_dist=0.15, max_dist=min(6.0, Config.MAX_RANGE))
        tracked = self.obstacle_tracker.update(
            clusters,
            robot_pose=(self.pose.x, self.pose.y, self.pose.theta)
        )
        bounds = getattr(Config, "ARENA_BOUNDS_TUPLE", (-0.1, 2.1, -0.6, 0.6))
        wall_margin = 0.15
        x_lo = bounds[0] + wall_margin
        x_hi = bounds[1] - wall_margin
        y_lo = bounds[2] + wall_margin
        y_hi = bounds[3] - wall_margin

        c_th = math.cos(self.pose.theta)
        s_th = math.sin(self.pose.theta)
        valid_clusters = []
        out_list = []
        for cl in tracked:
            cw_x = round(self.pose.x + cl.centroid[0] * c_th - cl.centroid[1] * s_th, 3)
            cw_y = round(self.pose.y + cl.centroid[0] * s_th + cl.centroid[1] * c_th, 3)

            # Strictly ignore anything outside or within 15cm of perimeter walls
            if not (x_lo <= cw_x <= x_hi and y_lo <= cw_y <= y_hi):
                continue

            # Filter out wall-like structures
            if getattr(cl, "length_m", 0.0) > 0.45 or getattr(cl, "width_m", 0.0) > 0.45:
                continue

            valid_clusters.append(cl)
            cd = cl.to_dict()
            cd["centroid_world"] = [cw_x, cw_y]
            cd["bounding_box_world"] = [
                [
                    round(self.pose.x + px * c_th - py * s_th, 3),
                    round(self.pose.y + px * s_th + py * c_th, 3)
                ]
                for px, py in cl.bounding_box_2d
            ]
            out_list.append(cd)

        self.latest_clusters = valid_clusters
        self.latest_clusters_dict = out_list

    async def get_motion_xai(self) -> dict:
        """Returns explainable motion and obstacle metrics for Web Dashboard."""
        return {
            "status": self.motion_xai_info.get("status", "IDLE"),
            "speed_scale": self.motion_xai_info.get("speed_scale", 1.0),
            "curvature_w": self.motion_xai_info.get("curvature_w", 0.0),
            "evasion_active": self.motion_xai_info.get("evasion_active", False),
            "evasion_reason": self.motion_xai_info.get("evasion_reason", ""),
            "lookahead_target": self.motion_xai_info.get("lookahead_target", None),
            "clusters": getattr(self, "latest_clusters_dict", []),
            "low_obstacles": getattr(self, "latest_low_obstacles", []),
            "social_bubbles": getattr(self, "latest_social_bubbles", [])
        }

    async def reset_map(self):
        """Clear map data and trajectory completely, reset origin, and re-scan environment."""
        self.pose = RobotPose(x=0.0, y=0.0, theta=0.0)
        self.grid_map.reset()
        self._anchor_initialized = False
        self._static_map_locked = False
        self.map_data = self.grid_map.to_dashboard_data()
        self.map_version = getattr(self, "map_version", 0) + 1
        self.trajectory.clear()
        if hasattr(self, "lidar_odometry"):
            self.lidar_odometry.reset_pose(0.0, 0.0, 0.0)
            self._last_odom_scan_time = time.time()
        self.current_scan = LidarScan(points=[])
        self.latest_clusters = []
        self.latest_clusters_dict = []
        self.latest_low_obstacles = []
        self.latest_social_bubbles = []
        self.controller.clear_path()
        self._stop_start_time = 0.0
        self._replan_attempts = 0
        self._static_map_scans = 0
        self.motion_xai_info = {
            "status": "IDLE",
            "speed_scale": 1.0,
            "curvature_w": 0.0,
            "evasion_active": False,
            "evasion_reason": "",
            "lookahead_target": None
        }
        self._add_event("WARNING", "🧹 Map & Origin cleared! LiDAR rescanning 4-wall arena...")
        print("[SLAM] >>> 🧹 Map & Origin cleared by user. Rescanning static arena...", flush=True)

    async def reset_pose(self):
        """Reset robot pose to origin (0, 0, 0) and unlock static map re-anchor"""
        self.pose = RobotPose(x=0.0, y=0.0, theta=0.0)
        self.trajectory.clear()
        if hasattr(self, "lidar_odometry"):
            self.lidar_odometry.reset_pose(0.0, 0.0, 0.0)
            self._last_odom_scan_time = time.time()
        self.grid_map.clear_footprint(0.0, 0.0, radius=0.18)
        self.controller.clear_path()
        self._stop_start_time = 0.0
        self._replan_attempts = 0
        self._static_map_locked = False
        self._static_map_scans = 0
        self.map_version = getattr(self, "map_version", 0) + 1
        self._add_event("INFO", "🎯 Robot pose reset to (0, 0, 0). SLAM anchor re-calibrated.")
        print("[SLAM] >>> 🎯 Robot pose reset to (0, 0, 0). Ready for fresh scanning.", flush=True)

    async def handle_real_telemetry(self, data: dict):
        """Processes real Camera + LiDAR + Battery from physical JetBot"""
        # Wipe fake simulation walls on first real telemetry packet
        if not getattr(self, '_has_reset_real_map', False):
            self.grid_map.reset()
            self.map_data = self.grid_map.to_dashboard_data()
            self.trajectory.clear()
            if hasattr(self, "lidar_odometry"):
                self.lidar_odometry.reset_pose(self.pose.x, self.pose.y, self.pose.theta)
                self._last_odom_scan_time = time.time()
            self._has_reset_real_map = True
            self._add_event("SUCCESS", "Cleared simulated room. Mapping Real Environment!")

        self.is_real_connected = True
        self._last_real_data_time = time.time()

        # 1. Real Camera frame (WebSocket video bloat eliminated; native HTTP MJPEG on port 8080)
        cam_b64 = data.get("camera")
        if cam_b64 and len(cam_b64) > 100:
            try:
                self._camera_frame = base64.b64decode(cam_b64)
                self._last_camera_time = time.time()
                self._camera_connected = True
            except Exception:
                pass
        elif self.is_real_connected:
            self._camera_connected = True
            self._last_camera_time = time.time()

        # 2. Real Battery
        bat = data.get("battery")
        if bat is not None:
            try:
                self.battery = float(bat)
                self._last_battery_time = time.time()
                self._battery_connected = True
            except (ValueError, TypeError):
                pass

        # 3. Real 2D LiDAR Scan (Clean Non-blocking Vectorized Mapping)
        raw_lidar = data.get("lidar")
        if raw_lidar and isinstance(raw_lidar, list) and len(raw_lidar) > 15:
            self._last_lidar_time = time.time()
            self._lidar_connected = True
            ranges = [0.0] * 360

            for item in raw_lidar:
                if len(item) >= 2:
                    ang_deg = float(item[0])
                    dist = float(item[1])

                    # Filter noise: ignore reflections closer than 15cm (chassis) or out of range
                    if dist < 0.15 or dist > Config.MAX_RANGE:
                        continue

                    ranges[int(round(ang_deg)) % 360] = dist

            # 2D LiDAR Scan Matching Odometry (Triệt tiêu Wheel Slip Dead-Reckoning Drift)
            now_t = time.time()
            if hasattr(self, "lidar_odometry"):
                dt_scan = max(0.01, min(0.3, now_t - getattr(self, '_last_odom_scan_time', now_t - 0.1)))
                self._last_odom_scan_time = now_t
                ox, oy, oth, odom_diag = self.lidar_odometry.update(
                    ranges,
                    dt=dt_scan,
                    v_cmd=getattr(self, "linear_vel", 0.0),
                    w_cmd=getattr(self, "angular_vel", 0.0),
                    grid_map=self.grid_map
                )
                # Luôn luôn cập nhật pose: nếu ICP tin cậy thì dùng ICP, nếu ICP từ chối thì dùng Dead-Reckoning Kinematic Fallback
                # Tuyệt đối KHÔNG ĐƯỢC bỏ qua cập nhật khiến pose bị đóng băng khi robot xoay trên sàn trơn!
                self.pose.x = ox
                self.pose.y = oy
                self.pose.theta = oth

            # Transform raw ranges to global point cloud with corrected pose
            scan_points = []
            for ang_deg, dist in enumerate(ranges):
                if 0.15 <= dist <= Config.MAX_RANGE:
                    rad = math.radians(ang_deg) + self.pose.theta
                    hit_x = self.pose.x + dist * math.cos(rad)
                    hit_y = self.pose.y + dist * math.sin(rad)
                    scan_points.append(LidarPoint(x=hit_x, y=hit_y))

            # Store latest raw ranges for 10Hz Safety Reflex in controller.py
            self.latest_raw_ranges = ranges
            
            # Process 1D Range Jump O(N) Obstacle Clusters & Threat Levels
            self._process_lidar_clusters(ranges)

            if scan_points:
                self.current_scan = LidarScan(points=scan_points)

            # STEP 2: Capture the first real 360° LiDAR scan as pristine Anchor Wall Map
            if not getattr(self, "_anchor_initialized", False):
                valid_ranges = [r for r in ranges if 0.12 <= r <= Config.MAX_RANGE]
                if len(valid_ranges) >= 15:
                    locked = self.grid_map.set_anchor_from_scan(
                        ranges, self.pose.x, self.pose.y, self.pose.theta, max_range=Config.MAX_RANGE
                    )
                    if locked:
                        self._anchor_initialized = True
                        self._static_map_locked = True
                        self.map_data = self.grid_map.to_dashboard_data()
                        self.map_version = getattr(self, "map_version", 0) + 1
                        self._add_event("SUCCESS", f"🔒 Real LiDAR Anchor Map Locked ({len(valid_ranges)} real wall hits)!")
                        print(f"[SLAM] >>> 🔒 Real LiDAR Anchor Map Locked ({len(valid_ranges)} real wall hits)!", flush=True)

            # Clean 5Hz Map Update: updates rolling obstacle buffer inside floor while walls remain locked
            if (now_t - getattr(self, '_last_map_update_time', 0.0) >= 0.20):
                self.grid_map.update_scan(
                    self.pose.x, self.pose.y, self.pose.theta,
                    ranges=ranges,
                    max_valid_range=Config.MAX_RANGE - 0.15
                )
                if hasattr(self, "latest_clusters") and self.latest_clusters:
                    self.grid_map.insert_obstacle_clusters(
                        self.latest_clusters, self.pose.x, self.pose.y, self.pose.theta
                    )
                self.grid_map.clear_footprint(self.pose.x, self.pose.y, radius=0.15)
                self.map_data = self.grid_map.to_dashboard_data()
                self.map_version = getattr(self, "map_version", 0) + 1
                self._last_map_update_time = now_t
        else:
            if time.time() - getattr(self, '_last_lidar_time', 0.0) > 2.0:
                self._lidar_connected = False
                self.current_scan = LidarScan(points=[])

        # 4. Real YOLO Detections from JetBot
        raw_dets = data.get("detections")
        yolo_active = data.get("yolo_active", False)
        if raw_dets or yolo_active:
            self._last_yolo_time = time.time()
            self._yolo_connected = True
        else:
            if time.time() - getattr(self, '_last_yolo_time', 0.0) > 3.0:
                self._yolo_connected = False

        if raw_dets is not None and isinstance(raw_dets, list):
            obj_list = []
            for d in raw_dets:
                obj_list.append(Detection(
                    class_name=d.get("class_name") or d.get("class", "object"),
                    confidence=float(d.get("confidence", 0.0)),
                    bbox=d.get("bbox", [0, 0, 0, 0]),
                    distance=float(d.get("distance", 0.0)),
                    azimuth_deg=float(d.get("azimuth_deg", 0.0))
                ))
            self.current_detections = DetectionList(objects=obj_list)
            # TẦNG 1: Project Social Cost Bubbles (0.40m) onto 2D Occupancy Grid Map
            self.grid_map.update_social_bubbles_from_detections(
                detections=raw_dets,
                robot_x=self.pose.x,
                robot_y=self.pose.y,
                robot_theta=self.pose.theta,
                bubble_radius_m=0.40
            )
            self.latest_social_bubbles = [
                {"x": round(b[0], 3), "y": round(b[1], 3), "radius": round(b[2], 2)}
                for b in getattr(self.grid_map, "social_bubbles", [])
            ]
            self.map_data = self.grid_map.to_dashboard_data()

        # 5. CV Floor IPM Low-Obstacles: DISABLED to prevent glossy tile floor reflections from generating phantom obstacles
        self.latest_low_obstacles = []

    # ─── RobotController ───

    async def get_pose(self) -> RobotPose:
        return self.pose

    async def get_battery(self) -> float:
        return self.battery

    async def set_velocity(self, linear: float, angular: float):
        if self.mode == ModeEnum.MANUAL:
            self.linear_vel = max(-Config.MAX_SPEED, min(Config.MAX_SPEED, linear))
            self.angular_vel = max(-2.5, min(2.5, angular))
            await self.send_hardware_cmd_vel(self.linear_vel, self.angular_vel)

    async def stop(self):
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        await self.send_hardware_stop()

    async def emergency_stop(self):
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        await self.send_hardware_stop()
        if self.nav_status.status in (NavigationStatusEnum.NAVIGATING, NavigationStatusEnum.PLANNING):
            self.nav_status.status = NavigationStatusEnum.ERROR
        self._add_event("ERROR", "EMERGENCY STOP activated")

    async def set_mode(self, mode: str):
        self.mode = ModeEnum(mode)
        if self.mode == ModeEnum.MANUAL:
            if self.nav_status.status in (NavigationStatusEnum.NAVIGATING, NavigationStatusEnum.PLANNING):
                self.nav_status.status = NavigationStatusEnum.CANCELLED
                self.linear_vel = 0.0
                self.angular_vel = 0.0
        self._add_event("INFO", f"Mode changed to {mode}")

    async def get_mode(self) -> str:
        return self.mode.value

    async def get_status(self) -> RobotStatus:
        return RobotStatus(
            connected=True,
            battery=self.battery,
            mode=self.mode,
            pose=self.pose,
            linear_vel=self.linear_vel,
            angular_vel=self.angular_vel,
            sensors=await self.get_sensor_health()
        )

    # ─── SensorProvider ───

    async def get_lidar_scan(self) -> LidarScan:
        return self.current_scan

    async def get_camera_frame(self) -> bytes:
        return self._camera_frame

    async def get_sensor_health(self) -> List[SensorHealth]:
        now = time.time()
        
        # 1. Camera: ACTIVE only if real camera frame received within 2s
        cam_active = (now - getattr(self, "_last_camera_time", 0.0) < 2.0) and getattr(self, "_camera_connected", False)
        camera_status = "ACTIVE" if cam_active else "OFFLINE"
        
        # 2. LiDAR: ACTIVE only if real scan received with >15 valid points within 2s
        lidar_active = (now - getattr(self, "_last_lidar_time", 0.0) < 2.0) and getattr(self, "_lidar_connected", False)
        lidar_status = "ACTIVE" if lidar_active else "OFFLINE"
        
        # 3. IMU: OFFLINE unless real IMU data stream is present
        imu_status = "OFFLINE"
        
        # 4. Odometry: ACTIVE if robot is running or connected
        odom_status = "ACTIVE" if getattr(self, "is_real_connected", False) or getattr(self, "running", False) else "OFFLINE"
        
        # 5. SLAM: ACTIVE if LiDAR is actively updating the map, else STANDBY
        slam_status = "ACTIVE" if lidar_active else "STANDBY"
        
        # 6. YOLO: ACTIVE only if YOLO thread active within 3s on JetBot
        yolo_active = (now - getattr(self, "_last_yolo_time", 0.0) < 3.0) and getattr(self, "_yolo_connected", False)
        yolo_status = "ACTIVE" if yolo_active else "OFFLINE"
        
        # 7. Navigation: ACTIVE if planner ready
        nav_status = "ACTIVE" if hasattr(self, "planner") else "OFFLINE"
        
        return [
            SensorHealth(name="Camera", status=camera_status, last_update=getattr(self, "_last_camera_time", now)),
            SensorHealth(name="LiDAR", status=lidar_status, last_update=getattr(self, "_last_lidar_time", now)),
            SensorHealth(name="IMU", status=imu_status, last_update=now),
            SensorHealth(name="Odometry", status=odom_status, last_update=now),
            SensorHealth(name="SLAM", status=slam_status, last_update=now),
            SensorHealth(name="YOLO", status=yolo_status, last_update=getattr(self, "_last_yolo_time", now)),
            SensorHealth(name="Navigation", status=nav_status, last_update=now),
        ]

    # ─── MapProvider ───

    async def get_map(self) -> MapData:
        return MapData(
            resolution=Config.MAP_RESOLUTION,
            width=Config.MAP_WIDTH,
            height=Config.MAP_HEIGHT,
            origin_x=Config.MAP_ORIGIN_X,
            origin_y=Config.MAP_ORIGIN_Y,
            data=self.map_data.copy()
        )

    async def get_trajectory(self) -> List[LidarPoint]:
        return self.trajectory

    # ─── NavigationManager ───

    async def set_goal(self, x: float, y: float):
        if self.mode != ModeEnum.AUTONOMOUS:
            self.mode = ModeEnum.AUTONOMOUS
            self._add_event("INFO", "🤖 Auto-switched to AUTONOMOUS mode for navigation.")
        
        # 1. Inject any detected obstacle clusters (table legs, boxes) into costmap
        if getattr(self, "latest_clusters", None):
            self.grid_map.insert_obstacle_clusters(
                self.latest_clusters, self.pose.x, self.pose.y, self.pose.theta
            )

        # 2. Clear footprint under the robot chassis to guarantee valid A* start cell
        self.grid_map.clear_footprint(self.pose.x, self.pose.y, radius=0.20)

        # 3. Reset navigation & controller state
        self.nav_status.status = NavigationStatusEnum.PLANNING
        self.nav_status.goal_x = x
        self.nav_status.goal_y = y
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        self._stop_start_time = 0.0
        self._replan_attempts = 0
        self._last_replan_time = 0.0
        self.controller.clear_path()
        self.controller.is_goal_reached = False
        self.controller.is_path_blocked = False
        self.controller._blocked_persistence_count = 0
        await self.send_hardware_stop()
        self._add_event("INFO", f"🎯 Goal received: ({x:.2f}, {y:.2f}). Computing optimal path...")
        print(f"\n[NAV-DEBUG] >>> 🎯 GOAL RECEIVED: ({x:.2f}, {y:.2f}) | Mode: {self.mode.value} | Status: PLANNING", flush=True)
        
        # Run A* in a background thread to prevent freezing the FastAPI asyncio event loop
        try:
            path = await asyncio.wait_for(
                _async_run_in_thread(self.planner.plan, (self.pose.x, self.pose.y), (x, y), True, 10.0, 25000, self.pose.theta),
                timeout=12.0
            )
        except asyncio.TimeoutError:
            path = None
            self._add_event("ERROR", f"⏱️ A* search timed out (> 10s)! Destination ({x:.2f}, {y:.2f}) is too complex or unreachable.")
            print(f"[NAV-DEBUG] >>> ⏱️ A* SEARCH TIMEOUT (>10s) to ({x:.2f}, {y:.2f})", flush=True)
        except Exception as e:
            path = None
            self._add_event("ERROR", f"A* Planner error: {e}")
            print(f"[NAV-DEBUG] >>> ❌ A* PLANNER ERROR: {e}", flush=True)

        if path is not None and len(path) >= 2:
            self.planned_path.points = [LidarPoint(x=px, y=py) for px, py in path]
            self.controller.set_path(path)
            self._initial_goal_distance = math.hypot(x - self.pose.x, y - self.pose.y)
            self.nav_status.distance_remaining = self._initial_goal_distance
            self.nav_status.progress = 0.0
            self._add_event("SUCCESS", f"✅ Path found ({len(path)} waypoints)! Validating clearance before moving...")
            print(f"[NAV-DEBUG] >>> ✅ PATH FOUND: {len(path)} waypoints | Dist: {self._initial_goal_distance:.2f}m", flush=True)
            
            # Brief pause to verify trajectory and allow web UI to render
            await asyncio.sleep(0.5)
            
            if self.nav_status.status == NavigationStatusEnum.PLANNING:
                self.nav_status.status = NavigationStatusEnum.NAVIGATING
                self._add_event("INFO", "🚀 Trajectory verified! Commencing navigation...")
                print(f"[NAV-DEBUG] >>> 🚀 COMMENCING NAVIGATION: Status=NAVIGATING", flush=True)
        else:
            self.nav_status.status = NavigationStatusEnum.BLOCKED
            self.planned_path.points = []
            self.controller.clear_path()
            print(f"[NAV-DEBUG] >>> ❌ PATH BLOCKED to ({x:.2f}, {y:.2f})", flush=True)
            if not any("timed out" in ev.message for ev in self.events[-3:]):
                self._add_event("ERROR", f"❌ A* path blocked by obstacles! Cannot reach ({x:.2f}, {y:.2f})")

    async def cancel_goal(self):
        if self.nav_status.status in (NavigationStatusEnum.PLANNING, NavigationStatusEnum.NAVIGATING):
            self.nav_status.status = NavigationStatusEnum.CANCELLED
            self.linear_vel = 0.0
            self.angular_vel = 0.0
            self.planned_path.points = []
            self._add_event("WARNING", "Navigation cancelled")
            print(f"[NAV-DEBUG] >>> ⚠️ NAVIGATION CANCELLED by user", flush=True)

    async def get_nav_status(self) -> NavigationStatus:
        return self.nav_status

    async def get_planned_path(self) -> PlannedPath:
        return self.planned_path

    # ─── DetectionProvider ───

    async def get_detections(self) -> DetectionList:
        return self.current_detections

    # ─── Internal simulation methods ───

    def _update_map(self, px: float, py: float, hit: bool):
        """Update a single cell in the occupancy grid."""
        mx = int((px - Config.MAP_ORIGIN_X) / Config.MAP_RESOLUTION)
        my = int((py - Config.MAP_ORIGIN_Y) / Config.MAP_RESOLUTION)
        if 0 <= mx < Config.MAP_WIDTH and 0 <= my < Config.MAP_HEIGHT:
            idx = my * Config.MAP_WIDTH + mx
            self.map_data[idx] = 100 if hit else 0

    def _raycast(self, angle: float) -> float:
        """Cast a ray from robot position at given angle, return distance to nearest hit."""
        ray_end = (
            self.pose.x + math.cos(angle) * Config.MAX_RANGE,
            self.pose.y + math.sin(angle) * Config.MAX_RANGE
        )
        p1 = (self.pose.x, self.pose.y)
        min_dist = Config.MAX_RANGE
        
        for w in Config.WALLS:
            intersect = line_intersection(p1, ray_end, (w[0], w[1]), (w[2], w[3]))
            if intersect:
                d = math.hypot(intersect[0] - p1[0], intersect[1] - p1[1])
                min_dist = min(min_dist, d)
                
        for obs in Config.OBSTACLES:
            intersect = circle_intersection(p1, ray_end, obs[0], obs[1], obs[2])
            if intersect:
                d = math.hypot(intersect[0] - p1[0], intersect[1] - p1[1])
                min_dist = min(min_dist, d)
                
        return min_dist + random.gauss(0, 0.02)

    async def update(self):
        """Main simulation loop running at ROBOT_UPDATE_RATE Hz."""
        self.running = True
        while self.running:
            # Battery drain (only simulate when physical robot is NOT connected)
            if not getattr(self, "is_real_connected", False):
                self.battery -= 0.01 * self._dt
                if self.battery < 0:
                    self.battery = 0
            
            # Autonomous navigation control via Pure Pursuit + Curvature Scaling + Plateau Detour + 10Hz Reflex
            if (self.mode == ModeEnum.AUTONOMOUS and 
                self.nav_status.status == NavigationStatusEnum.NAVIGATING):
                
                # ─── Nav2 Back-Up Recovery Execution Loop ───
                if getattr(self, "_is_backing_up", False):
                    now_t = time.time()
                    elapsed = now_t - getattr(self, "_backup_start_time", now_t)
                    
                    # Continuous rear collision check (150 to 210 deg)
                    rear_pts = [r for idx, r in enumerate(self.latest_raw_ranges) if 150 <= idx <= 210 and 0.12 <= r <= 1.8]
                    rear_min = min(rear_pts) if rear_pts else 2.0
                    
                    if elapsed >= getattr(self, "_backup_duration", 1.2) or rear_min < 0.18:
                        self._is_backing_up = False
                        self.linear_vel = 0.0
                        self.angular_vel = 0.0
                        await self.send_hardware_stop()
                        
                        # Clear footprint around new position to ensure replanner starts clean
                        self.grid_map.clear_footprint(self.pose.x, self.pose.y, radius=0.18)
                        
                        # Reset controller reflex stop & blocked timers
                        self.controller.is_reflex_stopped = False
                        if hasattr(self.controller, "_obstacle_blocked_start_time"):
                            self.controller._obstacle_blocked_start_time = 0.0
                        self.controller.clear_path()
                        
                        self._add_event("SUCCESS", f"✅ [Nav2 Recovery] Back-Up completed (cleared {round(elapsed * 0.09, 2)}m). Triggering replan.")
                        print(f"[NAV-DEBUG] >>> ✅ BACK-UP COMPLETE: Cleared physical clearance space, replanning now.", flush=True)
                        
                        # Trigger immediate A* replan
                        goal_x = self.nav_status.goal_x
                        goal_y = self.nav_status.goal_y
                        try:
                            new_path = await asyncio.wait_for(
                                _async_run_in_thread(self.planner.plan, (self.pose.x, self.pose.y), (goal_x, goal_y), True, 3.0, 10000, self.pose.theta),
                                timeout=3.5
                            )
                        except Exception:
                            new_path = None
                        
                        if new_path and len(new_path) >= 2:
                            self.controller.set_path(new_path)
                            self.planned_path.points = [LidarPoint(x=p[0], y=p[1]) for p in new_path]
                            self._stop_start_time = 0.0
                            self._replan_attempts = 0
                            print(f"[NAV-DEBUG] >>> 🔄 Post-Backup Replan Success: {len(new_path)} waypoints", flush=True)
                        
                        await asyncio.sleep(self._dt)
                        continue
                    else:
                        v_back = -0.09
                        w_back = 0.0
                        self.linear_vel = v_back
                        self.angular_vel = w_back
                        self.motion_xai_info["status"] = "BACKUP_RECOVERY"
                        await self.send_hardware_cmd_vel(v_back, w_back)
                        await asyncio.sleep(self._dt)
                        continue

                v, w, info = self.controller.compute_command(
                    self.pose.x, self.pose.y, self.pose.theta,
                    lidar_ranges=self.latest_raw_ranges,
                    clusters=self.latest_clusters,
                    grid_map=self.grid_map,
                    auto_evade=True
                )
                
                status_str = info.get("status")
                evasion_active = info.get("evasion_active", False)
                evasion_reason = info.get("evasion_reason", "")

                # ─── Proactive Real-Time A* Dynamic Replanning (0.3ms latency) ───
                # If frontal path is blocked or local detour cannot fit, immediately query A*
                # to route around the obstacle while the robot is still moving!
                now_t = time.time()
                stop_t = getattr(self, "_stop_start_time", 0.0)
                is_stalled = (abs(v) <= 0.01 and abs(w) <= 0.01 and self.nav_status.distance_remaining > 0.20)
                is_stopped_or_blocked = (status_str in ("IMPASSABLE_STOP", "TACTICAL_YIELD", "WAITING_CLEARANCE") or info.get("is_blocked", False) or is_stalled)
                if is_stopped_or_blocked:
                    if stop_t == 0.0:
                        self._stop_start_time = now_t
                        stop_t = now_t
                    blocked_dur = now_t - stop_t
                else:
                    blocked_dur = 0.0

                replan_cooldown = now_t - getattr(self, "_last_replan_time", 0.0)
                if is_stopped_or_blocked and (blocked_dur >= 0.6) and (replan_cooldown >= 1.5):
                    goal_x = self.nav_status.goal_x
                    goal_y = self.nav_status.goal_y
                    if getattr(self, "latest_clusters", None):
                        self.grid_map.insert_obstacle_clusters(
                            self.latest_clusters, self.pose.x, self.pose.y, self.pose.theta
                        )
                    self.grid_map.clear_footprint(self.pose.x, self.pose.y, radius=0.15)
                    try:
                        new_path = self.planner.plan((self.pose.x, self.pose.y), (goal_x, goal_y), smooth=True)
                    except Exception:
                        new_path = None

                    if new_path and len(new_path) >= 2:
                        self.controller.set_path(new_path)
                        self.planned_path.points = [LidarPoint(x=p[0], y=p[1]) for p in new_path]
                        self._last_replan_time = now_t
                        self._stop_start_time = 0.0
                        self._replan_attempts = 0
                        self._add_event("SUCCESS", "🔄 [Proactive A*] Re-routed around obstacle! Resuming motion...")
                        # Recompute velocity command immediately for the new path with auto_evade=True
                        v, w, info = self.controller.compute_command(
                            self.pose.x, self.pose.y, self.pose.theta,
                            lidar_ranges=self.latest_raw_ranges,
                            clusters=self.latest_clusters,
                            grid_map=self.grid_map,
                            auto_evade=True
                        )
                        status_str = info.get("status")
                        evasion_active = info.get("evasion_active", False)

                target_pt = info.get("target")
                if target_pt is None and self.controller.current_path:
                    p_idx = min(len(self.controller.current_path) - 1, getattr(self.controller, "path_index", 0) + 1)
                    target_pt = self.controller.current_path[p_idx]

                # Stable path update: Update planned_path points only on state transitions
                # to prevent continuous path flickering on the Dashboard!
                if evasion_active and not self._last_evasion_active:
                    if self.controller.current_path:
                        self.planned_path.points = [LidarPoint(x=p[0], y=p[1]) for p in self.controller.current_path]
                    tag = "MPPI Predictive Evasion" if isinstance(self.controller, MPPIController) else "XAI Plateau Detour"
                    self._add_event("WARNING", f"🛡️ [{tag}] {evasion_reason or 'Evading obstacle with smooth arc'}")
                elif not evasion_active and self._last_evasion_active:
                    if hasattr(self.controller, "global_reference_path") and self.controller.global_reference_path:
                        self.planned_path.points = [LidarPoint(x=p[0], y=p[1]) for p in self.controller.global_reference_path]
                    self._add_event("SUCCESS", "✅ Resumed nominal path to destination.")
                self._last_evasion_active = evasion_active

                # Pudu/Keenon FSM Wait-and-See & Tactical Yield Event Logging
                last_fsm_status = getattr(self, "_last_fsm_ctrl_status", "")
                if status_str == "TACTICAL_YIELD" and last_fsm_status != "TACTICAL_YIELD":
                    self._add_event("WARNING", f"🛑 [Tactical Yield] {info.get('reason', 'Tactical Yield: Holding position at 0.3m to prevent wall crash.')}")
                elif status_str == "WAITING_CLEARANCE" and last_fsm_status != "WAITING_CLEARANCE":
                    self._add_event("INFO", f"⏳ [Pudu FSM Wait] {info.get('reason', 'Obstacle detected. Waiting 1.5s for dynamic clearance...')}")
                elif status_str not in ("WAITING_CLEARANCE", "TACTICAL_YIELD") and last_fsm_status in ("WAITING_CLEARANCE", "TACTICAL_YIELD"):
                    if not evasion_active and status_str not in ("EMERGENCY_STOP", "IMPASSABLE_STOP"):
                        self._add_event("SUCCESS", "✅ [Clearance Resumed] Path clear. Resuming nominal navigation.")
                self._last_fsm_ctrl_status = status_str

                # SOTA 2026: CBF & DDE Explainable AI Event Logging
                if info.get("cbf_active"):
                    now_t = time.time()
                    if now_t - getattr(self, "_last_cbf_log_time", 0.0) > 3.0:
                        self._add_event("WARNING", f"🛡️ [CBF Safety Shield] {evasion_reason or 'Control Barrier Function Active'}")
                        self._last_cbf_log_time = now_t
                elif info.get("dde_active"):
                    now_t = time.time()
                    if now_t - getattr(self, "_last_dde_log_time", 0.0) > 3.0:
                        self._add_event("INFO", f"🧭 [DDE Gap Selection] Steering {info.get('dde_target_deg', 0):.0f}° towards safe opening")
                        self._last_dde_log_time = now_t

                # Curvature velocity scaling XAI logging
                base_v = self.controller.max_linear_speed
                curv_scale = round(v / max(0.01, base_v), 2) if v > 0 else 0.0
                if abs(w) > 0.35 and curv_scale < 0.85:
                    now_t = time.time()
                    if now_t - self._last_curv_log_time > 4.0:
                        pct_down = int(round((1.0 - curv_scale) * 100))
                        self._add_event("INFO", f"⚡ [XAI Curvature Scaling] -{pct_down}% speed for sharp turn (w={round(w, 2)} rad/s)")
                        self._last_curv_log_time = now_t

                # Update Motion XAI info for Dashboard
                self.motion_xai_info = {
                    "status": status_str,
                    "speed_scale": curv_scale,
                    "curvature_w": round(w, 2),
                    "evasion_active": evasion_active,
                    "evasion_reason": evasion_reason,
                    "lookahead_target": [round(target_pt[0], 3), round(target_pt[1], 3)] if target_pt else None,
                    "cbf_active": info.get("cbf_active", False),
                    "cbf_min_h": info.get("cbf_min_h", 1.0),
                    "dde_active": info.get("dde_active", False),
                    "dde_target_deg": info.get("dde_target_deg", 0.0)
                }

                # Real-Time Algorithm Diagnostics (Every 0.5s or on Status Change)
                now_t = time.time()
                last_log_t = getattr(self, "_last_nav_diag_time", 0.0)
                last_logged_status = getattr(self, "_last_logged_ctrl_status", "")
                if (now_t - last_log_t >= 0.5) or (status_str != last_logged_status):
                    self._last_nav_diag_time = now_t
                    self._last_logged_ctrl_status = status_str

                    # Compute nearest frontal obstacle in body frame
                    front_dist = None
                    if getattr(self, "latest_raw_ranges", None):
                        f_pts = [r for idx, r in enumerate(self.latest_raw_ranges) if (idx <= 30 or idx >= 330) and 0.12 <= r < 3.0]
                        if f_pts:
                            front_dist = round(min(f_pts), 2)

                    hw_str = "CONNECTED" if (self.real_ws and self.is_real_connected) else "SIMULATION"
                    ctrl_name = "MPPI" if isinstance(self.controller, MPPIController) else "PurePursuit"
                    log_line = (
                        f"[NAV-DEBUG] [{ctrl_name}] Status={status_str:<14} | "
                        f"Cmd: v={v:.3f} m/s, w={w:+.3f} rad/s | "
                        f"Pose: ({self.pose.x:.2f}, {self.pose.y:.2f}, {math.degrees(self.pose.theta):+.1f}°) | "
                        f"Goal: ({self.nav_status.goal_x:.2f}, {self.nav_status.goal_y:.2f}) rem={info.get('distance_remaining', 0.0):.2f}m | "
                        f"FrontMin: {str(front_dist)+'m' if front_dist else 'CLEAR':<5} | "
                        f"Reflex: {str(self.controller.is_reflex_stopped):<5} | "
                        f"CBF: {str(info.get('cbf_active', False)):<5} | "
                        f"HW: {hw_str}"
                    )
                    print(log_line, flush=True)

                    try:
                        log_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../../04-hieu-MotionControl"))
                        if os.path.exists(log_dir):
                            log_path = os.path.join(log_dir, "nav_debug.log")
                            with open(log_path, "a", encoding="utf-8") as lf:
                                lf.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')} {log_line}\n")
                    except Exception:
                        pass

                # Terminal Goal Bypass: If robot is within 0.50m of goal and FrontMin > 0.35m,
                # do NOT stop on IMPASSABLE_STOP or TACTICAL_YIELD, continue creeping to goal at slow speed
                rem_goal_dist = self.nav_status.distance_remaining
                if rem_goal_dist <= 0.50 and front_dist is not None and front_dist > 0.35:
                    if status_str in ("IMPASSABLE_STOP", "TACTICAL_YIELD", "WAITING_CLEARANCE"):
                        status_str = "TERMINAL_APPROACH"
                        v = 0.08
                        if target_pt:
                            d_x = target_pt[0] - self.pose.x
                            d_y = target_pt[1] - self.pose.y
                            t_alpha = (math.atan2(d_y, d_x) - self.pose.theta + math.pi) % (2.0 * math.pi) - math.pi
                            w = max(-self.controller.max_angular_speed, min(self.controller.max_angular_speed, v * (2.0 * math.sin(t_alpha) / 0.15)))
                        else:
                            w = 0.0

                if status_str in ("EMERGENCY_STOP", "IMPASSABLE_STOP", "WAITING_CLEARANCE"):
                    self.linear_vel = 0.0
                    self.angular_vel = 0.0
                    await self.send_hardware_stop()
                    now_t = time.time()
                    if not hasattr(self, "_stop_start_time") or self._stop_start_time == 0.0:
                        self._stop_start_time = now_t
                        if status_str == "EMERGENCY_STOP":
                            tag = "Reflex Brake"
                        elif status_str == "WAITING_CLEARANCE":
                            tag = "Wait-and-See (2s)"
                        else:
                            tag = "Corridor Impassable"
                        self._add_event("WARNING", f"🛑 [{tag}] {info.get('reason', 'Obstacle blocking path!')}. Waiting before replanning...")
                        print(f"[NAV-DEBUG] >>> 🛑 {status_str}! Reason: {info.get('reason', 'Obstacle blocking path!')}", flush=True)

                    # Terminal Goal Vicinity Check:
                    rem_goal_dist = self.nav_status.distance_remaining
                    if rem_goal_dist <= max(0.18, self.controller.goal_tolerance):
                        self.nav_status.status = NavigationStatusEnum.REACHED
                        self.nav_status.progress = 100.0
                        self.nav_status.distance_remaining = 0.0
                        self.planned_path.points = []
                        self.motion_xai_info["status"] = "GOAL_REACHED"
                        self._add_event("SUCCESS", "🎯 Reached target destination accurately!")
                        print(f"[NAV-DEBUG] >>> 🎯 GOAL REACHED (rem={rem_goal_dist:.3f}m <= {max(0.18, self.controller.goal_tolerance):.3f}m)", flush=True)
                        return

                    # ─── Nav2 Back-Up Recovery Trigger ───
                    # When reflex brake stops the robot or corridor is impassable, back up 1.2s to create physical clearance
                    last_backup_t = getattr(self, "_last_backup_time", 0.0)
                    stopped_duration = now_t - getattr(self, "_stop_start_time", now_t)
                    rem_goal_dist = self.nav_status.distance_remaining
                    should_backup = (
                        (status_str == "EMERGENCY_STOP" and stopped_duration >= 0.5) or
                        (status_str == "IMPASSABLE_STOP" and stopped_duration >= 2.0)
                    ) and (rem_goal_dist > 0.35)
                    if should_backup and (now_t - last_backup_t > 4.0):
                        rear_pts = [r for idx, r in enumerate(self.latest_raw_ranges) if 150 <= idx <= 210 and 0.12 <= r <= 1.8]
                        rear_min = min(rear_pts) if rear_pts else 2.0
                        if rear_min >= 0.22:
                            self._is_backing_up = True
                            self._backup_start_time = now_t
                            self._backup_duration = 1.2
                            self._last_backup_time = now_t
                            self._stop_start_time = 0.0
                            self._add_event("WARNING", f"⏪ [Nav2 Recovery] Obstacle blocking forward corridor! Reversing 1.2s (Rear={rear_min:.2f}m)...")
                            print(f"[NAV-DEBUG] >>> ⏪ BACKUP_RECOVERY TRIGGERED: Reversing 1.2s (rear clearance {rear_min:.2f}m)", flush=True)
                            await asyncio.sleep(self._dt)
                            continue
                        else:
                            print(f"[NAV-DEBUG] >>> ⚠️ Cannot back up: rear is obstructed ({rear_min:.2f}m < 0.22m)", flush=True)

                    # Intelligent recovery with patience (wait 2.0s for dynamic obstacles or to compute new path)
                    if not hasattr(self, "_last_replan_time"):
                        self._last_replan_time = 0.0
                    replan_delay = 2.0
                    if (now_t - self._stop_start_time >= replan_delay) and (now_t - self._last_replan_time > 2.0):
                        self._last_replan_time = now_t
                        self._replan_attempts += 1
                        goal_x = self.nav_status.goal_x
                        goal_y = self.nav_status.goal_y

                        # 1. Re-inject obstacle clusters before replan
                        if getattr(self, "latest_clusters", None):
                            self.grid_map.insert_obstacle_clusters(
                                self.latest_clusters, self.pose.x, self.pose.y, self.pose.theta
                            )

                        # 2. Clear immediate footprint to ensure replan start cell is clean
                        self.grid_map.clear_footprint(self.pose.x, self.pose.y, radius=0.15)

                        try:
                            new_path = await asyncio.wait_for(
                                _async_run_in_thread(self.planner.plan, (self.pose.x, self.pose.y), (goal_x, goal_y), True, 3.0, 10000, self.pose.theta),
                                timeout=3.5
                            )
                        except Exception:
                            new_path = None

                        if new_path and len(new_path) >= 2:
                            self.controller.set_path(new_path)
                            self.planned_path.points = [LidarPoint(x=p[0], y=p[1]) for p in new_path]
                            self._stop_start_time = 0.0
                            if hasattr(self.controller, "_obstacle_blocked_start_time"):
                                self.controller._obstacle_blocked_start_time = 0.0
                            self._replan_attempts = 0
                            self._add_event("SUCCESS", "🔄 Found new path around obstacle! Resuming navigation...")
                            print(f"[NAV-DEBUG] >>> 🔄 REPLAN SUCCESS: New path found ({len(new_path)} waypoints)", flush=True)
                        elif self._replan_attempts >= 3 or (now_t - self._stop_start_time > 8.0):
                            self.nav_status.status = NavigationStatusEnum.BLOCKED
                            self.planned_path.points = []
                            self.controller.clear_path()
                            self._stop_start_time = 0.0
                            self._replan_attempts = 0
                            self._add_event("ERROR", "🚨 All routes blocked by obstacles! Navigation terminated.")
                            print(f"[NAV-DEBUG] >>> 🚨 ALL ROUTES BLOCKED: Navigation terminated.", flush=True)
                elif status_str == "GOAL_REACHED":
                    self.linear_vel = 0.0
                    self.angular_vel = 0.0
                    self._stop_start_time = 0.0
                    self._replan_attempts = 0
                    await self.send_hardware_stop()
                    self.nav_status.status = NavigationStatusEnum.REACHED
                    self.nav_status.progress = 100.0
                    self.nav_status.distance_remaining = 0.0
                    self.planned_path.points = []
                    self.motion_xai_info["status"] = "GOAL_REACHED"
                    self._add_event("SUCCESS", "🎯 Target Goal Reached Successfully!")
                    print(f"[NAV-DEBUG] >>> 🎯 GOAL REACHED! Successfully arrived at goal.", flush=True)
                else:
                    if abs(v) <= 0.01 and abs(w) <= 0.01:
                        # Robot is commanded zero motion (stalled/stopped by obstacle or corridor)
                        now_t = time.time()
                        if not hasattr(self, "_stop_start_time") or self._stop_start_time == 0.0:
                            self._stop_start_time = now_t
                    else:
                        self._stop_start_time = 0.0
                        self._replan_attempts = 0

                    self.linear_vel = v
                    self.angular_vel = w
                    self.nav_status.velocity = v
                    rem_dist = info.get("distance_remaining", 0.0)
                    self.nav_status.distance_remaining = rem_dist
                    if self._initial_goal_distance > 0:
                        self.nav_status.progress = max(0.0, min(100.0, (1.0 - rem_dist / self._initial_goal_distance) * 100.0))
                    await self.send_hardware_cmd_vel(v, w)

                    # Keep planned path in sync with controller detour path on Dashboard
                    if info.get("evasion_active") and self.controller.current_path:
                        self.planned_path.points = [LidarPoint(x=p[0], y=p[1]) for p in self.controller.current_path]
            else:
                self.motion_xai_info["status"] = "IDLE"
                self.motion_xai_info["evasion_active"] = False
                self.motion_xai_info["speed_scale"] = 1.0
                self.motion_xai_info["curvature_w"] = 0.0
                self.motion_xai_info["lookahead_target"] = None
            
            # LiDAR scan (Use real scan if hardware connected, otherwise simulate)
            is_hardware_active = self.is_real_connected and (time.time() - self._last_real_data_time < 3.0)

            # Update physics (Simulated kinematic integration when hardware is not driving pose via LiDAR Odometry)
            if not is_hardware_active:
                self.pose.theta += self.angular_vel * self._dt
                self.pose.theta = (self.pose.theta + math.pi) % (2 * math.pi) - math.pi
                self.pose.x += self.linear_vel * math.cos(self.pose.theta) * self._dt
                self.pose.y += self.linear_vel * math.sin(self.pose.theta) * self._dt
            
            # Record trajectory
            t = time.time()
            if t - self.last_traj_time > 0.1:
                self.trajectory.append(LidarPoint(x=self.pose.x, y=self.pose.y))
                self.last_traj_time = t
                if len(self.trajectory) > 5000:
                    self.trajectory = self.trajectory[-3000:]

            if not is_hardware_active:
                scan_points = []
                sim_ranges = []
                for i in range(Config.NUM_RAYS):
                    angle = self.pose.theta + (i * 2 * math.pi / Config.NUM_RAYS)
                    dist = self._raycast(angle)
                    sim_ranges.append(dist)
                    hit_x = self.pose.x + math.cos(angle) * dist
                    hit_y = self.pose.y + math.sin(angle) * dist
                    scan_points.append(LidarPoint(x=hit_x, y=hit_y))
                    
                    # Update SLAM map
                    if dist < Config.MAX_RANGE - 0.1:
                        self._update_map(hit_x, hit_y, True)
                    
                    # Mark free space along the ray (sample a few points)
                    for frac in [0.3, 0.5, 0.7]:
                        free_x = self.pose.x + math.cos(angle) * (dist * frac)
                        free_y = self.pose.y + math.sin(angle) * (dist * frac)
                        self._update_map(free_x, free_y, False)
                    
                self.current_scan = LidarScan(points=scan_points)
                self.latest_raw_ranges = sim_ranges
                self._process_lidar_clusters(sim_ranges)

                # Throttle simulation grid map update to 5 Hz synchronized with WebSocket
                now_sim_t = time.time()
                if now_sim_t - getattr(self, '_last_sim_map_time', 0.0) >= 0.18:
                    self.grid_map.update_scan(
                        self.pose.x, self.pose.y, self.pose.theta,
                        ranges=sim_ranges,
                        max_valid_range=Config.MAX_RANGE - 0.15
                    )
                    if getattr(self, "latest_clusters", None):
                        self.grid_map.insert_obstacle_clusters(
                            self.latest_clusters, self.pose.x, self.pose.y, self.pose.theta
                        )
                    self.map_data = self.grid_map.to_dashboard_data()
                    self.map_version = getattr(self, "map_version", 0) + 1
                    self._last_sim_map_time = now_sim_t
            
            # YOLO detections (Only simulate if NO real hardware connected)
            if not is_hardware_active:
                dets = []
                for obj in self.mock_objects:
                    dx = obj["x"] - self.pose.x
                    dy = obj["y"] - self.pose.y
                    dist = math.hypot(dx, dy)
                    if dist < 5.0 and dist > 0.3:
                        angle_to_obj = math.atan2(dy, dx)
                        angle_diff = (angle_to_obj - self.pose.theta + math.pi) % (2 * math.pi) - math.pi
                        if abs(angle_diff) < math.pi / 4:
                            # Calculate approximate bounding box in pixel space
                            # Object appears at horizontal position based on angle_diff
                            cx = 320 + int(angle_diff / (math.pi / 4) * 300)
                            # Size inversely proportional to distance
                            size = int(200 / max(dist, 0.5))
                            x1 = max(0, cx - size // 2)
                            y1 = max(0, 240 - size)
                            x2 = min(640, cx + size // 2)
                            y2 = min(480, 240 + size)
                            
                            dets.append(Detection(
                                class_name=obj["class"],
                                confidence=round(random.uniform(0.72, 0.98), 2),
                                bbox=[x1, y1, x2, y2],
                                distance=round(dist, 2)
                            ))
                if self.mock_objects:
                    self.current_detections = DetectionList(objects=dets)

                # Social bubbles from person detections (Simulation & CV)
                sim_person_dets = [
                    {
                        "class_name": d.class_name,
                        "distance": d.distance,
                        "azimuth_deg": getattr(d, "azimuth_deg", 0.0)
                    }
                    for d in self.current_detections.objects
                    if d.class_name.lower() == "person" and d.distance and d.distance > 0
                ]
                if sim_person_dets:
                    self.grid_map.update_social_bubbles_from_detections(
                        detections=sim_person_dets,
                        robot_x=self.pose.x,
                        robot_y=self.pose.y,
                        robot_theta=self.pose.theta,
                        bubble_radius_m=0.9
                    )
                    self.latest_social_bubbles = [
                        {"x": round(b[0], 3), "y": round(b[1], 3), "radius": round(b[2], 2)}
                        for b in getattr(self.grid_map, "social_bubbles", [])
                    ]
                elif getattr(self.grid_map, "social_bubbles", None):
                    self.grid_map.social_bubbles = []
                    self.latest_social_bubbles = []
            
            await asyncio.sleep(self._dt)
