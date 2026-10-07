import os
import asyncio
import math
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
        self.planner = AStarPlanner(self.grid_map)
        self.controller = PurePursuitController(
            lookahead_dist=0.30,
            max_linear_speed=Config.MAX_SPEED * 0.6,
            max_angular_speed=Config.MAX_ANGULAR_SPEED,
            track_width=0.115,
            reflex_dist_threshold=0.20
        )
        self.latest_raw_ranges = [0.0] * 360
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

    async def reset_map(self):
        """Clear map data and trajectory completely."""
        self.grid_map.reset()
        self.map_data = self.grid_map.to_dashboard_data()
        self.trajectory.clear()
        self.current_scan = LidarScan(points=[])
        self._add_event("WARNING", "2D SLAM Map reset by user!")

    async def reset_pose(self):
        """Reset robot pose to origin (0, 0, 0)"""
        self.pose = RobotPose(x=0.0, y=0.0, theta=0.0)
        self.trajectory.clear()
        self._add_event("INFO", "Robot pose reset to (0, 0, 0)")

    async def handle_real_telemetry(self, data: dict):
        """Processes real Camera + LiDAR + Battery from physical JetBot"""
        # Wipe fake simulation walls on first real telemetry packet
        if not getattr(self, '_has_reset_real_map', False):
            self.grid_map.reset()
            self.map_data = self.grid_map.to_dashboard_data()
            self.trajectory.clear()
            self._has_reset_real_map = True
            self._add_event("SUCCESS", "Cleared simulated room. Mapping Real Environment!")

        self.is_real_connected = True
        self._last_real_data_time = time.time()

        # 1. Real Camera frame
        cam_b64 = data.get("camera")
        if cam_b64 and len(cam_b64) > 100:
            try:
                self._camera_frame = base64.b64decode(cam_b64)
                self._last_camera_time = time.time()
                self._camera_connected = True
            except Exception:
                pass

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
            scan_points = []

            for item in raw_lidar:
                if len(item) >= 2:
                    ang_deg = float(item[0])
                    dist = float(item[1])

                    # Filter noise: ignore reflections closer than 12cm (chassis) or out of range
                    if dist < 0.12 or dist > Config.MAX_RANGE:
                        continue

                    ranges[int(round(ang_deg)) % 360] = dist

                    rad = math.radians(ang_deg) + self.pose.theta
                    hit_x = self.pose.x + dist * math.cos(rad)
                    hit_y = self.pose.y + dist * math.sin(rad)
                    scan_points.append(LidarPoint(x=hit_x, y=hit_y))

            # Store latest raw ranges for 10Hz Safety Reflex in controller.py
            self.latest_raw_ranges = ranges

            if scan_points:
                self.current_scan = LidarScan(points=scan_points)

            # Throttle grid map update to 4-5 Hz (period >= 0.2s) to prevent CPU event loop blocking
            now_t = time.time()
            if now_t - getattr(self, '_last_map_update_time', 0.0) >= 0.2:
                self.grid_map.update_scan(
                    self.pose.x, self.pose.y, self.pose.theta,
                    ranges=ranges,
                    max_valid_range=Config.MAX_RANGE - 0.15
                )
                self.map_data = self.grid_map.to_dashboard_data()
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
            # TẦNG 1: Project Social Cost Bubbles (0.9m) onto 2D Occupancy Grid Map
            self.grid_map.update_social_bubbles_from_detections(
                detections=raw_dets,
                robot_x=self.pose.x,
                robot_y=self.pose.y,
                robot_theta=self.pose.theta,
                bubble_radius_m=0.9
            )
            self.map_data = self.grid_map.to_dashboard_data()

    # ─── RobotController ───

    async def get_pose(self) -> RobotPose:
        return self.pose

    async def get_battery(self) -> float:
        return self.battery

    async def set_velocity(self, linear: float, angular: float):
        if self.mode == ModeEnum.MANUAL:
            self.linear_vel = max(-Config.MAX_SPEED, min(Config.MAX_SPEED, linear))
            self.angular_vel = max(-Config.MAX_ANGULAR_SPEED, min(Config.MAX_ANGULAR_SPEED, angular))
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
            self._add_event("WARNING", "Cannot set goal in MANUAL mode. Switch to AUTONOMOUS!")
            return
        
        self.nav_status.status = NavigationStatusEnum.PLANNING
        self.nav_status.goal_x = x
        self.nav_status.goal_y = y
        self._add_event("INFO", f"Goal received: ({x:.2f}, {y:.2f}). Running A* Search...")
        
        path = self.planner.plan((self.pose.x, self.pose.y), (x, y), smooth=True)
        if path is not None and len(path) >= 2:
            self.planned_path.points = [LidarPoint(x=px, y=py) for px, py in path]
            self.controller.set_path(path)
            self._initial_goal_distance = math.hypot(x - self.pose.x, y - self.pose.y)
            self.nav_status.status = NavigationStatusEnum.NAVIGATING
            self.nav_status.distance_remaining = self._initial_goal_distance
            self.nav_status.progress = 0.0
            self._add_event("SUCCESS", f"A* found safe path ({len(path)} waypoints)! Tracking via Pure Pursuit...")
        else:
            self.nav_status.status = NavigationStatusEnum.BLOCKED
            self.planned_path.points = []
            self.controller.clear_path()
            self._add_event("ERROR", f"A* path blocked by obstacles! Cannot reach ({x:.2f}, {y:.2f})")

    async def cancel_goal(self):
        if self.nav_status.status in (NavigationStatusEnum.PLANNING, NavigationStatusEnum.NAVIGATING):
            self.nav_status.status = NavigationStatusEnum.CANCELLED
            self.linear_vel = 0.0
            self.angular_vel = 0.0
            self.planned_path.points = []
            self._add_event("WARNING", "Navigation cancelled")

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
            
            # Autonomous navigation control via Pure Pursuit + 10Hz Reflex Safety Brake
            if (self.mode == ModeEnum.AUTONOMOUS and 
                self.nav_status.status == NavigationStatusEnum.NAVIGATING):
                
                v, w, info = self.controller.compute_command(
                    self.pose.x, self.pose.y, self.pose.theta,
                    lidar_ranges=self.latest_raw_ranges
                )
                
                status_str = info.get("status")
                if status_str == "EMERGENCY_STOP":
                    self.linear_vel = 0.0
                    self.angular_vel = 0.0
                    await self.send_hardware_stop()
                    self.nav_status.status = NavigationStatusEnum.BLOCKED
                    self._add_event("ERROR", f"🚨 10Hz SAFETY REFLEX: {info.get('reason')}")
                elif status_str == "GOAL_REACHED":
                    self.linear_vel = 0.0
                    self.angular_vel = 0.0
                    await self.send_hardware_stop()
                    self.nav_status.status = NavigationStatusEnum.REACHED
                    self.nav_status.progress = 100.0
                    self.nav_status.distance_remaining = 0.0
                    self.planned_path.points = []
                    self._add_event("SUCCESS", "🎯 Target Goal Reached Successfully!")
                else:
                    self.linear_vel = v
                    self.angular_vel = w
                    self.nav_status.velocity = v
                    rem_dist = info.get("distance_remaining", 0.0)
                    self.nav_status.distance_remaining = rem_dist
                    if self._initial_goal_distance > 0:
                        self.nav_status.progress = max(0.0, min(100.0, (1.0 - rem_dist / self._initial_goal_distance) * 100.0))
                    await self.send_hardware_cmd_vel(v, w)
            
            # Update physics
            self.pose.theta += self.angular_vel * self._dt
            self.pose.theta = (self.pose.theta + math.pi) % (2 * math.pi) - math.pi
            self.pose.x += self.linear_vel * math.cos(self.pose.theta) * self._dt
            self.pose.y += self.linear_vel * math.sin(self.pose.theta) * self._dt
            
            # Record trajectory
            t = time.time()
            if t - self.last_traj_time > 0.1:
                self.trajectory.append(LidarPoint(x=self.pose.x, y=self.pose.y))
                self.last_traj_time = t
                # Limit trajectory length
                if len(self.trajectory) > 5000:
                    self.trajectory = self.trajectory[-3000:]
                
            # LiDAR scan (Use real scan if hardware connected, otherwise simulate)
            is_hardware_active = self.is_real_connected and (time.time() - self._last_real_data_time < 3.0)
            if not is_hardware_active:
                scan_points = []
                for i in range(Config.NUM_RAYS):
                    angle = self.pose.theta + (i * 2 * math.pi / Config.NUM_RAYS)
                    dist = self._raycast(angle)
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
                self.current_detections = DetectionList(objects=dets)
            
            await asyncio.sleep(self._dt)
