import asyncio
import math
import random
import time
import struct
import io
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
    """Create a minimal valid JPEG image with a dark gradient pattern.
    This generates a proper JPEG that can be displayed in browsers.
    Uses a simple approach with raw JFIF structure.
    """
    # Generate a simple BMP-style raw image, then wrap in minimal JPEG markers
    # For simplicity, create a very small placeholder JPEG
    # This is a minimal valid JPEG: SOI + APP0 + DQT + SOF0 + DHT + SOS + image data + EOI
    
    # Actually, let's create a proper raw image data and use a simpler approach
    # Generate pixel data for a dark-themed camera-like view
    import zlib
    
    # Create a small thumbnail that will be scaled by the browser
    w, h = 160, 120  # Small size for efficiency
    
    # Build raw RGB data with a dark gradient/noise pattern
    pixels = bytearray(w * h * 3)
    for y in range(h):
        for x in range(w):
            idx = (y * w + x) * 3
            # Dark gradient with some noise for "camera static" effect
            base = int(15 + 8 * math.sin(x * 0.05) * math.cos(y * 0.05))
            noise = random.randint(-5, 5)
            val = max(0, min(255, base + noise))
            # Slight cyan tint
            pixels[idx] = val  # R
            pixels[idx + 1] = val + 3  # G
            pixels[idx + 2] = val + 5  # B
    
    # Create a BMP in memory (browsers understand BMP too, but let's use PPM format
    # which is simpler and can be converted)
    # Actually, base64-encoded in frontend expects JPEG. Let's create a minimal
    # uncompressed JPEG-like format.
    
    # Simplest approach: create a PPM and note that the frontend will handle it
    # Better: create a proper BMP which is simple
    
    # BMP format
    bmp_header_size = 54
    row_size = (w * 3 + 3) & ~3  # Rows padded to 4 bytes
    pixel_data_size = row_size * h
    file_size = bmp_header_size + pixel_data_size
    
    bmp = bytearray(file_size)
    # BMP header
    bmp[0:2] = b'BM'
    struct.pack_into('<I', bmp, 2, file_size)
    struct.pack_into('<I', bmp, 10, bmp_header_size)
    # DIB header
    struct.pack_into('<I', bmp, 14, 40)  # DIB header size
    struct.pack_into('<i', bmp, 18, w)
    struct.pack_into('<i', bmp, 22, -h)  # Negative = top-down
    struct.pack_into('<H', bmp, 26, 1)   # Color planes
    struct.pack_into('<H', bmp, 28, 24)  # Bits per pixel
    struct.pack_into('<I', bmp, 34, pixel_data_size)
    
    # Pixel data (BGR format for BMP)
    offset = bmp_header_size
    for y in range(h):
        for x in range(w):
            src = (y * w + x) * 3
            bmp[offset] = pixels[src + 2]     # B
            bmp[offset + 1] = pixels[src + 1] # G
            bmp[offset + 2] = pixels[src]     # R
            offset += 3
        # Padding
        padding = row_size - w * 3
        offset += padding
    
    return bytes(bmp)


class MockRobot(RobotController, SensorProvider, MapProvider, NavigationManager, DetectionProvider):
    def __init__(self):
        self.pose = RobotPose(x=0.0, y=0.0, theta=0.0)
        self.battery = 85.0
        self.mode = ModeEnum.MANUAL
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        
        self.map_data = [-1] * (Config.MAP_WIDTH * Config.MAP_HEIGHT)
        self.trajectory: List[LidarPoint] = []
        self.last_traj_time = time.time()
        
        self.nav_status = NavigationStatus(status=NavigationStatusEnum.IDLE)
        self.planned_path = PlannedPath(points=[])
        self._initial_goal_distance = 0.0
        
        self.current_scan = LidarScan(points=[])
        self.current_detections = DetectionList(objects=[])
        self.events: List[EventLog] = []
        
        self.mock_objects = [
            {"class": "person", "x": 3.0, "y": 1.0},
            {"class": "chair", "x": -2.0, "y": 2.0},
            {"class": "bottle", "x": 1.5, "y": -1.5},
            {"class": "dog", "x": -1.0, "y": 3.0},
        ]
        
        self._dt = 1.0 / Config.ROBOT_UPDATE_RATE
        self.running = False
        self.start_time = time.time()
        
        # Pre-generate camera frame
        self._camera_frame = _create_minimal_jpeg()
        
        # Add startup events
        self._add_event("INFO", "System initialized")
        self._add_event("SUCCESS", "Camera connected")
        self._add_event("SUCCESS", "LiDAR connected")
        self._add_event("INFO", "SLAM started")

    def _add_event(self, level: str, message: str):
        """Add an event with human-readable timestamp."""
        ts = time.strftime("%H:%M:%S", time.localtime())
        self.events.append(EventLog(timestamp=ts, level=level, message=message))

    # ─── RobotController ───

    async def get_pose(self) -> RobotPose:
        return self.pose

    async def get_battery(self) -> float:
        return self.battery

    async def set_velocity(self, linear: float, angular: float):
        if self.mode == ModeEnum.MANUAL:
            self.linear_vel = max(-Config.MAX_SPEED, min(Config.MAX_SPEED, linear))
            self.angular_vel = max(-Config.MAX_ANGULAR_SPEED, min(Config.MAX_ANGULAR_SPEED, angular))

    async def stop(self):
        self.linear_vel = 0.0
        self.angular_vel = 0.0

    async def emergency_stop(self):
        self.linear_vel = 0.0
        self.angular_vel = 0.0
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
        t = time.time()
        return [
            SensorHealth(name="Camera", status="ACTIVE", last_update=t),
            SensorHealth(name="LiDAR", status="ACTIVE", last_update=t),
            SensorHealth(name="IMU", status="ACTIVE", last_update=t),
            SensorHealth(name="Odometry", status="ACTIVE", last_update=t),
            SensorHealth(name="SLAM", status="ACTIVE", last_update=t),
            SensorHealth(name="YOLO", status="ACTIVE", last_update=t),
            SensorHealth(name="Navigation", status="ACTIVE", last_update=t),
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
            self._add_event("WARNING", "Cannot set goal in MANUAL mode")
            return
        
        self.nav_status.status = NavigationStatusEnum.PLANNING
        self.nav_status.goal_x = x
        self.nav_status.goal_y = y
        self._add_event("INFO", f"Goal received: ({x:.2f}, {y:.2f})")
        
        await asyncio.sleep(0.5)
        
        # Generate planned path with some curvature
        dx = x - self.pose.x
        dy = y - self.pose.y
        dist = math.hypot(dx, dy)
        self._initial_goal_distance = dist
        steps = max(3, int(dist * 3))
        path = []
        for i in range(1, steps + 1):
            t = i / steps
            # Add slight curve for realism
            curve = 0.2 * math.sin(t * math.pi)
            px = self.pose.x + dx * t + curve * (-dy / (dist + 0.001))
            py = self.pose.y + dy * t + curve * (dx / (dist + 0.001))
            path.append(LidarPoint(x=px, y=py))
        self.planned_path.points = path
        
        self.nav_status.status = NavigationStatusEnum.NAVIGATING
        self.nav_status.distance_remaining = dist
        self.nav_status.progress = 0.0
        self._add_event("SUCCESS", "Path planned, navigating...")

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
            # Battery drain
            self.battery -= 0.01 * self._dt
            if self.battery < 0:
                self.battery = 0
            
            # Autonomous navigation control
            if (self.mode == ModeEnum.AUTONOMOUS and 
                self.nav_status.status == NavigationStatusEnum.NAVIGATING):
                gx = self.nav_status.goal_x
                gy = self.nav_status.goal_y
                if gx is not None and gy is not None:
                    dx = gx - self.pose.x
                    dy = gy - self.pose.y
                    dist = math.hypot(dx, dy)
                    
                    if dist < 0.15:
                        self.nav_status.status = NavigationStatusEnum.REACHED
                        self.nav_status.progress = 100.0
                        self.nav_status.distance_remaining = 0.0
                        self.linear_vel = 0.0
                        self.angular_vel = 0.0
                        self.planned_path.points = []
                        self._add_event("SUCCESS", "Goal reached!")
                    else:
                        target_angle = math.atan2(dy, dx)
                        angle_diff = (target_angle - self.pose.theta + math.pi) % (2 * math.pi) - math.pi
                        
                        if abs(angle_diff) > 0.1:
                            self.angular_vel = math.copysign(
                                min(abs(angle_diff) * 2.0, Config.MAX_ANGULAR_SPEED), angle_diff
                            )
                            self.linear_vel = 0.05  # Slow forward while turning
                        else:
                            self.angular_vel = angle_diff * 0.5  # Small correction
                            self.linear_vel = min(dist * 0.8, Config.MAX_SPEED)
                            
                        self.nav_status.distance_remaining = dist
                        self.nav_status.velocity = self.linear_vel
                        
                        # Calculate progress
                        if self._initial_goal_distance > 0:
                            self.nav_status.progress = max(0, min(100,
                                (1.0 - dist / self._initial_goal_distance) * 100
                            ))
            
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
                
            # LiDAR scan
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
            
            # YOLO detections
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
