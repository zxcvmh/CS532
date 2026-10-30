"""
Navigation Coordinator and State Machine.
Coordinates motion planning, hardware bridge, simulation fallback, and telemetry synchronization.
"""
import asyncio
import logging
import math
import os
import time
from typing import List, Dict, Any, Optional
from app.config import Config
from app.models.schemas import (
    RobotPose, RobotStatus, LidarScan, LidarPoint, MapData,
    NavigationStatus, PlannedPath, DetectionList, ModeEnum,
    NavigationStatusEnum, EventLog
)
from app.hardware.bridge_client import BridgeClient
from app.hardware.telemetry_parser import TelemetryParser
from app.hardware.scan_gate import ScanGate
from app.simulation.mock_arena import MockArena
from app.streaming.mjpeg_streamer import MjpegStreamer
from app.navigation.motion_adapter import MotionAdapter

logger = logging.getLogger("jetbot.coordinator")


class NavigationCoordinator:
    """Central state machine coordinating hardware, planning, streaming, and UI events."""

    def __init__(self, bridge: BridgeClient = None, streamer: MjpegStreamer = None):
        self.bridge = bridge or BridgeClient()
        self.streamer = streamer or MjpegStreamer()
        self.motion = MotionAdapter()
        self.sim = MockArena()
        self.scan_gate = ScanGate()

        self.pose = RobotPose(x=0.0, y=0.0, theta=0.0)
        self.battery, self.mode = 85.0, ModeEnum.MANUAL
        self.linear_vel, self.angular_vel = 0.0, 0.0

        self.nav_status = NavigationStatus(status=NavigationStatusEnum.IDLE)
        self.planned_path, self.current_scan = PlannedPath(points=[]), LidarScan(points=[])
        self.current_detections = DetectionList(objects=[])
        self.trajectory, self.events = [], []

        self.latest_raw_ranges = [0.0] * 360
        self.latest_clusters, self.latest_clusters_dict, self.latest_social_bubbles = [], [], []

        self.map_data = [-1] * (Config.MAP_WIDTH * Config.MAP_HEIGHT)
        self.map_data_b64, self.map_version = None, 1

        self.motion_xai_info: Dict[str, Any] = {"status": "IDLE", "speed_scale": 1.0, "curvature_w": 0.0}
        # CV low-obstacle detections in the ROBOT frame, surfaced to the MapViewer
        # amber-diamond layer (Phase 4.2).
        self.low_obstacles: List[List[float]] = []
        # Wall-clock stamp of the last accepted ScanTick. 0.0 means "no hardware
        # telemetry yet", which disables the staleness guard (sim mode).
        self._last_scan_tick_time: float = 0.0
        self._stale_warn_time: float = 0.0
        # Phase 8 — 1 Hz health probe state. `lidar_diag` is the raw driver diagnostics
        # dict relayed by the bridge (Phase 1.3 evidence counters live on the Jetson).
        self.lidar_diag: Dict[str, Any] = {}
        self.latest_scan_seq: int = 0
        self.latest_tick_seq: int = 0
        self._last_health_log: float = 0.0
        self.running, self._dt = False, 1.0 / Config.ROBOT_UPDATE_RATE
        self._last_traj_time = self._last_map_update = self._last_odom_time = 0.0
        self._stop_start_time = self._last_replan_time = self._initial_goal_dist = 0.0
        self._replan_attempts, self._has_reset_real_map = 0, False

        # Module 07: Imitation Learning ONNX Navigation Policy
        self.il_runner = None
        try:
            from app.navigation.il_onnx_bridge import get_il_policy_runner
            self.il_runner = get_il_policy_runner()
            if self.il_runner:
                logger.info(f"[ONNX IL ENGINE] Initialized Production Engine: {self.il_runner.engine_type}")
                self.add_event("SUCCESS", "[ONNX IL Engine] Da tich hop mo hinh no-ron tu hanh vao Web Dashboard!")
        except Exception as ex_il:
            logger.warning(f"Failed to load ONNX IL runner: {ex_il}")


        self.add_event("INFO", "System initialized cleanly (Modular Architecture)")

    @property
    def is_real_connected(self) -> bool: return self.bridge.is_connected
    @property
    def grid_map(self): return self.motion.grid_map
    @property
    def controller(self): return self.motion.controller
    @property
    def planner(self): return self.motion.planner

    def on_hardware_disconnected(self):
        """Cleanly transition state when physical hardware drops."""
        self._has_reset_real_map = False
        self.add_event("WARNING", "Physical JetBot disconnected. Falling back to simulation.")

    def add_event(self, level: str, message: str):
        """Append human-readable system event."""
        ts = time.strftime("%H:%M:%S", time.localtime())
        self.events.append(EventLog(timestamp=ts, level=level, message=message))

    async def set_velocity(self, linear: float, angular: float):
        """Set robot velocities in MANUAL mode."""
        if self.mode == ModeEnum.MANUAL:
            self.linear_vel = max(-Config.MAX_SPEED, min(Config.MAX_SPEED, linear))
            self.angular_vel = max(-Config.MAX_ANGULAR_SPEED, min(Config.MAX_ANGULAR_SPEED, angular))
            await self.bridge.send_cmd_vel(self.linear_vel, self.angular_vel, mode=self.mode.value)

    async def stop(self):
        """Stop motors."""
        self.linear_vel, self.angular_vel = 0.0, 0.0
        await self.bridge.send_stop()

    async def emergency_stop(self):
        """Emergency stop and set error status if navigating."""
        await self.stop()
        if self.nav_status.status in (NavigationStatusEnum.NAVIGATING, NavigationStatusEnum.PLANNING):
            self.nav_status.status = NavigationStatusEnum.ERROR
        self.add_event("ERROR", "EMERGENCY STOP activated")

    async def set_mode(self, mode_str: str):
        """Change control mode."""
        self.mode = ModeEnum(mode_str)
        if self.mode == ModeEnum.MANUAL and self.nav_status.status in (NavigationStatusEnum.NAVIGATING, NavigationStatusEnum.PLANNING):
            self.nav_status.status = NavigationStatusEnum.CANCELLED
            await self.stop()
        await self.bridge.send_mode(self.mode.value)
        self.add_event("INFO", f"Mode changed to {mode_str}")

    async def reset_map(self):
        """Reset SLAM grid map, trajectory, and odometry."""
        self.pose = RobotPose(x=0.0, y=0.0, theta=0.0)
        self.motion.reset()
        self.map_data = self.motion.grid_map.to_dashboard_data()
        self.map_data_b64 = self.motion.grid_map.to_dashboard_b64()
        self.map_version += 1
        self.trajectory.clear()
        self.current_scan = LidarScan(points=[])
        self.latest_clusters, self.latest_clusters_dict = [], []
        self._stop_start_time = 0.0
        self.add_event("WARNING", "🧹 Map cleared! Rescanning environment...")

    async def reset_pose(self):
        """Reset pose to origin (0,0,0)."""
        self.pose = RobotPose(x=0.0, y=0.0, theta=0.0)
        self.trajectory.clear()
        self.motion.lidar_odometry.reset_pose(0.0, 0.0, 0.0)
        self.motion.grid_map.clear_footprint(
            0.0, 0.0, radius=Config.ROBOT_RADIUS, evict_clusters=False
        )
        self.motion.controller.clear_path()
        self.map_version += 1
        self.add_event("INFO", "🎯 Robot pose reset to (0, 0, 0).")

    async def set_goal(self, x: float, y: float):
        """Dispatch navigation goal and trigger ONNX autonomous navigation."""
        if self.mode != ModeEnum.AUTONOMOUS:
            self.mode = ModeEnum.AUTONOMOUS
            await self.bridge.send_mode(self.mode.value)

        self.motion.grid_map.clear_footprint(
            self.pose.x, self.pose.y, radius=Config.ROBOT_RADIUS, evict_clusters=False
        )
        self.nav_status.status = NavigationStatusEnum.PLANNING
        self.nav_status.goal_x, self.nav_status.goal_y = x, y
        self._initial_goal_dist = math.hypot(x - self.pose.x, y - self.pose.y)
        self.nav_status.distance_remaining = self._initial_goal_dist
        self.nav_status.progress = 0.0
        await self.stop()

        # Generate visual path line for web dashboard
        path = await asyncio.to_thread(self.motion.plan, (self.pose.x, self.pose.y), (x, y), self.pose.theta)
        if path and len(path) >= 2:
            self.planned_path.points = [LidarPoint(x=px, y=py) for px, py in path]
            self.motion.controller.set_path(path)
        else:
            # Direct line to goal if A* is blocked by noisy grid cells
            self.planned_path.points = [
                LidarPoint(x=self.pose.x, y=self.pose.y),
                LidarPoint(x=x, y=y)
            ]

        self.nav_status.status = NavigationStatusEnum.NAVIGATING
        self.add_event("SUCCESS", f"🚀 [ONNX IL] Mục tiêu mới ({x:.2f}m, {y:.2f}m) - Bắt đầu tự hành né cản!")

    async def cancel_goal(self):
        """Cancel current autonomous navigation goal."""
        if self.nav_status.status in (NavigationStatusEnum.PLANNING, NavigationStatusEnum.NAVIGATING):
            self.nav_status.status = NavigationStatusEnum.CANCELLED
            await self.stop()
            self.planned_path.points = []
            self.add_event("WARNING", "Navigation cancelled by user")

    async def handle_real_telemetry(self, data: Dict[str, Any]):
        """Ingest raw telemetry from physical JetBot."""
        TelemetryParser.apply_telemetry(data, self)

    async def handle_binary_telemetry(self, raw_bytes: bytes):
        """Ingest ultra-fast 1,130-byte JB-Binary frame from physical JetBot."""
        TelemetryParser.apply_binary_telemetry(raw_bytes, self)

    async def handle_camera_frame(self, data: Dict[str, Any]):
        """Ingest asynchronous camera and detection frame from physical JetBot."""
        cam = TelemetryParser.parse_camera_frame(data)
        if cam:
            self.streamer.update_frame(cam)
            self.bridge.last_camera_time = time.time()
            self.bridge.camera_connected = True
        bat = TelemetryParser.parse_battery(data)
        if bat is not None:
            self.battery = bat
        raw_dets = data.get("detections")
        dets, yolo_act = TelemetryParser.parse_detections(data)
        if dets.objects or yolo_act:
            self.current_detections = dets
            self.bridge.last_yolo_time = time.time()
            self.bridge.yolo_connected = True
            if raw_dets and isinstance(raw_dets, list):
                self.motion.grid_map.update_social_bubbles_from_detections(
                    detections=raw_dets,
                    robot_x=self.pose.x,
                    robot_y=self.pose.y,
                    robot_theta=self.pose.theta,
                    bubble_radius_m=0.40
                )
                self.latest_social_bubbles = [
                    {"x": round(b[0], 3), "y": round(b[1], 3), "radius": round(b[2], 2)}
                    for b in getattr(self.motion.grid_map, "social_bubbles", [])
                ]

    async def get_binary_telemetry(self) -> bytes:
        """Returns 1,130-byte JB-Binary frame (real hardware or simulated)."""
        now = time.time()
        if self.is_real_connected and hasattr(self, "latest_binary_telemetry") and self.latest_binary_telemetry:
            if now - self.bridge.last_telemetry_time < 2.0:
                from app.streaming.binary_protocol import patch_binary_pose
                return patch_binary_pose(
                    self.latest_binary_telemetry,
                    self.pose.x,
                    self.pose.y,
                    self.pose.theta,
                    self.linear_vel,
                    self.angular_vel,
                )

        if not hasattr(self, "_sim_binary_packer"):
            from app.streaming.binary_protocol import BinaryTelemetryPacker
            self._sim_binary_packer = BinaryTelemetryPacker()

        near_d = 0.0
        if hasattr(self, "current_scan") and self.current_scan and self.current_scan.points:
            vr = [math.hypot(p.x, p.y) for p in self.current_scan.points if 0.03 < math.hypot(p.x, p.y) < 12.0]
            if vr:
                near_d = min(vr)

        ranges = self.latest_raw_ranges if hasattr(self, "latest_raw_ranges") and self.latest_raw_ranges else [0.0]*360
        return self._sim_binary_packer.pack(
            pose_x=self.pose.x,
            pose_y=self.pose.y,
            pose_theta=self.pose.theta,
            linear_vel=self.linear_vel,
            angular_vel=self.angular_vel,
            nearest_obs=near_d,
            battery=self.battery,
            cpu_temp=43.5,
            ranges_m=ranges,
            timestamp=now
        )

    async def _step_navigation(self):
        """Execute one autonomous navigation cycle (10Hz)."""
        # Phase 4.5: never command motion on stale perception.
        last_tick = getattr(self, "_last_scan_tick_time", 0.0)
        if last_tick and (time.time() - last_tick) > 0.5:
            await self.stop()
            now_w = time.time()
            if now_w - getattr(self, "_stale_warn_time", 0.0) > 2.0:
                self._stale_warn_time = now_w
                self.add_event(
                    "WARNING",
                    "⚠️ No fresh LiDAR scan for >0.5s — holding position",
                )
            return

        gx = getattr(self.nav_status, "goal_x", 0.0)
        gy = getattr(self.nav_status, "goal_y", 0.0)
        rem_dist = math.hypot(gx - self.pose.x, gy - self.pose.y)
        self.nav_status.distance_remaining = rem_dist
        if getattr(self, "_initial_goal_dist", 0.0) > 0:
            self.nav_status.progress = max(0.0, min(100.0, (1.0 - rem_dist / self._initial_goal_dist) * 100.0))

        # Check goal reached tolerance (0.18m)
        if rem_dist <= 0.18:
            await self.stop()
            self.nav_status.status = NavigationStatusEnum.REACHED
            self.nav_status.progress = 100.0
            self.planned_path.points = []
            self.motion_xai_info["status"] = "GOAL_REACHED"
            self.add_event("SUCCESS", "🎯 [ONNX IL] Đã đến đích thành công!")
            return

        # ─── PRIMARY: IMITATION LEARNING ONNX POLICY RUNNER ───
        if self.il_runner is not None:
            ranges = self.latest_raw_ranges if (hasattr(self, "latest_raw_ranges") and self.latest_raw_ranges) else [3.0] * 360

            # Local lookahead waypoint extraction along planned path
            # For direct/room-scale goals (<= 2.0m), guide directly toward the real destination
            # to prevent straight-line lookaheads from pinning goals inside obstacles
            tx, ty = gx, gy
            if rem_dist > 2.0 and hasattr(self, "planned_path") and self.planned_path and self.planned_path.points and len(self.planned_path.points) >= 5:
                for pt in self.planned_path.points:
                    d_pt = math.hypot(pt.x - self.pose.x, pt.y - self.pose.y)
                    if d_pt >= 0.60:
                        tx, ty = pt.x, pt.y
                        break

            v, w, info = self.il_runner.compute_command(
                lidar_ranges_360=ranges,
                goal_x=tx,
                goal_y=ty,
                robot_x=self.pose.x,
                robot_y=self.pose.y,
                robot_theta=self.pose.theta,
                curr_v=self.linear_vel,
                curr_w=self.angular_vel
            )

            status_str = info.get("status", "NAVIGATING")
            if status_str == "GOAL_REACHED":
                await self.stop()
                self.nav_status.status = NavigationStatusEnum.REACHED
                self.nav_status.progress = 100.0
                self.planned_path.points = []
                self.add_event("SUCCESS", "🎯 [ONNX IL] Đã đến đích thành công!")
                return

            self.linear_vel, self.angular_vel = v, w
            self.nav_status.velocity = v
            self.motion_xai_info.update({
                "status": f"ONNX_{status_str}",
                "speed_scale": round(v / 0.16, 2) if v > 0 else 0.0,
                "curvature_w": round(w, 2),
                "latency_ms": info.get("latency_ms", 0.02)
            })
            await self.bridge.send_cmd_vel(v, w, mode=self.mode.value)
            return

        # ─── FALLBACK: Classical Motion Controller ───
        v, w, info = self.motion.compute_command(
            self.pose.x, self.pose.y, self.pose.theta,
            self.latest_raw_ranges, self.latest_clusters, auto_evade=True
        )
        status_str = info.get("status", "NAVIGATING")

        if status_str == "GOAL_REACHED" or rem_dist <= max(0.18, self.motion.controller.goal_tolerance):
            await self.stop()
            self.nav_status.status = NavigationStatusEnum.REACHED
            self.nav_status.progress = 100.0
            self.planned_path.points = []
            self.motion_xai_info["status"] = "GOAL_REACHED"
            self.add_event("SUCCESS", "🎯 Target Goal Reached Successfully!")
            return

        if status_str in ("EMERGENCY_STOP", "IMPASSABLE_STOP", "WAITING_CLEARANCE", "BLOCKED_WAIT"):
            await self.stop()
            return

        self.linear_vel, self.angular_vel = v, w
        self.nav_status.velocity = v
        await self.bridge.send_cmd_vel(v, w, mode=self.mode.value)


    async def _step_recovery(self):
        """Periodic background replan probe when autonomous navigation is blocked."""
        now_t = time.time()
        if now_t - getattr(self, "_last_recovery_time", 0.0) < 0.80:
            return
        self._last_recovery_time = now_t
        gx = getattr(self.nav_status, "goal_x", None)
        gy = getattr(self.nav_status, "goal_y", None)
        if gx is None or gy is None:
            return
        new_path = await asyncio.to_thread(
            self.motion.plan, (self.pose.x, self.pose.y), (gx, gy)
        )
        if new_path and len(new_path) >= 2:
            self.motion.controller.set_path(new_path)
            self.planned_path.points = [LidarPoint(x=p[0], y=p[1]) for p in new_path]
            self._stop_start_time, self._replan_attempts = 0.0, 0
            self.nav_status.status = NavigationStatusEnum.NAVIGATING
            self.add_event("SUCCESS", "✅ Obstacle cleared! Auto-resuming navigation to goal.")

    async def update(self):
        """Main lifecycle update loop running at ROBOT_UPDATE_RATE Hz."""
        self.running = True
        while self.running:
            now_t = time.time()
            is_hw = self.is_real_connected and (now_t - self.bridge.last_telemetry_time < 3.0)

            if not is_hw:
                self.battery = self.sim.simulate_battery(self.battery, self._dt)
                nx, ny, nth = self.sim.update_kinematics(self.pose.x, self.pose.y, self.pose.theta, self.linear_vel, self.angular_vel, self._dt)
                self.pose.x, self.pose.y, self.pose.theta = nx, ny, nth
                
                # Only inject simulated arena walls into grid_map if explicitly configured in pure sim mode
                enable_sim_map = os.environ.get("ENABLE_SIMULATION_MAP", "0") == "1"
                if enable_sim_map:
                    ranges, pts = self.sim.simulate_scan(nx, ny, nth)
                    self.latest_raw_ranges, self.current_scan = ranges, LidarScan(points=pts)
                    valid_cls, out_cls = self.motion.process_clusters(ranges, nx, ny, nth)
                    self.latest_clusters, self.latest_clusters_dict = valid_cls, out_cls

                    if now_t - self._last_map_update >= 0.09:
                        self.motion.grid_map.update_scan(nx, ny, nth, ranges=ranges, max_valid_range=Config.MAX_RANGE - 0.15)
                        self.map_data_b64 = self.motion.grid_map.to_dashboard_b64()
                        self.map_data = self.motion.grid_map.to_dashboard_data()
                        self.map_version += 1
                        self._last_map_update = now_t

            if now_t - self._last_traj_time > 0.1:
                self.trajectory.append(LidarPoint(x=self.pose.x, y=self.pose.y))
                self._last_traj_time = now_t
                if len(self.trajectory) > 5000:
                    self.trajectory = self.trajectory[-3000:]

            if self.mode == ModeEnum.AUTONOMOUS:
                if self.nav_status.status == NavigationStatusEnum.NAVIGATING:
                    await self._step_navigation()
                elif self.nav_status.status == NavigationStatusEnum.BLOCKED:
                    await self._step_recovery()

            self._maybe_log_health(now_t)

            await asyncio.sleep(self._dt)

    def _maybe_log_health(self, now_t: float) -> None:
        """One INFO line per second (Phase 8): enough to prove what the robot believed.

        Contains exactly the fields the operator needed and did not have: the LiDAR sequence
        numbers, the pose, the nav status, the active arena bounds, the map census, and the
        driver's intensity/range rejection counters.
        """
        if now_t - self._last_health_log < 10.0:
            return
        self._last_health_log = now_t

        try:
            raw = self.motion.grid_map.to_dashboard_bytes()
            total = len(raw) or 1
            unknown = 100.0 * raw.count(b"\xff") / total
            occupied = 100.0 * raw.count(b"\x64") / total
            free = 100.0 - unknown - occupied
        except Exception:
            unknown = free = occupied = float("nan")

        diag = self.lidar_diag or {}
        logger.info(
            "HEALTH scan_seq=%s tick_seq=%s pose=(%.2f,%.2f,%.1fdeg) nav=%s arena=%s "
            "map[unknown=%.1f%% free=%.1f%% occ=%.1f%%] rx_intensity=%s rx_range=%s "
            "plan=%s mode=%s hw=%s",
            self.latest_scan_seq,
            self.latest_tick_seq,
            self.pose.x,
            self.pose.y,
            math.degrees(self.pose.theta),
            self.nav_status.status.value
            if hasattr(self.nav_status.status, "value")
            else self.nav_status.status,
            Config.ARENA_BOUNDS,
            unknown,
            free,
            occupied,
            diag.get("rejected_by_intensity", "-"),
            diag.get("rejected_by_range", "-"),
            self.motion.last_plan_status,
            self.mode.value if hasattr(self.mode, "value") else self.mode,
            self.is_real_connected,
        )

    # UI / Dashboard Accessors
    async def get_pose(self) -> RobotPose: return self.pose
    async def get_battery(self) -> float: return self.battery
    async def get_mode(self) -> str: return self.mode.value
    async def get_map(self) -> MapData:
        data_copy = self.map_data.copy() if self.map_data is not None else None
        return MapData(resolution=Config.MAP_RESOLUTION, width=Config.MAP_WIDTH, height=Config.MAP_HEIGHT,
                       origin_x=Config.MAP_ORIGIN_X, origin_y=Config.MAP_ORIGIN_Y, data=data_copy, data_b64=self.map_data_b64)
    async def get_lidar_scan(self) -> LidarScan: return self.current_scan
    async def get_camera_frame(self) -> bytes: return self.streamer.get_frame()
    async def get_detections(self) -> DetectionList: return self.current_detections
    async def get_sensor_health(self) -> List[Any]: return self.bridge.get_sensor_health(self.running)
    async def get_trajectory(self) -> List[LidarPoint]: return self.trajectory
    async def get_nav_status(self) -> NavigationStatus: return self.nav_status
    async def get_planned_path(self) -> PlannedPath: return self.planned_path
    async def get_status(self) -> RobotStatus:
        return RobotStatus(connected=True, battery=self.battery, mode=self.mode, pose=self.pose,
                           linear_vel=self.linear_vel, angular_vel=self.angular_vel, sensors=await self.get_sensor_health())
    async def get_motion_xai(self) -> Dict[str, Any]:
        return {
            **self.motion_xai_info,
            "clusters": self.latest_clusters_dict,
            "social_bubbles": self.latest_social_bubbles,
            "low_obstacles": self.low_obstacles,
        }
