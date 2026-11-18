"""
Hardware Bridge Client for Physical JetBot.
Manages WebSocket connection /ws/robot to communicate with jetbot_bridge.py.
"""
import time
from typing import Optional, List
from fastapi import WebSocket
from app.models.schemas import SensorHealth


class BridgeClient:
    """Manages low-latency bidirectional communication with the physical JetBot bridge."""

    def __init__(self):
        self.ws: Optional[WebSocket] = None
        self._is_connected_override: Optional[bool] = None
        self.last_telemetry_time: float = 0.0
        self.last_lidar_time: float = 0.0
        self.last_camera_time: float = 0.0
        self.last_yolo_time: float = 0.0
        self.last_battery_time: float = 0.0

        self.camera_connected: bool = False
        self.lidar_connected: bool = False
        self.yolo_connected: bool = False
        self.battery_connected: bool = False

    @property
    def is_connected(self) -> bool:
        """Dynamic single source of truth: socket must exist and telemetry must be fresh (< 4.0s)."""
        if self._is_connected_override is False:
            return False
        return (self.ws is not None) and (time.time() - self.last_telemetry_time < 4.0)

    @is_connected.setter
    def is_connected(self, value: bool):
        self._is_connected_override = value

    async def register_connection(self, websocket: WebSocket):
        """Register newly connected physical JetBot WebSocket, evicting any stale socket."""
        if self.ws is not None and self.ws != websocket:
            try:
                await self.ws.close(code=1000, reason="Superseded by new connection")
            except Exception:
                pass
        self.ws = websocket
        self._is_connected_override = True
        self.last_telemetry_time = time.time()

    def unregister_connection(self, websocket: Optional[WebSocket] = None) -> bool:
        """Handle physical JetBot disconnection safely without killing active superseding connections."""
        if websocket is not None and self.ws is not None and self.ws != websocket:
            # Stale connection closing; do not touch active superseding connection
            return False
        self._is_connected_override = False
        self.ws = None
        self.camera_connected = False
        self.lidar_connected = False
        self.yolo_connected = False
        self.battery_connected = False
        return True

    async def send_mode(self, mode: str) -> bool:
        """Send operating mode (MANUAL or AUTONOMOUS) to JetBot bridge."""
        if self.ws and self.is_connected:
            try:
                await self.ws.send_json({"type": "set_mode", "mode": mode})
                return True
            except Exception:
                self.unregister_connection(self.ws)
        return False

    async def send_cmd_vel(self, linear: float, angular: float, mode: str = "MANUAL") -> bool:
        """Send differential drive velocity command to JetBot motors."""
        if self.ws and self.is_connected:
            try:
                await self.ws.send_json({
                    "type": "cmd_vel",
                    "linear": round(linear, 3),
                    "angular": round(angular, 3),
                    "mode": mode
                })
                return True
            except Exception:
                self.unregister_connection(self.ws)
        return False

    async def send_stop(self) -> bool:
        """Send emergency stop to JetBot motors."""
        if self.ws and self.is_connected:
            try:
                await self.ws.send_json({"type": "stop"})
                return True
            except Exception:
                self.unregister_connection(self.ws)
        return False

    def check_timeouts(self, now: float = None):
        """Update sensor connection flags based on recent telemetry timeouts."""
        if now is None:
            now = time.time()
        if now - self.last_camera_time > 3.5:
            self.camera_connected = False
        if now - self.last_lidar_time > 3.5:
            self.lidar_connected = False
        if now - self.last_yolo_time > 4.0:
            self.yolo_connected = False
        if now - self.last_telemetry_time > 4.0:
            self._is_connected_override = False

    def get_sensor_health(self, is_sim_running: bool = False) -> List[SensorHealth]:
        """Generate real-time health status for all hardware and software sensors."""
        now = time.time()
        self.check_timeouts(now)

        cam_status = "ACTIVE" if self.camera_connected else "OFFLINE"
        lidar_status = "ACTIVE" if self.lidar_connected else "OFFLINE"
        imu_status = "OFFLINE"
        odom_status = "ACTIVE" if (self.is_connected or is_sim_running) else "OFFLINE"
        slam_status = "ACTIVE" if (self.lidar_connected or is_sim_running) else "STANDBY"
        yolo_status = "ACTIVE" if self.yolo_connected else "OFFLINE"
        nav_status = "ACTIVE"

        return [
            SensorHealth(name="Camera", status=cam_status, last_update=self.last_camera_time or now),
            SensorHealth(name="LiDAR", status=lidar_status, last_update=self.last_lidar_time or now),
            SensorHealth(name="IMU", status=imu_status, last_update=now),
            SensorHealth(name="Odometry", status=odom_status, last_update=now),
            SensorHealth(name="SLAM", status=slam_status, last_update=now),
            SensorHealth(name="YOLO", status=yolo_status, last_update=self.last_yolo_time or now),
            SensorHealth(name="Navigation", status=nav_status, last_update=now),
        ]
