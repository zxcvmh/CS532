"""
WebSocket Hub for JetBot Web Dashboard.
Manages dashboard clients (/ws), handles teleoperation commands, and broadcasts telemetry at configured rates.
"""
import asyncio
import json
import base64
import time
import math
from typing import List, Dict, Any
from fastapi import WebSocket
from app.config import Config


def _dump(o):
    return o.model_dump() if hasattr(o, "model_dump") else o.dict()


class WebSocketHub:
    """Manages Web Dashboard WebSocket clients and real-time telemetry dispatching."""

    def __init__(self, coordinator):
        self.active_connections: List[WebSocket] = []
        self.binary_connections: set = set()
        self.robot = coordinator
        self._last_broadcast_map_ver: int = -1

    async def connect(self, websocket: WebSocket):
        await websocket.accept()
        self.active_connections.append(websocket)
        self._last_broadcast_map_ver = -1

    def disconnect(self, websocket: WebSocket):
        if websocket in self.active_connections:
            self.active_connections.remove(websocket)
        self.binary_connections.discard(websocket)

    async def broadcast(self, message: dict):
        if not self.active_connections:
            return
        disconnected, msg_text = [], json.dumps(message)
        for conn in list(self.active_connections):
            try:
                await conn.send_text(msg_text)
            except Exception:
                disconnected.append(conn)
        for conn in disconnected:
            self.disconnect(conn)

    async def broadcast_bytes(self, raw_bytes: bytes):
        """Broadcast raw binary frame to subscribed binary clients."""
        if not self.binary_connections:
            return
        disconnected = []
        for conn in list(self.binary_connections):
            try:
                await conn.send_bytes(raw_bytes)
            except Exception:
                disconnected.append(conn)
        for conn in disconnected:
            self.disconnect(conn)

    async def handle_message(self, websocket: WebSocket, message: dict):
        """Dispatch incoming commands from frontend Dashboard."""
        msg_t = str(message.get("type", "")).strip().lower()
        cmd = str(message.get("cmd") or message.get("command") or "").strip().lower()

        if msg_t == "ping" or cmd == "ping":
            await websocket.send_json({"type": "pong", "t": message.get("t", time.time()), "server_time": time.time()})
            return

        if msg_t == "enable_binary_telemetry":
            if message.get("enabled", True):
                self.binary_connections.add(websocket)
                await websocket.send_json({"type": "binary_telemetry_status", "enabled": True})
            else:
                self.binary_connections.discard(websocket)
            return

        if cmd == "drive":
            l, r = float(message.get("left", 0.0)), float(message.get("right", 0.0))
            lin = (l + r) / 2.0 * Config.MAX_SPEED
            ang = (r - l) / 0.115 * Config.MAX_ANGULAR_SPEED * 0.5
            await self.robot.set_velocity(lin, ang)
            if message.get("ack"):
                await websocket.send_json({"ok": True, "status": "drive"})
            return

        if cmd in ("stop", "emergency_stop") or msg_t in ("stop", "emergency_stop"):
            if "emergency" in cmd or "emergency" in msg_t:
                await self.robot.emergency_stop()
            else:
                await self.robot.stop()
            if message.get("ack"):
                await websocket.send_json({"ok": True, "status": "stopped"})
            return

        if msg_t == "set_goal":
            await self.robot.set_goal(message["x"], message["y"])
        elif msg_t == "cancel_goal":
            await self.robot.cancel_goal()
        elif msg_t == "reset_map":
            await self.robot.reset_map()
        elif msg_t == "reset_pose":
            await self.robot.reset_pose()
        elif msg_t == "set_mode":
            await self.robot.set_mode(message["mode"])
        elif msg_t == "manual_control":
            if "linear" in message and "angular" in message:
                lin = max(-Config.MAX_SPEED, min(Config.MAX_SPEED, float(message.get("linear", 0.0))))
                ang = max(-Config.MAX_ANGULAR_SPEED, min(Config.MAX_ANGULAR_SPEED, float(message.get("angular", 0.0))))
                await self.robot.set_velocity(lin, ang)
                return
            mcmd = str(message.get("command", "")).strip().upper()
            spd = float(message.get("speed", Config.MAX_SPEED))
            lin, ang = 0.0, 0.0
            if mcmd == "FORWARD": lin = spd
            elif mcmd == "BACKWARD": lin = -spd
            elif mcmd in ("LEFT", "ROTATE_LEFT"): ang = Config.MAX_ANGULAR_SPEED * (spd / Config.MAX_SPEED if Config.MAX_SPEED > 0 else 1.0)
            elif mcmd in ("RIGHT", "ROTATE_RIGHT"): ang = -Config.MAX_ANGULAR_SPEED * (spd / Config.MAX_SPEED if Config.MAX_SPEED > 0 else 1.0)
            elif mcmd == "STOP": await self.robot.stop(); return
            await self.robot.set_velocity(lin, ang)

    async def _broadcast_channel(self, key: str):
        """Broadcast payload for a specific channel key."""
        now = time.time()
        if key == "robot_pose":
            pose = await self.robot.get_pose()
            near_d = None
            if hasattr(self.robot, "current_scan") and self.robot.current_scan:
                vr = [math.hypot(p.x, p.y) for p in self.robot.current_scan.points if 0.03 < math.hypot(p.x, p.y) < 12.0]
                if vr: near_d = round(min(vr), 2)
            await self.broadcast({
                "type": "robot_pose", "x": round(pose.x, 4), "y": round(pose.y, 4), "theta": round(pose.theta, 4),
                "linear_speed": round(getattr(self.robot, "linear_vel", 0.0), 2),
                "angular_speed": round(getattr(self.robot, "angular_vel", 0.0), 2),
                "nearest_obstacle": near_d, "cpu_temp": round(43.5 + 2.0 * abs(getattr(self.robot, "linear_vel", 0.0)), 1),
                "timestamp": now
            })
        elif key == "lidar_scan":
            scan = await self.robot.get_lidar_scan()
            if scan and getattr(scan, "points", None):
                pts = [{"x": round(p.x, 3), "y": round(p.y, 3)} for p in scan.points]
                if pts:
                    await self.broadcast({"type": "lidar_scan", "points": pts, "timestamp": now})
        elif key == "map_update":
            cur_ver = getattr(self.robot, "map_version", 0)
            if cur_ver != self._last_broadcast_map_ver:
                m = await self.robot.get_map()
                msg = {"type": "map_update", "resolution": m.resolution, "width": m.width, "height": m.height,
                       "origin_x": m.origin_x, "origin_y": m.origin_y, "timestamp": now}
                if m.data_b64: msg["data_b64"] = m.data_b64
                if m.data is not None and not m.data_b64: msg["data"] = m.data
                await self.broadcast(msg)
                self._last_broadcast_map_ver = cur_ver
        elif key == "detections":
            dets = await self.robot.get_detections()
            await self.broadcast({"type": "detections", "objects": [_dump(d) for d in dets.objects], "timestamp": now})
        elif key == "navigation_status":
            nav = await self.robot.get_nav_status()
            await self.broadcast({"type": "navigation_status", **_dump(nav), "timestamp": now})
        elif key == "battery":
            bat = await self.robot.get_battery()
            await self.broadcast({"type": "battery", "percentage": round(bat, 1), "timestamp": now})
        elif key == "sensor_health":
            sh = await self.robot.get_sensor_health()
            await self.broadcast({"type": "sensor_health", "sensors": [_dump(s) for s in sh],
                                  "is_real_connected": getattr(self.robot, "is_real_connected", False), "timestamp": now})
        elif key == "camera_frame":
            frame = await self.robot.get_camera_frame()
            if frame:
                await self.broadcast({"type": "camera_frame", "data": base64.b64encode(frame).decode('ascii'),
                                      "width": 640, "height": 480, "timestamp": now})
        elif key == "trajectory":
            traj = await self.robot.get_trajectory()
            step = max(1, len(traj) // 500)
            pts = [{"x": round(p.x, 3), "y": round(p.y, 3)} for i, p in enumerate(traj) if i % step == 0]
            await self.broadcast({"type": "trajectory", "points": pts, "timestamp": now})
        elif key == "planned_path":
            path = await self.robot.get_planned_path()
            await self.broadcast({"type": "planned_path", "points": [{"x": round(p.x, 3), "y": round(p.y, 3)} for p in path.points], "timestamp": now})
        elif key == "motion_xai":
            xai = await self.robot.get_motion_xai()
            await self.broadcast({"type": "motion_xai", **xai, "timestamp": now})

    async def broadcaster_task(self):
        """Background broadcast loop running at configured rates."""
        timers = {k: 0.0 for k in Config.WS_RATES.keys()}
        dt = 0.02
        while True:
            if not self.active_connections:
                await asyncio.sleep(dt)
                continue
            for key, rate in Config.WS_RATES.items():
                timers[key] += dt
                period = 1.0 / rate if rate > 0 else 999999
                if timers[key] >= period:
                    timers[key] = 0.0
                    try:
                        await self._broadcast_channel(key)
                    except Exception as e:
                        import sys
                        print(f"[WARN] Broadcast error on {key}: {e}", file=sys.stderr)

            # Ultra-fast JB-Binary telemetry broadcast (28 Hz) to subscribed binary clients
            if self.binary_connections:
                timers.setdefault("binary_telemetry", 0.0)
                timers["binary_telemetry"] += dt
                if timers["binary_telemetry"] >= 0.035:
                    timers["binary_telemetry"] = 0.0
                    try:
                        bin_frame = await self.robot.get_binary_telemetry()
                        if bin_frame:
                            await self.broadcast_bytes(bin_frame)
                    except Exception as e:
                        import sys
                        print(f"[WARN] Binary broadcast error: {e}", file=sys.stderr)

            while self.robot.events:
                ev = self.robot.events.pop(0)
                await self.broadcast({"type": "event", **_dump(ev)})
            await asyncio.sleep(dt)
