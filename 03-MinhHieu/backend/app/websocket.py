import asyncio
import json
import base64
import time
from typing import List
from fastapi import WebSocket, WebSocketDisconnect
from app.robot.mock_robot import MockRobot
from app.config import Config


class WebSocketManager:
    def __init__(self, robot: MockRobot):
        self.active_connections: List[WebSocket] = []
        self.robot = robot

    async def connect(self, websocket: WebSocket):
        await websocket.accept()
        self.active_connections.append(websocket)

    def disconnect(self, websocket: WebSocket):
        if websocket in self.active_connections:
            self.active_connections.remove(websocket)

    async def broadcast(self, message: dict):
        if not self.active_connections:
            return
        disconnected = []
        try:
            msg_text = json.dumps(message)
        except Exception:
            return
        for connection in self.active_connections:
            try:
                await connection.send_text(msg_text)
            except Exception:
                disconnected.append(connection)
        for conn in disconnected:
            self.disconnect(conn)

    async def handle_message(self, websocket: WebSocket, message: dict):
        """Handle incoming WebSocket messages from frontend."""
        msg_type = message.get("type", "")
        cmd = message.get("cmd") or message.get("command") or ""
        cmd_lower = str(cmd).strip().lower()
        msg_type_lower = str(msg_type).strip().lower()

        # UIT Tut 3 & Heartbeat: ping / pong
        if msg_type_lower == "ping" or cmd_lower == "ping":
            await websocket.send_json({
                "type": "pong",
                "t": message.get("t", time.time()),
                "server_time": time.time()
            })
            return

        # UIT Tut 3 differential drive command: {"cmd": "drive", "left": f, "right": f}
        if cmd_lower == "drive":
            l = float(message.get("left", 0.0))
            r = float(message.get("right", 0.0))
            track_width = 0.115
            linear = (l + r) / 2.0 * Config.MAX_SPEED
            angular = (r - l) / track_width * Config.MAX_ANGULAR_SPEED * 0.5
            await self.robot.set_velocity(linear, angular)
            if message.get("ack"):
                await websocket.send_json({"ok": True, "status": "drive"})
            return

        # Global stop / Emergency stop
        if cmd_lower in ("stop", "emergency_stop") or msg_type_lower in ("stop", "emergency_stop"):
            if "emergency" in cmd_lower or "emergency" in msg_type_lower:
                await self.robot.emergency_stop()
            else:
                await self.robot.stop()
            if message.get("ack"):
                await websocket.send_json({"ok": True, "status": "stopped"})
            return

        if msg_type == "set_goal":
            await self.robot.set_goal(message["x"], message["y"])
        elif msg_type == "cancel_goal":
            await self.robot.cancel_goal()
        elif msg_type == "emergency_stop":
            await self.robot.emergency_stop()
        elif msg_type == "reset_map":
            await self.robot.reset_map()
        elif msg_type == "reset_pose":
            await self.robot.reset_pose()
        elif msg_type == "set_mode":
            await self.robot.set_mode(message["mode"])
        elif msg_type == "manual_control":
            # Direct continuous velocity input from Virtual Joystick
            if "linear" in message and "angular" in message:
                try:
                    linear = float(message.get("linear", 0.0))
                    angular = float(message.get("angular", 0.0))
                    # Clamp within safe limits
                    linear = max(-Config.MAX_SPEED, min(Config.MAX_SPEED, linear))
                    angular = max(-Config.MAX_ANGULAR_SPEED, min(Config.MAX_ANGULAR_SPEED, angular))
                    await self.robot.set_velocity(linear, angular)
                    return
                except (ValueError, TypeError):
                    pass

            manual_cmd = str(message.get("command", "")).strip().upper()
            speed_limit = float(message.get("speed", Config.MAX_SPEED))
            linear = 0.0
            angular = 0.0
            if manual_cmd == "FORWARD":
                linear = speed_limit
            elif manual_cmd == "BACKWARD":
                linear = -speed_limit
            elif manual_cmd in ("LEFT", "ROTATE_LEFT"):
                angular = Config.MAX_ANGULAR_SPEED * (speed_limit / Config.MAX_SPEED if Config.MAX_SPEED > 0 else 1.0)
            elif manual_cmd in ("RIGHT", "ROTATE_RIGHT"):
                angular = -Config.MAX_ANGULAR_SPEED * (speed_limit / Config.MAX_SPEED if Config.MAX_SPEED > 0 else 1.0)
            elif manual_cmd == "STOP":
                linear = 0.0
                angular = 0.0
                await self.robot.stop()
                return

            await self.robot.set_velocity(linear, angular)

    async def broadcaster_task(self):
        """Background task that broadcasts data to all connected clients at configured rates."""
        timers = {k: 0.0 for k in Config.WS_RATES.keys()}
        dt = 0.05  # 20Hz broadcast loop
        
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
                        if key == "robot_pose":
                            pose = await self.robot.get_pose()
                            # Calculate nearest obstacle distance from laser points if available
                            nearest_dist = None
                            if hasattr(self.robot, "latest_scan") and self.robot.latest_scan:
                                valid_ranges = [math.hypot(p.x, p.y) for p in self.robot.latest_scan.points if 0.03 < math.hypot(p.x, p.y) < 12.0]
                                if valid_ranges:
                                    nearest_dist = round(min(valid_ranges), 2)
                            await self.broadcast({
                                "type": "robot_pose",
                                "x": round(pose.x, 4),
                                "y": round(pose.y, 4),
                                "theta": round(pose.theta, 4),
                                "linear_speed": round(getattr(self.robot, "linear_vel", 0.0), 2),
                                "angular_speed": round(getattr(self.robot, "angular_vel", 0.0), 2),
                                "nearest_obstacle": nearest_dist,
                                "cpu_temp": round(getattr(self.robot, "cpu_temp", 43.5 + 2.0 * abs(getattr(self.robot, "linear_vel", 0.0))), 1),
                                "timestamp": time.time()
                            })
                            
                        elif key == "lidar_scan":
                            scan = await self.robot.get_lidar_scan()
                            # Downsample for bandwidth: send every 4th point
                            points = [{"x": round(p.x, 3), "y": round(p.y, 3)} 
                                      for i, p in enumerate(scan.points) if i % 4 == 0]
                            await self.broadcast({
                                "type": "lidar_scan",
                                "points": points,
                                "timestamp": time.time()
                            })
                            
                        elif key == "map_update":
                            m = await self.robot.get_map()
                            await self.broadcast({
                                "type": "map_update",
                                "resolution": m.resolution,
                                "width": m.width,
                                "height": m.height,
                                "origin_x": m.origin_x,
                                "origin_y": m.origin_y,
                                "data": m.data,
                                "timestamp": time.time()
                            })
                            
                        elif key == "detections":
                            dets = await self.robot.get_detections()
                            # ponytail: compatible with both pydantic v1 (Python 3.6) and v2
                            def _dump(o): return o.model_dump() if hasattr(o, "model_dump") else o.dict()
                            await self.broadcast({
                                "type": "detections",
                                "objects": [_dump(d) for d in dets.objects],
                                "timestamp": time.time()
                            })
                            
                        elif key == "navigation_status":
                            nav = await self.robot.get_nav_status()
                            def _dump(o): return o.model_dump() if hasattr(o, "model_dump") else o.dict()
                            await self.broadcast({
                                "type": "navigation_status",
                                **_dump(nav),
                                "timestamp": time.time()
                            })
                            
                        elif key == "battery":
                            bat = await self.robot.get_battery()
                            await self.broadcast({
                                "type": "battery",
                                "percentage": round(bat, 1),
                                "timestamp": time.time()
                            })
                            
                        elif key == "sensor_health":
                            sh = await self.robot.get_sensor_health()
                            def _dump(o): return o.model_dump() if hasattr(o, "model_dump") else o.dict()
                            await self.broadcast({
                                "type": "sensor_health",
                                "sensors": [_dump(s) for s in sh],
                                "is_real_connected": getattr(self.robot, "is_real_connected", False),
                                "timestamp": time.time()
                            })
                            
                        elif key == "camera_frame":
                            frame = await self.robot.get_camera_frame()
                            if frame:
                                await self.broadcast({
                                    "type": "camera_frame",
                                    "data": base64.b64encode(frame).decode('ascii'),
                                    "width": 640,
                                    "height": 480,
                                    "timestamp": time.time()
                                })
                            
                        elif key == "trajectory":
                            traj = await self.robot.get_trajectory()
                            # Downsample trajectory for bandwidth
                            step = max(1, len(traj) // 500)
                            points = [{"x": round(p.x, 3), "y": round(p.y, 3)} 
                                      for i, p in enumerate(traj) if i % step == 0]
                            await self.broadcast({
                                "type": "trajectory",
                                "points": points,
                                "timestamp": time.time()
                            })
                            
                        elif key == "planned_path":
                            path = await self.robot.get_planned_path()
                            await self.broadcast({
                                "type": "planned_path",
                                "points": [{"x": round(p.x, 3), "y": round(p.y, 3)} for p in path.points],
                                "timestamp": time.time()
                            })
                            
                    except Exception as e:
                        pass  # Don't crash on broadcast errors
                        
            # Flush events
            while self.robot.events:
                ev = self.robot.events.pop(0)
                def _dump(o): return o.model_dump() if hasattr(o, "model_dump") else o.dict()
                await self.broadcast({
                    "type": "event",
                    **_dump(ev)
                })
                
            await asyncio.sleep(dt)
