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
        disconnected = []
        for connection in self.active_connections:
            try:
                await connection.send_text(json.dumps(message))
            except Exception:
                disconnected.append(connection)
        for conn in disconnected:
            self.disconnect(conn)

    async def handle_message(self, websocket: WebSocket, message: dict):
        """Handle incoming WebSocket messages from frontend."""
        msg_type = message.get("type")
        
        if msg_type == "set_goal":
            await self.robot.set_goal(message["x"], message["y"])
        elif msg_type == "cancel_goal":
            await self.robot.cancel_goal()
        elif msg_type == "emergency_stop":
            await self.robot.emergency_stop()
        elif msg_type == "set_mode":
            await self.robot.set_mode(message["mode"])
        elif msg_type == "manual_control":
            cmd = message.get("command")
            linear = 0.0
            angular = 0.0
            if cmd == "FORWARD":
                linear = Config.MAX_SPEED
            elif cmd == "BACKWARD":
                linear = -Config.MAX_SPEED
            elif cmd == "LEFT" or cmd == "ROTATE_LEFT":
                angular = Config.MAX_ANGULAR_SPEED
            elif cmd == "RIGHT" or cmd == "ROTATE_RIGHT":
                angular = -Config.MAX_ANGULAR_SPEED
            elif cmd == "STOP":
                linear = 0.0
                angular = 0.0
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
                            await self.broadcast({
                                "type": "robot_pose",
                                "x": round(pose.x, 4),
                                "y": round(pose.y, 4),
                                "theta": round(pose.theta, 4),
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
                            await self.broadcast({
                                "type": "detections",
                                "objects": [d.model_dump() for d in dets.objects],
                                "timestamp": time.time()
                            })
                            
                        elif key == "navigation_status":
                            nav = await self.robot.get_nav_status()
                            await self.broadcast({
                                "type": "navigation_status",
                                **nav.model_dump(),
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
                            await self.broadcast({
                                "type": "sensor_health",
                                "sensors": [s.model_dump() for s in sh],
                                "timestamp": time.time()
                            })
                            
                        elif key == "camera_frame":
                            frame = await self.robot.get_camera_frame()
                            await self.broadcast({
                                "type": "camera_frame",
                                "data": base64.b64encode(frame).decode(),
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
                await self.broadcast({
                    "type": "event",
                    **ev.model_dump()
                })
                
            await asyncio.sleep(dt)
