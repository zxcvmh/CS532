"""
Bridge Adapter for Seamless Integration with JetBot WebSocket Backend
======================================================================
CS532 Autonomous JetBot — Module 07-imitation-learning
Based on UIT ITCourses RL-101 (Bài 10: Học bắt chước - Imitation Learning)

Usage:
  1. Standalone Autonomous Agent:
     python bridge_il_adapter.py --host 192.168.1.8 --port 8000 --goal-x 1.8 --goal-y 0.0
  2. Importable into jetbot_bridge.py or backend navigation coordinator
"""

import os
import sys
import json
import time
import asyncio
import argparse
from typing import Optional, List, Dict, Any, Tuple

try:
    import websockets
    HAS_WEBSOCKETS = True
except ImportError:
    HAS_WEBSOCKETS = False

from config import (
    DEFAULT_BRIDGE_HOST,
    DEFAULT_BRIDGE_PORT,
    CONTROL_HZ
)
from il_policy_runner import ILPolicyRunner


class BridgeILAdapter:
    """
    Connects trained Imitation Learning Policy to the WebSocket Bridge.
    Listens for LiDAR scans and sends optimal (v, w) commands.
    """
    def __init__(
        self,
        model_path: str = "07-imitation-learning/test_artifacts/models/dagger_policy.onnx",
        server_host: str = DEFAULT_BRIDGE_HOST,
        server_port: int = DEFAULT_BRIDGE_PORT
    ):
        self.runner = ILPolicyRunner(model_path=model_path)
        self.server_host = server_host
        self.server_port = server_port
        self.ws_url = f"ws://{server_host}:{server_port}/ws/robot"


        # Robot state
        self.goal_x = 1.80
        self.goal_y = 0.00
        self.robot_x = 0.00
        self.robot_y = 0.00
        self.robot_theta = 0.00
        self.curr_v = 0.00
        self.curr_w = 0.00

        self.latest_ranges: List[float] = [3.0] * 360
        self.running = False

    def set_goal(self, gx: float, gy: float):
        """Updates navigation goal target."""
        self.goal_x = float(gx)
        self.goal_y = float(gy)
        print(f"[IL ADAPTER] New Goal Set: ({self.goal_x:.2f}m, {self.goal_y:.2f}m)")

    def process_lidar_telemetry(self, ranges_360: List[float], pose_x: float, pose_y: float, pose_th: float) -> Tuple[float, float, Dict[str, Any]]:
        """Processes incoming LiDAR telemetry and returns autonomous control command."""
        self.latest_ranges = ranges_360
        self.robot_x = pose_x
        self.robot_y = pose_y
        self.robot_theta = pose_th

        v_cmd, w_cmd, info = self.runner.compute_command(
            lidar_ranges_360=self.latest_ranges,
            goal_x=self.goal_x,
            goal_y=self.goal_y,
            robot_x=self.robot_x,
            robot_y=self.robot_y,
            robot_theta=self.robot_theta,
            curr_v=self.curr_v,
            curr_w=self.curr_w
        )
        self.curr_v = v_cmd
        self.curr_w = w_cmd
        return v_cmd, w_cmd, info

    async def run_standalone_loop(self):
        """Standalone WebSocket client loop interacting with backend dashboard."""
        if not HAS_WEBSOCKETS:
            print("[ERROR] 'websockets' library is required to run standalone bridge adapter.")
            return

        print(f"[INFO] Connecting IL Policy Agent to: {self.ws_url}")
        self.running = True

        while self.running:
            try:
                async with websockets.connect(self.ws_url, ping_interval=3.0, ping_timeout=3.0) as ws:
                    print(f"[SUCCESS] IL Policy Agent connected to {self.ws_url}!")

                    # Set robot to autonomous mode
                    await ws.send(json.dumps({"type": "set_mode", "mode": "AUTONOMOUS"}))

                    last_ctrl_time = 0.0
                    interval = 1.0 / CONTROL_HZ

                    while self.running:
                        try:
                            msg_raw = await asyncio.wait_for(ws.recv(), timeout=interval)
                            # Handle incoming text JSON or binary telemetry
                            if isinstance(msg_raw, str):
                                msg = json.loads(msg_raw)
                                mtype = msg.get("type", "")
                                if mtype == "goal" or "goal_x" in msg:
                                    self.set_goal(float(msg.get("goal_x", 1.8)), float(msg.get("goal_y", 0.0)))
                        except asyncio.TimeoutError:
                            pass

                        now = time.time()
                        if now - last_ctrl_time >= interval:
                            v, w, info = self.process_lidar_telemetry(
                                self.latest_ranges,
                                self.robot_x, self.robot_y, self.robot_theta
                            )
                            # Dispatch cmd_vel back to robot
                            cmd_payload = {
                                "type": "cmd_vel",
                                "linear": v,
                                "angular": w,
                                "mode": "AUTONOMOUS"
                            }
                            await ws.send(json.dumps(cmd_payload))
                            last_ctrl_time = now

            except Exception as e:
                print(f"[WARN] Connection lost ({e}). Reconnecting in 2.0s...")
                await asyncio.sleep(2.0)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Imitation Learning Bridge Adapter (Standard ONNX Engine)")
    parser.add_argument("--host", type=str, default=DEFAULT_BRIDGE_HOST, help="Server host")
    parser.add_argument("--port", type=int, default=DEFAULT_BRIDGE_PORT, help="Server port")
    parser.add_argument("--goal-x", type=float, default=1.80, help="Initial goal x")
    parser.add_argument("--goal-y", type=float, default=0.00, help="Initial goal y")
    parser.add_argument("--model", "--weights", type=str, default="07-imitation-learning/test_artifacts/models/dagger_policy.onnx", help="Path to ONNX policy (.onnx)")
    args = parser.parse_args()

    adapter = BridgeILAdapter(model_path=args.model, server_host=args.host, server_port=args.port)
    adapter.set_goal(args.goal_x, args.goal_y)


    if HAS_WEBSOCKETS:
        asyncio.run(adapter.run_standalone_loop())
    else:
        print("[INFO] Standalone WebSocket test skipped (websockets not installed in this environment).")
