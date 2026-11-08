"""
Live JetBot Flight Recorder & Comprehensive Evaluation Benchmark
================================================================
CS532 Autonomous JetBot — Module 07-imitation-learning
Evaluates live physical/simulated performance of Imitation Learning Navigation:

Metrics Measured:
  1. Loop Latency & Jitter: LiDAR packet timestamp -> Policy inference -> Motor dispatch (ms)
  2. Obstacle Clearance Profile: Minimum distance to nearest obstacle over time (m)
  3. Slip & Jerk Indices: Rate of change of angular velocity (rad/s^2) and linear acceleration (m/s^2)
  4. Deadband Compliance: 0% violation rate of motor stiction deadband (v in [0.01, 0.09] m/s)
  5. Trajectory Efficiency: Actual trajectory length vs. Straight-line Euclidean distance
  6. Safety Reflex Triggers: Emergency stop / evasion activations count
  7. Blackbox Flight Log Export: Full session telemetry saved to CSV & JSON
"""

import os
import sys
import time
import math
import json
import csv
import asyncio
import argparse
from typing import List, Tuple, Dict, Any, Optional


try:
    import websockets
    HAS_WEBSOCKETS = True
except ImportError:
    HAS_WEBSOCKETS = False

import numpy as np

from config import (
    DEFAULT_BRIDGE_HOST,
    DEFAULT_BRIDGE_PORT,
    CONTROL_HZ,
    MIN_DRIVE_SPEED,
    MAX_LINEAR_SPEED,
    MAX_ANGULAR_SPEED,
    GOAL_TOLERANCE_M
)
from il_policy_runner import ILPolicyRunner

# ANSI Color codes
CLR_RESET = "\033[0m"
CLR_BOLD = "\033[1m"
CLR_GREEN = "\033[92m"
CLR_RED = "\033[91m"
CLR_YELLOW = "\033[93m"
CLR_CYAN = "\033[96m"
CLR_MAGENTA = "\033[95m"


class LiveFlightRecorder:
    """
    Flight Telemetry Logger & Real-Time Performance Evaluator for JetBot.
    """
    def __init__(self, log_dir: str = "logs"):
        self.log_dir = log_dir
        os.makedirs(self.log_dir, exist_ok=True)
        self.session_id = time.strftime("run_%Y%m%d_%H%M%S")
        self.csv_path = os.path.join(self.log_dir, f"{self.session_id}_telemetry.csv")
        self.summary_path = os.path.join(self.log_dir, f"{self.session_id}_summary.json")

        self.records: List[Dict[str, Any]] = []
        self.start_time: float = 0.0
        self.last_record_time: float = 0.0

        # Accumulated metrics
        self.latencies: List[float] = []
        self.min_clearances: List[float] = []
        self.linear_accels: List[float] = []
        self.angular_jerks: List[float] = []
        self.deadband_violations: int = 0
        self.reflex_triggers: int = 0
        self.total_path_length_m: float = 0.0

        self.last_pose: Optional[Tuple[float, float]] = None
        self.last_v: float = 0.0
        self.last_w: float = 0.0
        self.initial_goal_dist: float = 0.0
        self.final_goal_dist: float = 0.0
        self.goal_reached: bool = False

    def start_session(self, initial_pose: Tuple[float, float], goal: Tuple[float, float]):
        self.start_time = time.time()
        self.last_record_time = self.start_time
        self.last_pose = initial_pose
        self.initial_goal_dist = math.hypot(goal[0] - initial_pose[0], goal[1] - initial_pose[1])
        print(f"\n{CLR_BOLD}{CLR_CYAN}========================================================================{CLR_RESET}")
        print(f"{CLR_BOLD}  JETBOT FLIGHT RECORDER ACTIVE: Session {self.session_id}{CLR_RESET}")
        print(f"  Initial Pose: ({initial_pose[0]:.2f}m, {initial_pose[1]:.2f}m) | Goal: ({goal[0]:.2f}m, {goal[1]:.2f}m) | Dist: {self.initial_goal_dist:.2f}m")
        print(f"{CLR_BOLD}{CLR_CYAN}========================================================================{CLR_RESET}\n")

    def log_step(
        self,
        robot_x: float,
        robot_y: float,
        robot_theta: float,
        cmd_v: float,
        cmd_w: float,
        nearest_obs_m: float,
        policy_latency_ms: float,
        status: str,
        goal_x: float,
        goal_y: float
    ):
        now = time.time()
        dt = max(1e-4, now - self.last_record_time)
        self.last_record_time = now

        # Path integration
        if self.last_pose is not None:
            step_dist = math.hypot(robot_x - self.last_pose[0], robot_y - self.last_pose[1])
            self.total_path_length_m += step_dist
        self.last_pose = (robot_x, robot_y)

        # Acceleration and Jerk calculation (anti-wheel-slip metrics)
        accel_v = abs(cmd_v - self.last_v) / dt
        jerk_w = abs(cmd_w - self.last_w) / dt
        self.last_v = cmd_v
        self.last_w = cmd_w

        # Deadband violation check: did command fall into motor stall zone?
        is_deadband_violation = (0.01 < cmd_v < MIN_DRIVE_SPEED)
        if is_deadband_violation:
            self.deadband_violations += 1

        if "REFLEX" in status or "EMERGENCY" in status:
            self.reflex_triggers += 1

        dist_goal = math.hypot(goal_x - robot_x, goal_y - robot_y)
        self.final_goal_dist = dist_goal
        if dist_goal <= GOAL_TOLERANCE_M or status == "GOAL_REACHED":
            self.goal_reached = True

        self.latencies.append(policy_latency_ms)
        self.min_clearances.append(nearest_obs_m)
        self.linear_accels.append(accel_v)
        self.angular_jerks.append(jerk_w)

        row = {
            "timestamp": round(now - self.start_time, 3),
            "x": round(robot_x, 3),
            "y": round(robot_y, 3),
            "theta_deg": round(math.degrees(robot_theta), 1),
            "cmd_v": round(cmd_v, 3),
            "cmd_w": round(cmd_w, 3),
            "accel_v": round(accel_v, 2),
            "jerk_w": round(jerk_w, 2),
            "nearest_obs_m": round(nearest_obs_m, 3),
            "dist_goal_m": round(dist_goal, 3),
            "latency_ms": round(policy_latency_ms, 3),
            "status": status,
            "deadband_violation": int(is_deadband_violation)
        }
        self.records.append(row)

    def finish_session_and_report(self) -> Dict[str, Any]:
        duration_s = max(1e-4, time.time() - self.start_time)
        avg_latency = float(np.mean(self.latencies)) if self.latencies else 0.0
        max_latency = float(np.max(self.latencies)) if self.latencies else 0.0
        min_clearance = float(np.min(self.min_clearances)) if self.min_clearances else 0.0
        avg_clearance = float(np.mean(self.min_clearances)) if self.min_clearances else 0.0
        max_accel = float(np.max(self.linear_accels)) if self.linear_accels else 0.0
        max_jerk = float(np.max(self.angular_jerks)) if self.angular_jerks else 0.0

        # Trajectory efficiency: straight line / actual path length
        efficiency = (self.initial_goal_dist / max(0.01, self.total_path_length_m)) * 100.0 if self.goal_reached else 0.0

        summary = {
            "session_id": self.session_id,
            "duration_seconds": round(duration_s, 2),
            "goal_reached": self.goal_reached,
            "initial_distance_m": round(self.initial_goal_dist, 3),
            "final_distance_m": round(self.final_goal_dist, 3),
            "total_path_length_m": round(self.total_path_length_m, 3),
            "trajectory_efficiency_pct": round(min(100.0, efficiency), 1),
            "average_latency_ms": round(avg_latency, 3),
            "max_latency_ms": round(max_latency, 3),
            "min_obstacle_clearance_m": round(min_clearance, 3),
            "avg_obstacle_clearance_m": round(avg_clearance, 3),
            "max_linear_accel_mps2": round(max_accel, 2),
            "max_angular_jerk_rads2": round(max_jerk, 2),
            "deadband_violations": self.deadband_violations,
            "reflex_triggers": self.reflex_triggers,
            "total_samples": len(self.records)
        }

        # 1. Export CSV
        if self.records:
            fieldnames = list(self.records[0].keys())
            with open(self.csv_path, "w", newline="", encoding="utf-8") as f:
                writer = csv.DictWriter(f, fieldnames=fieldnames)
                writer.writeheader()
                writer.writerows(self.records)

        # 2. Export Summary JSON
        with open(self.summary_path, "w", encoding="utf-8") as f:
            json.dump(summary, f, indent=2)

        # 3. Print Comprehensive Benchmark Report to Console
        print("\n" + CLR_BOLD + CLR_CYAN + "+" + "=" * 72 + "+" + CLR_RESET)
        print(CLR_BOLD + f"|       JETBOT FLIGHT EVALUATION REPORT -- {self.session_id}       |" + CLR_RESET)
        print(CLR_BOLD + CLR_CYAN + "+" + "=" * 72 + "+" + CLR_RESET)

        res_color = CLR_GREEN if self.goal_reached else CLR_RED
        print(f"| Outcome                     : {res_color}{'TARGET REACHED (PASS)' if self.goal_reached else 'INCOMPLETE / TIMEOUT'}{CLR_RESET}")
        print(f"| Duration                    : {summary['duration_seconds']} s (Steps: {len(self.records)})")
        print(f"| Path Traveled / Direct Dist : {summary['total_path_length_m']}m / {summary['initial_distance_m']}m")
        print(f"| Trajectory Efficiency       : {summary['trajectory_efficiency_pct']}%")
        print(f"| ---------------------------------------------------------------------- |")

        lat_color = CLR_GREEN if avg_latency < 0.50 else CLR_YELLOW
        print(f"| Policy Latency (Avg / Max)  : {lat_color}{summary['average_latency_ms']} ms / {summary['max_latency_ms']} ms{CLR_RESET} (< 0.5ms SLA)")

        clr_color = CLR_GREEN if min_clearance >= 0.15 else CLR_RED
        print(f"| Nearest Obstacle Margin     : {clr_color}{summary['min_obstacle_clearance_m']} m{CLR_RESET} (Avg: {summary['avg_obstacle_clearance_m']} m)")

        db_color = CLR_GREEN if self.deadband_violations == 0 else CLR_RED
        print(f"| Deadband Stiction Violations: {db_color}{self.deadband_violations}{CLR_RESET} (Expected: 0)")

        jerk_color = CLR_GREEN if max_jerk < 8.0 else CLR_YELLOW
        print(f"| Max Angular Jerk (Anti-Slip): {jerk_color}{summary['max_angular_jerk_rads2']} rad/s2{CLR_RESET}")
        print(f"| Safety Reflex Activations   : {self.reflex_triggers}")
        print(f"| ---------------------------------------------------------------------- |")
        print(f"| CSV Telemetry Log           : {self.csv_path}")
        print(f"| JSON Summary File           : {self.summary_path}")
        print(CLR_BOLD + CLR_CYAN + "+" + "=" * 72 + "+" + CLR_RESET + "\n")


        return summary


async def run_live_closed_loop_test(
    server_host: str = DEFAULT_BRIDGE_HOST,
    server_port: int = DEFAULT_BRIDGE_PORT,
    goal_x: float = 1.80,
    goal_y: float = 0.00,
    weights_path: str = "models/dagger_policy_numpy.json",
    max_duration_s: float = 45.0
):
    """
    Connects to live JetBot WebSocket, runs Imitation Learning Policy, and benchmarks performance.
    """
    if not HAS_WEBSOCKETS:
        print(f"{CLR_RED}[ERROR] 'websockets' library is required to run live evaluator.{CLR_RESET}")
        return

    runner = ILPolicyRunner(weights_path=weights_path)
    recorder = LiveFlightRecorder(log_dir="07-imitation-learning/logs")
    ws_url = f"ws://{server_host}:{server_port}/ws"

    print(f"\n[EVALUATOR] Connecting to JetBot WebSocket Hub at: {ws_url}...")

    try:
        async with websockets.connect(ws_url, ping_interval=3.0, ping_timeout=3.0) as ws:
            print(f"{CLR_GREEN}[SUCCESS] Connected to Backend Hub! Subscribed to live telemetry.{CLR_RESET}")
            # Subscribe to real-time binary LiDAR stream & set manual control
            await ws.send(json.dumps({"type": "enable_binary_telemetry", "enabled": True}))
            await ws.send(json.dumps({"type": "set_mode", "mode": "MANUAL"}))

            # Initialize recorder session
            recorder.start_session(initial_pose=(0.0, 0.0), goal=(goal_x, goal_y))

            t_start = time.time()
            robot_pose = [0.0, 0.0, 0.0]
            latest_ranges = [3.0] * 360
            curr_v = 0.0
            curr_w = 0.0

            step_interval = 1.0 / CONTROL_HZ

            while time.time() - t_start < max_duration_s:
                t_cycle_start = time.perf_counter()

                # Read incoming telemetry from JetBot
                try:
                    raw_msg = await asyncio.wait_for(ws.recv(), timeout=step_interval)
                    if isinstance(raw_msg, bytes) and len(raw_msg) >= 1130 and raw_msg[:2] == b"JB":
                        # Fast unpack ranges from binary packet
                        body = raw_msg[18:]
                        # Ranges are 360 uint16 values in mm
                        ranges_mm = np.frombuffer(body[32:32 + 720], dtype=np.uint16)
                        latest_ranges = (ranges_mm / 1000.0).tolist()
                    elif isinstance(raw_msg, str):
                        data = json.loads(raw_msg)
                        mtype = data.get("type")
                        if mtype == "robot_pose":
                            robot_pose = [data.get("x", 0.0), data.get("y", 0.0), data.get("theta", 0.0)]
                        elif "pose" in data:
                            robot_pose = [data["pose"].get("x", 0.0), data["pose"].get("y", 0.0), data["pose"].get("theta", 0.0)]
                        elif mtype in ("navigation_status", "goal", "set_goal") or ("goal_x" in data and "goal_y" in data):
                            new_gx = data.get("goal_x") if data.get("goal_x") is not None else data.get("x")
                            new_gy = data.get("goal_y") if data.get("goal_y") is not None else data.get("y")
                            if new_gx is not None and new_gy is not None:
                                goal_x, goal_y = float(new_gx), float(new_gy)
                                print(f"\n[GOAL UPDATED] New Target Goal from Website: ({goal_x:.2f}m, {goal_y:.2f}m)")
                except asyncio.TimeoutError:
                    pass


                # Execute policy step (< 0.1ms)
                v_cmd, w_cmd, info = runner.compute_command(
                    lidar_ranges_360=latest_ranges,
                    goal_x=goal_x, goal_y=goal_y,
                    robot_x=robot_pose[0], robot_y=robot_pose[1], robot_theta=robot_pose[2],
                    curr_v=curr_v, curr_w=curr_w
                )
                curr_v, curr_w = v_cmd, w_cmd

                # Compute nearest obstacle distance
                valid_r = [r for r in latest_ranges if 0.05 < r < 8.0]
                nearest_obs = min(valid_r) if valid_r else 3.0

                # Log step to flight recorder
                recorder.log_step(
                    robot_x=robot_pose[0],
                    robot_y=robot_pose[1],
                    robot_theta=robot_pose[2],
                    cmd_v=v_cmd,
                    cmd_w=w_cmd,
                    nearest_obs_m=nearest_obs,
                    policy_latency_ms=info.get("latency_ms", 0.1),
                    status=info.get("status", "NAVIGATING"),
                    goal_x=goal_x,
                    goal_y=goal_y
                )

                # Send command to JetBot via WebSocket Hub
                cmd_payload = {
                    "type": "manual_control",
                    "linear": v_cmd,
                    "angular": w_cmd
                }
                await ws.send(json.dumps(cmd_payload))


                # Display live terminal telemetry line
                print(
                    f"\r  [{time.time()-t_start:4.1f}s] Pose: ({robot_pose[0]:.2f}, {robot_pose[1]:.2f}) | "
                    f"Cmd: v={v_cmd:.2f}m/s w={w_cmd:+.2f}r/s | NearObs: {nearest_obs:.2f}m | "
                    f"DistGoal: {math.hypot(goal_x - robot_pose[0], goal_y - robot_pose[1]):.2f}m | "
                    f"Status: {info.get('status')}   ",
                    end="",
                    flush=True
                )

                if info.get("status") == "GOAL_REACHED":
                    print(f"\n{CLR_GREEN}[SUCCESS] Target Goal Reached!{CLR_RESET}")
                    # Stop robot
                    await ws.send(json.dumps({"type": "cmd_vel", "linear": 0.0, "angular": 0.0}))
                    break

                elapsed = time.perf_counter() - t_cycle_start
                await asyncio.sleep(max(0.001, step_interval - elapsed))

    except Exception as e:
        print(f"\n{CLR_RED}[ERROR] Evaluation connection failed: {e}{CLR_RESET}")

    finally:
        recorder.finish_session_and_report()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Live JetBot Flight Evaluator & Benchmark (Standard ONNX Engine)")
    parser.add_argument("--host", type=str, default=DEFAULT_BRIDGE_HOST, help="Server host (e.g. 127.0.0.1 or laptop IP)")
    parser.add_argument("--port", type=int, default=DEFAULT_BRIDGE_PORT, help="Server port (default: 8000)")
    parser.add_argument("--goal-x", type=float, default=1.80, help="Target Goal X in meters")
    parser.add_argument("--goal-y", type=float, default=0.00, help="Target Goal Y in meters")
    parser.add_argument("--model", "--weights", type=str, default="07-imitation-learning/test_artifacts/models/dagger_policy.onnx", help="Path to standard ONNX policy (.onnx)")
    parser.add_argument("--timeout", type=float, default=30.0, help="Maximum test duration in seconds")
    args = parser.parse_args()

    asyncio.run(run_live_closed_loop_test(
        server_host=args.host,
        server_port=args.port,
        goal_x=args.goal_x,
        goal_y=args.goal_y,
        weights_path=args.model,
        max_duration_s=args.timeout
    ))

