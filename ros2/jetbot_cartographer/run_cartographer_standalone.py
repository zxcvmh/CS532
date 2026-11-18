#!/usr/bin/env python3
"""
Standalone Google Cartographer 2D SLAM Runner (Zero ROS 2 Dependency)
=====================================================================
Reference: Wolfgang Hess et al. (Google Cartographer 2016), Edwin Olson (2009).
Runs pure Python Cartographer 2D SLAM on any Linux OS (Arch, CachyOS, Ubuntu)
without requiring ROS 2 or colcon.

Pipeline:
1. Connects to JetBot stream (ws://127.0.0.1:8000/ws/robot or direct bridge).
2. Unpacks 1,130-byte JB-Binary composite frames.
3. Real-Time Correlative Scan Matcher (CSM): Resolves translation and rotation without drift.
4. ActiveSubmaps2D: Dual overlapping submaps with odds updates.
5. PoseGraph2D: Sparse Pose Adjustment and Loop Closure.
6. Publishes OccupancyGrid and robot pose directly to Web Dashboard (ws://127.0.0.1:8000/ws).
"""
import sys
import os
import math
import time
import base64
import json
import asyncio
import struct
import numpy as np
from typing import Tuple, List, Optional

# Add cartographer module to sys.path
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from cartographer_core.probability_grid import ProbabilityGrid2D
from cartographer_core.correlative_scan_matcher import RealTimeCorrelativeScanMatcher2D
from cartographer_core.submap_2d import ActiveSubmaps2D, Submap2D
from cartographer_core.pose_graph_2d import PoseGraph2D
from scripts.jetbot_ros_bridge_node import unpack_jb_frame, TOTAL_FRAME_SIZE

try:
    import websockets
except ImportError:
    print("[ERROR] 'websockets' library is required. Install via: pip install websockets")
    sys.exit(1)


class StandaloneCartographer2D:
    """Standalone Cartographer 2D SLAM Pipeline."""

    def __init__(
        self,
        resolution: float = 0.05,
        map_width: int = 160,
        map_height: int = 160,
        submap_scans: int = 35,
        min_score_loop: float = 0.65
    ):
        self.resolution = float(resolution)
        self.map_width = int(map_width)
        self.map_height = int(map_height)
        self.origin_x = -0.5 * map_width * resolution
        self.origin_y = -0.5 * map_height * resolution

        # Global combined map for dashboard visualization
        self.global_map = ProbabilityGrid2D(
            resolution=self.resolution,
            width=self.map_width,
            height=self.map_height,
            origin_x=self.origin_x,
            origin_y=self.origin_y
        )

        # Local SLAM: Real-Time Correlative Scan Matcher
        self.csm = RealTimeCorrelativeScanMatcher2D(
            linear_search_window=0.15,
            angular_search_window_rad=math.radians(20.0),
            translation_weight=0.1,
            rotation_weight=0.1
        )

        # Submaps Manager
        self.submaps_mgr = ActiveSubmaps2D(
            num_range_data=submap_scans,
            resolution=self.resolution,
            grid_width=self.map_width,
            grid_height=self.map_height
        )

        # Global SLAM: Pose Graph & Loop Closure
        self.pose_graph = PoseGraph2D(min_score=min_score_loop)

        self.current_pose: Tuple[float, float, float] = (0.0, 0.0, 0.0)
        self.last_pose: Tuple[float, float, float] = (0.0, 0.0, 0.0)
        self.last_stamp: float = 0.0
        self.node_counter: int = 0
        self.total_scans_processed: int = 0
        self.is_first_scan: bool = True

    def process_scan(self, frame: dict) -> Tuple[Tuple[float, float, float], float, int]:
        """
        Process incoming 1,130-byte JB-Binary scan:
        1. Extract and filter range points with lever arm offset (+0.08m).
        2. Scan match against active submap.
        3. Insert range data into active submaps.
        4. Register pose graph node and evaluate loop closures.
        """
        ranges_mm = frame["ranges_mm"]
        stamp = frame.get("timestamp", time.time())
        v = frame.get("linear_vel", 0.0)
        w = frame.get("angular_vel", 0.0)

        # 1. Transform 360 ranges into local 2D point cloud in base_link frame
        pts_local = []
        angle_step = (2.0 * math.pi) / 360.0
        laser_offset_x = 0.08  # Lever arm 8cm

        for deg in range(360):
            r_mm = ranges_mm[deg]
            if 50 < r_mm <= 6000:
                r_m = float(r_mm) / 1000.0
                ang = deg * angle_step
                lx = r_m * math.cos(ang) + laser_offset_x
                ly = r_m * math.sin(ang)
                pts_local.append([lx, ly])

        if len(pts_local) < 10:
            return self.current_pose, 0.0, 0

        pts_arr = np.array(pts_local, dtype=np.float32)

        # 2. Predict initial pose via dead-reckoning motion model
        dt = max(0.01, min(0.2, stamp - self.last_stamp)) if self.last_stamp > 0 else 0.1
        self.last_stamp = stamp

        pred_x = self.current_pose[0] + v * dt * math.cos(self.current_pose[2])
        pred_y = self.current_pose[1] + v * dt * math.sin(self.current_pose[2])
        pred_th = self.current_pose[2] + w * dt

        if self.is_first_scan:
            self.current_pose = (0.0, 0.0, 0.0)
            self.is_first_scan = False
            score = 1.0
        else:
            # Correlative Scan Match against primary active submap
            active_sub = self.submaps_mgr.submaps[0] if self.submaps_mgr.submaps else None
            target_grid = active_sub.grid if active_sub else self.global_map
            matched_pose, score = self.csm.match(target_grid, pts_arr, (pred_x, pred_y, pred_th))
            self.current_pose = matched_pose

        # 3. Insert range data into ActiveSubmaps2D
        cx, cy, cth = self.current_pose
        c, s = math.cos(cth), math.sin(cth)
        world_pts = np.column_stack([
            pts_arr[:, 0] * c - pts_arr[:, 1] * s + cx,
            pts_arr[:, 0] * s + pts_arr[:, 1] * c + cy
        ])

        active_subs = self.submaps_mgr.insert_range_data(self.current_pose, world_pts)

        # Also update global visualization grid
        self.global_map.insert_scan(cx, cy, world_pts)

        # 4. Add node to PoseGraph2D and check loop closures
        node = self.pose_graph.add_node(
            node_id=self.node_counter,
            pose=self.current_pose,
            timestamp=stamp,
            points_local=pts_arr,
            active_submaps=active_subs
        )
        self.node_counter += 1
        self.total_scans_processed += 1

        new_closures = 0
        if self.node_counter % 5 == 0 and len(self.submaps_mgr.finished_submaps) > 0:
            new_closures = self.pose_graph.check_loop_closures(node, self.submaps_mgr.finished_submaps)
            if new_closures > 0:
                self.current_pose = node.pose

        return self.current_pose, score, new_closures

    def get_map_payload(self) -> dict:
        """Encode global probability grid into OccupancyGrid JSON format for Web Dashboard."""
        occ = self.global_map.to_occupancy_grid()
        raw_bytes = bytes([b if b >= 0 else (256 + b) for b in occ.ravel()])
        b64_str = base64.b64encode(raw_bytes).decode('ascii')
        return {
            "type": "map_update",
            "resolution": self.resolution,
            "width": self.map_width,
            "height": self.map_height,
            "origin_x": round(self.origin_x, 4),
            "origin_y": round(self.origin_y, 4),
            "data_b64": b64_str,
            "pose": {
                "x": round(self.current_pose[0], 4),
                "y": round(self.current_pose[1], 4),
                "theta": round(self.current_pose[2], 4)
            },
            "source": "standalone_cartographer_2d",
            "timestamp": time.time(),
            "loop_closures": self.pose_graph.total_loop_closures
        }


async def run_standalone_pipeline(robot_ws_url: str, dashboard_ws_url: str):
    print("=" * 65)
    print("  STANDALONE GOOGLE CARTOGRAPHER 2D SLAM RUNNER")
    print(f"  Robot Input:     {robot_ws_url}")
    print(f"  Dashboard Sync:  {dashboard_ws_url}")
    print(f"  Resolution:      5 cm (0.05m)")
    print(f"  Submap Lifecycle: 35 scans / submap (Dual overlapping)")
    print("=" * 65)

    slam = StandaloneCartographer2D()
    last_dashboard_push = 0.0

    while True:
        try:
            print(f"[WS] Connecting to JetBot hardware stream at {robot_ws_url}...")
            async with websockets.connect(
                robot_ws_url,
                ping_interval=3.0,
                ping_timeout=3.0,
                close_timeout=2.0,
                max_size=10_000_000
            ) as robot_ws:
                print(f"[OK] Connected to JetBot! Ingesting JB-Binary frames...")

                # Connect to dashboard if different URL
                dashboard_ws = None
                if dashboard_ws_url and dashboard_ws_url != robot_ws_url:
                    try:
                        dashboard_ws = await websockets.connect(dashboard_ws_url)
                        print(f"[OK] Connected to Dashboard at {dashboard_ws_url}!")
                    except Exception as ed:
                        print(f"[WARN] Dashboard WS connect failed: {ed}")

                while True:
                    message = await robot_ws.recv()
                    # Real-time lock: If incoming frames accumulated in the client deque,
                    # drain to the newest binary frame so Cartographer runs at 0 lag.
                    msgs_deque = getattr(robot_ws, "messages", None)
                    if msgs_deque and len(msgs_deque) > 0:
                        while msgs_deque:
                            item = msgs_deque.popleft()
                            if isinstance(item, bytes):
                                message = item

                    now = time.time()
                    if isinstance(message, bytes):
                        frame = unpack_jb_frame(message)
                        if frame:
                            pose, score, closures = slam.process_scan(frame)

                            # Log status at ~1Hz
                            if slam.total_scans_processed % 10 == 0:
                                deg = math.degrees(pose[2])
                                print(
                                    f"[CARTOGRAPHER] Scan #{slam.total_scans_processed:04d} | "
                                    f"Pose: ({pose[0]:.2f}m, {pose[1]:.2f}m, {deg:.1f}°) | "
                                    f"CSM Score: {score:.3f} | Loops: {slam.pose_graph.total_loop_closures}",
                                    flush=True
                                )

                            # Push map update to dashboard at 2.0 Hz
                            if now - last_dashboard_push >= 0.5:
                                payload = slam.get_map_payload()
                                target_sock = dashboard_ws if dashboard_ws else robot_ws
                                try:
                                    await target_sock.send(json.dumps(payload))
                                    last_dashboard_push = now
                                except Exception:
                                    pass

        except asyncio.CancelledError:
            break
        except Exception as e:
            print(f"[WARN] Stream connection dropped: {e}. Reconnecting in 2.0s...")
            await asyncio.sleep(2.0)


def main():
    import argparse
    parser = argparse.ArgumentParser(description="Standalone Cartographer 2D SLAM Runner")
    parser.add_argument("--robot-url", default="ws://127.0.0.1:8000/ws/robot", help="JetBot telemetry WebSocket URL")
    parser.add_argument("--dashboard-url", default="ws://127.0.0.1:8000/ws", help="Web Dashboard WebSocket URL")
    args = parser.parse_args()

    try:
        asyncio.run(run_standalone_pipeline(args.robot_url, args.dashboard_url))
    except KeyboardInterrupt:
        print("\n[INFO] Cartographer stopped cleanly.")


if __name__ == '__main__':
    main()
