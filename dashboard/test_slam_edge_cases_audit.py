#!/usr/bin/env python3
"""
Edge-Case Stress Test & Quality Audit Suite for JetBot SLAM Architecture
========================================================================
Audits:
1. Continuous 360° Pivot Rotation (5 full rotations = 1800°, 100 sweeps):
   - Anchor map preservation
   - Bounded pivot translation drift (lever arm compensation)
   - Angle wrapping consistency
2. Stationary Stability (v=0, w=0, 50 cycles):
   - ScanGate force-acceptance (prevents starvation)
   - Coordinator staleness guard non-triggering
   - Zero-drift guarantee
3. Hardware Disconnect/Reconnect Resilience:
   - Clean sim fallback without NameError or crash
   - Clean state reset on hardware reattachment
4. Array Boundary & Extreme Cluster Geometries:
   - OBB quad crossing map boundaries
   - Bounded values in dashboard exports
5. Actuation Deadlock Escape:
   - CBF and Controller pivot clearance gating
"""
import os
import sys
import time
import math
import unittest
import numpy as np

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(BASE_DIR)
MOTION_LIB = os.path.join(PROJECT_ROOT, "05-d500lib-motion")
BACKEND_APP = os.path.join(BASE_DIR, "backend")

for p in [MOTION_LIB, BACKEND_APP, BASE_DIR]:
    if p not in sys.path:
        sys.path.insert(0, p)

from app.config import Config
from app.hardware.scan_gate import ScanGate, STALE_FORCE_ACCEPT_SEC
from app.hardware.telemetry_parser import TelemetryParser
from app.streaming.binary_protocol import BinaryTelemetryPacker
from app.navigation.coordinator import NavigationCoordinator
from app.models.schemas import ModeEnum, RobotPose
from planning.occupancy_grid import OccupancyGridMap
from planning.lidar_odometry import Lidar2DOdometry
from planning.cbf_safety_filter import CBFSafetyFilter
from planning.controller import PurePursuitController


class TestSlamEdgeCasesAudit(unittest.TestCase):

    def test_01_continuous_360_rotation_stability(self):
        """Stress test: 5 full rotations (1800°) must preserve anchor map and bound drift."""
        grid = OccupancyGridMap(width=100, height=100, resolution=0.05, origin_x=-2.5, origin_y=-2.5)
        odom = Lidar2DOdometry(min_range_m=0.05, max_range_m=6.0)

        # 1. Initialize anchor map with a 2m square room
        initial_ranges = [2.0] * 360
        grid.set_anchor_from_scan(initial_ranges, robot_x=0.0, robot_y=0.0, robot_theta=0.0)
        wall_mask = grid._anchor_log_odds > 0.62
        initial_wall_count = int(np.count_nonzero(wall_mask))
        self.assertGreater(initial_wall_count, 30, "Anchor walls must be initialized")

        # 2. Simulate 5 full rotations at calibrated MAX_ANGULAR_SPEED = 0.35 rad/s
        dt = 0.1  # 10Hz
        dth_per_step = Config.MAX_ANGULAR_SPEED * dt
        steps_per_rev = int((2.0 * math.pi) / dth_per_step)
        total_steps = steps_per_rev * 5  # 5 full 360° revolutions

        current_sim_th = 0.0
        for step in range(total_steps):
            current_sim_th = (current_sim_th + dth_per_step + math.pi) % (2.0 * math.pi) - math.pi
            
            # Simulated range returns in room: distance 2.0m regardless of yaw in circular room
            sim_ranges = [2.0] * 360
            
            # Grid update with is_rotating=True
            grid.update_scan(
                robot_x=odom.x, robot_y=odom.y, robot_theta=odom.theta,
                ranges=sim_ranges, is_rotating=True, defer_costmap=True
            )

            # Odometry update
            ox, oy, oth, diag = odom.update(sim_ranges, dt=dt, v_cmd=0.0, w_cmd=Config.MAX_ANGULAR_SPEED)

        # Verify walls remain preserved without artificial freeze (>= 80% retention under continuous rotation)
        remaining_wall_count = int(np.count_nonzero(grid.log_odds[wall_mask] > 0.62))
        self.assertGreaterEqual(remaining_wall_count, int(initial_wall_count * 0.80),
                                f"Walls must remain preserved after 5 full rotations! ({remaining_wall_count}/{initial_wall_count})")

        # Verify pivot translation drift is bounded (< 0.05m)
        drift = math.hypot(odom.x, odom.y)
        self.assertLess(drift, 0.05, f"Translation drift during in-place pivot must be < 5cm, got {drift*100:.2f}cm")

    def test_02_stationary_robot_scangate_and_staleness(self):
        """Stationary robot (v=0, w=0) must not starve coordinator or trigger stale warnings."""
        coord = NavigationCoordinator()
        coord.mode = ModeEnum.AUTONOMOUS
        packer = BinaryTelemetryPacker()

        # Send 30 scans with identical seq=42 and identical ranges
        now = time.time()
        for i in range(30):
            # Advance time by 50ms per iteration
            fake_time = now + i * 0.05
            payload = packer.pack(
                pose_x=0.0, pose_y=0.0, pose_theta=0.0, linear_vel=0.0, angular_vel=0.0,
                nearest_obs=2.0, battery=90.0, cpu_temp=42.0, ranges_m=[2.0] * 360,
                timestamp=fake_time, seq=42
            )
            TelemetryParser.apply_binary_telemetry(payload, coord)

        # Check: Did ScanGate force-accept scans every ~0.20s?
        # Over 1.5s (30 * 0.05s), force-accept should have fired multiple times
        self.assertGreater(coord.latest_tick_seq, 0)
        self.assertAlmostEqual(coord.pose.x, 0.0, places=3)
        self.assertAlmostEqual(coord.pose.y, 0.0, places=3)

        # Verify NO 'LiDAR odometry degraded' warnings while stationary
        degraded_warns = [e.message for e in coord.events if "LiDAR odometry degraded" in getattr(e, "message", str(e))]
        self.assertEqual(len(degraded_warns), 0, "No odometry warnings should trigger while stationary")

    def test_03_hardware_disconnect_and_reconnect(self):
        """Coordinator must transition smoothly between hardware drop and reconnection."""
        coord = NavigationCoordinator()
        coord.mode = ModeEnum.MANUAL
        coord.linear_vel = 0.10
        coord.angular_vel = 0.0

        # 1. Simulate initial hardware connection (socket registered)
        coord.bridge.ws = object()
        packer = BinaryTelemetryPacker()
        p1 = packer.pack(
            pose_x=0.0, pose_y=0.0, pose_theta=0.0, linear_vel=0.10, angular_vel=0.0,
            nearest_obs=1.5, battery=95.0, cpu_temp=43.0, ranges_m=[1.5] * 360,
            timestamp=time.time(), seq=1
        )
        TelemetryParser.apply_binary_telemetry(p1, coord)
        self.assertTrue(coord.is_real_connected)
        self.assertTrue(coord._has_reset_real_map)

        # 2. Trigger disconnect event
        coord.on_hardware_disconnected()
        self.assertFalse(coord._has_reset_real_map)
        coord.bridge.is_connected = False
        coord.bridge.last_telemetry_time = time.time() - 10.0  # stale

        # 3. Step simulation kinematics for 10 cycles (must NOT crash)
        for _ in range(10):
            nx, ny, nth = coord.sim.update_kinematics(coord.pose.x, coord.pose.y, coord.pose.theta, coord.linear_vel, coord.angular_vel, 0.04)
            coord.pose.x, coord.pose.y, coord.pose.theta = nx, ny, nth

        self.assertGreater(coord.pose.x, 0.02, "Sim kinematics should update forward position")

        # 4. Reconnect hardware
        p2 = packer.pack(
            pose_x=0.0, pose_y=0.0, pose_theta=0.0, linear_vel=0.0, angular_vel=0.0,
            nearest_obs=1.5, battery=95.0, cpu_temp=43.0, ranges_m=[1.5] * 360,
            timestamp=time.time(), seq=2
        )
        TelemetryParser.apply_binary_telemetry(p2, coord)
        self.assertTrue(coord.is_real_connected)
        self.assertTrue(coord._has_reset_real_map, "Clean reset of real map flag on reconnect")

    def test_04_array_bounds_and_clipped_obb_clusters(self):
        """Dashboard serialization and extreme out-of-bounds OBB polygons."""
        grid = OccupancyGridMap(width=100, height=100, resolution=0.05, origin_x=-2.5, origin_y=-2.5)

        # Create an OBB cluster whose bounding box partially crosses the map boundary
        # Map world span: [-2.5, 2.5] x [-2.5, 2.5]
        # Place cluster at (2.45, 2.45) with size 0.20m so corners extend to (2.55, 2.55) (off-grid)
        cluster_boundary = type("Cluster", (), {
            "points_count": 15,
            "obstacle_type": "BOX_OBSTACLE",
            "centroid": (2.45, 2.45),
            "effective_radius": 0.12,
            "bounding_box_2d": [(2.35, 2.35), (2.55, 2.35), (2.55, 2.55), (2.35, 2.55)],
            "to_dict": lambda: {}
        })()

        # Should NOT raise IndexError
        grid.insert_obstacle_clusters([cluster_boundary], robot_x=0.0, robot_y=0.0, robot_theta=0.0)

        raw_bytes = grid.to_dashboard_bytes()
        self.assertEqual(len(raw_bytes), 100 * 100, "Dashboard bytes length must exactly match total cell count")

        data_list = grid.to_dashboard_data()
        self.assertEqual(len(data_list), 100 * 100, "Dashboard data list length must exactly match total cell count")

        # Check values: only -1, 0, 100 allowed in dashboard map
        unique_vals = set(data_list)
        self.assertTrue(unique_vals.issubset({-1, 0, 100}), f"Unexpected values in dashboard data: {unique_vals}")

    def test_05_cbf_controller_decoupled_pivot_clearance(self):
        """When blocked at close range, CBF and PurePursuit must decouple w from v (v=0, w!=0)."""
        cbf = CBFSafetyFilter(v_max=0.14, w_max=0.35)
        ranges = [2.0] * 360
        ranges[0] = 0.12  # Very close frontal obstacle (12cm)

        # Commanded forward creep (v=0.14) with steer (w=0.20)
        v_out, w_out, info = cbf.filter_control(v_nom=0.14, w_nom=0.20, raw_ranges=ranges)
        self.assertEqual(v_out, 0.0, "Linear velocity must be 0 at 12cm")
        self.assertGreaterEqual(abs(w_out), 0.16, "Angular speed must be >= 0.16 to allow in-place rotation")
        self.assertLessEqual(abs(w_out), 0.35, "Angular speed must not exceed w_max 0.35")

        # PurePursuitController pivot turn check
        ctrl = PurePursuitController()
        ctrl.set_path([(0.0, 0.0), (0.0, 1.0)])  # 90° turn from heading 0.0
        v_ctrl, w_ctrl, c_info = ctrl.compute_command(0.0, 0.0, 0.0, lidar_ranges=[2.0]*360)
        self.assertEqual(v_ctrl, 0.0, "Pivot turn requires v=0.0")
        self.assertGreaterEqual(abs(w_ctrl), 0.15, "Pivot turn must command w >= 0.15")
        self.assertLessEqual(abs(w_ctrl), 0.35, "Pivot turn must command w <= 0.35")
        self.assertEqual(c_info.get("status"), "PIVOT_ALIGNING")


if __name__ == "__main__":
    unittest.main()
