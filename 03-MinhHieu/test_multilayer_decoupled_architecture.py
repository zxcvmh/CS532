#!/usr/bin/env python3
"""
Unit Test Suite for Multi-Layer Decoupled Architecture Fixes
===========================================================
Tests:
1. Solid Black Obstacle Cluster Rasterization (to_dashboard_bytes, to_dashboard_data)
2. Dynamic Obstacle TTL Decay (0.35s evaporation of non-anchor footprints)
3. Pivot Clearance Gating in CBF & Controller (omega != 0 when v = 0)
4. Manual Mode Odometry Warning Suppression (only active in AUTONOMOUS)
"""
import os
import sys
import time
import math
import unittest
import numpy as np

# Ensure 03-MinhHieu and 05-d500lib-motion are in sys.path
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(BASE_DIR)
MOTION_LIB = os.path.join(PROJECT_ROOT, "05-d500lib-motion")
BACKEND_APP = os.path.join(BASE_DIR, "backend")

for p in [MOTION_LIB, BACKEND_APP, BASE_DIR]:
    if p not in sys.path:
        sys.path.insert(0, p)

from planning.occupancy_grid import OccupancyGridMap
from planning.cbf_safety_filter import CBFSafetyFilter
from planning.controller import PurePursuitController
from app.hardware.telemetry_parser import TelemetryParser, BinaryTelemetryPacker
from app.hardware.scan_gate import ScanGate
from app.models.schemas import ModeEnum


class MockCoordinator:
    def __init__(self):
        self.mode = ModeEnum.MANUAL
        self.linear_vel = 0.2
        self.angular_vel = 0.0
        self.events = []
        self._pose_stale_warn_time = 0.0
        self._last_scan_tick_time = time.time()
        self._has_reset_real_map = True
        self.latest_raw_ranges = [1.5] * 360
        self.latest_scan_seq = 0
        self.bridge = type("Bridge", (), {
            "last_telemetry_time": 0.0, "is_connected": False,
            "last_lidar_time": 0.0, "lidar_connected": False,
            "last_battery_time": 0.0, "battery_connected": False
        })()
        self.pose = type("Pose", (), {"x": 0.0, "y": 0.0, "theta": 0.0})()
        self.motion = type("Motion", (), {
            "anchor_initialized": True,
            "grid_map": OccupancyGridMap(),
            "update_odometry": lambda *args, **kwargs: (0.0, 0.0, 0.0, {"status": "POSE_STALE", "is_reliable": False}),
            "process_clusters": lambda *args, **kwargs: ([], [])
        })()
        self.scan_gate = ScanGate()
        self._last_map_update = 0.0
        self.map_version = 0

    def add_event(self, level, msg):
        self.events.append((level, msg))


class TestMultiLayerDecoupledArchitecture(unittest.TestCase):

    def test_01_solid_cluster_rasterization_dashboard(self):
        """Active obstacle clusters must be rasterized as solid 100 in dashboard exports."""
        grid = OccupancyGridMap(width=100, height=100, resolution=0.05, origin_x=-2.5, origin_y=-2.5)
        # Place a box cluster at world (0.5, 0.5) with radius 0.15m
        cluster_mock = type("Cluster", (), {
            "points_count": 20,
            "obstacle_type": "BOX_OBSTACLE",
            "centroid": (0.5, 0.5),
            "effective_radius": 0.15,
            "bounding_box_2d": [(0.4, 0.4), (0.6, 0.4), (0.6, 0.6), (0.4, 0.6)],
            "to_dict": lambda: {}
        })()

        grid.insert_obstacle_clusters([cluster_mock], robot_x=0.0, robot_y=0.0, robot_theta=0.0)

        # Export to dashboard
        raw_bytes = grid.to_dashboard_bytes()
        arr = np.frombuffer(raw_bytes, dtype=np.int8)
        c_p, r_p = grid.world_to_grid(0.5, 0.5)
        center_val = arr[r_p * grid.width + c_p]

        self.assertEqual(center_val, 100, "Cluster center cell must be solid occupied 100 on dashboard")
        data_list = grid.to_dashboard_data()
        self.assertEqual(data_list[r_p * grid.width + c_p], 100, "Dashboard data list must also be 100")

    def test_02_dynamic_obstacle_ttl_decay(self):
        """Dynamic obstacle hits outside Anchor Map must evaporate after TTL (0.35s)."""
        grid = OccupancyGridMap(width=100, height=100, resolution=0.05, origin_x=-2.5, origin_y=-2.5)
        # Lock anchor map with initial scan
        grid.set_anchor_from_scan([2.0] * 360, robot_x=0.0, robot_y=0.0, robot_theta=0.0)
        self.assertTrue(grid.anchor_map_locked)

        # Now simulate a dynamic obstacle (human foot) at 0.8m ahead (angle 0)
        scan = [2.0] * 360
        scan[0] = 0.8  # Foot at 0.8m
        grid.update_scan(robot_x=0.0, robot_y=0.0, robot_theta=0.0, ranges=scan)

        c_foot, r_foot = grid.world_to_grid(0.88, 0.0)  # lidar_x is 0.08 + 0.80 = 0.88
        foot_idx = r_foot * grid.width + c_foot

        # Verify foot cell is in dynamic obstacle tracking buffer
        self.assertIn(foot_idx, grid._dynamic_obstacle_cells, "Dynamic foot cell must be tracked in TTL buffer")

        # Advance time beyond 0.35s TTL
        grid._dynamic_obstacle_cells[foot_idx] = time.time() - 0.1  # expired

        # Next scan update without the foot (person walked away)
        clear_scan = [2.0] * 360
        grid.update_scan(robot_x=0.0, robot_y=0.0, robot_theta=0.0, ranges=clear_scan)

        self.assertNotIn(foot_idx, grid._dynamic_obstacle_cells, "Expired cell must be evicted from TTL buffer")
        self.assertLessEqual(grid.log_odds.ravel()[foot_idx], 0.0,
                             "Expired dynamic obstacle cell must evaporate to <= 0.0 (free space, no ghost wall)")

    def test_03_pivot_clearance_gating_cbf_and_controller(self):
        """When v=0 due to critical obstacle, omega != 0 must be preserved to escape deadlock."""
        # 1. Test CBFSafetyFilter
        cbf = CBFSafetyFilter(v_max=0.14, w_max=0.35)
        # Create critical obstacle at 0.14m directly in front
        ranges = [2.0] * 360
        ranges[0] = 0.14  # Obstacle right at bumper
        v_safe, w_safe, info = cbf.filter_control(v_nom=0.14, w_nom=0.25, raw_ranges=ranges)

        self.assertEqual(v_safe, 0.0, "Forward motion must be halted at 0.14m")
        self.assertGreater(abs(w_safe), 0.05, "Angular rotation must be preserved for pivot escape")

        # 2. Test PurePursuitController
        ctrl = PurePursuitController(reflex_dist_threshold=0.30)
        ctrl.set_path([(0.0, 0.0), (1.0, 0.0)])
        # Simulate lidar ranges with critical obstacle directly in front cone
        critical_ranges = [2.0] * 360
        for i in range(-15, 16):
            critical_ranges[i % 360] = 0.14

        v, w, info = ctrl.compute_command(0.0, 0.0, 0.0, lidar_ranges=critical_ranges, auto_evade=False)
        self.assertEqual(v, 0.0, "Reflex must halt v to 0.0")
        self.assertGreater(abs(w), 0.10, "Controller must command in-place pivot turn (omega != 0)")
        self.assertEqual(info["status"], "REFLEX_PIVOT_EVADE")

    def test_04_manual_mode_suppresses_odometry_warning(self):
        """In MANUAL mode, POSE_STALE must NOT trigger warning event; in AUTONOMOUS mode it should."""
        coord = MockCoordinator()
        coord.mode = ModeEnum.MANUAL
        packer = BinaryTelemetryPacker()
        payload = packer.pack(
            pose_x=0.0, pose_y=0.0, pose_theta=0.0, linear_vel=0.2, angular_vel=0.0,
            nearest_obs=1.0, battery=90.0, cpu_temp=40.0, ranges_m=[1.0] * 360,
            timestamp=time.time(), seq=100
        )

        TelemetryParser.apply_binary_telemetry(payload, coord)
        warn_events = [msg for level, msg in coord.events if "LiDAR odometry degraded" in msg]
        self.assertEqual(len(warn_events), 0, "MANUAL mode must NOT produce LiDAR odometry degraded warning")

        # Now switch to AUTONOMOUS mode
        coord.mode = ModeEnum.AUTONOMOUS
        coord._pose_stale_warn_time = 0.0
        payload2 = packer.pack(
            pose_x=0.0, pose_y=0.0, pose_theta=0.0, linear_vel=0.2, angular_vel=0.0,
            nearest_obs=1.0, battery=90.0, cpu_temp=40.0, ranges_m=[1.0] * 360,
            timestamp=time.time() + 0.1, seq=101
        )
        TelemetryParser.apply_binary_telemetry(payload2, coord)
        warn_events_auto = [msg for level, msg in coord.events if "LiDAR odometry degraded" in msg]
        self.assertEqual(len(warn_events_auto), 1, "AUTONOMOUS mode MUST log degraded warning when pose is dead-reckoned")


if __name__ == "__main__":
    unittest.main()
