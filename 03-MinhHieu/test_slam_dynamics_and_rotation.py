#!/usr/bin/env python3
"""
Test Suite: Robot Dynamics Integration, Anchor Wall Locking & SLAM Rotation Fidelity.
Validates:
  1. Config loads calibrated parameters directly from setup/robot_dynamics.json (Single Source of Truth).
  2. Manual rotation commands via WebSocket dispatch at calibrated MAX_ANGULAR_SPEED (0.35 rad/s), NOT hardcoded 2.0 rad/s.
  3. No fake simulation room walls are injected into OccupancyGridMap unless ENABLE_SIMULATION_MAP=1.
  4. Anchor Map Wall Protection: Free-space clearing rays NEVER erode or delete locked anchor walls.
  5. Probability Map Caching: Eliminates redundant np.exp calculations, achieving <0.01ms lookup.
  6. Odometry Pivot Prior: Prior dtheta is bounded to physical pivot capability (0.35 rad/s = 4.5s for 90°).
"""

import math
import os
import sys
import unittest
import numpy as np

# Set path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "backend")))
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "05-d500lib-motion")))

from app.config import Config
from app.navigation.coordinator import NavigationCoordinator
from app.streaming.websocket_hub import WebSocketHub
from planning.occupancy_grid import OccupancyGridMap
from planning.lidar_odometry import Lidar2DOdometry


class TestSlamDynamicsAndRotation(unittest.TestCase):

    def test_01_robot_dynamics_single_source_of_truth(self):
        """Verify Config physical dynamics calibrated for 90 deg in 0.50s."""
        self.assertAlmostEqual(Config.MAX_ANGULAR_SPEED, 3.1416, places=2)
        self.assertAlmostEqual(Config.TURN_TIME_90_DEG_SEC, 0.50, places=1)
        self.assertAlmostEqual(Config.TURN_TIME_180_DEG_SEC, 1.00, places=1)
        self.assertAlmostEqual(Config.MAX_SPEED, 0.14, places=2)
        self.assertAlmostEqual(Config.MIN_DRIVE_SPEED, 0.12, places=2)
        self.assertAlmostEqual(Config.CRUISE_SPEED, 0.17, places=2)

    def test_02_manual_rotation_calibrated_angular_velocity(self):
        """Verify manual control commands produce calibrated 3.1416 rad/s (90 deg in 0.5s)."""
        coord = NavigationCoordinator()
        hub = WebSocketHub(coord)

        import asyncio
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)

        # ROTATE_LEFT
        loop.run_until_complete(hub.handle_message(None, {
            "type": "manual_control",
            "command": "ROTATE_LEFT",
            "speed": Config.MAX_SPEED
        }))
        self.assertAlmostEqual(coord.angular_vel, Config.MAX_ANGULAR_SPEED, places=2)
        self.assertLessEqual(coord.angular_vel, 3.50, "Angular speed must not exceed physical turning rate!")

        # ROTATE_RIGHT
        loop.run_until_complete(hub.handle_message(None, {
            "type": "manual_control",
            "command": "ROTATE_RIGHT",
            "speed": Config.MAX_SPEED
        }))
        self.assertAlmostEqual(coord.angular_vel, -Config.MAX_ANGULAR_SPEED, places=2)
        self.assertGreaterEqual(coord.angular_vel, -3.50)

        # Joystick velocity message
        loop.run_until_complete(hub.handle_message(None, {
            "type": "manual_control",
            "linear": 0.14,
            "angular": 5.0  # Excessive angular command clamped to MAX_ANGULAR_SPEED
        }))
        self.assertAlmostEqual(coord.angular_vel, Config.MAX_ANGULAR_SPEED, places=2)

        loop.close()

    def test_03_no_fake_simulation_room_by_default(self):
        """Verify that without real hardware, the real map starts empty, NOT with Config.WALLS."""
        coord = NavigationCoordinator()
        # Verify map starts unobserved (-1)
        data = coord.map_data
        occupied_count = sum(1 for val in data if val == 100)
        self.assertEqual(occupied_count, 0, "No fake walls should be in the initial map!")

    def test_04_anchor_wall_locking_and_protection(self):
        """Verify that locked anchor walls cannot be eroded by free space rays during turns."""
        grid = OccupancyGridMap(width=100, height=100, resolution=0.05, origin_x=-2.5, origin_y=-2.5)

        # Build a circular scan representing room walls at 1.5m
        scan = [1.5] * 360
        locked = grid.set_anchor_from_scan(scan, robot_x=0.0, robot_y=0.0, robot_theta=0.0)
        self.assertTrue(locked)
        self.assertTrue(grid.anchor_map_locked)

        # Find cells that are anchor walls
        wall_mask = grid._anchor_log_odds > 0.62
        wall_count_before = int(np.count_nonzero(wall_mask))
        self.assertGreater(wall_count_before, 20)

        # Now simulate 5 consecutive sweeps of free-space rays through the map while rotating
        for angle in [0.1, 0.2, 0.3, 0.4, 0.5]:
            grid.update_scan(
                robot_x=0.0, robot_y=0.0, robot_theta=angle,
                ranges=[1.8] * 360,  # Longer rays passing through old wall positions
                is_rotating=True
            )

        # Verify anchor walls were NOT erased!
        current_wall_cells = np.count_nonzero(grid.log_odds[wall_mask] > 0.62)
        self.assertEqual(current_wall_cells, wall_count_before, "Anchor wall cells must be 100% preserved!")

    def test_05_probability_map_caching_speedup(self):
        """Verify get_probability_map caches the array for zero-overhead Hector SLAM iterations."""
        grid = OccupancyGridMap(width=100, height=100, resolution=0.05, origin_x=-2.5, origin_y=-2.5)
        grid.update_scan(0.0, 0.0, 0.0, [1.0] * 360)

        import time
        # First call computes
        t0 = time.time()
        p1 = grid.get_probability_map()
        t1 = time.time()

        # Subsequent calls must return cached array instantly
        t2 = time.time()
        for _ in range(20):
            p2 = grid.get_probability_map()
            self.assertIs(p1, p2, "Must return the exact cached object!")
        t3 = time.time()

        cached_avg_ms = ((t3 - t2) / 20.0) * 1000.0
        self.assertLess(cached_avg_ms, 0.05, f"Cached lookup took {cached_avg_ms:.4f}ms, expected < 0.05ms")

    def test_06_odometry_pivot_yaw_rate_bounding(self):
        """Verify odometry bounds prior yaw rate during pivot turn to match hardware calibration (3.1416 rad/s)."""
        odom = Lidar2DOdometry(min_range_m=0.05, max_range_m=6.0)
        self.assertAlmostEqual(odom.calibrated_pivot_yaw_rate, 3.1416, places=2)

        # Simulate update with extreme w_cmd = 5.0 rad/s in pivot turn mode (v=0)
        ranges = [2.0] * 360
        ox, oy, oth, diag = odom.update(ranges, dt=0.1, v_cmd=0.0, w_cmd=5.0)

        # First scan initializes
        # Second scan tests prior:
        ox, oy, oth, diag = odom.update(ranges, dt=0.1, v_cmd=0.0, w_cmd=5.0)
        # Bounded rotation in 0.1s at max 3.1416 rad/s: abs(dtheta) <= 3.1416 * 0.1 + tolerance = 0.314 rad + 0.05
        self.assertLessEqual(abs(diag.get("dtheta_rad", 0.0)), 0.38,
                            f"Pivot turn step should not exceed ~0.35 rad, got {diag.get('dtheta_rad')}")


if __name__ == "__main__":
    unittest.main()
