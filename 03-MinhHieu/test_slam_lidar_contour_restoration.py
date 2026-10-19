#!/usr/bin/env python3
"""
Test LiDAR Contour & Obstacle Instant Recognition Restoration
CS532 JetBot SLAM — Verification of anh_lidar_cu.png restoration

Checks:
1. Real foam box obstacle at 0.27m (matching OZ cushion in anh_lidar.png):
   - Hit cell immediately registered with solid 100 in Occupancy Grid.
   - Remains solid 100 on subsequent cycles while obstacle persists.
   - Raycasting does not turn obstacle into white free space (0).
2. LiDAR scan contour and beam extraction:
   - 360-ray ranges accurately convert to world points with +0.08m lever-arm compensation.
   - Dense scan points correctly outline walls and obstacles.
3. Dashboard map format integrity:
   - Unknown cells: -1 (transparent on canvas)
   - Free cells: 0 (white navigable space)
   - Obstacle cells: 100 (solid black walls)
"""

import unittest
import math
import numpy as np
import time

from app.config import Config
from planning.occupancy_grid import OccupancyGridMap
from app.hardware.telemetry_parser import TelemetryParser


class TestSlamLidarContourRestoration(unittest.TestCase):
    def setUp(self):
        self.grid = OccupancyGridMap(
            width=100,
            height=100,
            resolution=0.05,
            origin_x=-2.5,
            origin_y=-2.5
        )

    def test_01_foam_box_at_27cm_instant_solid_obstacle(self):
        """Object at 0.27m (like OZ cushion) must be instantly recognized as solid 100, never free space."""
        # Robot at origin (0, 0, theta=0) facing +X
        # Obstacle 'OZ' is at 0.27m in front of LiDAR
        # 360-degree range array where front rays [350..359, 0..10] see the box at 0.27m
        ranges = [2.5] * 360  # Room walls at 2.5m
        for deg in list(range(350, 360)) + list(range(0, 11)):
            ranges[deg] = 0.27

        # Perform scan update
        self.grid.update_scan(robot_x=0.0, robot_y=0.0, robot_theta=0.0, ranges=ranges)

        # Coordinate of box hit:
        # lidar_x = 0.0 + 0.08 = 0.08m
        # hit_x = 0.08 + 0.27 = 0.35m, hit_y = 0.0m
        box_c, box_r = self.grid.world_to_grid(0.35, 0.0)
        box_idx = box_r * self.grid.width + box_c

        # 1. Log-odds must be positive and strong
        self.assertGreater(self.grid.log_odds.ravel()[box_idx], 0.62,
                           "Box at 0.27m must have log-odds > 0.62 for occupied status")

        # 2. Dashboard rasterization must be solid 100 (Black obstacle / White chalk)
        dashboard_data = self.grid.to_dashboard_data()
        self.assertEqual(dashboard_data[box_idx], 100,
                         "Box at 0.27m must be rasterized as solid 100 on dashboard")

        # 3. Check cells between robot and box: must be free space (0), NOT unobserved (-1)
        free_c, free_r = self.grid.world_to_grid(0.20, 0.0)
        free_idx = free_r * self.grid.width + free_c
        self.assertEqual(dashboard_data[free_idx], 0,
                         "Space between robot and box must be cleared navigable free space (0)")

        # 4. Multi-cycle persistence: simulate 10 subsequent sweeps with the box still there
        for _ in range(10):
            time.sleep(0.01)
            self.grid.update_scan(robot_x=0.0, robot_y=0.0, robot_theta=0.0, ranges=ranges)

        dashboard_data_persistent = self.grid.to_dashboard_data()
        self.assertEqual(dashboard_data_persistent[box_idx], 100,
                         "Box at 0.27m must remain solid 100 across multiple sweeps without disappearing")

    def test_02_lidar_contour_points_projection(self):
        """LiDAR range readings must accurately project into 2D world points forming boundary contours."""
        # Simulate room with walls at 2.0m and a column at 0.5m
        ranges = [2.0] * 360
        ranges[45] = 0.50  # 45 deg obstacle
        ranges[90] = 1.00  # Left wall at 1.0m

        pts = TelemetryParser.ranges_to_points(ranges, px=0.0, py=0.0, pth=0.0, max_range=6.0)
        self.assertGreaterEqual(len(pts), 350, "At least 350 valid LiDAR contour points must be extracted")

        # Verify point at 45 deg with 0.08m lever arm:
        # lidar_x = 0.08, lidar_y = 0.0
        # px = 0.08 + 0.5 * cos(45 deg) = 0.08 + 0.3536 = 0.4336
        # py = 0.00 + 0.5 * sin(45 deg) = 0.3536
        pt_45 = pts[45]
        expected_x = 0.08 + 0.50 * math.cos(math.radians(45))
        expected_y = 0.00 + 0.50 * math.sin(math.radians(45))
        self.assertAlmostEqual(pt_45.x, expected_x, places=2)
        self.assertAlmostEqual(pt_45.y, expected_y, places=2)

    def test_03_dashboard_bytes_and_b64_integrity(self):
        """Map serialization must correctly encode -1 (unknown), 0 (free), and 100 (obstacle)."""
        raw_bytes = self.grid.to_dashboard_bytes()
        self.assertEqual(len(raw_bytes), 100 * 100, "100x100 grid must produce exactly 10,000 bytes")

        b64 = self.grid.to_dashboard_b64()
        self.assertGreater(len(b64), 1000, "Base64 string must be non-empty and well-formed")

        # Initial state before scans: all cells must be -1 (unknown)
        arr = np.frombuffer(raw_bytes, dtype=np.int8)
        self.assertTrue(np.all(arr == -1), "Initial unobserved map must have 100% unknown cells (-1)")


if __name__ == "__main__":
    unittest.main()
