import sys
import os
import unittest
import math
import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "dashboard", "backend")))
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "motion")))

from app.navigation.motion_adapter import MotionAdapter


class TestHectorAnchorSLAM(unittest.TestCase):
    def test_01_anchor_wall_locking_after_initial_scans(self):
        adapter = MotionAdapter()
        # Simulate circular room of radius 1.5m
        room_ranges = [1.5] * 360
        for _ in range(6):
            ox, oy, oth, info = adapter.update_odometry(room_ranges, dt=0.1, v_cmd=0.0, w_cmd=0.0)

        self.assertTrue(adapter.anchor_initialized or getattr(adapter.grid_map, "anchor_locked", False))
        self.assertIsNotNone(getattr(adapter.grid_map, "static_anchor_walls", None))
        self.assertGreaterEqual(np.count_nonzero(adapter.grid_map.static_anchor_walls), 20)

    def test_02_in_place_rotation_tracks_robot_not_map(self):
        adapter = MotionAdapter()
        room_ranges = [1.5] * 360
        # Initialize anchor
        for _ in range(6):
            adapter.update_odometry(room_ranges, dt=0.1, v_cmd=0.0, w_cmd=0.0)

        # Record anchor wall positions in world frame
        anchor_walls_before = adapter.grid_map.static_anchor_walls.copy()

        # Simulate robot turning CCW with calibrated angular velocity
        ox, oy, oth, info = adapter.update_odometry(room_ranges, dt=0.1, v_cmd=0.0, w_cmd=0.5)

        # Robot position (ox, oy) must remain near origin (<= 0.05m) during in-place turn
        self.assertLess(math.hypot(ox, oy), 0.05)
        # TK-SLAM-09 LiDAR-only spec: in a featureless circular room without command dead-reckoning,
        # pure scan matching cannot observe rotation, so heading must NOT drift from w_cmd.
        self.assertEqual(oth, 0.0)
        # Static anchor walls must remain identical in world frame
        np.testing.assert_array_equal(adapter.grid_map.static_anchor_walls, anchor_walls_before)

    def test_03_free_rays_do_not_erode_anchor_walls(self):
        adapter = MotionAdapter()
        room_ranges = [1.5] * 360
        for _ in range(6):
            adapter.update_odometry(room_ranges, dt=0.1, v_cmd=0.0, w_cmd=0.0)

        wall_mask = adapter.grid_map.static_anchor_walls
        # Send closer scans (e.g. 1.0m) - free rays pass through area but should not clear anchor walls
        short_ranges = [1.0] * 360
        for _ in range(3):
            adapter.update_odometry(short_ranges, dt=0.1, v_cmd=0.0, w_cmd=0.0)

        # Confirmed anchor walls must still have positive log_odds
        wall_log_odds = adapter.grid_map.log_odds[wall_mask]
        self.assertTrue(np.all(wall_log_odds >= 1.0), "Anchor walls must not be eroded by free rays!")

    def test_04_costmap_preservation_after_update_costmap(self):
        adapter = MotionAdapter()
        room_ranges = [1.0] * 360
        adapter.update_odometry(room_ranges, dt=0.1, v_cmd=0.0, w_cmd=0.0)
        nonzeros_before = np.count_nonzero(adapter.grid_map.costmap > 0)
        self.assertGreater(nonzeros_before, 0, "Costmap should contain inflated obstacles from LiDAR scan")
        adapter.update_costmap()
        nonzeros_after = np.count_nonzero(adapter.grid_map.costmap > 0)
        self.assertEqual(nonzeros_after, nonzeros_before, "update_costmap must not wipe costmap obstacles!")


if __name__ == "__main__":
    unittest.main()
