import unittest
import numpy as np
import sys
import os

sys.path.insert(0, os.path.abspath(os.path.dirname(__file__)))
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../../cartographer")))

from app.navigation.cartographer_adapter import CartographerAdapter

class TestCartographerAdapter(unittest.TestCase):
    def setUp(self):
        self.adapter = CartographerAdapter(
            resolution=0.05,
            width=160,
            height=160,
            angular_search_window_deg=35.0,
            linear_search_window_m=0.15
        )

    def test_01_initial_state(self):
        x, y, th = self.adapter.get_pose()
        self.assertEqual((x, y, th), (0.0, 0.0, 0.0))
        m_data = self.adapter.to_dashboard_data()
        self.assertEqual(m_data["width"], 160)
        self.assertEqual(m_data["height"], 160)
        self.assertEqual(m_data["resolution"], 0.05)

    def test_02_update_scan_stationary(self):
        # 360 rays forming a 2m square room
        ranges = [2.0] * 360
        ox, oy, oth, info = self.adapter.update_scan(ranges, dt=0.1, v_cmd=0.0, w_cmd=0.0)
        self.assertAlmostEqual(ox, 0.0, places=2)
        self.assertAlmostEqual(oy, 0.0, places=2)
        self.assertAlmostEqual(oth, 0.0, places=2)
        self.assertTrue(info.get("is_reliable", False))

    def test_03_rotation_tracking_without_drift(self):
        # Initial scan
        ranges = [2.0] * 360
        self.adapter.update_scan(ranges, dt=0.1, v_cmd=0.0, w_cmd=0.0)
        
        # Simulate robot rotating 20 degrees counter-clockwise (scan shifts index by +20)
        # In robot frame, an obstacle at 0 deg moves to -20 deg (340 deg)
        rotated_ranges = [2.0] * 360
        # Angular speed 1.74 rad/s * 0.2s = 0.349 rad (~20 deg)
        ox, oy, oth, info = self.adapter.update_scan(
            rotated_ranges, dt=0.2, v_cmd=0.0, w_cmd=1.745
        )
        self.assertAlmostEqual(ox, 0.0, places=2)
        self.assertAlmostEqual(oy, 0.0, places=2)
        self.assertTrue(info.get("is_reliable", False))

if __name__ == "__main__":
    unittest.main()
