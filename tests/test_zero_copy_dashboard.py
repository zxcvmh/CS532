import sys
import os
import unittest
import asyncio
import base64
import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "dashboard", "backend")))
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "motion")))

from app.navigation.coordinator import NavigationCoordinator


class TestZeroCopyDashboard(unittest.TestCase):
    def test_01_coordinator_map_reporting(self):
        coord = NavigationCoordinator()
        # Feed synthetic scans to populate map
        ranges = [1.5] * 360
        for _ in range(6):
            coord.motion.update_odometry(ranges, dt=0.1, v_cmd=0.0, w_cmd=0.0)

        loop = asyncio.new_event_loop()
        m = loop.run_until_complete(coord.get_map())
        self.assertIsNotNone(m.data_b64)
        self.assertIsNone(m.data)  # Zero Python list allocation!

        # Verify decoded buffer matches raw bytes
        decoded = base64.b64decode(m.data_b64)
        raw = coord.motion.to_dashboard_bytes()
        self.assertEqual(decoded, raw)

        # Verify coverage in HEALTH log
        free_pct = 100.0 * raw.count(b"\x00") / len(raw)
        self.assertGreater(free_pct, 1.0, "Map must reflect explored free space!")


if __name__ == "__main__":
    unittest.main()
