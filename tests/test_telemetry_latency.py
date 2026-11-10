import sys
import os
import unittest
import time
from unittest.mock import MagicMock

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "dashboard", "backend")))

from app.hardware.scan_gate import ScanGate
from app.hardware.telemetry_parser import TelemetryParser


class TestTelemetryLatency(unittest.TestCase):
    def test_01_scangate_preserves_hardware_timestamp(self):
        gate = ScanGate()
        hw_stamp = 1700000000.123
        data = {
            "lidar_ranges": [1.0] * 360,
            "scan_seq": 100,
            "scan_stamp": hw_stamp
        }
        tick = gate.accept(data)
        self.assertIsNotNone(tick)
        self.assertEqual(tick.stamp, hw_stamp)

        # Duplicate seq arrives immediately -> must be dropped
        dup = gate.accept(data)
        self.assertIsNone(dup)

    def test_02_scangate_stale_force_accept_monotonic(self):
        gate = ScanGate()
        hw_stamp = 1700000000.100
        data1 = {
            "lidar_ranges": [1.0] * 360,
            "scan_seq": 101,
            "scan_stamp": hw_stamp
        }
        tick1 = gate.accept(data1)
        self.assertIsNotNone(tick1)

        # Force stale condition by backdating _last_accept_wall
        gate._last_accept_wall = time.time() - 0.50

        # Same seq arrives after stale timeout
        tick2 = gate.accept(data1)
        self.assertIsNotNone(tick2)
        # Monotonicity check
        self.assertGreaterEqual(tick2.stamp, tick1.stamp)
        self.assertLessEqual(tick2.stamp, tick1.stamp + 1.0)

    def test_03_telemetry_parser_latency_bounded(self):
        coord = MagicMock()
        coord._last_odom_time = time.time() - 0.035  # 35ms ago
        coord._last_lidar_sense_log_time = 0.0
        coord._min_clock_skew = None
        coord._last_skew_reset_time = 0.0
        ranges = [1.0] * 360

        # Capture print or test clock skew initialization
        TelemetryParser._log_lidar_surroundings(coord, ranges)
        min_skew = getattr(coord, "_min_clock_skew", None)
        self.assertIsNotNone(min_skew)
        # Latency should be bounded around ~35ms, not 1400ms
        self.assertAlmostEqual(min_skew, 0.035, delta=0.030)


if __name__ == "__main__":
    unittest.main()
