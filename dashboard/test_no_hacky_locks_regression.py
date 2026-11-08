import unittest
import inspect
import sys
import os

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "backend")))
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../cartographer")))

from app.hardware.telemetry_parser import TelemetryParser
from app.navigation.coordinator import NavigationCoordinator

class TestNoHackyLocksRegression(unittest.TestCase):
    def test_no_rotation_map_lock_in_code(self):
        source = inspect.getsource(TelemetryParser.apply_telemetry)
        self.assertNotIn("is_rotating", source, "Hacky 'is_rotating' lock must be completely eliminated!")
        self.assertNotIn("anchor_map_locked", source, "Anchor locking must not block raycast evaporation!")

    def test_coordinator_uses_cartographer(self):
        coord = NavigationCoordinator()
        self.assertTrue(
            hasattr(coord.motion, "cartographer") or hasattr(coord, "cartographer"),
            "Coordinator or MotionAdapter must own Cartographer SLAM engine!"
        )

if __name__ == "__main__":
    unittest.main()
