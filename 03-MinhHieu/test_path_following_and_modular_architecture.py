"""
Comprehensive Test Suite:
1. Path Following Steering Verification (Fix for Bug 2: Robot driving straight on diagonal goals)
2. In-place Pivot Stall Breakout Torque Verification
3. Modular SLAM Package Architecture Verification
4. Modular Control Package Architecture Verification
"""
import math
import sys
import os
import unittest
import numpy as np

# Ensure root and 05-d500lib-motion are in sys.path
BASE_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
D500_DIR = os.path.join(BASE_DIR, "05-d500lib-motion")
if BASE_DIR not in sys.path:
    sys.path.insert(0, BASE_DIR)
if D500_DIR not in sys.path:
    sys.path.insert(0, D500_DIR)

from planning.control import MotorDynamics, VelocityRegulator, PurePursuitTracker
from planning.slam import (
    GridCore,
    AnchorLayer,
    DynamicLayer,
    CostmapEDT,
    DashboardRasterizer,
    HectorMatcher,
)
from planning.controller import PurePursuitController
from planning.occupancy_grid import OccupancyGridMap


class TestBug2PathFollowingAndSteering(unittest.TestCase):
    """Verifies that diagonal paths produce active differential steering rather than driving straight."""

    def setUp(self):
        self.controller = PurePursuitController(
            max_linear_speed=0.14,
            max_angular_speed=0.35,
            min_drive_speed=0.12,
            lookahead_dist=0.28,
            goal_tolerance=0.18,
        )
        self.motor_dyn = MotorDynamics(
            track_width=0.115,
            min_drive_speed=0.12,
            max_linear_speed=0.14,
            max_angular_speed=0.35,
            trim_factor=1.024,
        )

    def test_diagonal_path_continuous_curvature_steering(self):
        """
        When target is at 45 degrees, the controller must NOT freeze linear velocity (v > 0)
        and must command significant angular velocity (w != 0).
        """
        # Robot at (0, 0), facing theta = 0 (heading East)
        # Goal at (1.0, 1.0) -> 45 degree angle (diagonal)
        path = [(0.0, 0.0), (0.5, 0.5), (1.0, 1.0)]
        self.controller.set_path(path)

        v, w, info = self.controller.compute_command(0.0, 0.0, 0.0)

        # Must not be stopped in in-place pivot alignment
        self.assertGreater(v, 0.05, f"Linear speed should be positive for 45° diagonal path, got {v}")
        self.assertGreater(w, 0.05, f"Angular speed should be positive (turning left toward 45°), got {w}")
        self.assertIn(info["status"], ["NAVIGATING", "EVADING"])

    def test_motor_pwm_differential_authority(self):
        """
        When commanded with forward linear velocity and steering angular velocity,
        the motor driver must produce a significant PWM differential (>= 10 PWM counts)
        while strictly respecting [min_drive_speed, cruise_speed] ([0.12, 0.17]).
        """
        v = 0.14
        w = self.motor_dyn.max_angular_speed  # Max steering left

        # Test MotorDynamics
        (pwm_l, pwm_r), (vl, vr) = self.motor_dyn.compute_motor_pwm(v, w, dt=0.5)

        hw_pwm_l = int(round(abs(pwm_l) * 255))
        hw_pwm_r = int(round(abs(pwm_r) * 255))
        hw_diff = hw_pwm_r - hw_pwm_l

        # Expected: L=31 (min_drive_speed 0.12), R=43 (cruise_speed 0.17)
        self.assertGreaterEqual(hw_diff, 10,
            f"Hardware PWM difference must be >= 10 to turn robot! Got L={hw_pwm_l}, R={hw_pwm_r}, diff={hw_diff}")
        self.assertGreater(hw_pwm_r, hw_pwm_l, "Right wheel must turn faster than left wheel when turning left!")
        self.assertEqual(hw_pwm_l, 31, f"Inner wheel should be at min_drive_speed (31 PWM), got {hw_pwm_l}")
        self.assertEqual(hw_pwm_r, 43, f"Outer wheel should be at cruise_speed (43 PWM), got {hw_pwm_r}")

    def test_inplace_pivot_breakout_torque(self):
        """
        In-place pivot turn (v=0, w!=0) must output exactly turn_speed=0.14 from robot_dynamics.json
        (36-37 PWM counts), producing the calibrated 0.35 rad/s angular speed (4.5s for 90 deg).
        """
        v = 0.0
        w = self.motor_dyn.max_angular_speed  # In-place spin left at max calibrated angular rate

        (pwm_l, pwm_r), _ = self.motor_dyn.compute_motor_pwm(v, w, dt=0.5)

        hw_pwm_l = int(round(abs(pwm_l) * 255))
        hw_pwm_r = int(round(abs(pwm_r) * 255))

        self.assertEqual(hw_pwm_r, 36, f"Right wheel spin PWM must match turn_speed=0.14 (36 PWM), got {hw_pwm_r}")
        self.assertEqual(hw_pwm_l, 37, f"Left wheel spin PWM must match turn_speed*trim=0.14336 (37 PWM), got {hw_pwm_l}")
        self.assertLess(pwm_l, 0, "Left wheel must drive backward for CCW spin")
        self.assertGreater(pwm_r, 0, "Right wheel must drive forward for CCW spin")


class TestModularSLAMArchitecture(unittest.TestCase):
    """Verifies that the modular SLAM package operates correctly both standalone and via facade."""

    def setUp(self):
        self.grid_map = OccupancyGridMap(width=80, height=80, resolution=0.05, origin_x=-2.0, origin_y=-2.0)

    def test_grid_core_and_anchor_layer_initialization(self):
        """AnchorLayer correctly captures pristine wall cells and protects them from erosion."""
        # 16-ray scan of 1m circle
        ranges = [1.0] * 360
        success = self.grid_map.set_anchor_from_scan(ranges, 0.0, 0.0, 0.0, max_range=2.5)
        self.assertTrue(success)
        self.assertTrue(self.grid_map.anchor_map_locked)

        # Check anchor layer mask
        anchor_mask = self.grid_map._anchor_layer.anchor_mask
        self.assertGreater(np.count_nonzero(anchor_mask), 10)

    def test_dynamic_layer_rolling_ttl_decay(self):
        """Dynamic obstacle footprint decays cleanly after 0.35s TTL."""
        # Insert dynamic scan cell
        c, r = 40, 40
        self.grid_map._dynamic_layer.register_cell(c, r, ttl=0.35)
        self.assertTrue(self.grid_map._dynamic_layer.is_dynamic_cell(c, r))

        # Advance time by 0.40s
        import time
        now = time.time()
        self.grid_map._dynamic_layer.decay(now + 0.40)
        self.assertFalse(self.grid_map._dynamic_layer.is_dynamic_cell(c, r))

    def test_dashboard_rasterizer_solid_cluster_blocks(self):
        """Active clusters are rasterized as solid 100 cells in to_dashboard_bytes."""
        from types import SimpleNamespace
        cluster = SimpleNamespace(
            id=1,
            centroid=(0.3, 0.0),
            radius_m=0.15,
            bounding_box_2d=[(0.2, -0.1), (0.4, -0.1), (0.4, 0.1), (0.2, 0.1)],
            is_dynamic=True,
            ttc=1.2
        )
        data_bytes = self.grid_map.to_dashboard_bytes()
        arr_before = np.frombuffer(data_bytes, dtype=np.int8)

        # Now pass cluster
        data_with_cluster = self.grid_map._rasterizer.to_dashboard_bytes(
            active_clusters=[cluster],
            robot_x=0.0,
            robot_y=0.0,
            robot_theta=0.0
        )
        arr_after = np.frombuffer(data_with_cluster, dtype=np.int8)
        self.assertGreater(np.count_nonzero(arr_after == 100), np.count_nonzero(arr_before == 100))


class TestModularControlArchitecture(unittest.TestCase):
    """Verifies that PurePursuitTracker and VelocityRegulator operate accurately."""

    def test_pure_pursuit_projection_and_steering(self):
        tracker = PurePursuitTracker(lookahead_dist=0.28, goal_tolerance=0.18, pivot_entry_deg=60.0, align_exit_deg=30.0)
        path = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)]

        # Robot slightly off path at (0.5, 0.1)
        best_seg, best_proj, dist = tracker.project_to_path(0.5, 0.1, path)
        self.assertEqual(best_seg, 0)
        self.assertAlmostEqual(best_proj[0], 0.5, places=2)
        self.assertAlmostEqual(best_proj[1], 0.0, places=2)
        self.assertAlmostEqual(dist, 0.1, places=2)

        # Lookahead target
        target = tracker.find_lookahead_target(path, best_seg, best_proj, 0.28)
        self.assertAlmostEqual(target[0], 0.78, places=2)
        self.assertAlmostEqual(target[1], 0.0, places=2)

    def test_velocity_regulator_goal_ramp_and_curvature(self):
        regulator = VelocityRegulator(min_drive_speed=0.12, max_linear_speed=0.14, goal_tolerance=0.18)

        # Far from goal: full speed
        v_far = regulator.compute_goal_ramp_velocity(dist_to_goal=1.5)
        self.assertAlmostEqual(v_far, 0.14)

        # Near goal: ramped speed
        v_near = regulator.compute_goal_ramp_velocity(dist_to_goal=0.20)
        self.assertLess(v_near, 0.14)
        self.assertGreaterEqual(v_near, 0.12)


if __name__ == "__main__":
    unittest.main()
