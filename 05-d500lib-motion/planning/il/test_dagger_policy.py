#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Unit & Integration Test Suite for DAgger Imitation Learning (149D & 1D-CNN)
===========================================================================
CS532 Advanced Autonomous Mobile Robotics — JetBot GA25-370 Platform
"""

import os
import sys
import time
import unittest
import numpy as np
import torch
import onnx

try:
    from .dagger_policy import (
        NavPolicy1DCNN,
        NavPolicyMLP,
        preprocess_lidar_72,
        compute_relative_lidar_velocity,
        build_state_vector,
        apply_emergency_brake,
        apply_motor_deadband
    )
    from .expert_oracle import ExpertOracle
    from .sim_env import JetBot2DSimEnv
    from ..cbf_safety_filter import CBFSafetyFilter
except (ImportError, ValueError):
    cur_dir = os.path.dirname(os.path.abspath(__file__))
    sys.path.insert(0, os.path.abspath(os.path.join(cur_dir, "../..")))
    from planning.il.dagger_policy import (
        NavPolicy1DCNN,
        NavPolicyMLP,
        preprocess_lidar_72,
        compute_relative_lidar_velocity,
        build_state_vector,
        apply_emergency_brake,
        apply_motor_deadband
    )
    from planning.il.expert_oracle import ExpertOracle
    from planning.il.sim_env import JetBot2DSimEnv
    from planning.cbf_safety_filter import CBFSafetyFilter


class TestDAggerPolicy(unittest.TestCase):

    def setUp(self):
        self.policy = NavPolicy1DCNN(max_v=0.12, max_w=0.35)
        self.oracle = ExpertOracle(max_v=0.12, max_w=0.35)
        self.cbf = CBFSafetyFilter(v_max=0.12, w_max=0.35)
        self.env = JetBot2DSimEnv()

    def test_state_preprocessing_and_shapes(self):
        """Validates LiDAR downsampling, ego-warped Delta r, and 149-dim state vector."""
        raw_ranges = [1.5 + 0.1 * np.sin(i * 0.1) for i in range(360)]
        lidar_72 = preprocess_lidar_72(raw_ranges, num_sectors=72, min_range=0.10, max_range=6.0)

        self.assertEqual(lidar_72.shape, (72,))
        self.assertTrue(np.all(lidar_72 >= 0.0) and np.all(lidar_72 <= 1.0))

        # Test Delta r calculation with robot motion
        prev_lidar = lidar_72.copy()
        delta_pose = (0.01, 0.0, 0.05)
        delta_r = compute_relative_lidar_velocity(
            current_ranges_72=lidar_72,
            previous_ranges_72=prev_lidar,
            delta_pose=delta_pose,
            dt=0.10
        )
        self.assertEqual(delta_r.shape, (72,))
        self.assertTrue(np.all(delta_r >= -1.0) and np.all(delta_r <= 1.0))

        # Test 149D state builder
        robot_pose = (0.5, 0.5, 0.2)
        subgoal = (1.5, 1.0)
        state_149 = build_state_vector(
            lidar_72=lidar_72,
            robot_pose=robot_pose,
            subgoal=subgoal,
            delta_r_72=delta_r,
            v_prev=0.08,
            w_prev=0.1
        )

        self.assertEqual(state_149.shape, (149,))
        self.assertEqual(state_149.dtype, np.float32)

    def test_policy_forward_and_latency(self):
        """Verifies 1D-CNN forward pass, velocity bounds, and sub-millisecond CPU latency."""
        dummy_state = np.random.uniform(0.1, 1.0, size=(149,)).astype(np.float32)

        # Warm up
        for _ in range(10):
            _ = self.policy.predict_action(dummy_state)

        # Measure latency over 100 runs
        start_time = time.perf_counter()
        runs = 100
        for _ in range(runs):
            v, w = self.policy.predict_action(dummy_state)
        elapsed_ms = (time.perf_counter() - start_time) / runs * 1000.0

        # Physical limits verification
        self.assertGreaterEqual(v, 0.0)
        self.assertLessEqual(v, 0.12 + 1e-4)
        self.assertGreaterEqual(w, -0.35 - 1e-4)
        self.assertLessEqual(w, 0.35 + 1e-4)

        # Latency check: must be under 2.0 ms on CPU in eager mode (ONNX is ~0.14 ms)
        print(f"\n  ⚡ 1D-CNN Policy Inference Latency: {elapsed_ms:.3f} ms (Target < 2.0 ms eager, < 0.5 ms ONNX)")
        self.assertLess(elapsed_ms, 2.0)

    def test_expert_oracle_behavior(self):
        """Verifies Expert Oracle slows down and steers away when obstacle is within clearance."""
        robot_pose = (0.0, 0.0, 0.0)
        subgoal = (2.0, 0.0)

        # Clear path (149D input)
        clear_state = np.ones(149, dtype=np.float32) * 0.9
        v_clear, w_clear = self.oracle.compute_expert_action(robot_pose, subgoal, clear_state)
        self.assertGreater(v_clear, 0.08)

        # Blocked path: obstacle at 0.18m directly ahead
        blocked_state = np.ones(149, dtype=np.float32) * 0.9
        blocked_state[28:44] = 0.18 / 5.0  # frontal sector blocked
        v_blocked, w_blocked = self.oracle.compute_expert_action(robot_pose, subgoal, blocked_state)

        # Speed must be reduced and steering engaged (COLREGS right turn w < 0)
        self.assertLess(v_blocked, v_clear)
        self.assertGreater(abs(w_blocked), 0.10)

    def test_emergency_brake_and_deadband(self):
        """Verifies Pure IL Hard Safety Gate (< 0.15m => v=0) and GA25-370 deadband."""
        # 1. Critical proximity < 0.15m -> v must be clamped to 0.0
        v_brake, w_brake, e_active = apply_emergency_brake(v_nom=0.12, w_nom=0.2, min_range_m=0.14)
        self.assertEqual(v_brake, 0.0)
        self.assertEqual(w_brake, 0.2)
        self.assertTrue(e_active)

        # 2. Safe distance > 0.15m -> normal throughput
        v_safe, w_safe, e_active2 = apply_emergency_brake(v_nom=0.12, w_nom=0.2, min_range_m=0.30)
        self.assertEqual(v_safe, 0.12)
        self.assertEqual(w_safe, 0.2)
        self.assertFalse(e_active2)

        # 3. Deadband compensation
        u_compensated = apply_motor_deadband(0.10, deadband=0.12)
        expected = 0.12 + 0.88 * 0.10
        self.assertAlmostEqual(u_compensated, expected, places=4)
        print(f"\n  🛡️ Hard Emergency Brake & Deadband Verified: min_r=0.14m -> v_brake={v_brake}, min_r=0.30m -> v_safe={v_safe}")

    def test_onnx_export(self):
        """Verifies that 1D-CNN policy exports cleanly to ONNX format (opset 18)."""
        tmp_onnx = "/tmp/test_dagger_1dcnn_149d.onnx"
        self.policy.export_onnx(tmp_onnx)
        self.assertTrue(os.path.exists(tmp_onnx))

        # Check model validity with ONNX library
        model = onnx.load(tmp_onnx)
        onnx.checker.check_model(model)
        os.remove(tmp_onnx)

    def test_env_rollout_149d(self):
        """Verifies that 2D simulation environment produces 149D state and steps smoothly."""
        s0 = self.env.reset(num_static=3, num_dynamic=2)
        self.assertEqual(s0.shape, (149,))
        self.assertEqual(s0.dtype, np.float32)

        s1, reward, done, info = self.env.step((0.10, 0.05))
        self.assertEqual(s1.shape, (149,))
        self.assertIn("min_lidar_m", info)
        self.assertIn("reached_goal", info)
        self.assertIn("collision", info)


    def test_oracle_colregs_directional_clearance(self):
        """Verifies Expert Oracle steers away from directional obstacles and applies COLREGS starboard turn."""
        robot_pose = (0.0, 0.0, 0.0)
        subgoal = (2.0, 0.0)

        # 1. Obstacle at 0.50m should NOT trigger emergency zero-velocity brake
        state_05m = np.ones(149, dtype=np.float32)
        state_05m[36] = 0.50 / 5.0  # normalized 0.10
        v_05m, _ = self.oracle.compute_expert_action(robot_pose, subgoal, state_05m)
        self.assertGreater(v_05m, 0.05, "Obstacle at 0.5m must not halt the robot")

        # 2. Obstacle on Front-Right (sectors 20..30) -> steer LEFT (w > 0)
        state_right = np.ones(149, dtype=np.float32)
        state_right[20:30] = 0.18 / 5.0
        _, w_right = self.oracle.compute_expert_action(robot_pose, subgoal, state_right)
        self.assertGreater(w_right, 0.0, "Must steer LEFT away from right obstacle")

        # 3. Obstacle on Front-Left (sectors 42..52) -> steer RIGHT (w < 0)
        state_left = np.ones(149, dtype=np.float32)
        state_left[42:52] = 0.18 / 5.0
        _, w_left = self.oracle.compute_expert_action(robot_pose, subgoal, state_left)
        self.assertLess(w_left, 0.0, "Must steer RIGHT away from left obstacle")

    def test_lidar_alignment_and_edge_cases(self):
        """Verifies hardware scan roll alignment and empty array edge cases."""
        # 360-pt hardware scan with obstacle at index 0 (0 deg, Front)
        raw_hw = np.ones(360, dtype=np.float32) * 5.0
        raw_hw[0:10] = 0.20
        # With align_front_center=True, front obstacle rolls to sector 36
        s_aligned = preprocess_lidar_72(raw_hw, max_range=5.0, align_front_center=True)
        self.assertAlmostEqual(s_aligned[36] * 5.0, 0.20, places=2)

        # Edge case: empty range rate calculation
        delta_r_empty = compute_relative_lidar_velocity(np.array([]), np.array([]), (0, 0, 0), dt=0.0)
        self.assertEqual(len(delta_r_empty), 72)


if __name__ == "__main__":
    unittest.main()
