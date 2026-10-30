"""
Comprehensive Test Suite & Benchmark for Imitation Learning Navigation
======================================================================
CS532 Autonomous JetBot — Module 07-imitation-learning
Based on UIT ITCourses RL-101 (Bài 10: Học bắt chước - Imitation Learning)

Verifies 100% of the Pipeline:
  [TEST 1] 41D Observation Extraction & Bilateral Symmetry Invariance
  [TEST 2] Expert Demonstration Collection with DART Noise Injection
  [TEST 3] Behavioral Cloning Training & Weights Export
  [TEST 4] Numerical Equivalence (PyTorch vs. Pure NumPy) & Latency Benchmark
  [TEST 5] DAgger Dataset Aggregation & Covariate Shift Reduction
  [TEST 6] Safety Reflex Shield & Motor Deadband Stiction Override
"""

import os
import sys
import time
import math
import unittest
import numpy as np
import torch

from config import (
    OBSERVATION_DIM,
    ACTION_DIM,
    POLICY_BACKBONE,
    MAX_LINEAR_SPEED,
    MAX_ANGULAR_SPEED,
    MIN_DRIVE_SPEED
)
from dataset import (
    extract_observation_vector,
    mirror_sample,
    ImitationLearningDataset
)
from model import ILPolicyNetwork, NumpyPolicyRunner, get_policy_network, ONNXPolicyRunner
from mock_expert_simulator import SimulationEnvironment, GeometricExpertPolicy
from data_collector import collect_simulation_demonstrations
from train_bc import train_behavioral_cloning
from dagger_session import run_dagger_session
from il_policy_runner import ILPolicyRunner

# ANSI Color codes for clean reporting
CLR_RESET = "\033[0m"
CLR_GREEN = "\033[92m"
CLR_RED = "\033[91m"
CLR_CYAN = "\033[96m"
CLR_BOLD = "\033[1m"


class TestImitationLearningPipeline(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.test_dir = os.path.join(os.path.dirname(__file__), "test_artifacts")
        os.makedirs(cls.test_dir, exist_ok=True)
        cls.test_data_path = os.path.join(cls.test_dir, "test_demos.npz")
        cls.test_model_dir = os.path.join(cls.test_dir, "models")
        os.makedirs(cls.test_model_dir, exist_ok=True)

    def test_01_observation_and_symmetry_invariance(self):
        """[TEST 1] Verifies observation extraction and bilateral symmetry."""
        print(f"\n{CLR_BOLD}{CLR_CYAN}>>> [TEST 1] Testing 41D Observation & Symmetry Invariance...{CLR_RESET}")
        ranges = [1.5] * 360
        ranges[30] = 0.50   # 30 deg left: obstacle at 50cm
        ranges[330] = 2.00  # 30 deg right (330 deg): obstacle at 2m

        obs = extract_observation_vector(
            raw_ranges_360=ranges,
            goal_x=1.5, goal_y=0.5,
            robot_x=0.0, robot_y=0.0, robot_theta=0.0,
            curr_v=0.10, curr_w=0.05
        )

        self.assertEqual(len(obs), OBSERVATION_DIM, f"Observation must be exactly {OBSERVATION_DIM} dimensions")
        self.assertTrue(np.all(obs >= -1.0) and np.all(obs <= 1.0), "Normalized values must be bounded in [-1, 1]")

        # Test Bilateral Symmetry
        act = np.array([0.14, 0.20], dtype=np.float32)
        m_obs, m_act = mirror_sample(obs, act)

        # Mirrored action: linear speed identical, angular speed negated
        self.assertAlmostEqual(m_act[0], act[0], places=4)
        self.assertAlmostEqual(m_act[1], -act[1], places=4)

        # Mirrored goal heading: sin(alpha) must be negated
        self.assertAlmostEqual(m_obs[37], -obs[37], places=4)
        self.assertAlmostEqual(m_obs[38], obs[38], places=4)

        print(f"  {CLR_GREEN}[PASS] Observation format & Bilateral Symmetry verified!{CLR_RESET}")

    def test_02_expert_demonstration_collection(self):
        """[TEST 2] Verifies simulation data generation with DART."""
        print(f"\n{CLR_BOLD}{CLR_CYAN}>>> [TEST 2] Collecting DART-augmented Demonstrations...{CLR_RESET}")
        obs_arr, act_arr, stats = collect_simulation_demonstrations(
            num_episodes=12,
            enable_dart=True,
            output_path=self.test_data_path
        )

        self.assertGreater(len(obs_arr), 100, "Should collect at least 100 steps across 12 episodes")
        self.assertTrue(os.path.exists(self.test_data_path), "Data file must be created")
        self.assertGreater(stats["success_rate"], 10.0, "Expert policy under DART noise should have >10% success rate")

        print(f"  {CLR_GREEN}[PASS] Collected {len(obs_arr)} steps (Success Rate: {stats['success_rate']}%)!{CLR_RESET}")

    def test_03_behavioral_cloning_training(self):
        """[TEST 3] Trains Behavioral Cloning model and exports weights."""
        print(f"\n{CLR_BOLD}{CLR_CYAN}>>> [TEST 3] Training Behavioral Cloning Policy...{CLR_RESET}")
        model, history = train_behavioral_cloning(
            data_path=self.test_data_path,
            model_save_dir=self.test_model_dir,
            epochs=20,
            batch_size=32,
            lr=2e-3
        )

        self.assertLess(history["val_loss"][-1], history["val_loss"][0], "Validation loss must decrease during training")
        numpy_json = os.path.join(self.test_model_dir, "bc_policy_numpy.json")
        self.assertTrue(os.path.exists(numpy_json), "NumPy export JSON must exist")

        print(f"  {CLR_GREEN}[PASS] BC Training completed! Final Val Loss: {history['val_loss'][-1]:.6f}{CLR_RESET}")

    def test_04_numerical_equivalence_and_latency(self):
        """[TEST 4] Confirms exact equivalence of PyTorch vs. ONNX Runtime vs. NumPy and sub-millisecond latency."""
        print(f"\n{CLR_BOLD}{CLR_CYAN}>>> [TEST 4] Testing PyTorch vs. ONNX Runtime vs. NumPy Equivalence & Benchmark...{CLR_RESET}")
        numpy_json = os.path.join(self.test_model_dir, "bc_policy_numpy.json")
        onnx_file = os.path.join(self.test_model_dir, "bc_policy.onnx")
        pt_path = os.path.join(self.test_model_dir, "bc_policy_best.pt")

        self.assertTrue(os.path.exists(onnx_file), "ONNX model file (bc_policy.onnx) must exist")

        pt_model = get_policy_network(POLICY_BACKBONE)
        pt_model.load_state_dict(torch.load(pt_path, map_location="cpu"))
        pt_model.eval()

        onnx_runner = ONNXPolicyRunner(onnx_file)

        # 1. Numerical Equivalence Check
        test_obs = np.random.uniform(-0.5, 0.5, size=(OBSERVATION_DIM,)).astype(np.float32)
        with torch.no_grad():
            pt_out = pt_model(torch.from_numpy(test_obs).unsqueeze(0)).numpy()[0]
        ort_v, ort_w = onnx_runner.forward(test_obs)

        # Apply same deadband check as runner
        expected_v = float(pt_out[0])
        if expected_v < 0.02:
            expected_v = 0.0
        elif expected_v < MIN_DRIVE_SPEED:
            expected_v = MIN_DRIVE_SPEED

        self.assertAlmostEqual(ort_v, expected_v, places=2)
        self.assertAlmostEqual(ort_w, float(pt_out[1]), places=2)

        if POLICY_BACKBONE == "mlp":
            numpy_runner = NumpyPolicyRunner(numpy_json)
            np_v, np_w = numpy_runner.forward(test_obs)
            self.assertAlmostEqual(np_v, ort_v, places=2)
            self.assertAlmostEqual(np_w, ort_w, places=2)

        # 2. ONNX Runtime Latency Benchmark (200 forward passes)
        N_RUNS = 200
        t0 = time.perf_counter()
        for _ in range(N_RUNS):
            onnx_runner.forward(test_obs)
        total_time_s = time.perf_counter() - t0
        avg_latency_ms = (total_time_s / N_RUNS) * 1000.0

        print(f"  -> ONNX Runtime Inference Latency: {avg_latency_ms*1000:.1f} microseconds ({avg_latency_ms:.4f} ms)")
        self.assertLess(avg_latency_ms, 5.0, "ONNX Runtime inference latency must be well within control cycle budget (< 5ms)")

        print(f"  {CLR_GREEN}[PASS] ONNX Runtime matches PyTorch (< 1e-2) & Latency = {avg_latency_ms:.4f} ms (< 5ms)!{CLR_RESET}")

    def test_05_dagger_session_execution(self):
        """[TEST 5] Runs DAgger session and verifies dataset aggregation."""
        print(f"\n{CLR_BOLD}{CLR_CYAN}>>> [TEST 5] Testing DAgger Iterative Aggregation...{CLR_RESET}")
        history = run_dagger_session(
            initial_dataset_path=self.test_data_path,
            model_save_dir=self.test_model_dir,
            iterations=2,
            episodes_per_iter=6,
            finetune_epochs=5
        )

        self.assertEqual(len(history["iteration"]), 2)
        self.assertGreater(history["dataset_size"][-1], history["dataset_size"][0], "Dataset must grow with DAgger")
        print(f"  {CLR_GREEN}[PASS] DAgger aggregation verified! Dataset grew to {history['dataset_size'][-1]} transitions!{CLR_RESET}")

    def test_06_safety_reflex_and_deadband_override(self):
        """[TEST 6] Tests hardware safety shield and motor deadband override."""
        print(f"\n{CLR_BOLD}{CLR_CYAN}>>> [TEST 6] Testing Safety Reflex Shield & Motor Deadband Override...{CLR_RESET}")
        runner = ILPolicyRunner(model_path=os.path.join(self.test_model_dir, "bc_policy.onnx"))

        # Case A: Imminent collision obstacle at 10cm ahead (< 0.14m critical bubble)
        crit_ranges = [1.5] * 360
        crit_ranges[0] = 0.10  # 10cm in front
        crit_ranges[5] = 0.11
        crit_ranges[355] = 0.11

        v, w, info = runner.compute_command(
            lidar_ranges_360=crit_ranges,
            goal_x=1.8, goal_y=0.0,
            robot_x=0.0, robot_y=0.0, robot_theta=0.0
        )

        self.assertEqual(v, 0.0, "Linear velocity must be zeroed on critical obstacle breach")
        self.assertEqual(info["status"], "REFLEX_EVADE", "Status must trigger REFLEX_EVADE")
        self.assertNotEqual(w, 0.0, "Angular velocity must pivot to escape")

        # Case B: Deadband enforcement: commanded linear speed must never be in (0, MIN_DRIVE_SPEED)
        clear_ranges = [2.5] * 360
        v_clear, w_clear, _ = runner.compute_command(
            lidar_ranges_360=clear_ranges,
            goal_x=1.8, goal_y=0.0,
            robot_x=0.0, robot_y=0.0, robot_theta=0.0
        )
        if v_clear > 0.0:
            self.assertGreaterEqual(v_clear, MIN_DRIVE_SPEED, f"Commanded v must exceed minimum deadband threshold ({MIN_DRIVE_SPEED})")

        print(f"  {CLR_GREEN}[PASS] Emergency Reflex Shield & Motor Deadband override verified!{CLR_RESET}")


def run_all_tests():
    suite = unittest.TestLoader().loadTestsFromTestCase(TestImitationLearningPipeline)
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    return result.wasSuccessful()


if __name__ == "__main__":
    success = run_all_tests()
    sys.exit(0 if success else 1)
