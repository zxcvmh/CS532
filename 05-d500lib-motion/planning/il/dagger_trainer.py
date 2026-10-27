#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Iterative DAgger (Dataset Aggregation) Trainer for JetBot (149D + Huber Loss)
============================================================================
CS532 Advanced Autonomous Mobile Robotics — JetBot GA25-370 Platform

Eliminates the O(T^2 epsilon) Covariate Shift error of Behavioral Cloning
by iteratively aggregating expert labels on the policy's induced state distribution.
Uses 1D-CNN architecture (149D) and Huber Loss (Smooth L1).
"""

import os
import sys
import random
import numpy as np
import torch
import torch.nn as nn
from torch.utils.data import TensorDataset, DataLoader
from typing import List, Tuple, Dict, Any, Optional

try:
    from .dagger_policy import NavPolicy1DCNN, apply_emergency_brake
    from .expert_oracle import ExpertOracle
    from .sim_env import JetBot2DSimEnv
except (ImportError, ValueError):
    cur_dir = os.path.dirname(os.path.abspath(__file__))
    sys.path.insert(0, os.path.abspath(os.path.join(cur_dir, "../..")))
    from planning.il.dagger_policy import NavPolicy1DCNN, apply_emergency_brake
    from planning.il.expert_oracle import ExpertOracle
    from planning.il.sim_env import JetBot2DSimEnv


class DAggerTrainer:
    """
    Goal-Conditioned DAgger Trainer with 1D-CNN and Huber Loss.
    """

    def __init__(
        self,
        policy: Optional[nn.Module] = None,
        oracle: Optional[ExpertOracle] = None,
        env: Optional[JetBot2DSimEnv] = None,
        lr: float = 1e-3,
        batch_size: int = 64,
        epochs_per_iter: int = 15,
        save_dir: str = "05-d500lib-motion/planning/il/checkpoints"
    ):
        self.policy = policy if policy is not None else NavPolicy1DCNN()
        self.oracle = oracle if oracle is not None else ExpertOracle()
        self.env = env if env is not None else JetBot2DSimEnv()
        self.lr = lr
        self.batch_size = batch_size
        self.epochs_per_iter = epochs_per_iter
        self.save_dir = save_dir

        os.makedirs(self.save_dir, exist_ok=True)

        self.states_buffer: List[np.ndarray] = []
        self.actions_buffer: List[np.ndarray] = []
        self.optimizer = torch.optim.Adam(self.policy.parameters(), lr=self.lr, weight_decay=1e-5)
        # Huber Loss (Smooth L1 Loss) to mitigate gradient spikes from sharp evasions
        self.criterion = nn.SmoothL1Loss(beta=0.02)

    def train_epoch(self, dataloader: DataLoader) -> float:
        """Runs one training epoch over the aggregated dataset."""
        self.policy.train()
        total_loss = 0.0
        n_batches = 0

        for states, targets in dataloader:
            self.optimizer.zero_grad()
            preds = self.policy(states)
            loss = self.criterion(preds, targets)
            loss.backward()
            torch.nn.utils.clip_grad_norm_(self.policy.parameters(), max_norm=1.0)
            self.optimizer.step()

            total_loss += loss.item()
            n_batches += 1

        return total_loss / max(1, n_batches)

    def fit_model(self) -> float:
        """Trains the policy on all accumulated dataset samples."""
        if len(self.states_buffer) == 0:
            return 0.0

        x_tensor = torch.tensor(np.array(self.states_buffer), dtype=torch.float32)
        y_tensor = torch.tensor(np.array(self.actions_buffer), dtype=torch.float32)

        dataset = TensorDataset(x_tensor, y_tensor)
        dataloader = DataLoader(dataset, batch_size=self.batch_size, shuffle=True)

        last_loss = 0.0
        for _ in range(self.epochs_per_iter):
            last_loss = self.train_epoch(dataloader)

        return last_loss

    def evaluate_policy(self, num_episodes: int = 25) -> Dict[str, float]:
        """Evaluates current policy performance in the simulation environment (Pure IL + Emergency Brake)."""
        success_count = 0
        collision_count = 0
        timeout_count = 0

        for _ in range(num_episodes):
            state = self.env.reset()
            done = False
            while not done:
                v, w = self.policy.predict_action(state)

                # Pure IL + Hard Emergency Brake Gate (v=0 if min_range < 0.15m)
                min_range_m = float(np.min(state[:72])) * self.env.max_lidar_range
                v_safe, w_safe, _ = apply_emergency_brake(v, w, min_range_m, safety_distance_m=0.15)

                next_state, _, done, info = self.env.step((v_safe, w_safe))
                state = next_state

                if done:
                    if info["reached_goal"]:
                        success_count += 1
                    elif info["collision"]:
                        collision_count += 1
                    else:
                        timeout_count += 1

        return {
            "success_rate": success_count / num_episodes,
            "collision_rate": collision_count / num_episodes,
            "timeout_rate": timeout_count / num_episodes
        }

    def run_dagger_iterations(
        self,
        num_iterations: int = 4,
        episodes_per_iter: int = 30
    ) -> List[Dict[str, Any]]:
        """
        Runs the full 4-round DAgger loop with beta schedule: [1.0, 0.5, 0.2, 0.0].
        """
        history = []
        beta_schedule = [1.0, 0.5, 0.2, 0.0]

        print("=" * 68)
        print(" 🚀 STARTING GOAL-CONDITIONED DAGGER IMITATION LEARNING (149D 1D-CNN)")
        print(f" ⚙️ Iterations: {num_iterations} | Episodes/Iter: {episodes_per_iter}")
        print("=" * 68)

        for k in range(num_iterations):
            beta = beta_schedule[k] if k < len(beta_schedule) else 0.0

            for _ in range(episodes_per_iter):
                state = self.env.reset()
                done = False

                while not done:
                    # Query expert action at the current 149D state
                    expert_v, expert_w = self.oracle.compute_expert_action(
                        robot_pose=self.env.robot_pose,
                        subgoal=self.env.subgoal,
                        lidar_ranges=state,
                        dynamic_obstacles=self.env.dynamic_obstacles
                    )

                    # Store state and expert label into aggregated dataset
                    self.states_buffer.append(state)
                    self.actions_buffer.append(np.array([expert_v, expert_w], dtype=np.float32))

                    # Action selection according to Beta mixing
                    if random.random() < beta:
                        action = (expert_v, expert_w)
                    else:
                        action = self.policy.predict_action(state)

                    next_state, _, done, _ = self.env.step(action)
                    state = next_state

            # Retrain model on all accumulated data
            train_loss = self.fit_model()

            # Evaluate policy standalone
            eval_metrics = self.evaluate_policy(num_episodes=25)

            iter_info = {
                "iteration": k,
                "beta": round(beta, 3),
                "total_samples": len(self.states_buffer),
                "train_loss": round(train_loss, 5),
                "success_rate": eval_metrics["success_rate"],
                "collision_rate": eval_metrics["collision_rate"]
            }
            history.append(iter_info)

            print(f"[Iter {k}] Beta: {beta:.2f} | Total Data: {len(self.states_buffer):5d} | "
                  f"Huber Loss: {train_loss:.5f} | Success: {eval_metrics['success_rate']*100:5.1f}% | "
                  f"Collisions: {eval_metrics['collision_rate']*100:4.1f}%")

        # Save weights and export ONNX
        import shutil
        tmp_pth = "/tmp/dagger_policy.pth"
        tmp_onnx = "/tmp/dagger_policy.onnx"
        torch.save(self.policy.state_dict(), tmp_pth)
        self.policy.export_onnx(tmp_onnx)

        pth_path = os.path.join(self.save_dir, "dagger_policy.pth")
        onnx_path = os.path.join(self.save_dir, "dagger_policy.onnx")

        try:
            shutil.copyfile(tmp_pth, pth_path)
            shutil.copyfile(tmp_onnx, onnx_path)
            print("=" * 68)
            print(" ✅ DAgger 4-Round Training Complete!")
            print(f" 💾 Checkpoint saved: {pth_path}")
            print(f" ⚡ ONNX Engine exported: {onnx_path}")
            print("=" * 68)
        except OSError as e:
            print("=" * 68)
            print(" ✅ DAgger 4-Round Training Complete!")
            print(f" 💾 Checkpoint ready in /tmp: {tmp_pth}")
            print(f" ⚡ ONNX Engine ready in /tmp: {tmp_onnx}")
            print(f" ℹ️ Notice: Workspace file sync pending ({e})")
            print("=" * 68)

        return history


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="Train 149D 1D-CNN DAgger Policy on JetBot 2D Sim")
    parser.add_argument("--iters", type=int, default=4, help="Số vòng lặp DAgger (default: 4)")
    parser.add_argument("--episodes", type=int, default=25, help="Số episodes mỗi vòng lặp (default: 25)")
    parser.add_argument("--epochs", type=int, default=15, help="Số epochs train mạng mỗi vòng (default: 15)")
    parser.add_argument("--batch-size", type=int, default=64, help="Batch size (default: 64)")
    args = parser.parse_args()

    trainer = DAggerTrainer(epochs_per_iter=args.epochs, batch_size=args.batch_size)
    trainer.run_dagger_iterations(num_iterations=args.iters, episodes_per_iter=args.episodes)

