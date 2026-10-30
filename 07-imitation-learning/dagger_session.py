"""
DAgger (Dataset Aggregation) Iterative Learning Session
=======================================================
CS532 Autonomous JetBot — Module 07-imitation-learning
Based on UIT ITCourses RL-101 (Bài 10: Học bắt chước - Imitation Learning)

Algorithm (Ross, Gordon, Bagnell, AISTATS 2011):
-------------------------------------------------
  1. Initialize D_1 <- expert demonstrations D_0
  2. Train policy hat{pi}_1 on D_1 using Behavioral Cloning
  3. For iteration k = 1 to K:
       a. Let beta_k = p^k (prob of expert action)
       b. Roll out pi_k = beta_k * pi* + (1 - beta_k) * hat{pi}_k
          collecting trajectory tau = {s_1, ..., s_T}
       c. Query expert for optimal action on all visited states: a_t* = pi*(s_t)
       d. Aggregate dataset: D_{k+1} <- D_k U {(s_t, a_t*)}
       e. Retrain hat{pi}_{k+1} on D_{k+1}
  4. Guarantees linear compounding error O(T * epsilon) instead of O(T^2 * epsilon)
"""

import os
import sys
import random
import argparse
from typing import List, Tuple, Dict, Any, Optional
import numpy as np

import torch
import torch.nn as nn
from torch.utils.data import DataLoader

from config import (
    DAGGER_MAX_ITERATIONS,
    DAGGER_BETA_DECAY,
    DAGGER_FINETUNE_EPOCHS,
    DAGGER_FINETUNE_LR,
    BATCH_SIZE,
    W_LINEAR_LOSS,
    W_ANGULAR_LOSS,
    POLICY_BACKBONE
)
from dataset import (
    extract_observation_vector,
    ImitationLearningDataset
)
from model import get_policy_network, NumpyPolicyRunner
from mock_expert_simulator import (
    SimulationEnvironment,
    GeometricExpertPolicy
)


def run_dagger_session(
    initial_dataset_path: str = "data/expert_demonstrations.npz",
    model_save_dir: str = "models",
    iterations: int = DAGGER_MAX_ITERATIONS,
    episodes_per_iter: int = 15,
    finetune_epochs: int = DAGGER_FINETUNE_EPOCHS,
    backbone: str = POLICY_BACKBONE
) -> Dict[str, Any]:
    """
    Executes complete DAgger interactive aggregation and training loop.
    """
    os.makedirs(model_save_dir, exist_ok=True)
    best_model_path = os.path.join(model_save_dir, "dagger_policy_best.pt")
    dagger_onnx_path = os.path.join(model_save_dir, "dagger_policy.onnx")
    numpy_weights_path = os.path.join(model_save_dir, "dagger_policy_numpy.json")
    aggregated_data_path = os.path.join(os.path.dirname(initial_dataset_path), "dagger_aggregated_dataset.npz")

    # 1. Initialize Dataset D_1
    print(f"\n[DAgger] Loading initial demonstration dataset: {initial_dataset_path}")
    dataset = ImitationLearningDataset.load_npz(initial_dataset_path, augment_symmetry=True)
    print(f"[DAgger] Initial dataset size: {len(dataset)} transitions")

    # 2. Initialize Model
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"[DAgger] Initializing backbone: {backbone.upper()} on device: {device}")
    model = get_policy_network(backbone=backbone).to(device)

    # Check if a pre-trained BC model exists to jumpstart
    bc_model_path = os.path.join(model_save_dir, "bc_policy_best.pt")
    if os.path.exists(bc_model_path):
        try:
            print(f"[DAgger] Loading baseline pre-trained BC weights from: {bc_model_path}")
            model.load_state_dict(torch.load(bc_model_path, map_location=device))
        except Exception as e:
            print(f"[DAgger] Note: Could not load BC weights (architecture may differ): {e}")

    env = SimulationEnvironment()
    expert = GeometricExpertPolicy()

    dagger_history = {
        "iteration": [],
        "dataset_size": [],
        "success_rate": [],
        "collision_rate": [],
        "beta": []
    }

    print("\n" + "=" * 80)
    print(f"{'Iter':^6} | {'Beta (Expert)':^14} | {'Dataset Size':^14} | {'Success Rate':^14} | {'Collision Rate':^14}")
    print("=" * 80)

    for k in range(1, iterations + 1):
        # Mixing probability: beta_k = p^k
        beta_k = DAGGER_BETA_DECAY ** k
        if k == iterations:
            beta_k = 0.0  # Final iteration is 100% autonomous evaluation!

        new_obs_list = []
        new_act_list = []

        success_count = 0
        collision_count = 0

        # Rollout trajectories under mixed policy pi_k
        model.eval()
        for ep in range(episodes_per_iter):
            env.reset(num_obstacles=random.randint(1, 3))
            expert.reset_state()
            reached = False
            collided = False

            for step in range(250):
                ranges = env.get_lidar_scan()

                # Extract state s_t
                obs_t = extract_observation_vector(
                    ranges,
                    env.goal_x, env.goal_y,
                    env.x, env.y, env.theta,
                    env.v, env.w
                )

                # Query expert oracle for ground-truth optimal action a*_t = pi*(s_t)
                expert_v, expert_w = expert.get_action(
                    ranges,
                    env.goal_x, env.goal_y,
                    env.x, env.y, env.theta,
                    apply_dart_noise=False
                )

                # Query current policy hat{pi}_k(s_t)
                with torch.no_grad():
                    obs_t_tensor = torch.from_numpy(obs_t).unsqueeze(0).to(device)
                    pred_act = model(obs_t_tensor).cpu().numpy()[0]
                    policy_v, policy_w = float(pred_act[0]), float(pred_act[1])

                # Determine executed action based on mixing probability beta_k
                if random.random() < beta_k:
                    exec_v, exec_w = expert_v, expert_w
                else:
                    exec_v, exec_w = policy_v, policy_w

                # DAgger Step: State s_t visited under policy is labeled with EXPERT action a*_t
                new_obs_list.append(obs_t)
                new_act_list.append([expert_v, expert_w])

                reached, collided = env.step(exec_v, exec_w)
                if reached or collided:
                    break

            if reached:
                success_count += 1
            if collided:
                collision_count += 1

        succ_rate = round(success_count / episodes_per_iter * 100.0, 1)
        coll_rate = round(collision_count / episodes_per_iter * 100.0, 1)

        # Aggregate new transitions into D
        new_obs_arr = np.array(new_obs_list, dtype=np.float32)
        new_act_arr = np.array(new_act_list, dtype=np.float32)
        dataset.add_samples(new_obs_arr, new_act_arr, augment_symmetry=True)
        dataset.save_npz(aggregated_data_path)

        dagger_history["iteration"].append(k)
        dagger_history["beta"].append(round(beta_k, 3))
        dagger_history["dataset_size"].append(len(dataset))
        dagger_history["success_rate"].append(succ_rate)
        dagger_history["collision_rate"].append(coll_rate)

        print(f"{k:^6d} | {beta_k:^14.3f} | {len(dataset):^14d} | {succ_rate:^13.1f}% | {coll_rate:^13.1f}%")

        # Retrain / Fine-tune model on aggregated dataset D_{k+1}
        optimizer = torch.optim.AdamW(model.parameters(), lr=DAGGER_FINETUNE_LR, weight_decay=1e-4)
        loader = DataLoader(dataset, batch_size=BATCH_SIZE, shuffle=True)

        model.train()
        for f_ep in range(finetune_epochs):
            for b_obs, b_act in loader:
                b_obs = b_obs.to(device)
                b_act = b_act.to(device)
                pred = model(b_obs)
                lv = nn.functional.mse_loss(pred[:, 0], b_act[:, 0])
                lw = nn.functional.mse_loss(pred[:, 1], b_act[:, 1])
                loss = W_LINEAR_LOSS * lv + W_ANGULAR_LOSS * lw
                optimizer.zero_grad()
                loss.backward()
                optimizer.step()

        # Save latest checkpoints
        torch.save(model.state_dict(), best_model_path)
        model.save_numpy_weights(numpy_weights_path)
        try:
            model.export_onnx(dagger_onnx_path)
        except Exception as e:
            print(f"[WARN] ONNX export failed: {e}")

    print("=" * 80)
    print(f"\n[SUCCESS] DAgger iterations completed!")
    print(f"          Final Success Rate : {dagger_history['success_rate'][-1]}%")
    print(f"          Final Collision Rate: {dagger_history['collision_rate'][-1]}%")
    print(f"          Saved PyTorch model : {best_model_path}")
    print(f"          Saved ONNX model    : {dagger_onnx_path}")
    print(f"          Saved NumPy weights : {numpy_weights_path}\n")

    return dagger_history


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Run DAgger Dataset Aggregation Session")
    parser.add_argument("--data", type=str, default="data/expert_demonstrations.npz", help="Initial dataset")
    parser.add_argument("--save-dir", type=str, default="models", help="Model directory")
    parser.add_argument("--iterations", type=int, default=DAGGER_MAX_ITERATIONS, help="DAgger iterations")
    parser.add_argument("--backbone", type=str, default=POLICY_BACKBONE, choices=["spiral", "mlp"], help="Backbone architecture")
    args = parser.parse_args()

    run_dagger_session(
        initial_dataset_path=args.data,
        model_save_dir=args.save_dir,
        iterations=args.iterations,
        backbone=args.backbone
    )
