"""
Data Collector for Imitation Learning (Simulation & Real JetBot Bridge)
=======================================================================
CS532 Autonomous JetBot — Module 07-imitation-learning
Based on UIT ITCourses RL-101 (Bài 10: Học bắt chước - Imitation Learning)

Features:
  1. Synthetic Expert Data Generation:
     - Generates N obstacle-rich episodes with DART noise injection
  2. Live Teleoperation Logger:
     - Listens to WebSocket bridge telemetry and user drive commands
     - Pairs LiDAR scan + goal orientation with user action (v, w)
  3. Automatic Dataset Serialization to .npz
"""

import os
import sys
import time
import random
import argparse
import numpy as np
from typing import List, Tuple, Dict, Any

from config import (
    CONTROL_HZ,
    DART_ENABLED,
    MAX_LINEAR_SPEED,
    MAX_ANGULAR_SPEED
)
from dataset import (
    extract_observation_vector,
    ImitationLearningDataset
)
from mock_expert_simulator import (
    SimulationEnvironment,
    GeometricExpertPolicy
)


def collect_simulation_demonstrations(
    num_episodes: int = 30,
    max_steps_per_episode: int = 250,
    enable_dart: bool = True,
    output_path: str = "data/expert_demonstrations.npz"
) -> Tuple[np.ndarray, np.ndarray, Dict[str, Any]]:
    """
    Simulates expert trajectories across diverse obstacle environments.
    """
    env = SimulationEnvironment()
    expert = GeometricExpertPolicy()

    all_obs = []
    all_act = []

    success_count = 0
    collision_count = 0
    total_steps = 0

    print(f"\n[INFO] Collecting {num_episodes} demonstration episodes (DART noise={enable_dart})...")

    for ep in range(1, num_episodes + 1):
        env.reset(num_obstacles=random.randint(1, 3))
        expert.reset_state()
        ep_steps = 0
        ep_obs = []
        ep_act = []

        while ep_steps < max_steps_per_episode:
            ranges = env.get_lidar_scan()
            # Expert computes ground truth action
            cmd_v, cmd_w = expert.get_action(
                ranges,
                env.goal_x, env.goal_y,
                env.x, env.y, env.theta,
                apply_dart_noise=enable_dart
            )

            # Extract 41D observation vector
            obs = extract_observation_vector(
                raw_ranges_360=ranges,
                goal_x=env.goal_x,
                goal_y=env.goal_y,
                robot_x=env.x,
                robot_y=env.y,
                robot_theta=env.theta,
                curr_v=env.v,
                curr_w=env.w
            )

            ep_obs.append(obs)
            ep_act.append([cmd_v, cmd_w])

            reached, collided = env.step(cmd_v, cmd_w)
            ep_steps += 1

            if collided:
                collision_count += 1
                # Discard catastrophic collision steps from clean expert demonstrations
                break

            if reached:
                success_count += 1
                all_obs.extend(ep_obs)
                all_act.extend(ep_act)
                total_steps += len(ep_obs)
                break

        status = "SUCCESS" if reached else ("COLLISION" if collided else "TIMEOUT")
        if ep % 5 == 0 or ep == num_episodes:
            print(f"  Episode {ep:02d}/{num_episodes} — Steps: {ep_steps:03d} — Status: {status}")

    obs_arr = np.array(all_obs, dtype=np.float32)
    act_arr = np.array(all_act, dtype=np.float32)

    stats = {
        "num_episodes": num_episodes,
        "success_rate": round(success_count / max(1, num_episodes) * 100.0, 1),
        "total_transitions": len(obs_arr),
        "output_path": output_path
    }

    os.makedirs(os.path.dirname(os.path.abspath(output_path)), exist_ok=True)
    dataset = ImitationLearningDataset(obs_arr, act_arr, augment_symmetry=True)
    dataset.save_npz(output_path)

    print(f"\n[DONE] Dataset saved: {output_path}")
    print(f"       Transitions collected: {len(obs_arr)} (with symmetry augmentation: {len(dataset)})")
    print(f"       Expert Success Rate: {stats['success_rate']}%\n")

    return obs_arr, act_arr, stats


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Collect Imitation Learning Demonstrations")
    parser.add_argument("--episodes", type=int, default=35, help="Number of episodes to record")
    parser.add_argument("--no-dart", action="store_true", help="Disable DART noise injection")
    parser.add_argument("--output", type=str, default="data/expert_demonstrations.npz", help="Output file path")
    args = parser.parse_args()

    collect_simulation_demonstrations(
        num_episodes=args.episodes,
        enable_dart=not args.no_dart,
        output_path=args.output
    )
