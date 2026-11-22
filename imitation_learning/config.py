"""
Configuration for Imitation Learning Navigation System
======================================================
CS532 Autonomous JetBot — Module imitation_learning
Based on UIT ITCourses RL-101 (Bài 10: Học bắt chước - Imitation Learning)

Standardizes:
  - Observation Space (41 Dimensions)
  - Action Space (2 Dimensions: v, w)
  - Normalization parameters
  - DART noise injection settings
  - DAgger aggregation parameters
  - Neural Network Hyperparameters
"""

import os
import json
import math
from typing import Tuple

# =============================================================================
# 1. OBSERVATION & ACTION SPACE DEFINITION
# =============================================================================
LIDAR_NUM_BEAMS = 36          # 36 beams sampled uniformly every 10 degrees (0..350 deg)
LIDAR_MAX_RANGE_M = 3.00      # Normalization ceiling for LiDAR distance
LIDAR_MIN_RANGE_M = 0.05      # Minimum blind spot threshold

GOAL_MAX_DIST_M = 3.00        # Normalization ceiling for goal distance
GOAL_TOLERANCE_M = 0.15       # Goal arrival threshold

# Total observation dimensions:
# 36 (LiDAR beams) + 1 (d_goal) + 2 (sin/cos alpha_goal) + 2 (curr_v, curr_w) = 41
OBSERVATION_DIM = LIDAR_NUM_BEAMS + 1 + 2 + 2  # 41
OBSERVATION_DIM_V2 = OBSERVATION_DIM + 2 * LIDAR_NUM_BEAMS  # 113 (+36 person flags +36 ego-compensated motion)
MOTION_NORM_M = 0.10          # Normaliser for ego-compensated range delta (m per 0.1s step)

# Action dimensions: [v, w]
ACTION_DIM = 2

# Discrete Action set (Stop-Turn-Go)
ACTION_STOP = 0
ACTION_FORWARD = 1
ACTION_PIVOT_LEFT = 2
ACTION_PIVOT_RIGHT = 3

# Load physical hardware dynamics from configuration file (no hardcoded literals)
_this_dir = os.path.dirname(os.path.abspath(__file__))
_repo_root = os.path.abspath(os.path.join(_this_dir, ".."))
_dyn_env = os.environ.get("ROBOT_DYNAMICS_FILE")
if _dyn_env:
    _dyn_file = os.path.abspath(_dyn_env)
else:
    _dyn_file = os.path.join(_repo_root, "setup", "robot_dynamics.json")

with open(_dyn_file, "r", encoding="utf-8") as _f:
    _dyn = json.load(_f)

# Physical hardware bounds read directly from robot_dynamics.json
MAX_LINEAR_SPEED = float(_dyn["actual_speed_mps"])
MAX_ANGULAR_SPEED = (math.pi / 2.0) / float(_dyn["turn_time_90_deg_sec"])
MIN_DRIVE_SPEED = float(_dyn["min_drive_speed"])
ROBOT_RADIUS_M = 0.09         # m (JetBot physical chassis radius)



# =============================================================================
# 2. NEURAL NETWORK ARCHITECTURE (SpiralMLP - Mu et al. arXiv:2404.00648v2)
# =============================================================================
POLICY_BACKBONE = "spiral"    # "spiral" (SpiralMLP with Spiral FC) or "mlp" (baseline)
SPIRAL_EMBED_DIM = 32         # Channel dimension C for spatial tokens
SPIRAL_DEPTH = 3              # Number of stacked SpiralBlocks (enhanced 3-stage depth)
SPIRAL_MLP_RATIO = 2.0        # Expansion ratio in Channel Mixing MLP
SPIRAL_MAX_RADIUS = 2         # Maximum spiral offset radius R for 6x6 spatial grid
SPIRAL_PERIOD = 16            # Trigonometric wave period T for spiral offsets

# Baseline MLP fallback settings
HIDDEN_DIMS = (64, 64, 32)
ACTIVATION = "relu"           # "relu", "elu", or "tanh"
USE_LAYER_NORM = True
DROPOUT_RATE = 0.05

# =============================================================================
# 3. TRAINING HYPERPARAMETERS (Behavioral Cloning)
# =============================================================================
BATCH_SIZE = 64
LEARNING_RATE = 1e-3
WEIGHT_DECAY = 1e-4           # L2 regularization to prevent overfitting on small demos
NUM_EPOCHS = 80
COSINE_LR_MIN = 1e-5
VAL_SPLIT_RATIO = 0.15
EARLY_STOP_PATIENCE = 15

# Loss function weights
W_LINEAR_LOSS = 1.0           # Weight for linear velocity MSE
W_ANGULAR_LOSS = 1.5          # Weight for angular velocity MSE (steering accuracy is critical)

# =============================================================================
# 4. COVARIATE SHIFT MITIGATION (DART & DAgger - UIT RL-101 Bài 10)
# =============================================================================
# DART (Data Gathering with Asserted Random Trajectories):
DART_ENABLED = True
DART_PROBABILITY = 0.15       # 15% probability of injecting perturbation during expert driving
DART_NOISE_STD_V = 0.03       # Noise added to linear velocity (m/s)
DART_NOISE_STD_W = 0.08       # Noise added to angular velocity (rad/s)

# DAgger (Dataset Aggregation):
DAGGER_BETA_DECAY = 0.50      # Beta parameter decay per DAgger iteration: beta_k = p^k
DAGGER_MAX_ITERATIONS = 5
DAGGER_FINETUNE_EPOCHS = 25
DAGGER_FINETUNE_LR = 5e-4

# =============================================================================
# 5. WEBSOCKET & DEPLOYMENT SETTINGS
# =============================================================================
CONTROL_HZ = 10.0             # Control loop frequency (10Hz matching LiDAR 100ms sweep)
WATCHDOG_TIMEOUT_S = 0.40     # Emergency stop watchdog if no commands received
DEFAULT_BRIDGE_HOST = "127.0.0.1"
DEFAULT_BRIDGE_PORT = 8000
