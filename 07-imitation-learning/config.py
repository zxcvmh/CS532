"""
Configuration for Imitation Learning Navigation System
======================================================
CS532 Autonomous JetBot — Module 07-imitation-learning
Based on UIT ITCourses RL-101 (Bài 10: Học bắt chước - Imitation Learning)

Standardizes:
  - Observation Space (41 Dimensions)
  - Action Space (2 Dimensions: v, w)
  - Normalization parameters
  - DART noise injection settings
  - DAgger aggregation parameters
  - Neural Network Hyperparameters
"""

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

# Action dimensions: [v, w]
ACTION_DIM = 2

# Physical hardware bounds (calibrated to robot_dynamics.json)
MAX_LINEAR_SPEED = 0.16       # m/s (safe cruise on glossy ceramic tile)
MIN_DRIVE_SPEED = 0.10        # m/s (motor deadband threshold to overcome gearbox stiction)
MAX_ANGULAR_SPEED = 0.35      # rad/s (~20 deg/s, avoids wheel slip)
ROBOT_RADIUS_M = 0.09         # m (JetBot physical chassis radius)


# =============================================================================
# 2. NEURAL NETWORK ARCHITECTURE
# =============================================================================
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
