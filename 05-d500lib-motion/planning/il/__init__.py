"""
Imitation Learning (DAgger) Module for CS532 Autonomous JetBot.
Includes:
- NavPolicy1DCNN: Goal-conditioned 1D-CNN neural network policy (149D).
- NavPolicyMLP: Benchmark MLP policy.
- DAggerTrainer: Iterative Dataset Aggregator for reducing covariate shift.
- ExpertOracle: Teacher wrapper around KinodynamicRVO with COLREGS Rule 14/15.
- JetBot2DSimEnv: 2D kinematic simulation environment with 149D perception.
- apply_emergency_brake: Hard safety reflex (< 0.15m => v = 0).
- apply_motor_deadband: GA25-370 deadband compensation (0.12 + 0.88 * |u|).
"""
from .dagger_policy import (
    NavPolicy1DCNN,
    NavPolicyMLP,
    preprocess_lidar_72,
    compute_relative_lidar_velocity,
    build_state_vector,
    apply_emergency_brake,
    apply_motor_deadband,
)
from .expert_oracle import ExpertOracle
from .dagger_trainer import DAggerTrainer
from .sim_env import JetBot2DSimEnv

__all__ = [
    "NavPolicy1DCNN",
    "NavPolicyMLP",
    "preprocess_lidar_72",
    "compute_relative_lidar_velocity",
    "build_state_vector",
    "apply_emergency_brake",
    "apply_motor_deadband",
    "ExpertOracle",
    "DAggerTrainer",
    "JetBot2DSimEnv",
]
