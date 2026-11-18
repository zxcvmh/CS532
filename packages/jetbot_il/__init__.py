"""SpiralMLP + DAgger Imitation Learning Navigation Package."""

from .config import (
    OBSERVATION_DIM,
    ACTION_DIM,
    POLICY_BACKBONE,
    MAX_LINEAR_SPEED,
    MAX_ANGULAR_SPEED,
    MIN_DRIVE_SPEED,
    DATA_DIR,
    MODELS_DIR,
    LOGS_DIR,
)

__all__ = [
    "OBSERVATION_DIM",
    "ACTION_DIM",
    "POLICY_BACKBONE",
    "MAX_LINEAR_SPEED",
    "MAX_ANGULAR_SPEED",
    "MIN_DRIVE_SPEED",
    "DATA_DIR",
    "MODELS_DIR",
    "LOGS_DIR",
]
