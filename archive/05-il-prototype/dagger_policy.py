#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Goal-Conditioned DAgger Policy Network for JetBot
=================================================
CS532 Advanced Autonomous Mobile Robotics — JetBot GA25-370 Platform

State vector (149 dimensions):
  - 72 sectors of 2D LiDAR ranges (normalized [0, 1])
  - 72 sectors of odometry-compensated relative range velocity Delta r (normalized [-1, 1])
  - 1 relative distance to Subgoal rho (normalized [0, 1])
  - 1 sin(phi) of relative azimuth to Subgoal
  - 1 cos(phi) of relative azimuth to Subgoal
  - 1 previous linear velocity v_prev / v_max
  - 1 previous angular velocity w_prev / w_max

Action vector (2 dimensions):
  - v_nom in [0.0, v_max] (m/s)
  - w_nom in [-w_max, w_max] (rad/s)
"""

import math
import numpy as np
import torch
import torch.nn as nn
from typing import List, Tuple, Optional, Union


def preprocess_lidar_72(
    raw_ranges: Union[List[float], np.ndarray],
    num_sectors: int = 72,
    min_range: float = 0.05,
    max_range: float = 6.0,
    align_front_center: bool = False
) -> np.ndarray:
    """
    Downsamples full 360-degree LiDAR scan into 72 conservative sectors (5 degrees each).
    Takes the minimum valid range within each sector to ensure safety margin.
    Waveshare D500 DTOF hardware specs:
      - Blind zone floor: 0.03m (configured to 0.05m with 2cm chassis mounting margin).
      - Effective range: up to 12.0m (clipped to 6.0m for 5x5m indoor seminar room).
      - Ranging rate: 5000 Hz (~500 pts per scan at 10 Hz).
    If align_front_center=True (for standard hardware 360 CCW scans with index 0 at Front),
    rolls by -n_points // 2 so Sector 36 corresponds to Front (0 deg), matching [-pi, pi).
    Returns array of shape (72,) normalized to [0.0, 1.0].
    """
    if len(raw_ranges) == 0:
        return np.ones(num_sectors, dtype=np.float32)

    raw = np.asarray(raw_ranges, dtype=np.float32)
    n_points = len(raw)

    if align_front_center and n_points >= 360:
        raw = np.roll(raw, -n_points // 2)

    points_per_sector = max(1, n_points // num_sectors)
    sectors = np.ones(num_sectors, dtype=np.float32) * max_range

    for i in range(num_sectors):
        start_idx = i * points_per_sector
        end_idx = min(n_points, (i + 1) * points_per_sector) if i < num_sectors - 1 else n_points
        if start_idx < end_idx:
            chunk = raw[start_idx:end_idx]
            valid_mask = (chunk >= min_range) & (chunk <= max_range)
            if np.any(valid_mask):
                sectors[i] = np.min(chunk[valid_mask])
            else:
                sectors[i] = max_range

    # Normalize to [0.0, 1.0]
    return np.clip(sectors / max_range, 0.0, 1.0).astype(np.float32)


def compute_relative_lidar_velocity(
    current_ranges_72: np.ndarray,
    previous_ranges_72: np.ndarray,
    delta_pose: Tuple[float, float, float],
    dt: float = 0.10,
    max_lidar_range: float = 5.0,
    max_relative_speed: float = 1.5
) -> np.ndarray:
    """
    Computes odometry-compensated relative range rate Delta r across 72 sectors.
    Delta r = (r_t - r_{t-1}^warped) / dt.

    Ego-motion warping:
      1. Angular shift: Warps scan sectors by delta yaw dtheta.
      2. Radial shift: Projects robot translation (dx, dy) onto each sector beam.

    For static obstacles, ego-motion warping cancels out displacement -> Delta r ≈ 0.
    For approaching dynamic obstacles (pedestrians) -> Delta r < 0 (closing in).
    For receding dynamic obstacles -> Delta r > 0.

    Returns array of shape (72,) normalized to [-1.0, 1.0].
    """
    if dt <= 1e-4 or len(current_ranges_72) == 0:
        return np.zeros(72, dtype=np.float32)

    dx_local, dy_local, dth = delta_pose
    num_sectors = len(current_ranges_72)
    sector_res = 2.0 * math.pi / num_sectors

    # Convert normalized ranges back to meters for geometric calculation
    curr_m = current_ranges_72 * max_lidar_range
    prev_m = previous_ranges_72 * max_lidar_range

    # Sector angles in robot frame [-pi, pi)
    angles = np.linspace(-math.pi, math.pi, num_sectors, endpoint=False)

    # 1. Angular warping by rotation dth
    shift_sectors = int(round(dth / sector_res))
    if shift_sectors != 0:
        prev_warped_rot = np.roll(prev_m, shift_sectors)
    else:
        prev_warped_rot = prev_m

    # 2. Radial projection of local ego translation (dx, dy)
    proj_trans = dx_local * np.cos(angles) + dy_local * np.sin(angles)
    expected_static_prev = prev_warped_rot - proj_trans

    # 3. Difference: (current - warped_previous) / dt
    delta_r = (curr_m - expected_static_prev) / dt

    # Clip to physical bounds [-max_relative_speed, max_relative_speed] and normalize
    norm_delta_r = np.clip(delta_r / max(1e-3, max_relative_speed), -1.0, 1.0)
    return norm_delta_r.astype(np.float32)


def build_state_vector(
    lidar_72: np.ndarray,
    robot_pose: Tuple[float, float, float],
    subgoal: Tuple[float, float],
    delta_r_72: Optional[np.ndarray] = None,
    v_prev: float = 0.0,
    w_prev: float = 0.0,
    max_v: float = 0.12,
    max_w: float = 0.35,
    max_subgoal_dist: float = 5.0
) -> np.ndarray:
    """
    Builds the 149-dimensional state vector for the DAgger policy:
      - 72 sectors range (normalized [0, 1])
      - 72 sectors velocity difference Delta r (normalized [-1, 1])
      - 5 meta features: [dist_norm, sin_phi, cos_phi, v_norm, w_norm]
    """
    rx, ry, rth = robot_pose
    gx, gy = subgoal

    dx = gx - rx
    dy = gy - ry
    dist = math.hypot(dx, dy)
    dist_norm = min(dist / max_subgoal_dist, 1.0)

    # Relative azimuth in robot frame
    goal_angle_world = math.atan2(dy, dx)
    rel_phi = goal_angle_world - rth
    rel_phi = (rel_phi + math.pi) % (2.0 * math.pi) - math.pi

    sin_phi = math.sin(rel_phi)
    cos_phi = math.cos(rel_phi)

    v_norm = max(0.0, min(1.0, v_prev / max(max_v, 1e-4)))
    w_norm = max(-1.0, min(1.0, w_prev / max(max_w, 1e-4)))

    meta_features = np.array([dist_norm, sin_phi, cos_phi, v_norm, w_norm], dtype=np.float32)

    if delta_r_72 is None:
        delta_r_72 = np.zeros(72, dtype=np.float32)

    state_149 = np.concatenate([lidar_72, delta_r_72, meta_features]).astype(np.float32)
    return state_149


def apply_emergency_brake(
    v_nom: float,
    w_nom: float,
    min_range_m: float,
    safety_distance_m: float = 0.15
) -> Tuple[float, float, bool]:
    """
    Hard Safety Gate: Pure IL controls the vehicle, but if any obstacle violates
    the critical safety boundary (< 0.15m), linear speed is clamped immediately to 0.0.
    Returns: (v_safe, w_safe, emergency_active)
    """
    if min_range_m < safety_distance_m:
        return 0.0, float(w_nom), True
    return float(v_nom), float(w_nom), False


def apply_motor_deadband(u: float, deadband: float = 0.12) -> float:
    """
    GA25-370 Motor Deadband Compensation at the actuator layer:
    u_motor = sign(u) * (deadband + (1.0 - deadband) * |u|)
    """
    if abs(u) < 1e-4:
        return 0.0
    return math.copysign(deadband + (1.0 - deadband) * min(1.0, abs(u)), u)


class NavPolicy1DCNN(nn.Module):
    """
    Goal-Conditioned 1D-CNN Policy Network for JetBot.
    Processes 2-channel LiDAR state:
      - Channel 0: 72 normalized range sectors
      - Channel 1: 72 normalized relative range rates (Delta r)
    Fused with 5-dim meta-feature vector (subgoal rho, sin phi, cos phi, v_prev, w_prev).
    Inference latency < 0.5 ms on CPU, < 1.0 ms on Jetson Nano ARM.
    """

    def __init__(
        self,
        max_v: float = 0.12,
        max_w: float = 0.35
    ):
        super().__init__()
        self.max_v = max_v
        self.max_w = max_w

        # Feature extractor on 2x72 LiDAR signals
        self.conv = nn.Sequential(
            nn.Conv1d(in_channels=2, out_channels=16, kernel_size=5, stride=1, padding=2),
            nn.BatchNorm1d(16),
            nn.ReLU(),
            nn.MaxPool1d(kernel_size=2, stride=2),  # (B, 16, 36)

            nn.Conv1d(in_channels=16, out_channels=32, kernel_size=3, stride=1, padding=1),
            nn.BatchNorm1d(32),
            nn.ReLU(),
            nn.MaxPool1d(kernel_size=2, stride=2),  # (B, 32, 18)
        )

        self.lidar_dense = nn.Sequential(
            nn.Linear(32 * 18, 64),
            nn.ReLU()
        )

        # Fusion head: 64 (spatial features) + 5 (meta features) = 69
        self.fusion = nn.Sequential(
            nn.Linear(64 + 5, 64),
            nn.ReLU(),
            nn.Linear(64, 2)
        )

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        """
        Forward pass.
        Input x: shape (B, 149) or (149,)
        Output: shape (B, 2) [v_nom, w_nom]
        """
        if x.dim() == 1:
            x = x.unsqueeze(0)

        # Slice 2-channel LiDAR via zero-copy view and extract meta features
        lidar_2ch = x[:, :144].view(-1, 2, 72)
        meta = x[:, 144:149]

        conv_out = self.conv(lidar_2ch)
        flat_conv = conv_out.flatten(start_dim=1)
        lidar_feats = self.lidar_dense(flat_conv)

        fused_input = torch.cat([lidar_feats, meta], dim=-1)
        raw_out = self.fusion(fused_input)

        # Bounded actions
        v = torch.sigmoid(raw_out[..., 0]) * self.max_v
        w = torch.tanh(raw_out[..., 1]) * self.max_w
        return torch.stack([v, w], dim=-1)

    def predict_action(self, state_149: np.ndarray) -> Tuple[float, float]:
        """
        Convenience inference method for a single state vector.
        """
        self.eval()
        with torch.no_grad():
            inp = torch.from_numpy(state_149).unsqueeze(0)
            out = self.forward(inp).squeeze(0).cpu().numpy()
        return float(out[0]), float(out[1])

    def export_onnx(self, filepath: str):
        """
        Exports the trained policy to standard standalone ONNX for Jetson Nano TensorRT compilation.
        """
        self.eval()
        dummy_input = torch.randn(1, 149, dtype=torch.float32)
        torch.onnx.export(
            self,
            dummy_input,
            filepath,
            export_params=True,
            opset_version=18,
            do_constant_folding=True,
            input_names=["state_149"],
            output_names=["action_vw"],
            dynamo=False
        )


class NavPolicyMLP(nn.Module):
    """
    3-Layer Goal-Conditioned Navigation Policy MLP (Fallback & Comparative Benchmark).
    """

    def __init__(
        self,
        input_dim: int = 149,
        hidden_dim1: int = 128,
        hidden_dim2: int = 64,
        max_v: float = 0.12,
        max_w: float = 0.35
    ):
        super().__init__()
        self.max_v = max_v
        self.max_w = max_w
        self.input_dim = input_dim

        self.net = nn.Sequential(
            nn.Linear(input_dim, hidden_dim1),
            nn.LayerNorm(hidden_dim1),
            nn.ReLU(),
            nn.Linear(hidden_dim1, hidden_dim2),
            nn.ReLU(),
            nn.Linear(hidden_dim2, 2)
        )

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        raw_out = self.net(x)
        v = torch.sigmoid(raw_out[..., 0]) * self.max_v
        w = torch.tanh(raw_out[..., 1]) * self.max_w
        return torch.stack([v, w], dim=-1)

    def predict_action(self, state: np.ndarray) -> Tuple[float, float]:
        self.eval()
        with torch.no_grad():
            inp = torch.from_numpy(state).unsqueeze(0)
            out = self.forward(inp).squeeze(0).cpu().numpy()
        return float(out[0]), float(out[1])

    def export_onnx(self, filepath: str):
        self.eval()
        dummy_input = torch.randn(1, self.input_dim, dtype=torch.float32)
        torch.onnx.export(
            self,
            dummy_input,
            filepath,
            export_params=True,
            opset_version=18,
            do_constant_folding=True,
            input_names=[f"state_{self.input_dim}"],
            output_names=["action_vw"],
            dynamo=False
        )
