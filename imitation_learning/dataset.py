"""
Dataset & Observation Preprocessing for Imitation Learning Navigation
====================================================================
CS532 Autonomous JetBot — Module imitation_learning
Based on UIT ITCourses RL-101 (Bài 10: Học bắt chước - Imitation Learning)

Features:
  1. Standardized 41D Observation Extraction:
     - 36-beam uniform LiDAR sampling from 360 CCW scan
     - Goal distance & continuous (sin, cos) heading orientation
     - Current velocity normalization
  2. Bilateral Symmetry Augmentation:
     - Doubles dataset size by mirroring states & actions across robot centerline
     - Eliminates directional steering bias (left vs. right prejudice)
  3. Seamless Dataset Merging for DAgger Iterations
"""

import math
import numpy as np
from typing import List, Tuple, Dict, Any, Optional

try:
    import torch
    from torch.utils.data import Dataset
    HAS_TORCH = True
except ImportError:
    HAS_TORCH = False
    Dataset = object

from config import (
    LIDAR_NUM_BEAMS,
    LIDAR_MAX_RANGE_M,
    LIDAR_MIN_RANGE_M,
    GOAL_MAX_DIST_M,
    MAX_LINEAR_SPEED,
    MAX_ANGULAR_SPEED,
    OBSERVATION_DIM,
    OBSERVATION_DIM_V2,
    MOTION_NORM_M,
    ACTION_DIM
)

_DEGS_360 = np.arange(360, dtype=np.float64)
_RADS_360 = np.deg2rad(_DEGS_360)
_COS_360 = np.cos(_RADS_360)
_SIN_360 = np.sin(_RADS_360)


def extract_observation_vector(
    raw_ranges_360: List[float],
    goal_x: float,
    goal_y: float,
    robot_x: float,
    robot_y: float,
    robot_theta: float,
    curr_v: float = 0.0,
    curr_w: float = 0.0,
    max_range: float = LIDAR_MAX_RANGE_M,
    max_goal_dist: float = GOAL_MAX_DIST_M
) -> np.ndarray:
    """
    Extracts the normalized 41-dimensional observation vector s from sensor state.
    """
    obs = np.zeros(OBSERVATION_DIM, dtype=np.float32)

    # 1. Sample 36 LiDAR beams with Sector Min-Pooling (eliminates obstacle blind spots)
    step = 360 // LIDAR_NUM_BEAMS  # 10 degrees per beam
    half_step = step // 2
    for i in range(LIDAR_NUM_BEAMS):
        deg_center = i * step
        # Sector window: [deg_center - 5, deg_center + 4] CCW
        sector_vals = []
        for d in range(-half_step, half_step):
            idx = (deg_center + d) % 360
            if idx < len(raw_ranges_360):
                r_val = raw_ranges_360[idx]
                if LIDAR_MIN_RANGE_M <= r_val <= 8.0 and not (math.isnan(r_val) or math.isinf(r_val)):
                    sector_vals.append(r_val)

        r = min(sector_vals) if sector_vals else max_range
        # Normalize to [0.0, 1.0]
        obs[i] = np.clip(r / max_range, 0.0, 1.0)


    # 2. Goal Vector in Robot Frame
    dx = goal_x - robot_x
    dy = goal_y - robot_y
    dist_goal = math.hypot(dx, dy)
    angle_world = math.atan2(dy, dx)
    # Angle relative to robot heading in (-pi, pi]
    alpha_goal = (angle_world - robot_theta + math.pi) % (2.0 * math.pi) - math.pi

    obs[36] = np.clip(dist_goal / max_goal_dist, 0.0, 1.0)
    obs[37] = math.sin(alpha_goal)
    obs[38] = math.cos(alpha_goal)

    # 3. Current velocity normalization (Neutralized to 0.0 to prevent Inertial Copycat Shortcut)
    # Forces network to decide steering purely based on LiDAR obstacle geometry and Goal vector!
    obs[39] = 0.0
    obs[40] = 0.0

    return obs


def mirror_sample(obs: np.ndarray, act: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """
    Applies bilateral symmetry mirroring to state and action.
    Mirrors left <-> right.
    """
    mirrored_obs = np.copy(obs)
    mirrored_act = np.copy(act)

    # 1. Mirror LiDAR beams:
    # Index 0 is straight ahead (0 deg).
    # Beams 1..35 correspond to 10..350 deg.
    # Mirrored angle = (360 - angle) % 360.
    # Index 1 (10 deg left) swaps with Index 35 (350 deg right), etc.
    for i in range(1, LIDAR_NUM_BEAMS // 2 + 1):
        j = (LIDAR_NUM_BEAMS - i) % LIDAR_NUM_BEAMS
        mirrored_obs[i], mirrored_obs[j] = obs[j], obs[i]

    # 2. Mirror Goal direction:
    # alpha_flipped = -alpha
    # sin(-alpha) = -sin(alpha), cos(-alpha) = cos(alpha)
    mirrored_obs[37] = -obs[37]
    mirrored_obs[38] = obs[38]

    # 3. Mirror current angular speed:
    mirrored_obs[40] = -obs[40]

    # 4. Mirror action:
    # v remains the same, w is negated
    mirrored_act[0] = act[0]
    mirrored_act[1] = -act[1]

    return mirrored_obs, mirrored_act


def extract_observation_vector_v2(
    raw_ranges_360: List[float],
    goal_x: float,
    goal_y: float,
    robot_x: float,
    robot_y: float,
    robot_theta: float,
    prev_ranges_360: Optional[List[float]] = None,
    ego_dpose: Optional[Tuple[float, float, float]] = None,
    person_mask_360: Optional[Any] = None,
    curr_v: float = 0.0,
    curr_w: float = 0.0,
    max_range: float = LIDAR_MAX_RANGE_M,
    max_goal_dist: float = GOAL_MAX_DIST_M
) -> np.ndarray:
    """
    Extracts the 113-D observation v2:
      [0:41]   legacy 41-D observation (unchanged)
      [41:77]  per-beam person flag (0.0/1.0, OR-pooled over 10-deg sector [10b-5, 10b+4])
      [77:113] per-beam ego-compensated motion in [-1, 1] (median delta / MOTION_NORM_M)
    """
    obs = np.zeros(OBSERVATION_DIM_V2, dtype=np.float32)
    obs[:OBSERVATION_DIM] = extract_observation_vector(
        raw_ranges_360, goal_x, goal_y, robot_x, robot_y, robot_theta,
        curr_v=curr_v, curr_w=curr_w, max_range=max_range, max_goal_dist=max_goal_dist
    )

    step = 360 // LIDAR_NUM_BEAMS
    half_step = step // 2

    if person_mask_360 is not None:
        mask = np.asarray(person_mask_360, dtype=bool)
        if mask.size == 360:
            sectors = np.roll(mask, half_step).reshape(LIDAR_NUM_BEAMS, step)
            obs[OBSERVATION_DIM:OBSERVATION_DIM + LIDAR_NUM_BEAMS] = sectors.any(axis=1).astype(np.float32)

    if prev_ranges_360 is not None and ego_dpose is not None:
        r_prev = np.asarray(prev_ranges_360, dtype=np.float64)
        r_cur = np.asarray(raw_ranges_360, dtype=np.float64)
        if r_prev.size == 360 and r_cur.size == 360:
            dx, dy, dth = float(ego_dpose[0]), float(ego_dpose[1]), float(ego_dpose[2])
            c, s = math.cos(dth), math.sin(dth)
            vx = r_prev * _COS_360 - dx
            vy = r_prev * _SIN_360 - dy
            p1_x = c * vx + s * vy
            p1_y = -s * vx + c * vy
            r_comp = np.hypot(p1_x, p1_y)
            ang_deg = np.rad2deg(np.arctan2(p1_y, p1_x)) % 360.0

            order = np.argsort(ang_deg)
            ang_s = ang_deg[order]
            r_s = r_comp[order]
            ang_ext = np.concatenate(([ang_s[-1] - 360.0], ang_s, [ang_s[0] + 360.0]))
            r_ext = np.concatenate(([r_s[-1]], r_s, [r_s[0]]))
            prev_in_cur = np.interp(_DEGS_360, ang_ext, r_ext)

            diff = prev_in_cur - r_cur
            diff_sectors = np.roll(diff, half_step).reshape(LIDAR_NUM_BEAMS, step)
            med_delta = np.median(diff_sectors, axis=1)
            motion = np.clip(med_delta / MOTION_NORM_M, -1.0, 1.0)
            motion = np.where(np.isfinite(motion), motion, 0.0)
            obs[OBSERVATION_DIM + LIDAR_NUM_BEAMS:] = motion.astype(np.float32)

    return obs


def mirror_sample_v2(obs: np.ndarray, act: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """Mirrors a 113-D observation v2 and action across the robot centerline."""
    mirrored_obs = np.copy(obs)
    mirrored_obs[:OBSERVATION_DIM], mirrored_act = mirror_sample(obs[:OBSERVATION_DIM], act)
    for off in (OBSERVATION_DIM, OBSERVATION_DIM + LIDAR_NUM_BEAMS):
        for i in range(1, LIDAR_NUM_BEAMS // 2 + 1):
            j = (LIDAR_NUM_BEAMS - i) % LIDAR_NUM_BEAMS
            mirrored_obs[off + i], mirrored_obs[off + j] = obs[off + j], obs[off + i]
    return mirrored_obs, mirrored_act


class ImitationLearningDataset:
    """Container and PyTorch Dataset for Imitation Learning demonstrations."""
    def __init__(
        self,
        observations: Optional[np.ndarray] = None,
        actions: Optional[np.ndarray] = None,
        augment_symmetry: bool = True
    ):
        self.observations: np.ndarray = np.empty((0, OBSERVATION_DIM), dtype=np.float32)
        self.actions: np.ndarray = np.empty((0, ACTION_DIM), dtype=np.float32)

        if observations is not None and actions is not None:
            self.add_samples(observations, actions, augment_symmetry=augment_symmetry)

    def add_samples(
        self,
        new_obs: np.ndarray,
        new_act: np.ndarray,
        augment_symmetry: bool = True
    ):
        """Appends new samples, optionally adding mirrored counterparts."""
        new_obs = np.asarray(new_obs, dtype=np.float32)
        new_act = np.asarray(new_act, dtype=np.float32)

        if augment_symmetry:
            aug_obs = []
            aug_act = []
            for o, a in zip(new_obs, new_act):
                mo, ma = mirror_sample(o, a)
                aug_obs.append(mo)
                aug_act.append(ma)
            new_obs = np.vstack([new_obs, np.array(aug_obs, dtype=np.float32)])
            new_act = np.vstack([new_act, np.array(aug_act, dtype=np.float32)])

        if len(self.observations) == 0:
            self.observations = new_obs
            self.actions = new_act
        else:
            self.observations = np.vstack([self.observations, new_obs])
            self.actions = np.vstack([self.actions, new_act])

    def __len__(self) -> int:
        return len(self.observations)

    def __getitem__(self, idx: int) -> Tuple[Any, Any]:
        obs = self.observations[idx]
        act = self.actions[idx]
        if HAS_TORCH:
            return torch.from_numpy(obs), torch.from_numpy(act)
        return obs, act

    def save_npz(self, filepath: str):
        """Persists dataset to compressed numpy file."""
        np.savez_compressed(filepath, observations=self.observations, actions=self.actions)

    @classmethod
    def load_npz(cls, filepath: str, augment_symmetry: bool = False) -> "ImitationLearningDataset":
        """Loads dataset from compressed numpy file."""
        data = np.load(filepath)
        return cls(data["observations"], data["actions"], augment_symmetry=augment_symmetry)
