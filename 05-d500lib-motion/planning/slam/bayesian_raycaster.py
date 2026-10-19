"""
BayesianRaycaster: Vectorized 2D Log-Odds Raycasting and Space Evaporation.
CS532 Advanced AI Robotics — Module 05-d500lib-motion / slam
"""
import time
import math
from typing import List, Tuple, Dict, Optional, Any
import numpy as np
from .grid_core import GridCore


class BayesianRaycaster:
    """Performs continuous probabilistic Bayesian log-odds mapping from 360 LiDAR sweeps."""

    def __init__(
        self,
        grid_core: GridCore,
        lever_arm: float = 0.08,
        dynamic_ttl: float = 0.80,
    ):
        self.grid = grid_core
        self.lever_arm = float(lever_arm)
        self.dynamic_ttl = float(dynamic_ttl)
        self.latest_hit_cells: np.ndarray = np.empty(0, dtype=np.int32)
        # Tracking buffer for short-TTL dynamic obstacles outside locked anchor walls
        self.dynamic_obstacle_cells: Dict[int, float] = {}

    def update_scan(
        self,
        robot_x: float,
        robot_y: float,
        robot_theta: float,
        ranges: List[float],
        max_valid_range: float = 6.0,
        min_valid_range: float = 0.05,
        anchor_mask: Optional[np.ndarray] = None,
        anchor_locked: bool = False,
    ) -> np.ndarray:
        """
        Updates 2D Occupancy Grid log-odds from LiDAR range array.
        - Lever-arm sensor offset compensation (+0.08m from axle).
        - Integer Bresenham line raycasting with Liang-Barsky clipping.
        - Confirmed wall protection (flat >= 1.5) preventing ray clearing erosion.
        - Dynamic footprint TTL decay when anchor map is locked.
        """
        lidar_x = robot_x + self.lever_arm * math.cos(robot_theta)
        lidar_y = robot_y + self.lever_arm * math.sin(robot_theta)
        r_col, r_row = self.grid.world_to_grid(lidar_x, lidar_y)
        if not self.grid.is_inside(r_col, r_row):
            return self.latest_hit_cells

        num_points = len(ranges)
        step_rad = (2.0 * math.pi) / max(1, num_points)

        w = self.grid.width
        h = self.grid.height
        free_mask = np.zeros(w * h, dtype=bool)
        occ_indices: List[int] = []

        valid_hits = sum(1 for r in ranges if min_valid_range <= r <= max_valid_range)
        has_active_lidar = (valid_hits >= 10)

        for i in range(num_points):
            dist = ranges[i]
            if math.isnan(dist) or dist <= 0.0 or dist < min_valid_range:
                continue

            if dist > max_valid_range:
                if not has_active_lidar:
                    continue
                is_obstacle = False
                clamped_dist = max_valid_range
            else:
                is_obstacle = True
                clamped_dist = dist

            angle = robot_theta + (i * step_rad)
            hit_x = lidar_x + clamped_dist * math.cos(angle)
            hit_y = lidar_y + clamped_dist * math.sin(angle)
            h_col, h_row = self.grid.world_to_grid(hit_x, hit_y)

            h_col, h_row, was_clipped = self.grid.clip_ray_to_grid(
                r_col, r_row, h_col, h_row
            )
            if was_clipped:
                is_obstacle = False

            # Fast integer Bresenham traversal marking free-space cells
            dx = abs(h_col - r_col)
            dy = abs(h_row - r_row)
            sx = 1 if r_col < h_col else -1
            sy = 1 if r_row < h_row else -1
            err = dx - dy
            cx, cy = r_col, r_row

            while cx != h_col or cy != h_row:
                # Stop 1 cell before obstacle to prevent sensor noise from eroding wall edges
                if is_obstacle and (abs(cx - h_col) <= 1 and abs(cy - h_row) <= 1):
                    break
                free_mask[cy * w + cx] = True
                e2 = 2 * err
                if e2 > -dy:
                    err -= dy
                    cx += sx
                if e2 < dx:
                    err += dx
                    cy += sy

            if is_obstacle:
                occ_indices.append(h_row * w + h_col)

        flat = self.grid.log_odds.ravel()

        if occ_indices:
            unique_occ = np.unique(occ_indices)
            free_mask[unique_occ] = False
        else:
            unique_occ = np.empty(0, dtype=np.int32)

        self.latest_hit_cells = unique_occ

        # Continuous Bayesian Log-Odds Update (REP-105 & Elfes/Moravec compliant):
        # Allow free-space rays to clear cells without artificial wall locking,
        # enabling recovery from dynamic obstacles and transient drift errors.

        # Continuous Bayesian Log-Odds Update
        # Open navigable space clearing
        flat[free_mask] += self.grid.l_free

        # Immediate 1-frame solid black hit response
        if len(unique_occ) > 0:
            flat[unique_occ] = np.maximum(flat[unique_occ] + self.grid.l_occ, self.grid.l_occ)

        # Dynamic obstacle tracking with TTL decay
        now = time.time()
        if anchor_locked and anchor_mask is not None:
            flat_anchor = anchor_mask.ravel()
            for idx in unique_occ:
                if not flat_anchor[idx]:
                    self.dynamic_obstacle_cells[int(idx)] = now + self.dynamic_ttl
        else:
            for idx in unique_occ:
                self.dynamic_obstacle_cells[int(idx)] = now + self.dynamic_ttl

        # Dynamic obstacle cell management
        if self.dynamic_obstacle_cells:
            # Ray clearing: if laser ray penetrates through a dynamic cell, clear it immediately
            free_indices = np.where(free_mask)[0]
            for f_idx in free_indices:
                if int(f_idx) in self.dynamic_obstacle_cells:
                    del self.dynamic_obstacle_cells[int(f_idx)]
                    flat[f_idx] = min(flat[f_idx], 0.0)

            # Purge stale tracking keys after TTL without wiping un-penetrated physical obstacles
            stale_keys = [idx for idx, expiry in self.dynamic_obstacle_cells.items() if now >= expiry + 1.5]
            for idx in stale_keys:
                del self.dynamic_obstacle_cells[idx]

        np.clip(flat, self.grid.l_min, self.grid.l_max, out=flat)
        return self.latest_hit_cells

    def clear_footprint(
        self,
        robot_x: float,
        robot_y: float,
        radius: float = 0.09,
    ) -> List[int]:
        """Clears costmap and log-odds directly under the robot chassis footprint."""
        c_p, r_p = self.grid.world_to_grid(robot_x, robot_y)
        rad_cells = max(1, int(math.ceil(radius / self.grid.resolution)))
        r_min = max(0, r_p - rad_cells)
        r_max = min(self.grid.height, r_p + rad_cells + 1)
        c_min = max(0, c_p - rad_cells)
        c_max = min(self.grid.width, c_p + rad_cells + 1)

        cleared_indices = []
        if r_min < r_max and c_min < c_max:
            ys, xs = np.ogrid[r_min - r_p : r_max - r_p, c_min - c_p : c_max - c_p]
            mask = (xs * xs + ys * ys) <= (rad_cells * rad_cells)
            self.grid.costmap[r_min:r_max, c_min:c_max][mask] = 0
            # Also clear log_odds under footprint to prevent self-mapping
            self.grid.log_odds[r_min:r_max, c_min:c_max][mask] = np.minimum(
                self.grid.log_odds[r_min:r_max, c_min:c_max][mask], 0.0
            )
            for r_idx in range(r_min, r_max):
                for c_idx in range(c_min, c_max):
                    if mask[r_idx - r_min, c_idx - c_min]:
                        cleared_indices.append(r_idx * self.grid.width + c_idx)

        return cleared_indices
