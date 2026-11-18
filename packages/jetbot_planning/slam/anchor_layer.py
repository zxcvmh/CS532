"""
AnchorLayer: Static Anchor Map layer for fixed environmental boundaries.
Locks wall cells and protects them from raycasting erosion.
CS532 Advanced AI Robotics — Module 05-d500lib-motion / slam
"""
import math
import numpy as np
from typing import List, Tuple, Optional
from .grid_core import GridCore


class AnchorLayer:
    """Manages pristine static reference walls (Anchor Map)."""

    def __init__(self, grid_core: GridCore):
        self.grid = grid_core
        self.anchor_mask = np.zeros((grid_core.height, grid_core.width), dtype=bool)
        self.anchor_locked: bool = False
        self.anchor_log_odds: Optional[np.ndarray] = None

    def reset(self):
        """Reset anchor mask and unlock layer."""
        self.anchor_mask.fill(False)
        self.anchor_locked = False
        self.anchor_log_odds = None

    def lock_anchor_map(self):
        """Permanently locks the static anchor map against dynamic erosion."""
        self.anchor_locked = True
        self.anchor_log_odds = self.grid.log_odds.copy()

    def is_anchor_locked(self) -> bool:
        """Returns True if the static anchor map has been locked."""
        return self.anchor_locked

    def set_anchor_from_scan(
        self,
        ranges: List[float],
        robot_x: float,
        robot_y: float,
        robot_theta: float,
        max_range: float = 6.0,
        min_hits: int = 15,
        lever_arm: float = 0.08,
    ) -> bool:
        """
        Initializes the Pristine Anchor Map from an initial reference 360-ray sweep.
        Marks wall hits as permanent static obstacles in anchor_mask and locks them.
        """
        if not ranges or len(ranges) < 180:
            return False

        lx = robot_x + lever_arm * math.cos(robot_theta)
        ly = robot_y + lever_arm * math.sin(robot_theta)
        r_col, r_row = self.grid.world_to_grid(lx, ly)
        if not self.grid.is_inside(r_col, r_row):
            return False

        hits_marked = 0
        step_deg = 360.0 / len(ranges)

        for i, dist in enumerate(ranges):
            if dist < 0.05 or dist > max_range:
                continue

            beam_rad = math.radians(i * step_deg) + robot_theta
            hit_x = lx + dist * math.cos(beam_rad)
            hit_y = ly + dist * math.sin(beam_rad)
            hc, hr = self.grid.world_to_grid(hit_x, hit_y)

            if not self.grid.is_inside(hc, hr):
                continue

            self.anchor_mask[hr, hc] = True
            self.grid.log_odds[hr, hc] = self.grid.l_max
            hits_marked += 1

            # Raycast free line from robot to hit point (conservative clearance)
            line = self.grid.bresenham_line(r_col, r_row, hc, hr)
            for fc, fr in line[:-1]:
                if self.grid.is_inside(fc, fr) and not self.anchor_mask[fr, fc]:
                    self.grid.log_odds[fr, fc] = self.grid.l_min

        if hits_marked >= min_hits:
            self.anchor_locked = True
            self.anchor_log_odds = self.grid.log_odds.copy()
            return True

        return False

    def protect_walls_from_free_mask(self, free_mask: np.ndarray) -> np.ndarray:
        """
        Ensures free_mask NEVER clears or erodes protected anchor wall cells.
        Returns the sanitized free_mask.
        """
        if self.anchor_locked:
            free_mask[self.anchor_mask.ravel()] = False
        return free_mask
