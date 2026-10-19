"""
Google Cartographer 2D Probability Grid
=======================================
Reference: Wolfgang Hess, Damon Kohler, Holger Rapp, Daniel Andor (Google, 2016).
"Real-Time Loop Closure in 2D LIDAR SLAM", IEEE ICRA 2016.
Implementation follows Cartographer's exact odds representation and update rules:
- Probability bounded in [0.1, 0.9]
- odds(p) = p / (1 - p)
- Log-odds / Multiplicative odds update with p_hit = 0.55, p_miss = 0.49
"""
import math
import numpy as np
from typing import Tuple, List, Optional

P_MIN = 0.10
P_MAX = 0.90
P_HIT_DEFAULT = 0.55
P_MISS_DEFAULT = 0.49


def probability_to_odds(p: float) -> float:
    """Convert probability in (0, 1) to odds."""
    p_clamped = max(P_MIN, min(P_MAX, p))
    return p_clamped / (1.0 - p_clamped)


def odds_to_probability(odds: float) -> float:
    """Convert odds to probability in [P_MIN, P_MAX]."""
    p = odds / (1.0 + odds)
    return max(P_MIN, min(P_MAX, p))


class ProbabilityGrid2D:
    """
    2D Probability Grid representing spatial occupancy probabilities.
    Direct implementation of Cartographer's probability grid structure.
    """

    def __init__(
        self,
        resolution: float = 0.05,
        width: int = 160,
        height: int = 160,
        origin_x: float = -4.0,
        origin_y: float = -4.0,
        hit_prob: float = P_HIT_DEFAULT,
        miss_prob: float = P_MISS_DEFAULT,
    ):
        self.resolution = float(resolution)
        self.inv_resolution = 1.0 / self.resolution
        self.width = int(width)
        self.height = int(height)
        self.origin_x = float(origin_x)
        self.origin_y = float(origin_y)

        self.hit_odds = probability_to_odds(hit_prob)
        self.miss_odds = probability_to_odds(miss_prob)

        # 0.5 represents unknown probability (odds = 1.0)
        # We store odds directly for fast multiplicative updates
        self.odds = np.ones((self.height, self.width), dtype=np.float32)
        # Track which cells have been observed at least once
        self.observed = np.zeros((self.height, self.width), dtype=bool)

    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        """Convert world coordinates (meters) to grid indices (col, row)."""
        col = int(math.floor((x - self.origin_x) * self.inv_resolution))
        row = int(math.floor((y - self.origin_y) * self.inv_resolution))
        return col, row

    def grid_to_world(self, col: int, row: int) -> Tuple[float, float]:
        """Convert grid indices to center world coordinates."""
        x = self.origin_x + (col + 0.5) * self.resolution
        y = self.origin_y + (row + 0.5) * self.resolution
        return x, y

    def is_inside(self, col: int, row: int) -> bool:
        """Check whether grid coordinate lies within map boundaries."""
        return 0 <= col < self.width and 0 <= row < self.height

    def bresenham_line(self, x0: int, y0: int, x1: int, y1: int) -> List[Tuple[int, int]]:
        """Bresenham 2D line generation for fast ray casting."""
        points = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy

        curr_x, curr_y = x0, y0
        while True:
            points.append((curr_x, curr_y))
            if curr_x == x1 and curr_y == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                curr_x += sx
            if e2 < dx:
                err += dx
                curr_y += sy
        return points

    def apply_odds(self, col: int, row: int, update_odds: float):
        """Update a cell's odds multiplicatively, clamped to [P_MIN, P_MAX]."""
        if not self.is_inside(col, row):
            return
        curr_odds = self.odds[row, col]
        new_odds = curr_odds * update_odds
        # Clamp bounds
        min_odds = probability_to_odds(P_MIN)
        max_odds = probability_to_odds(P_MAX)
        self.odds[row, col] = max(min_odds, min(max_odds, new_odds))
        self.observed[row, col] = True

    def insert_scan(
        self,
        robot_x: float,
        robot_y: float,
        hit_points_world: np.ndarray,
        miss_max_range: float = 3.5
    ):
        """
        Insert range data into probability grid.
        Follows Cartographer's CastRays:
        - Free space rays updated with miss_odds
        - Hit points updated with hit_odds
        """
        origin_col, origin_row = self.world_to_grid(robot_x, robot_y)
        if not self.is_inside(origin_col, origin_row):
            return

        for pt in hit_points_world:
            hx, hy = float(pt[0]), float(pt[1])
            dist = math.hypot(hx - robot_x, hy - robot_y)
            if dist < 0.05:
                continue

            hit_col, hit_row = self.world_to_grid(hx, hy)
            line = self.bresenham_line(origin_col, origin_row, hit_col, hit_row)

            # Raycast free cells along line (excluding the final hit cell)
            for fc, fr in line[:-1]:
                if self.is_inside(fc, fr):
                    self.apply_odds(fc, fr, self.miss_odds)

            # Mark hit cell
            if self.is_inside(hit_col, hit_row):
                self.apply_odds(hit_col, hit_row, self.hit_odds)

    def get_probability(self, x: float, y: float) -> float:
        """Query continuous probability via bilinear interpolation."""
        col_f = (x - self.origin_x) * self.inv_resolution - 0.5
        row_f = (y - self.origin_y) * self.inv_resolution - 0.5

        c0 = int(math.floor(col_f))
        r0 = int(math.floor(row_f))
        c1 = c0 + 1
        r1 = r0 + 1

        if not (self.is_inside(c0, r0) and self.is_inside(c1, r1)):
            c_clamped = max(0, min(self.width - 1, int(round(col_f))))
            r_clamped = max(0, min(self.height - 1, int(round(row_f))))
            return odds_to_probability(float(self.odds[r_clamped, c_clamped]))

        fc = col_f - c0
        fr = row_f - r0

        p00 = odds_to_probability(float(self.odds[r0, c0]))
        p01 = odds_to_probability(float(self.odds[r0, c1]))
        p10 = odds_to_probability(float(self.odds[r1, c0]))
        p11 = odds_to_probability(float(self.odds[r1, c1]))

        val = (
            (1.0 - fc) * (1.0 - fr) * p00
            + fc * (1.0 - fr) * p01
            + (1.0 - fc) * fr * p10
            + fc * fr * p11
        )
        return float(val)

    def to_occupancy_grid(self) -> np.ndarray:
        """
        Convert to standard ROS nav_msgs/OccupancyGrid data format:
        -1 for unknown, [0..100] for probability percentage.
        """
        grid = np.full((self.height, self.width), -1, dtype=np.int8)
        mask = self.observed
        p = self.odds[mask] / (1.0 + self.odds[mask])
        grid[mask] = np.clip(np.round((p - P_MIN) / (P_MAX - P_MIN) * 100.0), 0, 100).astype(np.int8)
        return grid
