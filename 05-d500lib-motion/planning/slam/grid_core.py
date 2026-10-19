"""
GridCore: Fundamental 2D Occupancy Grid geometry, coordinate conversions, and ray algorithms.
CS532 Advanced AI Robotics — Module 05-d500lib-motion / slam
"""
import math
from typing import Tuple, List, Optional
import numpy as np


class GridCore:
    """Manages 2D grid allocation, world-to-grid mappings, and line raycasting."""

    def __init__(
        self,
        width: int = 200,
        height: int = 200,
        resolution: float = 0.05,
        origin_x: float = -5.0,
        origin_y: float = -5.0,
        l_occ: float = 1.38,
        l_free: float = -0.50,
        l_min: float = -5.0,
        l_max: float = 5.0,
        arena_bounds: Optional[Tuple[float, float, float, float]] = None,
        robot_radius: float = 0.09,
        inflation_radius: float = 0.045,
    ):
        self.width = int(width)
        self.height = int(height)
        self.resolution = float(resolution)
        self.origin_x = float(origin_x)
        self.origin_y = float(origin_y)
        self.arena_bounds = arena_bounds
        self.robot_radius = float(robot_radius)
        self.inflation_radius = float(inflation_radius)

        # Log-odds Bayesian update parameters (Thrun et al., 2005)
        # L_FREE = -0.50 (clean ray clearing), L_OCC = +1.38 (instant 1-frame solid black response)
        self.l_occ = float(l_occ)
        self.l_free = float(l_free)
        self.l_min = float(l_min)
        self.l_max = float(l_max)

        # Internal arrays:
        # log_odds: float32 representation of spatial belief
        self.log_odds = np.zeros((self.height, self.width), dtype=np.float32)
        # costmap: uint8 array [0: free, 1..253: inflated clearance, 254: lethal obstacle]
        self.costmap = np.zeros((self.height, self.width), dtype=np.uint8)
        # dist_m: continuous metric distance to nearest obstacle from EDT
        self.dist_m: Optional[np.ndarray] = None

    def reset(self):
        """Reset grid cells and log-odds to initial unobserved state."""
        self.log_odds.fill(0.0)
        self.costmap.fill(0)
        self.dist_m = None

    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        """Convert continuous metric world coordinates (x, y) to discrete grid indices (col, row)."""
        col = int(math.floor((float(x) - self.origin_x) / self.resolution))
        row = int(math.floor((float(y) - self.origin_y) / self.resolution))
        return col, row

    def world_to_grid_array(
        self, wx: np.ndarray, wy: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Vectorized world coordinates to grid indices conversion."""
        col = np.floor((wx - self.origin_x) / self.resolution).astype(np.int32)
        row = np.floor((wy - self.origin_y) / self.resolution).astype(np.int32)
        return col, row

    def grid_to_world(self, col: int, row: int) -> Tuple[float, float]:
        """Convert grid indices (col, row) to cell center world coordinates (x, y)."""
        x = self.origin_x + (float(col) + 0.5) * self.resolution
        y = self.origin_y + (float(row) + 0.5) * self.resolution
        return round(x, 4), round(y, 4)

    def is_inside(self, col: int, row: int) -> bool:
        """Check whether discrete grid indices (col, row) are within grid boundaries."""
        return 0 <= col < self.width and 0 <= row < self.height

    @staticmethod
    def bresenham_line(c0: int, r0: int, c1: int, r1: int) -> List[Tuple[int, int]]:
        """Standard integer Bresenham 2D line algorithm."""
        points = []
        dc = abs(c1 - c0)
        dr = abs(r1 - r0)
        sc = 1 if c0 < c1 else -1
        sr = 1 if r0 < r1 else -1
        err = dc - dr

        c, r = c0, r0
        while True:
            points.append((c, r))
            if c == c1 and r == r1:
                break
            e2 = 2 * err
            if e2 > -dr:
                err -= dr
                c += sc
            if e2 < dc:
                err += dc
                r += sr
        return points

    def clip_ray_to_grid(
        self, x0: int, y0: int, x1: int, y1: int
    ) -> Tuple[int, int, bool]:
        """
        Liang-Barsky parametric 2D line clipping algorithm.
        Clips (x1, y1) to the grid rectangle [0, width - 1] x [0, height - 1]
        assuming start point (x0, y0) is inside the grid.
        Returns: (clipped_x1, clipped_y1, was_clipped)
        """
        if 0 <= x1 < self.width and 0 <= y1 < self.height:
            return x1, y1, False

        dx = x1 - x0
        dy = y1 - y0
        t_max = 1.0

        p = [-dx, dx, -dy, dy]
        q = [x0, (self.width - 1) - x0, y0, (self.height - 1) - y0]

        for i in range(4):
            pi = p[i]
            qi = q[i]
            if pi > 0:
                t = qi / float(pi)
                if t < t_max:
                    t_max = max(0.0, t)

        cx1 = int(round(x0 + t_max * dx))
        cy1 = int(round(y0 + t_max * dy))

        cx1 = max(0, min(self.width - 1, cx1))
        cy1 = max(0, min(self.height - 1, cy1))

        return cx1, cy1, True
