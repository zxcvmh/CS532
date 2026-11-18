"""
CostmapEDT: 2D Euclidean Distance Field (EDT), Inflation, and Footprint Clearance.
Generates safety costmaps for A* and trajectory evaluation.
"""
import math
import numpy as np
from typing import Tuple, List, Optional, Any
from .grid_core import GridCore


class CostmapEDT:
    """Manages distance field computation, obstacle inflation, and chassis footprint clearing."""

    def __init__(
        self,
        grid_core: GridCore,
        robot_radius: float = 0.09,
        inflation_radius: float = 0.35,
        lethal_radius: float = 0.12,
    ):
        self.grid_core = grid_core
        self.robot_radius = float(robot_radius)
        self.inflation_radius = float(inflation_radius)
        self.lethal_radius = float(lethal_radius)

        # 2D arrays:
        # costmap: uint8 array [0: free, 1..253: inflated, 254: lethal obstacle, 255: unknown]
        self.costmap = np.zeros((grid_core.height, grid_core.width), dtype=np.uint8)
        # dist_m: float32 continuous metric distance to nearest obstacle
        self.dist_m = np.full((grid_core.height, grid_core.width), 10.0, dtype=np.float32)

    def reset(self):
        """Reset costmap and distance field."""
        self.costmap.fill(0)
        self.dist_m.fill(10.0)

    def update_costmap(self, occ_mask: Optional[np.ndarray] = None) -> np.ndarray:
        """
        Computes 2D Euclidean Distance Transform (EDT) and inflation costmap.
        Uses scipy.ndimage.distance_transform_edt if available, otherwise fast bounded kernel.
        """
        if occ_mask is None:
            occ_mask = self.grid_core.grid >= 50

        h, w = self.grid_core.height, self.grid_core.width
        res = self.grid_core.resolution

        if not np.any(occ_mask):
            self.dist_m.fill(10.0)
            self.costmap.fill(0)
            return self.costmap

        try:
            from scipy.ndimage import distance_transform_edt
            dist_cells = distance_transform_edt(~occ_mask)
            self.dist_m = (dist_cells * res).astype(np.float32)
        except ImportError:
            # Vectorized fallback distance approximation
            occ_coords = np.argwhere(occ_mask)
            if len(occ_coords) > 2000:
                step = len(occ_coords) // 2000 + 1
                occ_coords = occ_coords[::step]
            from scipy.spatial import cKDTree
            tree = cKDTree(occ_coords)
            all_r, all_c = np.indices((h, w))
            pts = np.column_stack([all_r.ravel(), all_c.ravel()])
            dists, _ = tree.query(pts)
            self.dist_m = (dists.reshape((h, w)) * res).astype(np.float32)

        # Construct Inflation Costmap:
        # Distance <= lethal_radius -> 254 (lethal collision)
        # lethal_radius < Distance <= inflation_radius -> decaying cost [1..253]
        # Distance > inflation_radius -> 0 (free navigable)
        self.costmap.fill(0)
        lethal_mask = self.dist_m <= self.lethal_radius
        self.costmap[lethal_mask] = 254

        infl_mask = (self.dist_m > self.lethal_radius) & (self.dist_m <= self.inflation_radius)
        if np.any(infl_mask):
            d_vals = self.dist_m[infl_mask]
            # Exponential decay formula per ROS Nav2 standard: cost = 253 * exp(-k * (d - lethal))
            alpha = (d_vals - self.lethal_radius) / max(0.01, self.inflation_radius - self.lethal_radius)
            decay = np.exp(-3.0 * alpha)
            costs = (1.0 + 252.0 * decay).astype(np.uint8)
            self.costmap[infl_mask] = costs

        return self.costmap

    def clear_footprint(
        self,
        robot_x: float,
        robot_y: float,
        radius: Optional[float] = None,
        clear_lethal_only: bool = False,
    ):
        """
        Clears costmap cells under the robot chassis footprint.
        Prevents self-collision and allows A* planner to initiate path from robot center.
        """
        if radius is None:
            radius = self.robot_radius

        rc, rr = self.grid_core.world_to_grid(robot_x, robot_y)
        cell_r = max(1, int(math.ceil(radius / self.grid_core.resolution)))

        c_min = max(0, rc - cell_r)
        c_max = min(self.grid_core.width - 1, rc + cell_r)
        r_min = max(0, rr - cell_r)
        r_max = min(self.grid_core.height - 1, rr + cell_r)

        r_sq = (radius / self.grid_core.resolution) ** 2
        for r in range(r_min, r_max + 1):
            for c in range(c_min, c_max + 1):
                if (c - rc) ** 2 + (r - rr) ** 2 <= r_sq:
                    if clear_lethal_only:
                        if self.costmap[r, c] >= 254:
                            self.costmap[r, c] = 0
                    else:
                        self.costmap[r, c] = 0
