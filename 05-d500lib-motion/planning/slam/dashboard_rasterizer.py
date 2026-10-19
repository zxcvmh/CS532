"""
DashboardRasterizer: High-Contrast Visual Rasterization and Zero-Overhead Serialization.
Restores the solid, thick, crisp visual aesthetic of the proven baseline map (anh_lidar.png).
CS532 Advanced AI Robotics — Module 05-d500lib-motion / slam
"""
import base64
import math
from typing import List, Dict, Any, Tuple, Optional
import numpy as np
from .grid_core import GridCore


class DashboardRasterizer:
    """Serializes 2D Occupancy Grid into high-contrast dashboard frames."""

    def __init__(self, grid_core: GridCore):
        self.grid = grid_core

    def rasterize_frame(
        self,
        active_clusters: Optional[List[Any]] = None,
        active_cluster_polys: Optional[List[Any]] = None,
        latest_hits: Optional[np.ndarray] = None,
        dynamic_cells: Optional[Dict[int, float]] = None,
        robot_x: float = 0.0,
        robot_y: float = 0.0,
        robot_theta: float = 0.0,
        **kwargs
    ) -> np.ndarray:
        """
        Renders the high-contrast occupancy grid frame:
        - Unobserved space: -1
        - Free navigable space: 0
        - Solid occupied obstacles / walls: 100
        """
        data = np.zeros((self.grid.height, self.grid.width), dtype=np.int8)

        # 1. Unobserved space (near zero log-odds)
        data[np.abs(self.grid.log_odds) < 0.25] = -1

        # 2. Confirmed physical obstacles from raycasting
        data[self.grid.log_odds > 0.62] = 100

        # 3. Stamp active rolling dynamic obstacle cells (persistent within TTL)
        if dynamic_cells:
            import time
            now_c = time.time()
            active_dyn = [idx for idx, expiry in dynamic_cells.items() if now_c <= expiry]
            if active_dyn:
                data.ravel()[active_dyn] = 100

        # 4. Stamp active obstacle cluster cores as solid 100
        if active_clusters:
            res = self.grid.resolution
            h, w = self.grid.height, self.grid.width
            polys = active_cluster_polys or [None] * len(active_clusters)
            if len(polys) < len(active_clusters):
                polys = polys + [None] * (len(active_clusters) - len(polys))

            for cl, poly in zip(active_clusters, polys):
                if isinstance(cl, (list, tuple)) and len(cl) >= 3:
                    wx, wy, obs_rad = float(cl[0]), float(cl[1]), float(cl[2])
                else:
                    cent = getattr(cl, "centroid", (0.0, 0.0))
                    wx, wy = float(cent[0]), float(cent[1])
                    eff_r = getattr(cl, "effective_radius", None)
                    obs_rad = float(eff_r if eff_r is not None else getattr(cl, "radius_m", 0.10))

                c_p, r_p = self.grid.world_to_grid(wx, wy)
                rad_cells = max(1, int(math.ceil(obs_rad / res)))
                r_min = max(0, r_p - rad_cells)
                r_max = min(h, r_p + rad_cells + 1)
                c_min = max(0, c_p - rad_cells)
                c_max = min(w, c_p + rad_cells + 1)

                if r_min < r_max and c_min < c_max:
                    ys, xs = np.ogrid[r_min - r_p : r_max - r_p, c_min - c_p : c_max - c_p]
                    disc_mask = (xs * xs + ys * ys) * (res * res) <= (obs_rad * obs_rad)
                    data[r_min:r_max, c_min:c_max][disc_mask] = 100

        # 5. Direct 1-frame LiDAR hit rasterization with 3x3 footprint dilation
        if latest_hits is not None and len(latest_hits) > 0:
            flat_data = data.ravel()
            flat_data[latest_hits] = 100
            w, h = self.grid.width, self.grid.height
            rows = latest_hits // w
            cols = latest_hits % w
            for dr in (-1, 0, 1):
                for dc in (-1, 0, 1):
                    if dr == 0 and dc == 0:
                        continue
                    nr = np.clip(rows + dr, 0, h - 1)
                    nc = np.clip(cols + dc, 0, w - 1)
                    flat_data[nr * w + nc] = 100

        return data

    def to_dashboard_bytes(
        self,
        active_clusters: Optional[List[Any]] = None,
        active_cluster_polys: Optional[List[Any]] = None,
        latest_hits: Optional[np.ndarray] = None,
        dynamic_cells: Optional[Dict[int, float]] = None,
        *args,
        **kwargs
    ) -> bytes:
        """Packs map into standard 8-bit signed integer buffer for low-latency WebSocket streaming."""
        data = self.rasterize_frame(
            active_clusters=active_clusters,
            active_cluster_polys=active_cluster_polys,
            latest_hits=latest_hits,
            dynamic_cells=dynamic_cells,
            *args,
            **kwargs
        )
        return data.tobytes()

    def to_dashboard_b64(
        self,
        active_clusters: Optional[List[Any]] = None,
        active_cluster_polys: Optional[List[Any]] = None,
        latest_hits: Optional[np.ndarray] = None,
        dynamic_cells: Optional[Dict[int, float]] = None,
        *args,
        **kwargs
    ) -> str:
        """Encodes dashboard byte buffer into ASCII base64 string."""
        raw_b = self.to_dashboard_bytes(
            active_clusters=active_clusters,
            active_cluster_polys=active_cluster_polys,
            latest_hits=latest_hits,
            dynamic_cells=dynamic_cells,
            *args,
            **kwargs
        )
        return base64.b64encode(raw_b).decode("ascii")

    def to_dashboard_data(
        self,
        active_clusters: Optional[List[Any]] = None,
        active_cluster_polys: Optional[List[Any]] = None,
        latest_hits: Optional[np.ndarray] = None,
        dynamic_cells: Optional[Dict[int, float]] = None,
        *args,
        **kwargs
    ) -> List[int]:
        """Serializes grid as standard flat 1D Python list of integers."""
        data = self.rasterize_frame(
            active_clusters=active_clusters,
            active_cluster_polys=active_cluster_polys,
            latest_hits=latest_hits,
            dynamic_cells=dynamic_cells,
            *args,
            **kwargs
        )
        return data.ravel().tolist()
