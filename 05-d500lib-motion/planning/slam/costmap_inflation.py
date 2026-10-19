"""
CostmapInflation: Euclidean Distance Field (EDT), C-Space Inflation, and Dynamic Obstacle Ingestion.
CS532 Advanced AI Robotics — Module 05-d500lib-motion / slam
"""
import time
import math
from typing import List, Tuple, Dict, Optional, Any
import numpy as np
from .grid_core import GridCore

try:
    import cv2
    HAS_CV2 = True
except ImportError:
    HAS_CV2 = False

try:
    from scipy.ndimage import distance_transform_edt, binary_dilation
    HAS_SCIPY = True
except ImportError:
    HAS_SCIPY = False


class CostmapInflation:
    """Computes 2D Euclidean Distance Transform (EDT), C-space inflation, and dynamic obstacle layers."""

    def __init__(
        self,
        grid_core: GridCore,
        robot_radius: float = 0.09,
        inflation_radius: float = 0.045,
        cost_scaling: float = 6.0,
        low_obstacle_ttl_sec: float = 0.30,
    ):
        self.grid = grid_core
        self.robot_radius = float(robot_radius)
        self.inflation_radius = float(inflation_radius)
        self.cost_scaling = float(cost_scaling)
        self.low_obstacle_ttl_sec = float(low_obstacle_ttl_sec)

        # Dynamic layers
        self.active_clusters: List[Tuple[float, float, float]] = []
        self.active_cluster_polys: List[Optional[List[Tuple[float, float]]]] = []
        self.social_bubbles: List[Tuple[float, float, float]] = []
        self.social_bubbles_ts: float = 0.0
        self._low_obstacle_cells: Dict[int, float] = {}

    def reset(self):
        """Reset dynamic costmap layers."""
        self.active_clusters.clear()
        self.active_cluster_polys.clear()
        self.social_bubbles.clear()
        self.social_bubbles_ts = 0.0
        self._low_obstacle_cells.clear()

    def _cluster_region(
        self,
        wx: float,
        wy: float,
        obs_rad: float,
        poly: Optional[List[Tuple[float, float]]] = None,
    ) -> Optional[Tuple[int, int, int, int, np.ndarray]]:
        """Computes subgrid slice and boolean mask for an obstacle cluster."""
        res = self.grid.resolution
        h, w = self.grid.height, self.grid.width

        if poly is not None and len(poly) >= 3 and HAS_CV2:
            pts_grid = np.array(
                [self.grid.world_to_grid(px, py) for px, py in poly], dtype=np.int32
            )
            c_min = max(0, int(pts_grid[:, 0].min()))
            c_max = min(w, int(pts_grid[:, 0].max()) + 1)
            r_min = max(0, int(pts_grid[:, 1].min()))
            r_max = min(h, int(pts_grid[:, 1].max()) + 1)
            if r_min < r_max and c_min < c_max:
                sub_h = r_max - r_min
                sub_w = c_max - c_min
                # If bounding box is not a massive room enclosure (<= 2.5m), fill the polygon
                if sub_w * res <= 2.5 and sub_h * res <= 2.5:
                    sub_pts = pts_grid - np.array([c_min, r_min], dtype=np.int32)
                    poly_mask = np.zeros((sub_h, sub_w), dtype=np.uint8)
                    cv2.fillPoly(poly_mask, [sub_pts], 1)
                    return r_min, r_max, c_min, c_max, poly_mask.astype(bool)

        c_p, r_p = self.grid.world_to_grid(wx, wy)
        rad_cells = max(1, int(math.ceil(obs_rad / res)))
        r_min = max(0, r_p - rad_cells)
        r_max = min(h, r_p + rad_cells + 1)
        c_min = max(0, c_p - rad_cells)
        c_max = min(w, c_p + rad_cells + 1)
        if r_min >= r_max or c_min >= c_max:
            return None
        ys, xs = np.ogrid[r_min - r_p : r_max - r_p, c_min - c_p : c_max - c_p]
        dist_sq = (xs * xs + ys * ys) * (res * res)
        disc_mask = dist_sq <= (obs_rad * obs_rad)
        return r_min, r_max, c_min, c_max, disc_mask

    def update_costmap(self):
        """
        Generates 2D costmap with Euclidean Distance Transform & Exponential Decay Inflation.
        Academic Standard (ROS 2 Nav2 costmap_2d::InflationLayer):
          - Inscribed lethal zone: d <= robot_radius (0.09m) -> cost = 254 (Lethal)
          - Inflation gradient zone: robot_radius < d <= robot_radius + inflation_radius
            -> cost = 253 * exp(-cost_scaling * (d - robot_radius))
          - Free zone: d > robot_radius + inflation_radius -> cost = 0
        """
        occ_mask = (self.grid.log_odds > 0.62).astype(np.uint8)

        # Include active obstacle clusters into the obstacle footprint mask
        if self.active_clusters:
            polys = self.active_cluster_polys
            if len(polys) < len(self.active_clusters):
                polys = polys + [None] * (len(self.active_clusters) - len(polys))
            for (wx, wy, obs_rad), poly in zip(self.active_clusters, polys):
                region = self._cluster_region(wx, wy, obs_rad, poly)
                if region is not None:
                    r_min, r_max, c_min, c_max, mask = region
                    occ_mask[r_min:r_max, c_min:c_max][mask] = 1

        self.grid.costmap.fill(0)

        if np.any(occ_mask):
            inscribed_m = self.robot_radius
            inflation_m = self.robot_radius + self.inflation_radius

            dist_cells = None
            if HAS_CV2:
                dist_cells = cv2.distanceTransform(1 - occ_mask, cv2.DIST_L2, 3)
            elif HAS_SCIPY:
                dist_cells = distance_transform_edt(1 - occ_mask)

            if dist_cells is not None:
                dist_m = dist_cells * self.grid.resolution
                self.grid.dist_m = dist_m

                # 1. Lethal core zone
                lethal_mask = dist_m <= inscribed_m
                self.grid.costmap[lethal_mask] = 254

                # 2. Smooth exponential decay inflation gradient
                decay_mask = (dist_m > inscribed_m) & (dist_m <= inflation_m)
                if np.any(decay_mask):
                    decay_costs = 253.0 * np.exp(
                        -self.cost_scaling * (dist_m[decay_mask] - inscribed_m)
                    )
                    self.grid.costmap[decay_mask] = np.clip(decay_costs, 1, 253).astype(np.uint8)
            else:
                self.grid.dist_m = None
                self.grid.costmap[occ_mask > 0] = 254
        else:
            self.grid.dist_m = None
            self.grid.costmap.fill(0)

        # Stamp active obstacle clusters as lethal core overlay
        if self.active_clusters:
            polys = self.active_cluster_polys
            if len(polys) < len(self.active_clusters):
                polys = polys + [None] * (len(self.active_clusters) - len(polys))
            for (wx, wy, obs_rad), poly in zip(self.active_clusters, polys):
                region = self._cluster_region(wx, wy, obs_rad, poly)
                if region is not None:
                    r_min, r_max, c_min, c_max, mask = region
                    self.grid.costmap[r_min:r_max, c_min:c_max][mask] = 254

        # Apply short-TTL low obstacles
        self._apply_low_obstacle_layer()

        # Re-apply active social bubbles (Hall's Proxemics)
        if self.social_bubbles:
            now_b = time.time()
            if now_b - self.social_bubbles_ts <= 2.0:
                for px, py, rad in self.social_bubbles:
                    self.add_social_bubble(px, py, radius_m=rad)
            else:
                self.social_bubbles = []

        # Enforce physical arena boundaries if specified
        if self.grid.arena_bounds is not None:
            min_x, max_x, min_y, max_y = self.grid.arena_bounds
            c_min, r_min = self.grid.world_to_grid(min_x, min_y)
            c_max, r_max = self.grid.world_to_grid(max_x, max_y)
            c_low = max(0, min(self.grid.width, min(c_min, c_max)))
            c_high = max(0, min(self.grid.width, max(c_min, c_max)))
            r_low = max(0, min(self.grid.height, min(r_min, r_max)))
            r_high = max(0, min(self.grid.height, max(r_min, r_max)))
            self.grid.costmap[:r_low, :] = 254
            self.grid.costmap[r_high:, :] = 254
            self.grid.costmap[:, :c_low] = 254
            self.grid.costmap[:, c_high:] = 254

    def insert_obstacle_clusters(
        self,
        clusters: List[Any],
        robot_x: float = 0.0,
        robot_y: float = 0.0,
        robot_theta: float = 0.0,
        defer_costmap: bool = False,
    ):
        """Integrates dynamic obstacle clusters into costmap."""
        if clusters is None or len(clusters) == 0:
            changed = bool(self.active_clusters)
            self.active_clusters = []
            self.active_cluster_polys = []
            if changed and not defer_costmap:
                self.update_costmap()
            return

        c_th = math.cos(robot_theta)
        s_th = math.sin(robot_theta)
        new_active = []
        new_polys = []

        for cl in clusters:
            pts_cnt = getattr(cl, "points_count", 0)
            if pts_cnt <= 0:
                continue

            cls_name = getattr(cl, "obstacle_type", "")
            if (
                cls_name == "WALL_SURFACE"
                or getattr(cl, "is_static_boundary", False)
                or getattr(cl, "is_wall", False)
            ):
                continue

            cent = getattr(cl, "centroid", (0.0, 0.0))
            if abs(cent[0]) < 1e-4 and abs(cent[1]) < 1e-4:
                continue

            wx = robot_x + cent[0] * c_th - cent[1] * s_th
            wy = robot_y + cent[0] * s_th + cent[1] * c_th

            eff_r = getattr(cl, "effective_radius", None)
            if eff_r is not None and eff_r > 0:
                obs_radius = min(0.25, max(0.04, float(eff_r)))
            else:
                obs_radius = min(0.25, max(0.04, float(getattr(cl, "radius_m", 0.10))))

            poly_body = getattr(cl, "bounding_box_2d", None)
            poly_world = None
            if poly_body is not None and len(poly_body) >= 3:
                poly_world = [
                    (
                        robot_x + px * c_th - py * s_th,
                        robot_y + px * s_th + py * c_th,
                    )
                    for px, py in poly_body
                ]

            new_active.append((wx, wy, obs_radius))
            new_polys.append(poly_world)

        self.active_clusters = new_active
        self.active_cluster_polys = new_polys
        if not defer_costmap:
            self.update_costmap()

    def insert_low_obstacles(
        self,
        points: Any,
        robot_x: Optional[float] = None,
        robot_y: Optional[float] = None,
        robot_theta: Optional[float] = None,
        defer_costmap: bool = False,
        ttl_sec: Optional[float] = None,
    ):
        """Ingests low-profile obstacle detections into short-TTL costmap layer."""
        if points is None or len(points) == 0:
            return

        pts_arr = np.asarray(points, dtype=np.float64)
        if pts_arr.ndim == 1 and len(pts_arr) == 2:
            pts_arr = pts_arr.reshape(1, 2)
        if pts_arr.ndim != 2 or pts_arr.shape[1] < 2:
            return

        if robot_x is not None and robot_y is not None and robot_theta is not None:
            c_th = math.cos(robot_theta)
            s_th = math.sin(robot_theta)
            xw = robot_x + pts_arr[:, 0] * c_th - pts_arr[:, 1] * s_th
            yw = robot_y + pts_arr[:, 0] * s_th + pts_arr[:, 1] * c_th
        else:
            xw = pts_arr[:, 0]
            yw = pts_arr[:, 1]

        cols, rows = self.grid.world_to_grid_array(xw, yw)
        valid = (cols >= 0) & (cols < self.grid.width) & (rows >= 0) & (rows < self.grid.height)
        if not np.any(valid):
            return

        ttl = self.low_obstacle_ttl_sec if ttl_sec is None else float(ttl_sec)
        expiry = time.time() + max(0.05, ttl)
        for idx in rows[valid] * self.grid.width + cols[valid]:
            self._low_obstacle_cells[int(idx)] = expiry

        if not defer_costmap:
            self.update_costmap()

    def _apply_low_obstacle_layer(self):
        """Sweeps expired CV cells and stamps active ones lethal on the costmap."""
        if not self._low_obstacle_cells:
            return
        now = time.time()
        for idx in [i for i, exp in self._low_obstacle_cells.items() if exp <= now]:
            del self._low_obstacle_cells[idx]
        if self._low_obstacle_cells:
            flat_cost = self.grid.costmap.ravel()
            flat_cost[list(self._low_obstacle_cells.keys())] = 254

    def add_social_bubble(
        self,
        center_x: float,
        center_y: float,
        radius_m: float = 0.40,
        core_radius_m: float = 0.20,
    ):
        """Injects Hall's Proxemics Social Safety Bubble around a detected person."""
        c_p, r_p = self.grid.world_to_grid(center_x, center_y)
        rad_cells = int(math.ceil(radius_m / self.grid.resolution))

        r_min = max(0, r_p - rad_cells)
        r_max = min(self.grid.height, r_p + rad_cells + 1)
        c_min = max(0, c_p - rad_cells)
        c_max = min(self.grid.width, c_p + rad_cells + 1)

        if r_min >= r_max or c_min >= c_max:
            return

        ys, xs = np.ogrid[r_min - r_p : r_max - r_p, c_min - c_p : c_max - c_p]
        dist_m = np.sqrt(xs * xs + ys * ys) * self.grid.resolution

        bubble_cost = np.zeros_like(dist_m, dtype=np.uint8)

        # Soft core zone (cost 230)
        core_mask = dist_m <= core_radius_m
        bubble_cost[core_mask] = 230

        # Social comfort gradient zone
        social_mask = (dist_m > core_radius_m) & (dist_m <= radius_m)
        if np.any(social_mask):
            decay = (radius_m - dist_m[social_mask]) / max(
                1e-3, (radius_m - core_radius_m)
            )
            bubble_cost[social_mask] = (110 + decay * 110).astype(np.uint8)

        self.grid.costmap[r_min:r_max, c_min:c_max] = np.maximum(
            self.grid.costmap[r_min:r_max, c_min:c_max], bubble_cost
        )

    def update_social_bubbles_from_detections(
        self,
        detections: list,
        robot_x: float,
        robot_y: float,
        robot_theta: float,
        bubble_radius_m: float = 0.40,
    ):
        """Extracts person detections and projects social bubbles into costmap."""
        self.social_bubbles = []
        self.social_bubbles_ts = time.time()
        for det in detections:
            cname = (
                det.get("class") or det.get("class_name")
                if isinstance(det, dict)
                else getattr(det, "class_name", "")
            )
            if cname == "person":
                dist = float(
                    det.get("estimated_dist", det.get("distance", 0.0))
                    if isinstance(det, dict)
                    else getattr(det, "estimated_dist", getattr(det, "distance", 0.0))
                )
                if 0.15 < dist < 5.0:
                    if isinstance(det, dict) and "azimuth_deg" in det:
                        azimuth_rad = -math.radians(float(det["azimuth_deg"]))
                    elif hasattr(det, "azimuth_deg"):
                        azimuth_rad = -math.radians(float(det.azimuth_deg))
                    else:
                        bbox = (
                            det.get("bbox", [320, 240, 320, 240])
                            if isinstance(det, dict)
                            else getattr(det, "bbox", [320, 240, 320, 240])
                        )
                        cx = (bbox[0] + bbox[2]) / 2.0
                        azimuth_rad = math.radians(((320.0 - cx) / 320.0) * 80.0)
                    abs_theta = robot_theta + azimuth_rad
                    px = robot_x + dist * math.cos(abs_theta)
                    py = robot_y + dist * math.sin(abs_theta)
                    margin = float(
                        det.get("safety_margin_m", bubble_radius_m)
                        if isinstance(det, dict)
                        else getattr(det, "safety_margin_m", bubble_radius_m)
                    )
                    self.social_bubbles.append((px, py, margin))

        for px, py, rad in self.social_bubbles:
            self.add_social_bubble(px, py, radius_m=rad)
