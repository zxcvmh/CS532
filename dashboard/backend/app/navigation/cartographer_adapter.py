"""
CartographerAdapter: Bridges Google Cartographer 2D Core into JetBot Dashboard.
Zero-drift 2D LiDAR SLAM with Real-Time Correlative Scan Matching (CSM).
"""
import math
import time
import base64
import os
import sys
from typing import List, Tuple, Dict, Any, Optional
import numpy as np
from scipy.ndimage import distance_transform_edt

# Add cartographer path dynamically
current_dir = os.path.dirname(os.path.abspath(__file__))
for rel_path in ["../../../../cartographer", "../../../cartographer", "../../cartographer"]:
    cand = os.path.abspath(os.path.join(current_dir, rel_path))
    if os.path.isdir(cand) and os.path.isdir(os.path.join(cand, "cartographer_core")):
        if cand not in sys.path:
            sys.path.insert(0, cand)
        break

from cartographer_core.probability_grid import ProbabilityGrid2D, probability_to_odds
from cartographer_core.correlative_scan_matcher import RealTimeCorrelativeScanMatcher2D
from cartographer_core.submap_2d import ActiveSubmaps2D
from cartographer_core.pose_graph_2d import PoseGraph2D


class CartographerAdapter:
    """Production Adapter for Cartographer 2D SLAM."""

    def __init__(
        self,
        resolution: float = 0.05,
        width: int = 160,
        height: int = 160,
        submap_scans: int = 35,
        angular_search_window_deg: float = 35.0,
        linear_search_window_m: float = 0.15,
        robot_radius: float = 0.09,
        inflation_radius: float = 0.045
    ):
        self.resolution = float(resolution)
        self.width = int(width)
        self.height = int(height)
        self.origin_x = -0.5 * self.width * self.resolution
        self.origin_y = -0.5 * self.height * self.resolution
        self.robot_radius = float(robot_radius)
        self.inflation_radius = float(inflation_radius)

        # 1. Global Combined Probability Grid
        self.global_grid = ProbabilityGrid2D(
            resolution=self.resolution,
            width=self.width,
            height=self.height,
            origin_x=self.origin_x,
            origin_y=self.origin_y
        )

        # 2. Local SLAM: Real-Time Correlative Scan Matcher (CSM)
        self.csm = RealTimeCorrelativeScanMatcher2D(
            linear_search_window=linear_search_window_m,
            angular_search_window_rad=math.radians(angular_search_window_deg),
            translation_weight=0.1,
            rotation_weight=0.1
        )

        # 3. Active Submaps Manager
        self.submaps_mgr = ActiveSubmaps2D(
            num_range_data=submap_scans,
            resolution=self.resolution,
            grid_width=self.width,
            grid_height=self.height
        )

        # 4. Global SLAM: Pose Graph & Loop Closure
        self.pose_graph = PoseGraph2D(min_score=0.65)

        self.current_pose = (0.0, 0.0, 0.0)  # (x, y, theta)
        self.is_first_scan = True
        self.total_scans = 0

        # Cached EDT & Costmap for A* / MPPI planner compatibility
        self.costmap = np.zeros((self.height, self.width), dtype=np.float32)
        self.dist_m = np.zeros((self.height, self.width), dtype=np.float32)
        self.costmap_version = 0

    def get_pose(self) -> Tuple[float, float, float]:
        return self.current_pose

    def reset(self, x: float = 0.0, y: float = 0.0, theta: float = 0.0):
        """Reset SLAM state to initial pose."""
        self.current_pose = (float(x), float(y), float(theta))
        self.global_grid = ProbabilityGrid2D(
            resolution=self.resolution,
            width=self.width,
            height=self.height,
            origin_x=self.origin_x,
            origin_y=self.origin_y
        )
        self.submaps_mgr = ActiveSubmaps2D(
            num_range_data=35,
            resolution=self.resolution,
            grid_width=self.width,
            grid_height=self.height
        )
        self.pose_graph = PoseGraph2D(min_score=0.65)
        self.is_first_scan = True
        self.total_scans = 0

    def update_scan(
        self,
        ranges: List[float],
        dt: float = 0.1,
        v_cmd: float = 0.0,
        w_cmd: float = 0.0
    ) -> Tuple[float, float, float, Dict[str, Any]]:
        """
        Ingest 360-ray LiDAR sweep, match against Submaps via CSM, update ProbabilityGrid.
        """
        t0 = time.time()
        dt = max(0.01, min(0.5, float(dt)))

        # Convert ranges to local 2D point cloud (x, y)
        angles = np.linspace(0, 2 * np.pi, len(ranges), endpoint=False)
        r_arr = np.array(ranges, dtype=np.float32)
        valid = (r_arr >= 0.05) & (r_arr <= 6.0) & (~np.isnan(r_arr))

        if not np.any(valid):
            return self.current_pose[0], self.current_pose[1], self.current_pose[2], {"status": "NO_POINTS", "is_reliable": False}

        local_pts = np.column_stack([
            r_arr[valid] * np.cos(angles[valid]),
            r_arr[valid] * np.sin(angles[valid])
        ])

        # Kinematic Feedforward Prior
        curr_x, curr_y, curr_th = self.current_pose
        prior_dx = v_cmd * dt * math.cos(curr_th + 0.5 * w_cmd * dt)
        prior_dy = v_cmd * dt * math.sin(curr_th + 0.5 * w_cmd * dt)
        prior_dth = w_cmd * dt
        initial_guess = (
            curr_x + prior_dx,
            curr_y + prior_dy,
            (curr_th + prior_dth + math.pi) % (2.0 * math.pi) - math.pi
        )

        if self.is_first_scan:
            best_pose = (0.0, 0.0, 0.0)
            score = 1.0
            self.is_first_scan = False
        else:
            # Match against currently active submap
            target_grid = self.submaps_mgr.submaps[0].grid if self.submaps_mgr.submaps else self.global_grid
            best_pose, score = self.csm.match(target_grid, local_pts, initial_pose=initial_guess)

        self.current_pose = best_pose
        bx, by, bth = best_pose

        # Transform local points to world coordinates
        c, s = math.cos(bth), math.sin(bth)
        world_pts = np.column_stack([
            bx + local_pts[:, 0] * c - local_pts[:, 1] * s,
            by + local_pts[:, 0] * s + local_pts[:, 1] * c
        ])

        # Insert range data into ActiveSubmaps2D
        active_submaps = self.submaps_mgr.insert_range_data(best_pose, world_pts)

        # Rolling Dynamic Aggregation: insert into global_grid for immediate display
        self.global_grid.insert_scan(bx, by, world_pts)

        # Register node into Pose Graph
        self.total_scans += 1
        if self.total_scans % 3 == 0:
            self.pose_graph.add_node(self.total_scans, best_pose, time.time(), local_pts, active_submaps)

        elapsed_ms = (time.time() - t0) * 1000.0
        info = {
            "status": "CARTOGRAPHER_CSM_MATCH",
            "is_reliable": bool(score >= 0.35),
            "score": float(score),
            "match_time_ms": float(elapsed_ms)
        }
        return bx, by, bth, info

    def to_dashboard_data(self) -> Dict[str, Any]:
        """Produce JSON-compatible int8 occupancy grid map for frontend MapViewer.tsx."""
        prob = self.global_grid.to_probability_array()  # [height, width] in [0.0, 1.0], -1 for unknown
        out = np.full((self.height, self.width), -1, dtype=np.int8)

        known = prob >= 0.0
        # Map [0.0, 1.0] to [0, 100]
        out[known] = np.clip(prob[known] * 100.0, 0, 100).astype(np.int8)

        return {
            "width": self.width,
            "height": self.height,
            "resolution": self.resolution,
            "originX": self.origin_x,
            "originY": self.origin_y,
            "data": out.flatten().tolist()
        }

    def to_dashboard_b64(self) -> str:
        """Produce Base64-encoded int8 stream for ultra-fast frontend transmission."""
        prob = self.global_grid.to_probability_array()
        out = np.full((self.height, self.width), -1, dtype=np.int8)
        known = prob >= 0.0
        out[known] = np.clip(prob[known] * 100.0, 0, 100).astype(np.int8)
        return base64.b64encode(out.tobytes()).decode("ascii")

    def update_costmap(self):
        """Update EDT distance field and inflated costmap for A* / MPPI planner."""
        prob = self.global_grid.to_probability_array()
        obstacles = (prob >= 0.55)
        if not np.any(obstacles):
            self.dist_m.fill(5.0)
            self.costmap.fill(0.0)
            return

        edt_cells = distance_transform_edt(~obstacles)
        self.dist_m = edt_cells.astype(np.float32) * self.resolution

        lethal_dist = self.robot_radius
        decay_dist = self.robot_radius + self.inflation_radius
        cost = np.zeros_like(self.dist_m)

        cost[self.dist_m <= lethal_dist] = 254.0
        inflation_mask = (self.dist_m > lethal_dist) & (self.dist_m <= decay_dist)
        decay_factor = (decay_dist - self.dist_m[inflation_mask]) / max(1e-4, self.inflation_radius)
        cost[inflation_mask] = np.clip(decay_factor * 253.0, 1.0, 253.0)

        self.costmap = cost
        self.costmap_version += 1

    def clear_footprint(self, x: float, y: float, radius: float = 0.09, **kwargs):
        """Clear footprint around robot center."""
        r_cells = int(math.ceil(radius / self.resolution))
        col, row = self.global_grid.world_to_grid(x, y)
        r_start = max(0, row - r_cells)
        r_end = min(self.height, row + r_cells + 1)
        c_start = max(0, col - r_cells)
        c_end = min(self.width, col + r_cells + 1)

        y_idx, x_idx = np.ogrid[r_start:r_end, c_start:c_end]
        dist_sq = (x_idx - col) ** 2 + (y_idx - row) ** 2
        mask = dist_sq <= (radius / self.resolution) ** 2
        # Set cells to free (odd < 0 or prob ~ 0.1)
        self.global_grid.odds[r_start:r_end, c_start:c_end][mask] = probability_to_odds(0.1)
        self.global_grid.observed[r_start:r_end, c_start:c_end][mask] = True
