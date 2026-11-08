"""
Real-Time Correlative Scan Matcher 2D (CSM)
===========================================
Reference: Edwin Olson (2009), "Real-Time Correlative Scan Matching"
and Wolfgang Hess et al. (Google Cartographer 2016).
Provides exhaustive grid search over a bounded (x, y, theta) window to guarantee
convergence into the global basin of attraction, preventing the divergence of pure
Gauss-Newton matchers on robot rotations > 3 degrees.
"""
import math
import numpy as np
from typing import Tuple, List, Optional
from .probability_grid import ProbabilityGrid2D


class RealTimeCorrelativeScanMatcher2D:
    """
    Real-Time Correlative Scan Matcher (CSM).
    Exhaustively scores discrete candidates in a 3D search space (dx, dy, dtheta).
    """

    def __init__(
        self,
        linear_search_window: float = 0.15,
        angular_search_window_rad: float = math.radians(20.0),
        translation_weight: float = 0.1,
        rotation_weight: float = 0.1,
    ):
        self.linear_search_window = float(linear_search_window)
        self.angular_search_window = float(angular_search_window_rad)
        self.translation_weight = float(translation_weight)
        self.rotation_weight = float(rotation_weight)

    def match(
        self,
        grid: ProbabilityGrid2D,
        point_cloud_local: np.ndarray,
        initial_pose: Tuple[float, float, float]
    ) -> Tuple[Tuple[float, float, float], float]:
        """
        Match point cloud against probability grid around initial_pose (x0, y0, th0).
        Returns:
            best_pose: (x, y, theta) in world frame.
            best_score: normalized score in [0.0, 1.0].
        """
        if len(point_cloud_local) == 0:
            return initial_pose, 0.0

        x0, y0, th0 = initial_pose
        res = grid.resolution

        # Compute search steps
        # Angular step size ensures that a point at max range moves by at most 1 grid cell
        max_r = float(np.max(np.hypot(point_cloud_local[:, 0], point_cloud_local[:, 1]))) if len(point_cloud_local) > 0 else 3.0
        max_r = max(1.0, max_r)
        d_theta = res / max_r

        # Discrete search ranges
        num_theta_steps = int(math.ceil(self.angular_search_window / d_theta))
        theta_candidates = [
            th0 + step * d_theta
            for step in range(-num_theta_steps, num_theta_steps + 1)
        ]

        num_lin_steps = int(math.ceil(self.linear_search_window / res))
        dx_candidates = [step * res for step in range(-num_lin_steps, num_lin_steps + 1)]
        dy_candidates = [step * res for step in range(-num_lin_steps, num_lin_steps + 1)]

        best_score = -1e9
        best_pose = (x0, y0, th0)

        # Precompute rotated point clouds for each candidate angle
        for cand_th in theta_candidates:
            c = math.cos(cand_th)
            s = math.sin(cand_th)
            # 2D rotation of local points
            rot_x = point_cloud_local[:, 0] * c - point_cloud_local[:, 1] * s
            rot_y = point_cloud_local[:, 0] * s + point_cloud_local[:, 1] * c

            # Penalty for angular deviation from initial estimate
            dth = abs(cand_th - th0)
            rot_penalty = self.rotation_weight * (dth ** 2)

            for dx in dx_candidates:
                cand_x = x0 + dx
                trans_x = rot_x + cand_x

                for dy in dy_candidates:
                    cand_y = y0 + dy
                    trans_y = rot_y + cand_y

                    # Penalty for linear translation deviation
                    trans_penalty = self.translation_weight * (dx * dx + dy * dy)

                    # Compute score across points
                    score_sum = 0.0
                    valid_pts = 0

                    cols = np.floor((trans_x - grid.origin_x) * grid.inv_resolution).astype(np.int32)
                    rows = np.floor((trans_y - grid.origin_y) * grid.inv_resolution).astype(np.int32)

                    valid_mask = (cols >= 0) & (cols < grid.width) & (rows >= 0) & (rows < grid.height)
                    if np.any(valid_mask):
                        v_cols = cols[valid_mask]
                        v_rows = rows[valid_mask]
                        odds_vals = grid.odds[v_rows, v_cols]
                        # Probability = odds / (1 + odds)
                        probs = odds_vals / (1.0 + odds_vals)
                        score_sum = float(np.sum(probs))
                        valid_pts = int(np.sum(valid_mask))

                    if valid_pts > 0:
                        norm_score = (score_sum / len(point_cloud_local)) - (trans_penalty + rot_penalty)
                        if norm_score > best_score:
                            best_score = norm_score
                            best_pose = (cand_x, cand_y, cand_th)

        # Local gradient refinement (Ceres emulation) around best discrete candidate
        refined_pose = self._refine_pose_gradient(grid, point_cloud_local, best_pose)
        return refined_pose, max(0.0, best_score)

    def _refine_pose_gradient(
        self,
        grid: ProbabilityGrid2D,
        points: np.ndarray,
        seed_pose: Tuple[float, float, float],
        max_iters: int = 10
    ) -> Tuple[float, float, float]:
        """Sub-pixel gradient refinement with trust region bounded inside seed cell."""
        x, y, th = seed_pose
        step_xy = 0.005
        step_th = 0.005
        N = max(1, len(points))
        max_delta_xy = grid.resolution * 0.5
        max_delta_th = math.radians(2.0)

        def eval_score(p: Tuple[float, float, float]) -> float:
            return self._eval_score(grid, points, p) / N

        curr_score = eval_score((x, y, th))
        for _ in range(max_iters):
            gx = (eval_score((x + step_xy, y, th)) - eval_score((x - step_xy, y, th))) / (2.0 * step_xy)
            gy = (eval_score((x, y + step_xy, th)) - eval_score((x, y - step_xy, th))) / (2.0 * step_xy)
            gth = (eval_score((x, y, th + step_th)) - eval_score((x, y, th - step_th))) / (2.0 * step_th)

            norm_g = math.hypot(gx, gy)
            if norm_g > 1e-4:
                nx = x + min(0.01, norm_g * 0.005) * (gx / norm_g)
                ny = y + min(0.01, norm_g * 0.005) * (gy / norm_g)
            else:
                nx, ny = x, y

            nth = th + max(-0.01, min(0.01, gth * 0.005))

            # Trust region: strictly bounded within the discrete candidate cell
            nx = max(seed_pose[0] - max_delta_xy, min(seed_pose[0] + max_delta_xy, nx))
            ny = max(seed_pose[1] - max_delta_xy, min(seed_pose[1] + max_delta_xy, ny))
            nth = max(seed_pose[2] - max_delta_th, min(seed_pose[2] + max_delta_th, nth))

            new_score = eval_score((nx, ny, nth))
            if new_score >= curr_score:
                x, y, th = nx, ny, nth
                curr_score = new_score
            else:
                break

        return (float(x), float(y), float(th))

    def _eval_score(self, grid: ProbabilityGrid2D, points: np.ndarray, pose: Tuple[float, float, float]) -> float:
        px, py, pth = pose
        c = math.cos(pth)
        s = math.sin(pth)
        tx = points[:, 0] * c - points[:, 1] * s + px
        ty = points[:, 0] * s + points[:, 1] * c + py

        cols = np.floor((tx - grid.origin_x) * grid.inv_resolution).astype(np.int32)
        rows = np.floor((ty - grid.origin_y) * grid.inv_resolution).astype(np.int32)
        valid = (cols >= 0) & (cols < grid.width) & (rows >= 0) & (rows < grid.height)
        if not np.any(valid):
            return 0.0
        odds_vals = grid.odds[rows[valid], cols[valid]]
        return float(np.sum(odds_vals / (1.0 + odds_vals)))

