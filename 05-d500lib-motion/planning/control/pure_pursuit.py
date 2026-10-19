"""
PurePursuitTracker: Path projection, adaptive lookahead search,
chord steering curvature calculation, and calibrated pivot gating.
"""
import math
from typing import List, Tuple, Optional


class PurePursuitTracker:
    """Calculates lookahead target points and steering curvature along reference paths."""

    def __init__(
        self,
        lookahead_dist: float = 0.28,
        goal_tolerance: float = 0.18,
        pivot_entry_deg: float = 60.0,
        align_exit_deg: float = 30.0,
    ):
        self.lookahead_dist = float(lookahead_dist)
        self.goal_tolerance = float(goal_tolerance)
        self.pivot_entry_thresh = math.radians(pivot_entry_deg)
        self.align_exit_thresh = math.radians(align_exit_deg)

    def project_to_path(
        self,
        robot_x: float,
        robot_y: float,
        path: List[Tuple[float, float]],
        start_idx: int = 0
    ) -> Tuple[int, Tuple[float, float], float]:
        """Orthogonal projection of robot coordinates onto piecewise linear path segments."""
        if not path or len(path) < 2:
            return 0, (robot_x, robot_y), 0.0

        best_dist = float("inf")
        best_seg = min(start_idx, len(path) - 2)
        best_proj = path[best_seg]

        for i in range(start_idx, len(path) - 1):
            p1 = path[i]
            p2 = path[i + 1]
            dx_seg = p2[0] - p1[0]
            dy_seg = p2[1] - p1[1]
            seg_len_sq = dx_seg * dx_seg + dy_seg * dy_seg

            if seg_len_sq < 1e-6:
                continue

            t = ((robot_x - p1[0]) * dx_seg + (robot_y - p1[1]) * dy_seg) / seg_len_sq
            t_clamped = max(0.0, min(1.0, t))
            proj_x = p1[0] + t_clamped * dx_seg
            proj_y = p1[1] + t_clamped * dy_seg
            d = math.hypot(robot_x - proj_x, robot_y - proj_y)

            if d < best_dist:
                best_dist = d
                best_seg = i
                best_proj = (proj_x, proj_y)

        return best_seg, best_proj, best_dist

    def find_lookahead_target(
        self,
        path: List[Tuple[float, float]],
        best_seg: int,
        best_proj: Tuple[float, float],
        effective_lookahead: float,
    ) -> Tuple[float, float]:
        """Traverses path polyline starting from best_proj until effective_lookahead is reached."""
        rem_L = effective_lookahead
        curr_pt = best_proj
        target_pt = path[-1]

        for j in range(best_seg, len(path) - 1):
            next_pt = path[j + 1]
            seg_dx = next_pt[0] - curr_pt[0]
            seg_dy = next_pt[1] - curr_pt[1]
            seg_len = math.hypot(seg_dx, seg_dy)

            if seg_len >= rem_L:
                target_pt = (
                    curr_pt[0] + (seg_dx / seg_len) * rem_L,
                    curr_pt[1] + (seg_dy / seg_len) * rem_L
                )
                return target_pt
            else:
                rem_L -= seg_len
                curr_pt = next_pt

        return target_pt

    def compute_steering(
        self,
        robot_x: float,
        robot_y: float,
        robot_theta: float,
        target_pt: Tuple[float, float],
        lookahead_dist: float,
    ) -> Tuple[float, float, float]:
        """
        Computes heading error alpha, curvature kappa, and distance to lookahead target.
        Returns (alpha, kappa, dist_to_target).
        """
        dx = target_pt[0] - robot_x
        dy = target_pt[1] - robot_y
        dist_to_target = math.hypot(dx, dy)
        target_angle = math.atan2(dy, dx)
        alpha = (target_angle - robot_theta + math.pi) % (2.0 * math.pi) - math.pi

        # RPP Chord Length Adaptation
        L_chord = max(0.08, min(lookahead_dist, dist_to_target))
        kappa = (2.0 * math.sin(alpha)) / L_chord

        return alpha, kappa, dist_to_target

    def update_pivot_state(self, alpha: float, in_pivot: bool) -> bool:
        """
        Calibrated Hysteresis Pivot Gating:
        - Enter in-place pivot only when error |alpha| > pivot_entry_thresh (60 deg).
        - Exit pivot when error |alpha| <= align_exit_thresh (30 deg).
        For diagonal targets (|alpha| <= 60 deg), continuous differential steering is used!
        """
        abs_alpha = abs(alpha)
        if in_pivot:
            if abs_alpha <= self.align_exit_thresh:
                return False
            return True
        else:
            if abs_alpha > self.pivot_entry_thresh:
                return True
            return False
