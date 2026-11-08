"""
Google Cartographer 2D Pose Graph & Loop Closure
================================================
Reference: Wolfgang Hess, Damon Kohler, Holger Rapp, Daniel Andor (Google, 2016).
Section II-B: "Global SLAM - Closing Loops".
Maintains a pose graph of Trajectory Nodes (scan poses) and Submap Poses.
Branch-and-Bound / Fast Correlative Scan Matching evaluates loop closure candidates
against finished submaps. When score >= min_score (0.65), loop closure constraints
are added and the global pose graph is optimized via non-linear least squares.
"""
import math
import numpy as np
from typing import List, Tuple, Dict, Optional
from .submap_2d import Submap2D
from .correlative_scan_matcher import RealTimeCorrelativeScanMatcher2D


class TrajectoryNode2D:
    """A node in the trajectory pose graph representing an inserted scan."""

    def __init__(self, node_id: int, pose: Tuple[float, float, float], timestamp: float, points_local: np.ndarray):
        self.node_id = int(node_id)
        self.pose = pose            # (x, y, theta)
        self.timestamp = float(timestamp)
        self.points_local = points_local


class Constraint2D:
    """A constraint between a node and a submap (intra or inter/loop closure)."""

    def __init__(
        self,
        node_id: int,
        submap_id: int,
        relative_pose: Tuple[float, float, float],
        weight_translation: float = 1.0,
        weight_rotation: float = 10.0,
        is_loop_closure: bool = False,
    ):
        self.node_id = int(node_id)
        self.submap_id = int(submap_id)
        self.relative_pose = relative_pose  # (dx, dy, dtheta) in submap frame
        self.weight_translation = float(weight_translation)
        self.weight_rotation = float(weight_rotation)
        self.is_loop_closure = bool(is_loop_closure)


class PoseGraph2D:
    """
    Global SLAM backend managing nodes, submaps, loop closures, and optimization.
    """

    def __init__(
        self,
        min_score: float = 0.65,
        global_search_window: float = 7.0,
        angular_search_window: float = math.radians(30.0)
    ):
        self.min_score = float(min_score)
        self.nodes: List[TrajectoryNode2D] = []
        self.submaps: Dict[int, Submap2D] = {}
        self.submap_poses: Dict[int, Tuple[float, float, float]] = {}
        self.constraints: List[Constraint2D] = []

        self.loop_closure_matcher = RealTimeCorrelativeScanMatcher2D(
            linear_search_window=global_search_window,
            angular_search_window_rad=angular_search_window
        )
        self.total_loop_closures = 0

    def add_node(
        self,
        node_id: int,
        pose: Tuple[float, float, float],
        timestamp: float,
        points_local: np.ndarray,
        active_submaps: List[Submap2D]
    ) -> TrajectoryNode2D:
        """Add a new trajectory node and register intra-submap constraints."""
        node = TrajectoryNode2D(node_id, pose, timestamp, points_local)
        self.nodes.append(node)

        # Register intra-submap constraints for currently active submaps
        for submap in active_submaps:
            sid = submap.submap_id
            if sid not in self.submaps:
                self.submaps[sid] = submap
                self.submap_poses[sid] = submap.local_pose

            # Relative pose: T_submap^-1 * T_node
            rel_pose = self._compute_relative_pose(self.submap_poses[sid], pose)
            self.constraints.append(
                Constraint2D(
                    node_id=node_id,
                    submap_id=sid,
                    relative_pose=rel_pose,
                    is_loop_closure=False
                )
            )

        return node

    def check_loop_closures(self, current_node: TrajectoryNode2D, finished_submaps: List[Submap2D]) -> int:
        """
        Evaluate loop closure candidates against all finished submaps.
        Follows Cartographer's Branch-and-Bound / Correlative candidate testing.
        """
        new_closures = 0
        cur_x, cur_y, cur_th = current_node.pose

        for submap in finished_submaps:
            sid = submap.submap_id
            sub_x, sub_y, sub_th = self.submap_poses[sid]
            dist = math.hypot(cur_x - sub_x, cur_y - sub_y)

            # Only check submaps within physical reach (e.g. within 5.0m)
            if dist > 5.0:
                continue

            # Skip checking if a constraint already exists for this pair
            if any(c.node_id == current_node.node_id and c.submap_id == sid for c in self.constraints):
                continue

            # Run correlative scan match against the finished submap
            matched_pose, score = self.loop_closure_matcher.match(
                submap.grid,
                current_node.points_local,
                current_node.pose
            )

            if score >= self.min_score:
                rel_pose = self._compute_relative_pose(self.submap_poses[sid], matched_pose)
                self.constraints.append(
                    Constraint2D(
                        node_id=current_node.node_id,
                        submap_id=sid,
                        relative_pose=rel_pose,
                        weight_translation=2.0,
                        weight_rotation=20.0,
                        is_loop_closure=True
                    )
                )
                self.total_loop_closures += 1
                new_closures += 1

                # Trigger global pose graph optimization
                self.optimize()

        return new_closures

    def optimize(self, iterations: int = 10):
        """
        Global Sparse Pose Adjustment (SPA).
        Minimizes constraint residuals across all nodes and submaps.
        """
        if len(self.nodes) < 2 or not any(c.is_loop_closure for c in self.constraints):
            return

        # Simple iterative relaxation solver (Gauss-Seidel) for pose graph
        for _ in range(iterations):
            for c in self.constraints:
                sub_pose = self.submap_poses[c.submap_id]
                node = next((n for n in self.nodes if n.node_id == c.node_id), None)
                if not node:
                    continue

                # Expected node pose based on constraint: T_submap * T_relative
                expected_node_pose = self._apply_transform(sub_pose, c.relative_pose)

                # Error vector
                err_x = expected_node_pose[0] - node.pose[0]
                err_y = expected_node_pose[1] - node.pose[1]
                err_th = self._normalize_angle(expected_node_pose[2] - node.pose[2])

                # Damped correction
                damping = 0.25 if c.is_loop_closure else 0.05
                node.pose = (
                    node.pose[0] + damping * err_x,
                    node.pose[1] + damping * err_y,
                    self._normalize_angle(node.pose[2] + damping * err_th)
                )

    def _compute_relative_pose(
        self,
        from_pose: Tuple[float, float, float],
        to_pose: Tuple[float, float, float]
    ) -> Tuple[float, float, float]:
        """Compute relative transform from from_pose to to_pose."""
        fx, fy, fth = from_pose
        tx, ty, tth = to_pose
        dx = tx - fx
        dy = ty - fy
        c = math.cos(fth)
        s = math.sin(fth)
        local_x = dx * c + dy * s
        local_y = -dx * s + dy * c
        local_th = self._normalize_angle(tth - fth)
        return (local_x, local_y, local_th)

    def _apply_transform(
        self,
        base_pose: Tuple[float, float, float],
        rel_pose: Tuple[float, float, float]
    ) -> Tuple[float, float, float]:
        """Apply relative transform to base_pose."""
        bx, by, bth = base_pose
        rx, ry, rth = rel_pose
        c = math.cos(bth)
        s = math.sin(bth)
        out_x = bx + rx * c - ry * s
        out_y = by + rx * s + ry * c
        out_th = self._normalize_angle(bth + rth)
        return (out_x, out_y, out_th)

    def _normalize_angle(self, angle: float) -> float:
        """Wrap angle into [-pi, pi]."""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle
