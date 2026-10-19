"""
Google Cartographer 2D Submap and ActiveSubmaps2D
=================================================
Reference: Wolfgang Hess, Damon Kohler, Holger Rapp, Daniel Andor (Google, 2016).
Section II-A: "Local SLAM - Submaps".
ActiveSubmaps2D holds at most two active submaps at any time. Scans are inserted
into both active submaps until the older one finishes after N range data insertions.
Finished submaps become immutable targets for Global SLAM loop closure.
"""
import math
import numpy as np
from typing import List, Tuple, Optional
from .probability_grid import ProbabilityGrid2D


class Submap2D:
    """A single 2D Submap containing an occupancy probability grid and pose."""

    def __init__(
        self,
        submap_id: int,
        local_pose: Tuple[float, float, float],
        resolution: float = 0.05,
        width: int = 160,
        height: int = 160
    ):
        self.submap_id = int(submap_id)
        self.local_pose = local_pose  # (x, y, theta)
        self.resolution = float(resolution)
        self.grid = ProbabilityGrid2D(
            resolution=resolution,
            width=width,
            height=height,
            origin_x=local_pose[0] - (width * resolution * 0.5),
            origin_y=local_pose[1] - (height * resolution * 0.5)
        )
        self.num_range_data = 0
        self.is_finished = False

    def insert_scan(self, robot_x: float, robot_y: float, hit_points_world: np.ndarray):
        """Insert scan points into this submap's probability grid."""
        self.grid.insert_scan(robot_x, robot_y, hit_points_world)
        self.num_range_data += 1


class ActiveSubmaps2D:
    """
    Manages active submaps according to Google Cartographer's dual-submap policy:
    - Maintains two active submaps overlapping by 50%.
    - New submap created every num_range_data insertions.
    - Submap finishes after 2 * num_range_data insertions.
    """

    def __init__(
        self,
        num_range_data: int = 35,
        resolution: float = 0.05,
        grid_width: int = 160,
        grid_height: int = 160
    ):
        self.num_range_data = int(num_range_data)
        self.resolution = float(resolution)
        self.grid_width = int(grid_width)
        self.grid_height = int(grid_height)

        self.submaps: List[Submap2D] = []
        self.finished_submaps: List[Submap2D] = []
        self._next_id = 0

    def insert_range_data(
        self,
        robot_pose: Tuple[float, float, float],
        hit_points_world: np.ndarray
    ) -> List[Submap2D]:
        """
        Insert range data into currently active submaps.
        Returns the list of active submaps after insertion.
        """
        rx, ry, _ = robot_pose

        # Initialize first submap if empty
        if not self.submaps:
            self._create_new_submap(robot_pose)

        # Check if it is time to create the overlapping second submap
        if self.submaps[0].num_range_data >= self.num_range_data and len(self.submaps) == 1:
            self._create_new_submap(robot_pose)

        # Insert scan into all active submaps
        for submap in self.submaps:
            submap.insert_scan(rx, ry, hit_points_world)

        # Check if the older submap has finished its full lifecycle (2 * num_range_data)
        if self.submaps[0].num_range_data >= 2 * self.num_range_data:
            finished = self.submaps.pop(0)
            finished.is_finished = True
            self.finished_submaps.append(finished)

        return list(self.submaps)

    def _create_new_submap(self, robot_pose: Tuple[float, float, float]):
        """Create a new active submap centered at the robot's current pose."""
        new_sub = Submap2D(
            submap_id=self._next_id,
            local_pose=robot_pose,
            resolution=self.resolution,
            width=self.grid_width,
            height=self.grid_height
        )
        self._next_id += 1
        self.submaps.append(new_sub)
