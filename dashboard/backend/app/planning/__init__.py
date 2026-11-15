"""
Planning namespace compatibility shim.
Forwards to planning module in motion.
"""
from planning.occupancy_grid import OccupancyGridMap

__all__ = ["OccupancyGridMap"]
