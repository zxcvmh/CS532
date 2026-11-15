"""
OccupancyGridMap compatibility shim.
Forwards to planning.occupancy_grid in motion.
"""
from planning.occupancy_grid import OccupancyGridMap

__all__ = ["OccupancyGridMap"]
