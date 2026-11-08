"""
OccupancyGridMap compatibility shim.
Forwards to planning.occupancy_grid in 05-d500lib-motion.
"""
from planning.occupancy_grid import OccupancyGridMap

__all__ = ["OccupancyGridMap"]
