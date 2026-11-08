"""
Planning namespace compatibility shim.
Forwards to planning module in 05-d500lib-motion.
"""
from planning.occupancy_grid import OccupancyGridMap

__all__ = ["OccupancyGridMap"]
