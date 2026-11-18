"""
DynamicLayer: Rolling Obstacle Layer with 0.35s TTL Decay.
Implements the Multi-Layer Decoupled Architecture footstep evaporation mechanism,
eliminating ghost walls and deadlocks caused by moving humans and dynamic obstacles.
"""
import time
from typing import Dict, Tuple, List, Optional
import numpy as np
from .grid_core import GridCore
from .anchor_layer import AnchorLayer


class DynamicLayer:
    """Manages rolling short-TTL obstacle cells that evaporate automatically."""

    def __init__(
        self,
        grid_core: GridCore,
        anchor_layer: Optional[AnchorLayer] = None,
        ttl_s: float = 0.35,
        dynamic_ttl: Optional[float] = None,
        low_obs_ttl: float = 0.30
    ):
        self.grid_core = grid_core
        self.anchor_layer = anchor_layer
        self.ttl_s = float(dynamic_ttl if dynamic_ttl is not None else ttl_s)
        self.low_obs_ttl = float(low_obs_ttl)
        # Mapping from (col, row) to expiry timestamp (float)
        self.dynamic_cells: Dict[Tuple[int, int], float] = {}

    def reset(self):
        """Clear all active dynamic cells."""
        self.dynamic_cells.clear()

    def register_hit(self, col: int, row: int, now_t: Optional[float] = None, ttl_s: Optional[float] = None):
        """
        Registers an obstacle return cell with an expiration deadline.
        If cell is part of the locked Anchor Map, it is not marked as dynamic.
        """
        if self.anchor_layer is not None and self.anchor_layer.anchor_mask[row, col]:
            return
        if now_t is None:
            now_t = time.time()
        expiry = now_t + (float(ttl_s) if ttl_s is not None else self.ttl_s)
        self.dynamic_cells[(col, row)] = expiry

    def register_cell(self, col: int, row: int, ttl: Optional[float] = None, now_t: Optional[float] = None):
        """Convenience alias for register_hit."""
        self.register_hit(col, row, now_t=now_t, ttl_s=ttl)

    def is_dynamic_cell(self, col: int, row: int, now_t: Optional[float] = None) -> bool:
        """Returns True if the cell is currently tracked and unexpired."""
        if (col, row) not in self.dynamic_cells:
            return False
        if now_t is None:
            now_t = time.time()
        return self.dynamic_cells[(col, row)] > now_t

    def decay_expired_cells(self, now_t: Optional[float] = None) -> int:
        """
        Evaporates expired dynamic obstacle cells whose TTL deadline has passed.
        Resets cell back to free space (0) unless it belongs to the locked Anchor Map.
        Returns the count of evaporated cells.
        """
        if now_t is None:
            now_t = time.time()

        expired = [cell for cell, expiry in self.dynamic_cells.items() if now_t >= expiry]
        for c, r in expired:
            del self.dynamic_cells[(c, r)]
            if self.anchor_layer is None or not self.anchor_layer.anchor_mask[r, c]:
                if self.grid_core.log_odds[r, c] < 1.5:
                    self.grid_core.costmap[r, c] = 0
                    self.grid_core.log_odds[r, c] = self.grid_core.l_min

        return len(expired)

    def decay(self, now_t: Optional[float] = None) -> int:
        """Convenience alias for decay_expired_cells."""
        return self.decay_expired_cells(now_t=now_t)

    def prune_outside_horizon(self, robot_col: int, robot_row: int, max_cell_dist: int = 120):
        """Optionally prune very distant dynamic cells to prevent unbounded memory growth."""
        if len(self.dynamic_cells) <= 1000:
            return
        prune_keys = []
        for (c, r) in self.dynamic_cells.keys():
            if abs(c - robot_col) > max_cell_dist or abs(r - robot_row) > max_cell_dist:
                prune_keys.append((c, r))
        for k in prune_keys:
            del self.dynamic_cells[k]
