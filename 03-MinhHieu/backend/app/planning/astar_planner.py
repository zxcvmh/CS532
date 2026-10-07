import math
import heapq
from typing import List, Tuple, Optional

try:
    from .occupancy_grid import OccupancyGridMap
except ImportError:
    from occupancy_grid import OccupancyGridMap

class Node:
    __slots__ = ('col', 'row', 'g', 'h', 'f', 'parent')
    def __init__(self, col: int, row: int, g: float, h: float, parent: Optional['Node'] = None):
        self.col, self.row, self.g, self.h = col, row, g, h
        self.f = g + h
        self.parent = parent
    def __lt__(self, other: 'Node') -> bool:
        return self.f < other.f

class AStarPlanner:
    def __init__(self, grid_map: OccupancyGridMap):
        self.grid_map = grid_map
        self.motions = [
            ( 1,  0, 1.0), (-1,  0, 1.0), ( 0,  1, 1.0), ( 0, -1, 1.0),
            ( 1,  1, 1.414), ( 1, -1, 1.414), (-1,  1, 1.414), (-1, -1, 1.414)
        ]

    def _heuristic(self, c1: int, r1: int, c2: int, r2: int) -> float:
        return math.hypot(c1 - c2, r1 - r2) * 1.001

    def find_nearest_free_cell(self, target_c: int, target_r: int, max_radius: int = 15) -> Optional[Tuple[int, int]]:
        if self.grid_map.is_cell_free(target_c, target_r):
            return target_c, target_r
        for rad in range(1, max_radius + 1):
            for dc in range(-rad, rad + 1):
                for dr in range(-rad, rad + 1):
                    if abs(dc) == rad or abs(dr) == rad:
                        nc, nr = target_c + dc, target_r + dr
                        if self.grid_map.is_cell_free(nc, nr):
                            return nc, nr
        return None

    def plan(self, start_world: Tuple[float, float], goal_world: Tuple[float, float], smooth: bool = True) -> Optional[List[Tuple[float, float]]]:
        start_c, start_r = self.grid_map.world_to_grid(*start_world)
        raw_gc, raw_gr = self.grid_map.world_to_grid(*goal_world)
        # 1. Clamp start and goal to valid grid dimensions (Robust Clamping from Duy Hieu)
        pad = 1
        clamped_start_c = max(pad, min(self.grid_map.width - 1 - pad, start_c))
        clamped_start_r = max(pad, min(self.grid_map.height - 1 - pad, start_r))

        clamped_goal_c = max(pad, min(self.grid_map.width - 1 - pad, raw_gc))
        clamped_goal_r = max(pad, min(self.grid_map.height - 1 - pad, raw_gr))

        # 2. Resolve start cell if robot is parked inside obstacle or inflation
        start_cell = self.find_nearest_free_cell(clamped_start_c, clamped_start_r, max_radius=15)
        if start_cell is not None:
            start_c, start_r = start_cell
        else:
            start_c, start_r = clamped_start_c, clamped_start_r

        # 3. Robust goal resolution: handles out-of-bounds clicks or obstacles
        goal_cell = self.find_nearest_free_cell(clamped_goal_c, clamped_goal_r, max_radius=20)
        if goal_cell is None:
            # Raycast from clamped goal straight back to start to find first reachable free cell
            line = self.grid_map.bresenham_line(clamped_goal_c, clamped_goal_r, start_c, start_r)
            for c, r in line:
                if self.grid_map.is_cell_free(c, r):
                    goal_cell = (c, r)
                    break

        if goal_cell is None:
            return None
        goal_c, goal_r = goal_cell
        open_set: List[Node] = []
        open_dict = {}
        closed_set = set()
        start_node = Node(start_c, start_r, 0.0, self._heuristic(start_c, start_r, goal_c, goal_r))
        heapq.heappush(open_set, start_node)
        open_dict[(start_c, start_r)] = start_node
        goal_node = None
        while open_set:
            current = heapq.heappop(open_set)
            pos = (current.col, current.row)
            if pos in closed_set:
                continue
            closed_set.add(pos)
            if pos in open_dict:
                del open_dict[pos]
            if current.col == goal_c and current.row == goal_r:
                goal_node = current
                break
            for dc, dr, move_cost in self.motions:
                nc, nr = current.col + dc, current.row + dr
                npos = (nc, nr)
                if npos in closed_set or not self.grid_map.is_cell_free(nc, nr):
                    continue
                if dc != 0 and dr != 0:
                    if not self.grid_map.is_cell_free(current.col + dc, current.row) or not self.grid_map.is_cell_free(current.col, current.row + dr):
                        continue
                # Costmap penalty: prefer paths with low cost (far from obstacles and humans)
                cost_penalty = (float(self.grid_map.costmap[nr, nc]) / 255.0) * 3.0
                tentative_g = current.g + move_cost + cost_penalty
                if npos in open_dict:
                    existing = open_dict[npos]
                    if tentative_g < existing.g:
                        existing.g = tentative_g
                        existing.f = tentative_g + existing.h
                        existing.parent = current
                        heapq.heapify(open_set)
                else:
                    h = self._heuristic(nc, nr, goal_c, goal_r)
                    neighbor = Node(nc, nr, tentative_g, h, parent=current)
                    heapq.heappush(open_set, neighbor)
                    open_dict[npos] = neighbor
        if goal_node is None:
            return None
        grid_path = []
        curr = goal_node
        while curr:
            grid_path.append((curr.col, curr.row))
            curr = curr.parent
        grid_path.reverse()
        world_path = [self.grid_map.grid_to_world(c, r) for c, r in grid_path]
        if smooth and len(world_path) > 2:
            world_path = self.smooth_path(world_path)
        return world_path

    def has_line_of_sight(self, p1: Tuple[float, float], p2: Tuple[float, float]) -> bool:
        c0, r0 = self.grid_map.world_to_grid(*p1)
        c1, r1 = self.grid_map.world_to_grid(*p2)
        line = self.grid_map.bresenham_line(c0, r0, c1, r1)
        return all(self.grid_map.is_cell_free(c, r) for c, r in line)

    def smooth_path(self, path: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        if len(path) <= 2:
            return path
        smoothed = [path[0]]
        curr_idx = 0
        while curr_idx < len(path) - 1:
            furthest = curr_idx + 1
            for next_idx in range(curr_idx + 2, len(path)):
                if self.has_line_of_sight(path[curr_idx], path[next_idx]):
                    furthest = next_idx
                else:
                    break
            smoothed.append(path[furthest])
            curr_idx = furthest
        return smoothed
