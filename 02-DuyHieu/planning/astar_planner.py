import math
import heapq
import time
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
        # 8-connected grid motion model with exact Euclidean sqrt(2) costs
        sqrt2 = math.sqrt(2.0)
        self.motions = [
            ( 1,  0, 1.0), (-1,  0, 1.0), ( 0,  1, 1.0), ( 0, -1, 1.0),
            ( 1,  1, sqrt2), ( 1, -1, sqrt2), (-1,  1, sqrt2), (-1, -1, sqrt2)
        ]

    def _heuristic(self, c1: int, r1: int, c2: int, r2: int, weight: float = 1.0) -> float:
        # Strictly admissible and consistent octile distance metric (h(n) <= c(n, n'))
        dx = abs(c1 - c2)
        dy = abs(r1 - r2)
        return (max(dx, dy) + (math.sqrt(2.0) - 1.0) * min(dx, dy)) * weight

    def find_nearest_free_cell(self, target_c: int, target_r: int, max_radius: int = 15) -> Optional[Tuple[int, int]]:
        if self.grid_map.is_inside(target_c, target_r) and self.grid_map.is_cell_free(target_c, target_r):
            return target_c, target_r
        for rad in range(1, max_radius + 1):
            candidates = []
            for dc in range(-rad, rad + 1):
                for dr in range(-rad, rad + 1):
                    if abs(dc) == rad or abs(dr) == rad:
                        nc, nr = target_c + dc, target_r + dr
                        if self.grid_map.is_inside(nc, nr) and self.grid_map.is_cell_free(nc, nr):
                            d_sq = dc * dc + dr * dr
                            cost = float(self.grid_map.costmap[nr, nc])
                            candidates.append((d_sq, cost, (nc, nr)))
            if candidates:
                candidates.sort(key=lambda item: (item[0], item[1]))
                return candidates[0][2]
        return None

    def plan(
        self,
        start_world: Tuple[float, float],
        goal_world: Tuple[float, float],
        smooth: bool = True,
        max_time_sec: float = 10.0,
        max_expansions: int = 25000,
        robot_theta: Optional[float] = None
    ) -> Optional[List[Tuple[float, float]]]:
        raw_sc, raw_sr = self.grid_map.world_to_grid(*start_world)
        raw_gc, raw_gr = self.grid_map.world_to_grid(*goal_world)

        # 1. Clamp start and goal to valid grid dimensions
        pad = 1
        clamped_start_c = max(pad, min(self.grid_map.width - 1 - pad, raw_sc))
        clamped_start_r = max(pad, min(self.grid_map.height - 1 - pad, raw_sr))
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

        width, height = self.grid_map.width, self.grid_map.height
        INF = 1e9
        g_score = [INF] * (width * height)
        start_idx = start_r * width + start_c
        g_score[start_idx] = 0.0

        h0 = self._heuristic(start_c, start_r, goal_c, goal_r)
        open_heap = [(h0, 0.0, start_c, start_r)]
        parent = {}
        found = False

        costmap_flat = self.grid_map.costmap.ravel()
        t_start = time.time()
        expansions = 0

        while open_heap:
            expansions += 1
            if expansions % 400 == 0:
                if (time.time() - t_start > max_time_sec) or (expansions > max_expansions):
                    # Time budget exceeded, cleanly abort
                    break

            f, g, c, r = heapq.heappop(open_heap)
            idx = r * width + c
            if g > g_score[idx]:
                continue
            if c == goal_c and r == goal_r:
                found = True
                break

            for dc, dr, move_cost in self.motions:
                nc, nr = c + dc, r + dr
                if not (0 <= nc < width and 0 <= nr < height):
                    continue
                nidx = nr * width + nc
                val = costmap_flat[nidx]
                if val >= 250:  # impassable lethal obstacle cell
                    continue
                if dc != 0 and dr != 0:
                    # Diagonal squeeze: prevent cutting between two diagonally touching lethal obstacles
                    if costmap_flat[r * width + nc] >= 250 and costmap_flat[nr * width + c] >= 250:
                        continue

                # 1. Continuous Costmap Penalty (scales smoothly with inflation gradient)
                # Prefers center of wide open corridors over hugging obstacles
                cost_penalty = (float(val) / 255.0) * 1.5

                tentative_g = g + move_cost + cost_penalty
                if tentative_g < g_score[nidx]:
                    g_score[nidx] = tentative_g
                    parent[(nc, nr)] = (c, r)
                    nh = self._heuristic(nc, nr, goal_c, goal_r)
                    heapq.heappush(open_heap, (tentative_g + nh, tentative_g, nc, nr))

        if not found:
            return None

        curr = (goal_c, goal_r)
        grid_path = []
        while curr in parent:
            grid_path.append(curr)
            curr = parent[curr]
        grid_path.append((start_c, start_r))
        grid_path.reverse()

        world_path = [self.grid_map.grid_to_world(c, r) for c, r in grid_path]
        if world_path:
            # Ensure path smoothly anchors to actual robot start position
            world_path[0] = (round(start_world[0], 3), round(start_world[1], 3))

        if smooth and len(world_path) > 2:
            world_path = self.smooth_path(world_path)
        return world_path

    def has_line_of_sight(self, p1: Tuple[float, float], p2: Tuple[float, float], max_cost: int = 240) -> bool:
        c0, r0 = self.grid_map.world_to_grid(*p1)
        c1, r1 = self.grid_map.world_to_grid(*p2)
        dx = abs(c1 - c0)
        dy = abs(r1 - r0)
        sx = 1 if c0 < c1 else -1
        sy = 1 if r0 < r1 else -1
        err = dx - dy
        cx, cy = c0, r0
        w = self.grid_map.width
        h = self.grid_map.height
        costmap_flat = self.grid_map.costmap.ravel()

        while True:
            if not (0 <= cx < w and 0 <= cy < h):
                return False
            if costmap_flat[cy * w + cx] >= max_cost:
                return False
            if cx == c1 and cy == r1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                cx += sx
            if e2 < dx:
                err += dx
                cy += sy
        return True

    def smooth_path(self, path: List[Tuple[float, float]], max_cost: int = 230) -> List[Tuple[float, float]]:
        if len(path) <= 2:
            return path
        # Downsample candidate checkpoints to avoid O(N^2) ray checks
        sampled = [path[i] for i in range(0, len(path), 2)]
        if sampled[-1] != path[-1]:
            sampled.append(path[-1])

        smoothed = [sampled[0]]
        curr_idx = 0
        n = len(sampled)
        while curr_idx < n - 1:
            furthest_idx = curr_idx + 1
            for next_idx in range(n - 1, curr_idx, -1):
                # Allow bounded clearance tolerance (max_cost=210, ~11cm margin) when connecting directly to the goal point
                cost_limit = 210 if (next_idx == n - 1) else max_cost
                if self.has_line_of_sight(sampled[curr_idx], sampled[next_idx], max_cost=cost_limit):
                    furthest_idx = next_idx
                    break
            smoothed.append(sampled[furthest_idx])
            curr_idx = furthest_idx
        return smoothed
