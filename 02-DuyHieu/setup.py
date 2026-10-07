import os

base_dir = os.path.expanduser("~/DH")
plan_dir = os.path.join(base_dir, "planning")
os.makedirs(plan_dir, exist_ok=True)

files = {}

# 1. __init__.py
files[os.path.join(plan_dir, "__init__.py")] = '''"""
CS532 Planning & Control Package (DH)
"""
try:
    from .lidar_parser import D500Parser
    from .occupancy_grid import OccupancyGridMap
    from .astar_planner import AStarPlanner
    from .controller import PurePursuitController
except ImportError:
    from lidar_parser import D500Parser
    from occupancy_grid import OccupancyGridMap
    from astar_planner import AStarPlanner
    from controller import PurePursuitController

__all__ = ["D500Parser", "OccupancyGridMap", "AStarPlanner", "PurePursuitController"]
'''

# 2. lidar_parser.py
files[os.path.join(plan_dir, "lidar_parser.py")] = '''import time
import math
from typing import List, Tuple, Dict, Optional

class D500Parser:
    def __init__(self):
        self.scan_cache: Dict[int, Tuple[float, int, float]] = {}
        self.latest_ranges = [0.0] * 360
        self.latest_intensities = [0] * 360

    def parse_packet(self, packet: bytes, current_time: Optional[float] = None) -> bool:
        if len(packet) != 47 or packet[0] != 0x54 or packet[1] != 0x2C:
            return False
        if current_time is None:
            current_time = time.time()
        start_angle = (packet[4] | (packet[5] << 8)) / 100.0
        end_angle = (packet[42] | (packet[43] << 8)) / 100.0
        if end_angle < start_angle:
            end_angle += 360.0
        step = (end_angle - start_angle) / 11.0
        for i in range(12):
            offset = 6 + (i * 3)
            dist_mm = packet[offset] | (packet[offset + 1] << 8)
            intensity = packet[offset + 2]
            raw_angle = (start_angle + (i * step)) % 360.0
            dist_m = dist_mm / 1000.0
            angle_ccw = (360 - int(round(raw_angle))) % 360
            if 0.03 <= dist_m <= 12.0:
                self.scan_cache[angle_ccw] = (round(dist_m, 3), intensity, current_time)
        return True

    def get_scan_360(self, max_age_sec: float = 0.5) -> Tuple[List[float], List[int]]:
        now = time.time()
        ranges = [0.0] * 360
        intensities = [0] * 360
        for ang, (dist, inten, ts) in self.scan_cache.items():
            if now - ts <= max_age_sec:
                ranges[ang] = dist
                intensities[ang] = inten
        self.latest_ranges = ranges
        self.latest_intensities = intensities
        return ranges, intensities
'''

# 3. occupancy_grid.py
files[os.path.join(plan_dir, "occupancy_grid.py")] = '''import math
import numpy as np
from typing import List, Tuple, Optional

try:
    import cv2
    HAS_CV2 = True
except ImportError:
    HAS_CV2 = False

class OccupancyGridMap:
    def __init__(self, width: int = 100, height: int = 100, resolution: float = 0.05, origin_x: float = -2.5, origin_y: float = -2.5):
        self.width = width
        self.height = height
        self.resolution = resolution
        self.origin_x = origin_x
        self.origin_y = origin_y
        self.L_FREE = -0.85
        self.L_OCC = 1.38
        self.L_MIN = -5.0
        self.L_MAX = 5.0
        self.log_odds = np.zeros((height, width), dtype=np.float32)
        self.robot_radius = 0.15
        self.inflation_radius = 0.25
        self.inflation_cells = max(2, int(round(self.inflation_radius / self.resolution)))
        self._inflation_mask = self._create_circular_mask(self.inflation_cells)
        self.costmap = np.zeros((height, width), dtype=np.uint8)

    def _create_circular_mask(self, radius_cells: int) -> np.ndarray:
        y, x = np.ogrid[-radius_cells:radius_cells + 1, -radius_cells:radius_cells + 1]
        return (x * x + y * y) <= radius_cells * radius_cells

    def world_to_grid(self, wx: float, wy: float) -> Tuple[int, int]:
        col = int(math.floor((wx - self.origin_x) / self.resolution))
        row = int(math.floor((wy - self.origin_y) / self.resolution))
        return col, row

    def grid_to_world(self, col: int, row: int) -> Tuple[float, float]:
        wx = self.origin_x + (col + 0.5) * self.resolution
        wy = self.origin_y + (row + 0.5) * self.resolution
        return wx, wy

    def is_inside(self, col: int, row: int) -> bool:
        return 0 <= col < self.width and 0 <= row < self.height

    @staticmethod
    def bresenham_line(x0: int, y0: int, x1: int, y1: int) -> List[Tuple[int, int]]:
        points = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        curr_x, curr_y = x0, y0
        while True:
            points.append((curr_x, curr_y))
            if curr_x == x1 and curr_y == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                curr_x += sx
            if e2 < dx:
                err += dx
                curr_y += sy
        return points

    def update_scan(self, robot_x: float, robot_y: float, robot_theta: float, ranges: List[float], max_valid_range: float = 8.0, min_valid_range: float = 0.12):
        r_col, r_row = self.world_to_grid(robot_x, robot_y)
        if not self.is_inside(r_col, r_row):
            return
        num_points = len(ranges)
        step_rad = (2.0 * math.pi) / num_points
        for i, dist in enumerate(ranges):
            if dist < min_valid_range or math.isnan(dist):
                continue
            angle = robot_theta + (i * step_rad)
            is_obstacle = (dist <= max_valid_range)
            clamped_dist = min(dist, max_valid_range)
            hit_x = robot_x + clamped_dist * math.cos(angle)
            hit_y = robot_y + clamped_dist * math.sin(angle)
            h_col, h_row = self.world_to_grid(hit_x, hit_y)
            line = self.bresenham_line(r_col, r_row, h_col, h_row)
            for col, row in line[:-1]:
                if self.is_inside(col, row):
                    self.log_odds[row, col] = max(self.L_MIN, self.log_odds[row, col] + self.L_FREE)
            if is_obstacle and self.is_inside(h_col, h_row):
                self.log_odds[h_row, h_col] = min(self.L_MAX, self.log_odds[h_row, h_col] + self.L_OCC)
        self._update_costmap()

    def _update_costmap(self):
        occ_mask = self.log_odds > 0.62
        self.costmap.fill(0)
        self.costmap[occ_mask] = 254
        if np.any(occ_mask) and HAS_CV2:
            kernel = self._inflation_mask.astype(np.uint8)
            inflated = cv2.dilate(occ_mask.astype(np.uint8), kernel).astype(bool)
            self.costmap[inflated & (~occ_mask)] = 200

    def to_dashboard_data(self) -> List[int]:
        flat_data = []
        for row in range(self.height):
            for col in range(self.width):
                lo = self.log_odds[row, col]
                if abs(lo) < 1e-4:
                    val = -1
                elif self.costmap[row, col] >= 200 or lo > 0.62:
                    val = 100
                else:
                    val = 0
                flat_data.append(val)
        return flat_data

    def is_cell_free(self, col: int, row: int, check_inflated: bool = True) -> bool:
        if not self.is_inside(col, row):
            return False
        if check_inflated:
            return self.costmap[row, col] < 200
        return self.log_odds[row, col] <= 0.62

    def reset(self):
        self.log_odds.fill(0.0)
        self.costmap.fill(0)
'''

# 4. astar_planner.py
files[os.path.join(plan_dir, "astar_planner.py")] = '''import math
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
        if not self.grid_map.is_inside(start_c, start_r):
            return None
        goal_cell = self.find_nearest_free_cell(raw_gc, raw_gr)
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
                tentative_g = current.g + move_cost
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
'''

# 5. controller.py
files[os.path.join(plan_dir, "controller.py")] = '''import math
from typing import List, Tuple, Optional, Dict, Any

class PurePursuitController:
    def __init__(self, lookahead_dist: float = 0.30, max_linear_speed: float = 0.25, max_angular_speed: float = 1.0, track_width: float = 0.115, goal_tolerance: float = 0.12, reflex_dist_threshold: float = 0.20):
        self.lookahead_dist = lookahead_dist
        self.max_linear_speed = max_linear_speed
        self.max_angular_speed = max_angular_speed
        self.track_width = track_width
        self.goal_tolerance = goal_tolerance
        self.reflex_dist_threshold = reflex_dist_threshold
        self.current_path: List[Tuple[float, float]] = []
        self.path_index: int = 0
        self.is_goal_reached: bool = False
        self.is_reflex_stopped: bool = False

    def set_path(self, path: List[Tuple[float, float]]):
        self.current_path = path
        self.path_index = 0
        self.is_goal_reached = False
        self.is_reflex_stopped = False

    def clear_path(self):
        self.current_path = []
        self.path_index = 0
        self.is_goal_reached = False

    def check_safety_reflex(self, ranges: List[float]) -> bool:
        if not ranges or len(ranges) < 360:
            return False
        frontal_indices = list(range(0, 36)) + list(range(325, 360))
        for idx in frontal_indices:
            r = ranges[idx]
            if 0.05 < r < self.reflex_dist_threshold:
                self.is_reflex_stopped = True
                return True
        self.is_reflex_stopped = False
        return False

    def compute_command(self, robot_x: float, robot_y: float, robot_theta: float, lidar_ranges: Optional[List[float]] = None) -> Tuple[float, float, Dict[str, Any]]:
        if lidar_ranges and self.check_safety_reflex(lidar_ranges):
            return 0.0, 0.0, {"status": "EMERGENCY_STOP", "reason": "Obstacle < 20cm in front cone!", "distance_remaining": 0.0}
        if not self.current_path:
            return 0.0, 0.0, {"status": "IDLE", "distance_remaining": 0.0}
        goal_x, goal_y = self.current_path[-1]
        dist_to_final_goal = math.hypot(goal_x - robot_x, goal_y - robot_y)
        if dist_to_final_goal <= self.goal_tolerance:
            self.is_goal_reached = True
            self.clear_path()
            return 0.0, 0.0, {"status": "GOAL_REACHED", "distance_remaining": 0.0}
        while self.path_index < len(self.current_path) - 1:
            curr_pt = self.current_path[self.path_index]
            next_pt = self.current_path[self.path_index + 1]
            if math.hypot(next_pt[0] - robot_x, next_pt[1] - robot_y) < math.hypot(curr_pt[0] - robot_x, curr_pt[1] - robot_y):
                self.path_index += 1
            else:
                break
        target_pt = self.current_path[-1]
        for i in range(self.path_index, len(self.current_path)):
            if math.hypot(self.current_path[i][0] - robot_x, self.current_path[i][1] - robot_y) >= self.lookahead_dist:
                target_pt = self.current_path[i]
                break
        dx = target_pt[0] - robot_x
        dy = target_pt[1] - robot_y
        alpha = (math.atan2(dy, dx) - robot_theta + math.pi) % (2.0 * math.pi) - math.pi
        if abs(alpha) > 0.65:
            v = 0.03
            w = math.copysign(self.max_angular_speed * 0.8, alpha)
        else:
            kappa = (2.0 * math.sin(alpha)) / self.lookahead_dist
            v = min(self.max_linear_speed, max(0.08, dist_to_final_goal * 0.6)) * max(0.2, math.cos(alpha))
            w = max(-self.max_angular_speed, min(self.max_angular_speed, v * kappa))
        return round(v, 3), round(w, 3), {"status": "NAVIGATING", "distance_remaining": round(dist_to_final_goal, 3)}
'''

# 6. test_planning.py
files[os.path.join(base_dir, "test_planning.py")] = '''import os, sys, time, math
import numpy as np

sys.path.insert(0, os.path.expanduser("~/DH"))
from planning.occupancy_grid import OccupancyGridMap
from planning.astar_planner import AStarPlanner
from planning.controller import PurePursuitController
from planning.lidar_parser import D500Parser

print("=================================================================")
print("     CS532 - KIEM TRA PLANNING & CONTROL AGENT (THUMUC ~/DH)")
print("=================================================================")

# 1. Test Grid
grid = OccupancyGridMap(width=100, height=100, resolution=0.05, origin_x=-2.5, origin_y=-2.5)
ranges = [3.0] * 360
ranges[0] = 1.5
t0 = time.time()
grid.update_scan(0.0, 0.0, 0.0, ranges)
dt = (time.time() - t0) * 1000.0
cw, rw = grid.world_to_grid(1.5, 0.0)
assert grid.log_odds[rw, cw] > 0.5
print(f"[TEST 1/4] Occupancy Grid Log-Odds & Costmap: PASS (Update time: {dt:.2f} ms)")

# 2. Test A*
for y in np.linspace(-0.6, 0.6, 25):
    c, r = grid.world_to_grid(1.0, y)
    grid.log_odds[r, c] = 3.0
    grid.costmap[r, c] = 254
planner = AStarPlanner(grid)
t0 = time.time()
path = planner.plan((0.0, 0.0), (1.8, 0.0), smooth=True)
dt = (time.time() - t0) * 1000.0
assert path is not None and len(path) >= 2
print(f"[TEST 2/4] Thuat toan A* Search & Smoothing: PASS ({len(path)} waypoints, Latency: {dt:.2f} ms)")

# 3. Test Pure Pursuit & Reflex
ctrl = PurePursuitController()
ctrl.set_path(path)
v, w, info = ctrl.compute_command(0.0, 0.0, 0.0)
assert v > 0.05
v_s, w_s, info_s = ctrl.compute_command(0.0, 0.0, 0.0, lidar_ranges=[0.15]*360)
assert v_s == 0.0 and w_s == 0.0 and info_s["status"] == "EMERGENCY_STOP"
print("[TEST 3/4] Pure Pursuit & Phanh phan xa 10Hz (<20cm): PASS")

# 4. Test Parser
parser = D500Parser()
pkt = bytearray([0x54, 0x2C, 0x00, 0x08, 0x00, 0x00]) + bytearray([0xE2, 0x04, 200]*12) + bytearray([0xB8, 0x0B, 0x00, 0x00, 0x00])
assert parser.parse_packet(bytes(pkt))
r, i = parser.get_scan_360()
assert len(r) == 360
print("[TEST 4/4] Parser LiDAR D500 (360 ranges): PASS")

print("=================================================================")
print("🎉 TOAN BO 4 MODULE DA KHOI TAO THANH CONG TAI ~/DH!")
print("=================================================================")
'''

for p, code in files.items():
    with open(p, "w") as f:
        f.write(code)
    print(f"Generated: {p}")

print("\nCopying planning module to ~/mhieu/backend/app/planning...")
os.system("cp -r ~/DH/planning ~/mhieu/backend/app/")
print("Done! Ready to run.")
