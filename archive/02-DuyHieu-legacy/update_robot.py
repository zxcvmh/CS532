import re

file_path = "/home/jetbot/mhieu/backend/app/robot/mock_robot.py"
with open(file_path, "r") as f:
    code = f.read()

# 1. Add imports if not present
if "from app.planning.occupancy_grid import OccupancyGridMap" not in code:
    import_hook = "from app.config import Config"
    new_imports = """from app.config import Config
from app.planning.occupancy_grid import OccupancyGridMap
from app.planning.astar_planner import AStarPlanner
from app.planning.controller import PurePursuitController"""
    code = code.replace(import_hook, new_imports)

# 2. Add planner & grid_map to __init__
if "self.planner = AStarPlanner" not in code:
    init_hook = "self.angular_vel = 0.0"
    new_init = """self.angular_vel = 0.0
        
        # CS532 Planning & Control Architecture
        self.grid_map = OccupancyGridMap(
            width=Config.MAP_WIDTH,
            height=Config.MAP_HEIGHT,
            resolution=Config.MAP_RESOLUTION,
            origin_x=Config.MAP_ORIGIN_X,
            origin_y=Config.MAP_ORIGIN_Y
        )
        self.planner = AStarPlanner(self.grid_map)
        self.controller = PurePursuitController(
            lookahead_dist=0.30,
            max_linear_speed=Config.MAX_SPEED * 0.6,
            max_angular_speed=Config.MAX_ANGULAR_SPEED,
            track_width=0.115,
            reflex_dist_threshold=0.20
        )
        self.latest_raw_ranges = [0.0] * 360"""
    code = code.replace(init_hook, new_init, 1)

# 3. Replace set_goal with real A*
old_set_goal = """    async def set_goal(self, x: float, y: float):
        if self.mode != ModeEnum.AUTONOMOUS:
            self._add_event("WARNING", "Cannot set goal in MANUAL mode")
            return
        
        self.nav_status.status = NavigationStatusEnum.PLANNING
        self.nav_status.goal_x = x
        self.nav_status.goal_y = y
        self._add_event("INFO", f"Goal received: ({x:.2f}, {y:.2f})")
        
        await asyncio.sleep(0.5)
        
        # Generate planned path with some curvature
        dx = x - self.pose.x
        dy = y - self.pose.y
        dist = math.hypot(dx, dy)
        self._initial_goal_distance = dist
        steps = max(3, int(dist * 3))
        path = []
        for i in range(1, steps + 1):
            t = i / steps
            # Add slight curve for realism
            curve = 0.2 * math.sin(t * math.pi)
            px = self.pose.x + dx * t + curve * (-dy / (dist + 0.001))
            py = self.pose.y + dy * t + curve * (dx / (dist + 0.001))
            path.append(LidarPoint(x=px, y=py))
        self.planned_path.points = path
        
        self.nav_status.status = NavigationStatusEnum.NAVIGATING
        self.nav_status.distance_remaining = dist
        self.nav_status.progress = 0.0
        self._add_event("SUCCESS", "Path planned, navigating...")"""

new_set_goal = """    async def set_goal(self, x: float, y: float):
        if self.mode != ModeEnum.AUTONOMOUS:
            self._add_event("WARNING", "Cannot set goal in MANUAL mode. Switch to AUTONOMOUS!")
            return
        
        self.nav_status.status = NavigationStatusEnum.PLANNING
        self.nav_status.goal_x = x
        self.nav_status.goal_y = y
        self._add_event("INFO", f"Goal received: ({x:.2f}, {y:.2f}). Running A* Search...")
        
        path = self.planner.plan((self.pose.x, self.pose.y), (x, y), smooth=True)
        if path is not None and len(path) >= 2:
            self.planned_path.points = [LidarPoint(x=px, y=py) for px, py in path]
            self.controller.set_path(path)
            self._initial_goal_distance = math.hypot(x - self.pose.x, y - self.pose.y)
            self.nav_status.status = NavigationStatusEnum.NAVIGATING
            self.nav_status.distance_remaining = self._initial_goal_distance
            self.nav_status.progress = 0.0
            self._add_event("SUCCESS", f"A* found safe path ({len(path)} waypoints)! Tracking via Pure Pursuit...")
        else:
            self.nav_status.status = NavigationStatusEnum.BLOCKED
            self.planned_path.points = []
            self.controller.clear_path()
            self._add_event("ERROR", f"A* path blocked by obstacles! Cannot reach ({x:.2f}, {y:.2f})")"""

code = code.replace(old_set_goal, new_set_goal)

with open(file_path, "w") as f:
    f.write(code)

print("SUCCESS: mock_robot.py updated with A* Planner and Log-odds Mapping!")
