import re
import os

print("=" * 65)
print("  NÂNG CẤP BỘ ĐIỀU HƯỚNG A* BẤT TỬ (ROBUST GOAL CLAMPING)")
print("=" * 65)

# 1. Update astar_planner.py with clamping & raycast fallback
for base_path in ["/home/jetbot/DH/planning", "/home/jetbot/mhieu/backend/app/planning"]:
    astar_file = os.path.join(base_path, "astar_planner.py")
    if not os.path.exists(astar_file):
        continue
    
    with open(astar_file, "r") as f:
        code = f.read()

    old_target = """        # Check start inside map
        if not self.grid_map.is_inside(start_c, start_r):
            return None

        # Resolve goal if inside obstacle
        goal_cell = self.find_nearest_free_cell(raw_goal_c, raw_goal_r)
        if goal_cell is None:
            return None
        goal_c, goal_r = goal_cell"""

    new_target = """        # 1. Clamp start and goal to valid grid dimensions
        pad = 1
        clamped_start_c = max(pad, min(self.grid_map.width - 1 - pad, start_c))
        clamped_start_r = max(pad, min(self.grid_map.height - 1 - pad, start_r))
        
        clamped_goal_c = max(pad, min(self.grid_map.width - 1 - pad, raw_goal_c))
        clamped_goal_r = max(pad, min(self.grid_map.height - 1 - pad, raw_goal_r))

        # 2. Resolve start cell if robot is parked inside obstacle or inflation
        start_cell = self.find_nearest_free_cell(clamped_start_c, clamped_start_r, max_search_radius=15)
        if start_cell is not None:
            start_c, start_r = start_cell
        else:
            start_c, start_r = clamped_start_c, clamped_start_r

        # 3. Robust goal resolution: handles out-of-bounds clicks or obstacles
        goal_cell = self.find_nearest_free_cell(clamped_goal_c, clamped_goal_r, max_search_radius=20)
        if goal_cell is None:
            # Raycast from clamped goal straight back to start to find first reachable free cell
            line = self.grid_map.bresenham_line(clamped_goal_c, clamped_goal_r, start_c, start_r)
            for c, r in line:
                if self.grid_map.is_cell_free(c, r):
                    goal_cell = (c, r)
                    break

        if goal_cell is None:
            return None
        goal_c, goal_r = goal_cell"""

    if old_target in code:
        code = code.replace(old_target, new_target)
        with open(astar_file, "w") as f:
            f.write(code)
        print(f"[OK] Đã tích hợp Robust Clamping vào {astar_file}")

# 2. Mở rộng kích thước Map lên 10m x 10m (200x200 cells)
cfg_path = "/home/jetbot/mhieu/backend/app/config.py"
if os.path.exists(cfg_path):
    with open(cfg_path, "r") as f:
        code = f.read()
    code = re.sub(r'MAP_WIDTH\s*=\s*\d+', 'MAP_WIDTH = 200', code)
    code = re.sub(r'MAP_HEIGHT\s*=\s*\d+', 'MAP_HEIGHT = 200', code)
    code = re.sub(r'MAP_ORIGIN_X\s*=\s*-?[\d\.]+', 'MAP_ORIGIN_X = -5.0', code)
    code = re.sub(r'MAP_ORIGIN_Y\s*=\s*-?[\d\.]+', 'MAP_ORIGIN_Y = -5.0', code)
    with open(cfg_path, "w") as f:
        f.write(code)
    print("[OK] Đã mở rộng không gian Map lên 10m x 10m (click thoải mái)")

print("\n🎉 HOÀN TẤT NÂNG CẤP!")
