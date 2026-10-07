import os, sys, time, math
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
