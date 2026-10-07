#!/usr/bin/env python3
"""
Bộ Kiểm Thử Thuật Toán Điều Hướng Trên Máy Tính Cá Nhân (Duy Hiếu)
===================================================================
Tự sinh dữ liệu quét LiDAR giả lập (Synthetic LaserScan) để kiểm thử
thuật toán A*, Occupancy Grid, Gom cụm 1D và Pure Pursuit mà KHÔNG CẦN ROBOT THẬT.
"""

import os
import sys
import time
import math
import numpy as np

# Thêm đường dẫn thư mục hiện tại vào sys.path
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, BASE_DIR)

from planning.occupancy_grid import OccupancyGridMap
from planning.astar_planner import AStarPlanner
from planning.controller import PurePursuitController
from planning.obstacle_clustering import LiDARClusterDetector

def test_1_grid_update_latency():
    print("\n--- [TEST 1/4] Kiểm tra Độ trễ Cập nhật Occupancy Grid (Ray Clamping) ---")
    grid = OccupancyGridMap(width=100, height=100, resolution=0.05, origin_x=-2.5, origin_y=-2.5)
    # Giả lập phòng rộng 8m (tia quét vượt ra ngoài biên bản đồ 5m x 5m)
    synthetic_ranges = [8.0] * 360
    synthetic_ranges[0] = 1.2  # Vật cản ở góc 0 độ cự ly 1.2m

    t0 = time.time()
    loops = 10
    for _ in range(loops):
        grid.update_scan(0.0, 0.0, 0.0, synthetic_ranges, max_valid_range=2.4)
    avg_ms = ((time.time() - t0) / loops) * 1000.0
    print(f"-> Thời gian cập nhật trung bình trên PC: {avg_ms:.2f} ms")
    if avg_ms > 25.0:
        print("⚠️ CẢNH BÁO: Thuật toán cần tối ưu hơn để đạt < 30ms trên chip ARM Jetson Nano!")
    else:
        print("✅ PASS: Đạt chỉ tiêu độ trễ!")
    return grid

def test_2_astar_path_planning(grid):
    print("\n--- [TEST 2/4] Kiểm tra Tìm đường A* và Làm mượt quỹ đạo ---")
    planner = AStarPlanner(grid)
    start_pos = (0.0, 0.0)
    goal_pos = (1.8, 0.0)
    
    t0 = time.time()
    path = planner.plan(start_pos, goal_pos, smooth=True)
    dt_ms = (time.time() - t0) * 1000.0
    
    assert path is not None and len(path) >= 2, "❌ A* không tìm được đường đi!"
    print(f"-> A* tìm thấy {len(path)} waypoints trong {dt_ms:.2f} ms")
    print(f"-> Điểm xuất phát: {path[0]} -> Đích đến: {path[-1]}")
    print("✅ PASS: Thuật toán A* hội tụ thành công!")
    return path

def test_3_detour_clearance(path):
    print("\n--- [TEST 3/4] Kiểm tra Quỹ đạo Né Tránh Hình thang (Khoảng hở an toàn) ---")
    ctrl = PurePursuitController()
    ctrl.set_path(path)
    
    # Giả lập vật cản xuất hiện chắn đường tại (1.0m, 0.0m)
    blocked_pt = (1.0, 0.0)
    if hasattr(ctrl, "generate_detour_splice"):
        detour = ctrl.generate_detour_splice(0.2, 0.0, 0.0, blocked_pt=blocked_pt, lateral_offset=0.52)
        if detour:
            min_dist_to_obs = min(math.hypot(x - blocked_pt[0], y - blocked_pt[1]) for x, y in detour)
            clearance_margin = min_dist_to_obs - 0.12  # Trừ bán kính robot 0.12m
            print(f"-> Khoảng hở mép xe tới vật cản: {clearance_margin:.2f} m")
            if clearance_margin >= 0.22:
                print("✅ PASS: Khoảng hở an toàn >= 0.22m, không bị chém mép hay chạm E-stop!")
            else:
                print("⚠️ CẢNH BÁO: Khoảng hở < 0.22m, cần tăng lateral_offset hoặc kéo dài plateau hold!")
        else:
            print("⚠️ generate_detour_splice trả về None!")
    else:
        print("⚠️ Chưa tìm thấy hàm generate_detour_splice trong controller.py!")

def test_4_curvature_velocity_scaling():
    print("\n--- [TEST 4/4] Kiểm tra Bộ Điều Tốc Vào Cua (Curvature-Aware Scaling) ---")
    ctrl = PurePursuitController(max_linear_speed=0.25)
    
    # 1. Đường thẳng
    ctrl.set_path([(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)])
    v_straight, w_straight, _ = ctrl.compute_command(0.0, 0.0, 0.0)
    
    # 2. Vào cua gắt vuông góc 90 độ
    ctrl.set_path([(0.0, 0.0), (0.0, 1.0)])
    v_curve, w_curve, _ = ctrl.compute_command(0.0, 0.0, 0.0)
    
    print(f"-> Vận tốc đường thẳng: v = {v_straight} m/s, w = {w_straight} rad/s")
    print(f"-> Vận tốc khi ôm cua gắt: v = {v_curve} m/s, w = {w_curve} rad/s")
    
    if v_curve < v_straight:
        print("✅ PASS: Xe tự động hãm tốc khi bẻ lái góc lớn chống trượt lật bánh vi sai!")
    else:
        print("ℹ️ GỢI Ý: Hãy tích hợp công thức v = v_base / (1 + k * |w|) vào compute_command()!")

if __name__ == "__main__":
    print("=" * 65)
    print("   CS532 - BỘ KIỂM THỬ THUẬT TOÁN ĐIỀU HƯỚNG TRÊN PC (DUY HIẾU)")
    print("=" * 65)
    try:
        grid = test_1_grid_update_latency()
        path = test_2_astar_path_planning(grid)
        test_3_detour_clearance(path)
        test_4_curvature_velocity_scaling()
        print("\n🎉 KIỂM THỬ HOÀN TẤT TRÊN MÁY TÍNH CÁ NHÂN!")
    except Exception as e:
        print(f"\n❌ LỖI TRONG QUÁ TRÌNH KIỂM THỬ: {e}")
        import traceback
        traceback.print_exc()
