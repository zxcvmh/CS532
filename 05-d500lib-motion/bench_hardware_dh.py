#!/usr/bin/env python3
"""
CS532 Autonomous JetBot — Hardware Benchmark & Validation Suite
=============================================================================
Dành riêng cho Team Lead nghiệm thu trên chip ARM Cortex-A57 (NVIDIA Jetson Nano).
Chạy độc lập bằng 1 lệnh duy nhất:
    python3 bench_hardware_dh.py

Đo đạc chính xác thời gian thực thi của 4 module thuật toán then chốt:
  1. OccupancyGridMap.update_scan() [Target <= 30.0ms]
  2. LiDARClusterDetector.cluster_point_cloud() 1D Jump [Target <= 2.5ms]
  3. AStarPlanner.plan() [Target <= 15.0ms]
  4. PurePursuitController.compute_command() [Target <= 5.0ms]
  -> Tổng chu kỳ vòng lặp 10Hz [Target <= 100.0ms (Ngân sách thời gian thực)]
=============================================================================
"""

import os
import sys
import time
import math
import platform
import numpy as np

# Ensure planning package is discoverable
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)
PARENT_DIR = os.path.dirname(SCRIPT_DIR)
if PARENT_DIR not in sys.path:
    sys.path.insert(0, PARENT_DIR)

from planning.occupancy_grid import OccupancyGridMap
from planning.astar_planner import AStarPlanner
from planning.controller import PurePursuitController
from planning.obstacle_clustering import LiDARClusterDetector


def run_benchmark():
    print("=" * 76)
    print("   CS532 AUTONOMOUS JETBOT — BÁO CÁO ĐO ĐẠC HIỆU NĂNG THỜI GIAN THỰC")
    print("=" * 76)

    # 1. System Hardware Profile
    arch = platform.machine()
    system_name = platform.system()
    py_ver = platform.python_version()
    print(f"[*] Nền tảng phần cứng : {arch} ({system_name})")
    print(f"[*] Phiên bản Python   : {py_ver}")
    print(f"[*] Môi trường kiểm thử: {'Jetson Nano ARM Cortex-A57' if 'aarch64' in arch or 'arm' in arch else 'Máy tính cá nhân / PC Simulator'}")
    print("-" * 76)

    results = []

    # -------------------------------------------------------------
    # BENCHMARK 1: Occupancy Grid Update & Ray Clamping
    # -------------------------------------------------------------
    grid = OccupancyGridMap(width=100, height=100, resolution=0.05, origin_x=-2.5, origin_y=-2.5)
    synthetic_ranges = [8.0] * 360
    synthetic_ranges[0] = 1.2
    for a in range(40, 50):
        synthetic_ranges[a] = 0.95

    # Warm-up
    for _ in range(5):
        grid.update_scan(0.0, 0.0, 0.0, synthetic_ranges, max_valid_range=2.4)

    iters_grid = 50
    times_grid = []
    for _ in range(iters_grid):
        t0 = time.perf_counter()
        grid.update_scan(0.0, 0.0, 0.0, synthetic_ranges, max_valid_range=2.4)
        times_grid.append((time.perf_counter() - t0) * 1000.0)

    mean_grid = float(np.mean(times_grid))
    max_grid = float(np.max(times_grid))
    pass_grid = mean_grid <= 30.0
    results.append(("OccupancyGrid.update_scan()", mean_grid, max_grid, 30.0, pass_grid))

    # -------------------------------------------------------------
    # BENCHMARK 2: 1D Range Jump Clustering O(N)
    # -------------------------------------------------------------
    detector = LiDARClusterDetector(corridor_width=0.35, min_range_m=0.12, max_range_m=2.0)
    pts_sample = []
    np.random.seed(123)
    # 2 sample obstacles in field of view
    for ang in range(-20, 21):
        r = 1.0 + float(np.random.normal(0, 0.01))
        rad = math.radians(ang)
        pts_sample.append([r * math.cos(rad), r * math.sin(rad)])
    for ang in range(60, 90):
        r = 1.6 + float(np.random.normal(0, 0.01))
        rad = math.radians(ang)
        pts_sample.append([r * math.cos(rad), r * math.sin(rad)])
    pts_np = np.array(pts_sample, dtype=np.float32)

    # Warm-up
    for _ in range(5):
        _ = detector.cluster_point_cloud(pts_np)

    iters_cluster = 50
    times_cluster = []
    for _ in range(iters_cluster):
        t0 = time.perf_counter()
        clusters = detector.cluster_point_cloud(pts_np)
        times_cluster.append((time.perf_counter() - t0) * 1000.0)

    mean_cluster = float(np.mean(times_cluster))
    max_cluster = float(np.max(times_cluster))
    pass_cluster = mean_cluster <= 2.5
    results.append(("LiDAR 1D Range Jump Cluster", mean_cluster, max_cluster, 2.5, pass_cluster))

    # -------------------------------------------------------------
    # BENCHMARK 3: A* Global Path Planning & Smoothing
    # -------------------------------------------------------------
    planner = AStarPlanner(grid)
    start_pos = (0.0, 0.0)
    goal_pos = (1.8, 0.0)

    # Warm-up
    _ = planner.plan(start_pos, goal_pos, smooth=True)

    iters_astar = 30
    times_astar = []
    for _ in range(iters_astar):
        t0 = time.perf_counter()
        path = planner.plan(start_pos, goal_pos, smooth=True)
        times_astar.append((time.perf_counter() - t0) * 1000.0)

    mean_astar = float(np.mean(times_astar))
    max_astar = float(np.max(times_astar))
    pass_astar = mean_astar <= 15.0
    results.append(("A* Planner & Smoothing", mean_astar, max_astar, 15.0, pass_astar))

    # -------------------------------------------------------------
    # BENCHMARK 4: Pure Pursuit & Curvature Scaling Controller
    # -------------------------------------------------------------
    ctrl = PurePursuitController()
    if path is not None and len(path) >= 2:
        ctrl.set_path(path)
    else:
        ctrl.set_path([(0.0, 0.0), (0.5, 0.0), (1.0, 0.0), (1.8, 0.0)])

    # Warm-up
    _ = ctrl.compute_command(0.1, 0.0, 0.0, lidar_ranges=synthetic_ranges, clusters=clusters, grid_map=grid)

    iters_ctrl = 100
    times_ctrl = []
    for _ in range(iters_ctrl):
        t0 = time.perf_counter()
        _ = ctrl.compute_command(0.1, 0.0, 0.0, lidar_ranges=synthetic_ranges, clusters=clusters, grid_map=grid)
        times_ctrl.append((time.perf_counter() - t0) * 1000.0)

    mean_ctrl = float(np.mean(times_ctrl))
    max_ctrl = float(np.max(times_ctrl))
    pass_ctrl = mean_ctrl <= 5.0
    results.append(("PurePursuit + Curvature Ctrl", mean_ctrl, max_ctrl, 5.0, pass_ctrl))

    # -------------------------------------------------------------
    # BENCHMARK 5: Full 10Hz Closed-Loop Pipeline Execution
    # -------------------------------------------------------------
    times_full = []
    iters_full = 30
    for _ in range(iters_full):
        t0 = time.perf_counter()
        grid.update_scan(0.0, 0.0, 0.0, synthetic_ranges, max_valid_range=2.4)
        c_list = detector.cluster_point_cloud(pts_np)
        _ = ctrl.compute_command(0.05, 0.0, 0.0, lidar_ranges=synthetic_ranges, clusters=c_list, grid_map=grid)
        times_full.append((time.perf_counter() - t0) * 1000.0)

    mean_full = float(np.mean(times_full))
    max_full = float(np.max(times_full))
    pass_full = mean_full <= 50.0  # Well within 100ms 10Hz loop budget
    results.append(("TOTAL 10Hz Pipeline Cycle", mean_full, max_full, 50.0, pass_full))

    # -------------------------------------------------------------
    # PRINT RESULTS TABLE
    # -------------------------------------------------------------
    header = f"{'THUẬT TOÁN / TÁC VỤ':<30} | {'TRUNG BÌNH':<11} | {'MAX':<9} | {'CHỈ TIÊU':<9} | {'KẾT QUẢ'}"
    print(header)
    print("-" * 76)
    for name, mean_t, max_t, target_t, is_pass in results:
        status_str = "PASS ✅" if is_pass else "FAIL ❌"
        row = f"{name:<30} | {mean_t:>7.2f} ms | {max_t:>5.2f} ms | <={target_t:>4.1f}ms | {status_str}"
        print(row)
    print("=" * 76)

    all_passed = all(r[4] for r in results)
    if all_passed:
        print("\n🎉 KẾT QUẢ NGHIỆM THU: ĐẠT 100% TIÊU CHÍ HIỆU NĂNG THỜI GIAN THỰC 10Hz!")
        print("   Hệ thống sẵn sàng vận hành trên robot tự hành JetBot thực tế.\n")
    else:
        print("\n⚠️ CẢNH BÁO: Một số module vượt ngưỡng thời gian thực. Cần kiểm tra lại.\n")

    # Save summary log for Team Lead to easily copy
    log_path = os.path.join(SCRIPT_DIR, "benchmark_results_hardware.log")
    with open(log_path, "w", encoding="utf-8") as f:
        f.write("=== JETBOT HARDWARE BENCHMARK LOG ===\n")
        f.write(f"Architecture: {arch}\n")
        f.write(f"Date: {time.strftime('%Y-%m-%d %H:%M:%S')}\n\n")
        f.write(header + "\n")
        f.write("-" * 76 + "\n")
        for name, mean_t, max_t, target_t, is_pass in results:
            status_str = "PASS" if is_pass else "FAIL"
            f.write(f"{name:<30} | {mean_t:>7.2f} ms | {max_t:>5.2f} ms | <={target_t:>4.1f}ms | {status_str}\n")
        f.write("=" * 76 + "\n")
        f.write(f"Verdict: {'ALL PASS' if all_passed else 'NEEDS ATTENTION'}\n")
    print(f"[*] Báo cáo nghiệm thu đã được lưu vào: {log_path}")


if __name__ == "__main__":
    run_benchmark()
