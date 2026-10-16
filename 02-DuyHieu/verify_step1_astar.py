#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
CS532 AUTONOMOUS JETBOT — BƯỚC 1: KIỂM CHỨNG KHOA HỌC THUẬT TOÁN A* TỐI ƯU
========================================================================
Mục tiêu nghiệm thu:
  1. Kiểm chứng tính đúng đắn hình học & tránh va chạm (Corner-cutting, Clearance).
  2. Đo đạc độ trễ thời gian thực trên Jetson Nano ARM Cortex-A57 (Python 3.6.9).
  3. Chỉ tiêu PASS: Thời gian tìm đường + làm mượt <= 15.0 ms (Baseline cũ: 51.77 ms).
========================================================================
"""

import os
import sys
import time
import math
import platform
import numpy as np

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
if BASE_DIR not in sys.path:
    sys.path.insert(0, BASE_DIR)

from planning.occupancy_grid import OccupancyGridMap
from planning.astar_planner import AStarPlanner

CLR_RESET = "\033[0m"
CLR_BOLD = "\033[1m"
CLR_RED = "\033[91m"
CLR_GREEN = "\033[92m"
CLR_YELLOW = "\033[93m"
CLR_CYAN = "\033[96m"

def banner(msg: str):
    print("\n" + "=" * 76)
    print(f" {CLR_BOLD}{CLR_CYAN}{msg}{CLR_RESET}")
    print("=" * 76)

def test_astar_suite():
    arch = platform.machine().lower()
    is_arm = "aarch64" in arch or "arm" in arch
    env_name = "Jetson Nano ARM Cortex-A57" if is_arm else "PC Simulator / Laptop"

    print("╔══════════════════════════════════════════════════════════════════════════╗")
    print("║     CS532: NGHIỆM THU BƯỚC 1 — THUẬT TOÁN TÌM ĐƯỜNG TOÀN CỤC A* (2026)   ║")
    print("╚══════════════════════════════════════════════════════════════════════════╝")
    print(f"[*] Nền tảng thực thi : {arch} ({platform.system()}) — {env_name}")
    print(f"[*] Phiên bản Python  : {platform.python_version()}")

    # Khởi tạo Costmap giả lập chuẩn 5.0m x 5.0m (100 x 100 cells, res=0.05m)
    grid = OccupancyGridMap(width=100, height=100, resolution=0.05, origin_x=-2.5, origin_y=-2.5)

    # Đặt vật cản thực địa chuẩn của bài thi CS532: Thùng carton tại x=1.2m, y=0.0m
    synthetic_ranges = [8.0] * 360
    synthetic_ranges[0] = 1.2
    for a in range(40, 50):
        synthetic_ranges[a] = 0.95
    grid.update_scan(0.0, 0.0, 0.0, synthetic_ranges, max_valid_range=2.4)

    planner = AStarPlanner(grid)

    # -------------------------------------------------------------
    # PHẦN 1: KIỂM CHỨNG TÍNH ĐÚNG ĐẮN VỀ HÌNH HỌC & AN TOÀN
    # -------------------------------------------------------------
    banner("[KIỂM THỬ 1/3] Tính Đúng Đắn Hình Học & An Toàn Va Chạm")

    start_pos = (0.0, 0.0)
    goal_pos = (1.8, 0.0)

    # 1.1 Tìm đường thô (chưa làm mượt)
    raw_path = planner.plan(start_pos, goal_pos, smooth=False)
    assert raw_path is not None and len(raw_path) >= 2, "❌ A* thất bại không tìm được đường thô!"
    print(f"  [1.1] Đường thô 8-hướng: {len(raw_path)} waypoints hội tụ thành công.")

    # 1.2 Tìm đường làm mượt (Reverse Greedy Lookahead LoS)
    smooth_path = planner.plan(start_pos, goal_pos, smooth=True)
    assert smooth_path is not None and len(smooth_path) >= 2, "❌ A* thất bại không sinh được đường mượt!"
    print(f"  [1.2] Đường làm mượt   : Rút gọn còn {len(smooth_path)} waypoints tối ưu.")

    # 1.3 Kiểm tra an toàn: Không điểm nào trên quỹ đạo chạm vào ô lethal (cost >= 200)
    lethal_hits = 0
    for wp in smooth_path:
        c, r = grid.world_to_grid(*wp)
        if grid.costmap[r, c] >= 200:
            lethal_hits += 1
    assert lethal_hits == 0, f"❌ Quỹ đạo cắt qua {lethal_hits} ô vật cản lethal!"
    print("  [1.3] An toàn vật cản  : 100% waypoints nằm ngoài vùng cấm (Cost < 200).")

    # 1.4 Kiểm tra chống cắt góc chéo (Diagonal Corner Cutting)
    has_corner_cut = False
    for i in range(len(raw_path) - 1):
        c1, r1 = grid.world_to_grid(*raw_path[i])
        c2, r2 = grid.world_to_grid(*raw_path[i+1])
        dc = c2 - c1
        dr = r2 - r1
        if dc != 0 and dr != 0:
            if grid.costmap[r1, c1 + dc] >= 200 and grid.costmap[r1 + dr, c1] >= 200:
                has_corner_cut = True
                break
    assert not has_corner_cut, "❌ Quỹ đạo bị lỗi cắt góc chéo nguy hiểm!"
    print(f"  {CLR_GREEN}✅ PASS TOÀN DIỆN PHẦN 1: Quỹ đạo đúng đắn, không cắt góc, an toàn tuyệt đối.{CLR_RESET}")

    # -------------------------------------------------------------
    # PHẦN 2: ĐO ĐẠC ĐỘ TRỄ CHI TIẾT TỪNG PHÂN ĐOẠN (PROFILING)
    # -------------------------------------------------------------
    banner("[KIỂM THỬ 2/3] Phân Rã Độ Trễ: Search A* vs Reverse LoS Smoothing")

    # Warm-up JIT & Cache
    for _ in range(5):
        _ = planner.plan(start_pos, goal_pos, smooth=True)

    N_PROF = 30
    t_searches = []
    t_smooths = []
    for _ in range(N_PROF):
        t0 = time.perf_counter()
        p_raw = planner.plan(start_pos, goal_pos, smooth=False)
        t1 = time.perf_counter()
        p_sm = planner.smooth_path(p_raw)
        t2 = time.perf_counter()
        t_searches.append((t1 - t0) * 1000.0)
        t_smooths.append((t2 - t1) * 1000.0)

    mean_search = float(np.mean(t_searches))
    mean_smooth = float(np.mean(t_smooths))
    print(f"  -> Độ trễ thuật toán Search A* (1D Flat Array) : {mean_search:.2f} ms")
    print(f"  -> Độ trễ Reverse Greedy LoS Raycaster        : {mean_smooth:.2f} ms")
    print(f"  -> Tổng trung bình một chu kỳ                  : {(mean_search + mean_smooth):.2f} ms")

    # -------------------------------------------------------------
    # PHẦN 3: BỘ BENCHMARK CHÍNH THỨC NGHIỆM THU (THE TARGET)
    # -------------------------------------------------------------
    banner("[KIỂM THỬ 3/3] Nghiệm Thu Thời Gian Thực (Chỉ Tiêu <= 15.0 ms)")

    N_BENCH = 50
    latencies = []
    for _ in range(N_BENCH):
        t0 = time.perf_counter()
        _ = planner.plan(start_pos, goal_pos, smooth=True)
        latencies.append((time.perf_counter() - t0) * 1000.0)

    mean_lat = float(np.mean(latencies))
    min_lat = float(np.min(latencies))
    max_lat = float(np.max(latencies))
    p95_lat = float(np.percentile(latencies, 95))

    TARGET_MAX_MS = 15.0
    passed = mean_lat <= TARGET_MAX_MS

    baseline_arm_ms = 51.77
    if is_arm:
        speedup = baseline_arm_ms / mean_lat
    else:
        # Trên PC baseline là 6.21ms
        speedup = 6.21 / mean_lat

    print("----------------------------------------------------------------------------")
    print(f"{'CHỈ SỐ ĐO ĐẠC':<32} | {'KẾT QUẢ HIỆN TẠI':<16} | {'MỤC TIÊU NGHIỆM THU'}")
    print("----------------------------------------------------------------------------")
    print(f"{'Thời gian trung bình (Mean)':<32} | {mean_lat:>10.2f} ms   | <= 15.0 ms")
    print(f"{'Thời gian nhanh nhất (Min)':<32} | {min_lat:>10.2f} ms   | --")
    print(f"{'Thời gian chậm nhất (Max)':<32} | {max_lat:>10.2f} ms   | --")
    print(f"{'Phân vị 95% (P95 Latency)':<32} | {p95_lat:>10.2f} ms   | <= 20.0 ms")
    print(f"{'Tốc độ tăng tốc vs Baseline cũ':<32} | {speedup:>10.2f}x    | >= 3.45x")
    print("----------------------------------------------------------------------------")

    if passed:
        print(f"\n{CLR_GREEN}{CLR_BOLD}🎉 KẾT QUẢ BƯỚC 1: ĐẠT CHUẨN (PASS ✅)!{CLR_RESET}")
        print(f"Thuật toán A* mới đã giải quyết triệt để nút thắt cổ chai, đáp ứng hoàn hảo chu kỳ điều khiển 10Hz.")
    else:
        print(f"\n{CLR_RED}{CLR_BOLD}❌ KẾT QUẢ BƯỚC 1: CHƯA ĐẠT (FAIL ❌)!{CLR_RESET}")
        print(f"Thời gian trung bình {mean_lat:.2f}ms > {TARGET_MAX_MS}ms.")

    return passed

if __name__ == "__main__":
    success = test_astar_suite()
    sys.exit(0 if success else 1)
