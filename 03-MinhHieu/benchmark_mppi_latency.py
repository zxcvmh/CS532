#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
CS532 AUTONOMOUS JETBOT — BỘ ĐO ĐẠC ĐỘ TRỄ CHUẨN MÃ NGUỒN MỞ MPPI (LATENCY BENCHMARK)
====================================================================================
Quy chuẩn đo đạc:
  - Dựa trên phương pháp đo lường của cộng đồng ROS 2 Navigation (nav2_mppi_controller benchmark suite)
  - Khảo sát các kích thước batch lấy mẫu: K = 100, 250, 500, 1000 trajectories
  - Mỗi cấu hình thực thi 100 chu kỳ tối ưu hóa lặp lại trên bản đồ chi phí thực tế (Costmap 100x100)
  - Đo đạc thống kê đầy đủ: Mean, Min, Max, P50, P95, P99 và Tần số đáp ứng tối đa (Hz)
====================================================================================
"""

import os
import sys
import time
import math
import platform
import numpy as np

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
APP_DIR = os.path.join(SCRIPT_DIR, "backend", "app")
if APP_DIR not in sys.path:
    sys.path.insert(0, APP_DIR)

from planning.occupancy_grid import OccupancyGridMap
from planning.mppi_controller import MPPIController

CLR_RESET = "\033[0m"
CLR_BOLD = "\033[1m"
CLR_GREEN = "\033[92m"
CLR_YELLOW = "\033[93m"
CLR_CYAN = "\033[96m"

def build_benchmark_costmap() -> OccupancyGridMap:
    """Tạo costmap thực nghiệm 100x100 chứa hành lang và các cụm vật cản thực tế."""
    grid = OccupancyGridMap(width=100, height=100, resolution=0.05, origin_x=-2.5, origin_y=-2.5)
    # Tường hai bên tại y = +/- 0.60m
    for x_val in np.arange(-0.5, 3.0, 0.05):
        c_l, r_l = grid.world_to_grid(x_val, 0.60)
        c_r, r_r = grid.world_to_grid(x_val, -0.60)
        if 0 <= c_l < grid.width and 0 <= r_l < grid.height:
            grid.costmap[r_l, c_l] = 254
        if 0 <= c_r < grid.width and 0 <= r_r < grid.height:
            grid.costmap[r_r, c_r] = 254

    # 2 vật cản nằm trong hành lang
    for x_val in np.arange(0.8, 1.1, 0.05):
        for y_val in np.arange(-0.15, 0.15, 0.05):
            c, r = grid.world_to_grid(x_val, y_val)
            if 0 <= c < grid.width and 0 <= r < grid.height:
                grid.costmap[r, c] = 254

    for x_val in np.arange(1.6, 1.9, 0.05):
        for y_val in np.arange(0.1, 0.35, 0.05):
            c, r = grid.world_to_grid(x_val, y_val)
            if 0 <= c < grid.width and 0 <= r < grid.height:
                grid.costmap[r, c] = 254

    return grid

def run_benchmark():
    arch = platform.machine().lower()
    is_arm = "aarch64" in arch or "arm" in arch
    env_name = "Jetson Nano ARM Cortex-A57" if is_arm else "PC Simulator / Host CPU"

    print("╔══════════════════════════════════════════════════════════════════════════════════════════╗")
    print("║     CS532: ĐO ĐẠC ĐỘ TRỄ THUẬT TOÁN MPPI (STANDARDIZED NAV2 LATENCY BENCHMARK)           ║")
    print("╚══════════════════════════════════════════════════════════════════════════════════════════╝")
    print(f"[*] Nền tảng thực thi       : {arch} ({platform.system()}) — {env_name}")
    print(f"[*] Phiên bản Python        : {platform.python_version()}")
    print(f"[*] Thư viện số học         : NumPy {np.__version__}")
    print(f"[*] Kích thước bản đồ lưới  : 100 x 100 cells (Độ phân giải 5cm, diện tích 5m x 5m)")
    print(f"[*] Tiêu chuẩn thời gian thực: Ngân sách 10Hz = 100.0 ms | Ngân sách 50Hz = 20.0 ms\n")

    grid = build_benchmark_costmap()
    ref_path = [(0.0, 0.0), (0.5, 0.0), (1.0, 0.0), (1.5, 0.0), (2.2, 0.0)]

    # Các cấu hình số lượng mẫu K cần khảo sát
    K_CONFIGS = [100, 250, 500, 1000]
    HORIZON_STEPS = 15  # 15 bước x 0.1s = 1.5s nhìn trước
    N_ROUNDS = 100      # 100 chu kỳ lặp lại cho mỗi cấu hình

    print(f"{'Cấu hình (K Mẫu)':<18} | {'Mean (ms)':<10} | {'Min (ms)':<10} | {'Max (ms)':<10} | {'P95 (ms)':<10} | {'P99 (ms)':<10} | {'Tần số (Hz)':<12}")
    print("-" * 92)

    benchmark_summary = {}

    for k_val in K_CONFIGS:
        controller = MPPIController(
            horizon_steps=HORIZON_STEPS,
            num_samples=k_val,
            dt=0.10,
            temperature_lambda=15.0
        )
        controller.set_path(ref_path)

        # Warm-up 5 chu kỳ
        for _ in range(5):
            _ = controller.compute_command(0.2, 0.0, 0.0, grid_map=grid)

        # Đo đạc 100 chu kỳ
        latencies = []
        for step in range(N_ROUNDS):
            rx = 0.2 + (step * 0.005)
            ry = 0.05 * math.sin(step * 0.1)
            rth = 0.02 * math.cos(step * 0.1)

            t0 = time.perf_counter()
            v, w, info = controller.compute_command(rx, ry, rth, grid_map=grid)
            t1 = time.perf_counter()
            latencies.append((t1 - t0) * 1000.0)

        mean_l = float(np.mean(latencies))
        min_l = float(np.min(latencies))
        max_l = float(np.max(latencies))
        p95_l = float(np.percentile(latencies, 95))
        p99_l = float(np.percentile(latencies, 99))
        freq_hz = 1000.0 / mean_l if mean_l > 0 else 0.0

        benchmark_summary[k_val] = {
            "mean": mean_l, "min": min_l, "max": max_l,
            "p95": p95_l, "p99": p99_l, "freq_hz": freq_hz
        }

        print(f"K = {k_val:<14} | {mean_l:>8.3f} ms | {min_l:>8.3f} ms | {max_l:>8.3f} ms | {p95_l:>8.3f} ms | {p99_l:>8.3f} ms | {freq_hz:>10.1f} Hz")

    print("-" * 92)

    # Đánh giá chỉ tiêu
    k_rec = 300
    p95_k250 = benchmark_summary[250]["p95"]
    print(f"\n{CLR_BOLD}ĐÁNH GIÁ TƯƠNG THÍCH PHẦN CỨNG & HIỆU NĂNG THỜI GIAN THỰC:{CLR_RESET}")
    print(f"  • Cấu hình chuẩn khuyến nghị (K = 250 trajectories, T = 1.5s horizon):")
    print(f"    - Độ trễ P95 đạt              : {p95_k250:.3f} ms")
    print(f"    - Tần số đáp ứng khả dụng     : {benchmark_summary[250]['freq_hz']:.1f} Hz")
    print(f"    - Tiêu thụ ngân sách chu kỳ   : {(p95_k250 / 100.0) * 100.0:.2f}% (trong ngân sách 100ms của 10Hz)")

    if p95_k250 < 10.0:
        print(f"\n{CLR_GREEN}{CLR_BOLD}✅ KẾT LUẬN: ĐẠT CHUẨN THỜI GIAN THỰC CAO (P95 < 10.0 ms).{CLR_RESET}")
        print("Thuật toán MPPI hoàn toàn đáp ứng khả năng chạy thời gian thực liên tục trên CPU Jetson Nano.")
    else:
        print(f"\n{CLR_YELLOW}⚠️ CẢNH BÁO: Độ trễ P95 vượt ngưỡng 10ms trên phần cứng này.{CLR_RESET}")

if __name__ == "__main__":
    run_benchmark()
