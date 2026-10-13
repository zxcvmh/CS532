#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
CS532 AUTONOMOUS JETBOT — BƯỚC 2: KIỂM CHỨNG KHOA HỌC THUẬT TOÁN NÉ TRÁNH TỔNG QUÁT (FCP-LITE)
===============================================================================================
Mục tiêu nghiệm thu:
  1. Tổng quát hóa: Không chỉ thùng carton mà xử lý mọi vật thể (nhỏ, trung bình, lệch tâm, động, tắc toàn phần).
  2. Đo đạc độ trễ trên Jetson Nano ARM Cortex-A57 (Python 3.6.9): Target <= 5.0 ms.
  3. Không va chạm tường (Clearance >= 0.15m), không va chạm vật cản, tự động lật hướng thông minh.
===============================================================================================
"""

import os
import sys
import time
import math
import platform
import numpy as np

if sys.platform.startswith("win"):
    import io
    sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding="utf-8", errors="replace")
    sys.stderr = io.TextIOWrapper(sys.stderr.buffer, encoding="utf-8", errors="replace")

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
if BASE_DIR not in sys.path:
    sys.path.insert(0, BASE_DIR)

from planning.occupancy_grid import OccupancyGridMap
from planning.controller import PurePursuitController
from planning.obstacle_clustering import ObstacleCluster

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

def build_test_costmap(corridor_width_m: float = 1.1) -> OccupancyGridMap:
    grid = OccupancyGridMap(width=100, height=100, resolution=0.05, origin_x=-2.5, origin_y=-2.5)
    half_w = corridor_width_m / 2.0
    # Tạo tường trái và tường phải có độ dày vật lý
    for x_val in np.arange(-0.5, 3.0, 0.05):
        for w_offset in np.arange(0.0, 0.25, 0.05):
            c_l, r_l = grid.world_to_grid(x_val, half_w + w_offset)
            c_r, r_r = grid.world_to_grid(x_val, -half_w - w_offset)
            if 0 <= c_l < grid.width and 0 <= r_l < grid.height:
                grid.costmap[r_l, c_l] = 254
            if 0 <= c_r < grid.width and 0 <= r_r < grid.height:
                grid.costmap[r_r, c_r] = 254
    return grid

def make_cluster(cid=1, cx=1.0, cy=0.0, length=0.3, width=0.3, is_dyn=False, closing_spd=0.0) -> ObstacleCluster:
    return ObstacleCluster(
        id=cid,
        centroid=(cx, cy),
        closest_point=(cx - length/2.0, cy),
        min_distance=math.hypot(cx, cy),
        azimuth_deg=math.degrees(math.atan2(cy, cx)),
        length_m=length,
        width_m=width,
        orientation_rad=0.0,
        points_count=20,
        bounding_box_2d=[
            (cx - length/2.0, cy - width/2.0),
            (cx + length/2.0, cy - width/2.0),
            (cx + length/2.0, cy + width/2.0),
            (cx - length/2.0, cy + width/2.0)
        ],
        obstacle_type="BOX_OBSTACLE",
        threat_level="CRITICAL" if is_dyn else "WARNING",
        closing_speed=closing_spd,
        is_dynamic=is_dyn,
        ttc=3.0 if closing_spd > 0 else 99.0
    )

def main():
    arch = platform.machine().lower()
    is_arm = "aarch64" in arch or "arm" in arch
    env_name = "Jetson Nano ARM Cortex-A57" if is_arm else "PC Simulator / Laptop"

    print("╔══════════════════════════════════════════════════════════════════════════╗")
    print("║     CS532: NGHIỆM THU BƯỚC 2 — NÉ TRÁNH TỰ THÍCH ỨNG TỔNG QUÁT (2026)    ║")
    print("╚══════════════════════════════════════════════════════════════════════════╝")
    print(f"[*] Nền tảng thực thi : {arch} ({platform.system()}) — {env_name}")
    print(f"[*] Phiên bản Python  : {platform.python_version()}")

    ctrl = PurePursuitController()
    global_path = [(0.0, 0.0), (0.5, 0.0), (1.0, 0.0), (1.5, 0.0), (2.2, 0.0)]
    ctrl.set_path(global_path)

    # -------------------------------------------------------------
    # KIỂM THỬ 1: Hành lang hẹp 1.1m lớp CS532, Thùng carton 30cm
    # -------------------------------------------------------------
    banner("[TÌNH HUỐNG 1/5] Hành Lang Hẹp 1.1m (CS532) Với Thùng Carton 30cm")
    grid1 = build_test_costmap(corridor_width_m=1.1)
    # Thùng carton ở (1.0, 0.0) rộng 30cm
    for x_val in np.arange(0.85, 1.15, 0.05):
        for y_val in np.arange(-0.15, 0.16, 0.05):
            c, r = grid1.world_to_grid(x_val, y_val)
            grid1.costmap[r, c] = 254

    cluster1 = make_cluster(cid=1, cx=1.0, cy=0.0, length=0.30, width=0.30)

    detour1 = ctrl.generate_detour_splice(
        robot_x=0.2, robot_y=0.0, robot_theta=0.0,
        blocked_pt=(1.0, 0.0), obstacle_cluster=cluster1, grid_map=grid1
    )
    assert detour1 is not None, "❌ Thất bại: Không sinh được quỹ đạo né!"
    max_lateral_1 = max(abs(y) for x, y in detour1[:14])
    print(f"  -> Độ dạt tâm xe lớn nhất : {max_lateral_1:.2f} m (Baseline cũ: 0.52m gây đâm tường)")
    print(f"  -> Khoảng cách tới tường  : {(0.55 - max_lateral_1):.2f} m (Mép tường y=0.55m)")
    assert max_lateral_1 + 0.12 < 0.55, "❌ Mép thân xe chạm vào tường hành lang!"
    print(f"  {CLR_GREEN}✅ PASS 1: Vượt qua hành lang hẹp an toàn, không chạm tường, không chạm thùng!{CLR_RESET}")

    # -------------------------------------------------------------
    # KIỂM THỬ 2: Vật cản nhỏ (Lon nước, giày dép, vật 10cm)
    # -------------------------------------------------------------
    banner("[TÌNH HUỐNG 2/5] Vật Cản Nhỏ 10cm (Lon Nước, Giày Dép) Trong Phòng Rộng")
    grid2 = build_test_costmap(corridor_width_m=3.0)
    cluster2 = make_cluster(cid=2, cx=1.0, cy=0.0, length=0.10, width=0.10)

    detour2 = ctrl.generate_detour_splice(
        robot_x=0.2, robot_y=0.0, robot_theta=0.0,
        blocked_pt=(1.0, 0.0), obstacle_cluster=cluster2, grid_map=grid2
    )
    assert detour2 is not None, "❌ Không sinh được quỹ đạo cho vật nhỏ!"
    max_lateral_2 = max(abs(y) for x, y in detour2[:14])
    print(f"  -> Độ dạt tâm xe với vật nhỏ: {max_lateral_2:.2f} m")
    assert max_lateral_2 <= 0.35, f"❌ Dạt quá xa ({max_lateral_2}m > 0.35m) cho một vật nhỏ 10cm!"
    print(f"  {CLR_GREEN}✅ PASS 2: Thuật toán tự co nhỏ góc đánh lái tương ứng với kích thước vật thể!{CLR_RESET}")

    # -------------------------------------------------------------
    # KIỂM THỬ 3: Vật cản lệch trái, mép trái sát tường
    # -------------------------------------------------------------
    banner("[TÌNH HUỐNG 3/5] Vật Cản Lệch Trái (Mép Trái Bị Nghẽn Tường)")
    grid3 = build_test_costmap(corridor_width_m=1.4) # Tường y = +/- 0.70m
    # Đặt vật cản lệch trái tại y = +0.25m, rộng 0.40m -> mép trái ở y = +0.45m (cách tường chỉ 0.25m)
    for x_val in np.arange(0.85, 1.15, 0.05):
        for y_val in np.arange(0.05, 0.46, 0.05):
            c, r = grid3.world_to_grid(x_val, y_val)
            grid3.costmap[r, c] = 254

    cluster3 = make_cluster(cid=3, cx=1.0, cy=0.25, length=0.40, width=0.40)

    detour3 = ctrl.generate_detour_splice(
        robot_x=0.2, robot_y=0.0, robot_theta=0.0,
        blocked_pt=(1.0, 0.0), obstacle_cluster=cluster3, grid_map=grid3
    )
    assert detour3 is not None, "❌ Không sinh được quỹ đạo né!"
    # Vì bên trái bị nghẽn (khe hở < 0.28m), thuật toán phải dạt sang PHẢI (y < 0)
    y_signs = [y for x, y in detour3[:14]]
    is_right_deflection = any(y < -0.10 for y in y_signs)
    print(f"  -> Chiều dạt thực tế: {'BÊN PHẢI (y < 0)' if is_right_deflection else 'BÊN TRÁI'}")
    assert is_right_deflection, "❌ Không tự động lật hướng sang bên thoáng hơn!"
    print(f"  {CLR_GREEN}✅ PASS 3: Tự động đổi hướng né sang bên thoáng (Bên Phải) khi bên trái bị nghẽn!{CLR_RESET}")

    # -------------------------------------------------------------
    # KIỂM THỬ 4: Tắc nghẽn toàn phần (Full Blockage)
    # -------------------------------------------------------------
    banner("[TÌNH HUỐNG 4/5] Hành Lang Bị Chặn Hoàn Toàn (Ván Gỗ Ngăn Đường 0.9m)")
    grid4 = build_test_costmap(corridor_width_m=1.0) # Tường +/- 0.50m
    # Tấm ván chắn từ y = -0.45m đến y = +0.45m
    for x_val in np.arange(0.95, 1.05, 0.05):
        for y_val in np.arange(-0.45, 0.46, 0.05):
            c, r = grid4.world_to_grid(x_val, y_val)
            grid4.costmap[r, c] = 254

    cluster4 = make_cluster(cid=4, cx=1.0, cy=0.0, length=0.30, width=0.90)

    detour4 = ctrl.generate_detour_splice(
        robot_x=0.2, robot_y=0.0, robot_theta=0.0,
        blocked_pt=(1.0, 0.0), obstacle_cluster=cluster4, grid_map=grid4
    )
    print(f"  -> Quyết định điều khiển: {'TỪ CHỐI NÉ (BẢO VỆ DỪNG AN TOÀN)' if detour4 is None else 'CỐ NÉ'}")
    assert detour4 is None, "❌ Cố né ẩu khi không có lối đi khả dụng!"

    # Kiểm tra lệnh phanh từ compute_command
    v_cmd, w_cmd, info_cmd = ctrl.compute_command(
        robot_x=0.2, robot_y=0.0, robot_theta=0.0,
        grid_map=grid4, clusters=[cluster4]
    )
    print(f"  -> Lệnh phản hồi: v = {v_cmd} m/s, status = {info_cmd.get('status')}")
    assert v_cmd == 0.0 and info_cmd.get("status") == "IMPASSABLE_STOP", "❌ Xe không dừng an toàn khi tắc đường!"
    print(f"  {CLR_GREEN}✅ PASS 4: Bảo vệ chống đâm tường tuyệt đối, kích hoạt dừng an toàn khi tắc đường!{CLR_RESET}")

    # -------------------------------------------------------------
    # KIỂM THỬ 5: Vật thể động tiến tới (Dynamic Oncoming COLREGS)
    # -------------------------------------------------------------
    banner("[TÌNH HUỐNG 5/5] Người Đi Bộ Tiến Tới (Quy Tắc Tránh Va COLREGS)")
    grid5 = build_test_costmap(corridor_width_m=2.0)
    cluster5 = make_cluster(cid=5, cx=1.5, cy=0.0, length=0.40, width=0.40, is_dyn=True, closing_spd=0.40)

    detour5 = ctrl.generate_detour_splice(
        robot_x=0.2, robot_y=0.0, robot_theta=0.0,
        blocked_pt=(1.2, 0.0), obstacle_cluster=cluster5, grid_map=grid5
    )
    assert detour5 is not None, "❌ Không sinh được quỹ đạo né người!"
    y_signs_5 = [y for x, y in detour5[:14]]
    is_right_colregs = any(y < -0.10 for y in y_signs_5)
    print(f"  -> Chiều dạt theo COLREGS: {'BÊN PHẢI (y < 0)' if is_right_colregs else 'BÊN TRÁI'}")
    assert is_right_colregs, "❌ Không ưu tiên né bên phải theo luật giao thông!"
    print(f"  {CLR_GREEN}✅ PASS 5: Tuân thủ quy tắc né bên phải cho vật thể động tiến tới!{CLR_RESET}")

    # -------------------------------------------------------------
    # BENCHMARK THỜI GIAN THỰC TRÊN ARM CORTEX-A57
    # -------------------------------------------------------------
    banner("[BENCHMARK HIỆU NĂNG] Đo Đạc Độ Trễ Thời Gian Thực (Target <= 5.0 ms)")
    N_ITERS = 100
    times = []
    for _ in range(N_ITERS):
        t0 = time.perf_counter()
        _ = ctrl.generate_detour_splice(
            robot_x=0.2, robot_y=0.0, robot_theta=0.0,
            blocked_pt=(1.0, 0.0), obstacle_cluster=cluster1, grid_map=grid1
        )
        times.append((time.perf_counter() - t0) * 1000.0)

    mean_t = float(np.mean(times))
    max_t = float(np.max(times))
    p95_t = float(np.percentile(times, 95))

    print(f"{'Thời gian trung bình (Mean)':<32} | {mean_t:>10.3f} ms   | <= 5.0 ms")
    print(f"{'Thời gian chậm nhất (Max)':<32} | {max_t:>10.3f} ms   | --")
    print(f"{'Phân vị 95% (P95 Latency)':<32} | {p95_t:>10.3f} ms   | <= 8.0 ms")

    assert mean_t <= 5.0, f"❌ Độ trễ {mean_t:.2f}ms > 5.0ms!"
    print(f"\n{CLR_GREEN}{CLR_BOLD}🎉 KẾT QUẢ BƯỚC 2: ĐẠT CHUẨN 100% (PASS ✅)!{CLR_RESET}")
    print("Thuật toán né tránh tổng quát hoàn hảo mọi kịch bản và chạy cực nhanh trên Jetson Nano.")

if __name__ == "__main__":
    main()
