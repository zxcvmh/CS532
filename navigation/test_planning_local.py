#!/usr/bin/env python3
"""
CS532 Autonomous JetBot — Bộ Kiểm Thử Thuật Toán Điều Hướng Trên PC (PC-First)
=============================================================================
Độc lập 100% không cần robot thật. Tự sinh dữ liệu LaserScan giả lập
để kiểm thử 4 tiêu chí cốt tử trước khi bàn giao phần cứng:
  1. Occupancy Grid Ray Clamping & Độ trễ update_scan (< 20ms PC, < 30ms ARM).
  2. Gom cụm 1D Range Jump O(N) thích ứng cự ly (< 2.5ms, không phân mảnh).
  3. Quỹ đạo né hình thang Plateau 3 đoạn (khoảng hở mép an toàn >= 0.22m).
  4. Bộ điều tốc vào cua Curvature-Aware Scaling (v_curve < v_straight).
  + Kiểm tra phụ: Đấu nối CV low_obstacles vào Costmap & A* Path Planning.
=============================================================================
"""

import os
import sys
import time
import math
import numpy as np

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, BASE_DIR)

from planning.occupancy_grid import OccupancyGridMap
from planning.astar_planner import AStarPlanner
from planning.controller import PurePursuitController
from planning.obstacle_clustering import LiDARClusterDetector, ObstacleCluster


def test_1_grid_update_and_clamping():
    print("\n" + "=" * 65)
    print(" [TEST 1/4] Occupancy Grid Update Latency, Ray Clamping & Low Obstacles")
    print("=" * 65)

    grid = OccupancyGridMap(width=100, height=100, resolution=0.05, origin_x=-2.5, origin_y=-2.5)

    # 1. Đo latency quét phòng rộng 8m (tia vượt biên bản đồ 5m x 5m)
    synthetic_ranges = [8.0] * 360
    synthetic_ranges[0] = 1.2  # Vật cản phía trước 1.2m

    t0 = time.time()
    loops = 10
    for _ in range(loops):
        grid.update_scan(0.0, 0.0, 0.0, synthetic_ranges, max_valid_range=2.4)
    avg_ms = ((time.time() - t0) / loops) * 1000.0

    print(f"-> Thời gian cập nhật lưới trung bình trên PC: {avg_ms:.2f} ms")
    assert avg_ms < 20.0, f"❌ Thất bại: Latency {avg_ms:.2f}ms >= 20.0ms!"
    print("  ✅ PASS 1.1: Đạt chỉ tiêu độ trễ (< 20ms PC, tương đương < 30ms Jetson Nano ARM)!")

    # 2. Kiểm tra Ray Clamping khi robot ở sát góc bản đồ
    grid.update_scan(2.2, 2.2, 0.0, synthetic_ranges, max_valid_range=2.4)
    print("  ✅ PASS 1.2: Ray Clamping xử lý mượt mà khi tia bắn vượt biên bản đồ!")

    # 3. Kiểm tra đấu nối CV low_obstacles
    low_pts = [[0.8, 0.0], [0.85, 0.05]]
    grid.insert_low_obstacles(low_pts, robot_x=0.0, robot_y=0.0, robot_theta=0.0)
    c, r = grid.world_to_grid(0.8, 0.0)
    assert grid.costmap[r, c] >= 200, "❌ Low obstacle chưa được nạp vào costmap!"
    print("  ✅ PASS 1.3: Giao diện nạp low_obstacles từ CV đã nạp thành công vào Costmap!")

    return grid


def test_2_clustering_and_astar(grid):
    print("\n" + "=" * 65)
    print(" [TEST 2/4] Gom Cụm 1D Range Jump O(N) & Tìm Đường A*")
    print("=" * 65)

    detector = LiDARClusterDetector(
        min_cluster_size=3,
        corridor_width=0.35,
        min_range_m=0.12,
        max_range_m=2.0
    )

    # 1. Giả lập 1 cụm vật thể tại cự ly 1.0m (31 điểm quét -15° đến +15°)
    pts = []
    np.random.seed(42)
    for ang in range(-15, 16):
        r = 1.0 + float(np.random.normal(0, 0.008))
        rad = math.radians(ang)
        pts.append([r * math.cos(rad), r * math.sin(rad)])

    pts_np = np.array(pts, dtype=np.float32)

    # Đo latency gom cụm
    t0 = time.time()
    loops = 20
    for _ in range(loops):
        clusters = detector.cluster_point_cloud(pts_np)
    avg_ms = ((time.time() - t0) / loops) * 1000.0

    print(f"-> Thời gian gom cụm 1D trung bình trên PC: {avg_ms:.3f} ms")
    assert avg_ms < 2.5, f"❌ Thất bại: Latency {avg_ms:.2f}ms >= 2.5ms!"
    print(f"-> Số lượng cụm nhận diện: {len(clusters)} (Kỳ vọng: 1 cụm)")
    assert len(clusters) == 1, f"❌ Cụm bị phân mảnh hoặc bỏ sót: tìm thấy {len(clusters)} cụm!"

    # 2. Kiểm tra phát hiện mối nguy CRITICAL ở cự ly gần (0.30m)
    pts_crit = []
    for ang in range(-10, 11):
        r = 0.30 + float(np.random.normal(0, 0.005))
        rad = math.radians(ang)
        pts_crit.append([r * math.cos(rad), r * math.sin(rad)])
    clusters_crit = detector.cluster_point_cloud(np.array(pts_crit, dtype=np.float32))
    assert len(clusters_crit) == 1 and clusters_crit[0].threat_level == "CRITICAL", "❌ Cụm gần không nhận diện là CRITICAL!"
    print("  ✅ PASS 2.1: Thuật toán 1D Range Jump O(N) gom cụm chuẩn xác, nhận diện đúng mức nguy hại!")

    # 3. Kiểm tra A* path planning
    planner = AStarPlanner(grid)
    t0 = time.time()
    path = planner.plan((0.0, 0.0), (1.8, 0.0), smooth=True)
    dt_ms = (time.time() - t0) * 1000.0
    assert path is not None and len(path) >= 2, "❌ A* không tìm được đường đi!"
    print(f"-> A* tìm thấy {len(path)} waypoints trong {dt_ms:.2f} ms")
    print("  ✅ PASS 2.2: Thuật toán A* hội tụ thành công với Costmap lạm phát!")

    return path


def test_3_detour_plateau_clearance(path):
    print("\n" + "=" * 65)
    print(" [TEST 3/4] Quỹ Đạo Né Tránh Hình Thang Plateau (Khoảng Hở >= 0.22m)")
    print("=" * 65)

    ctrl = PurePursuitController(lateral_detour_offset=0.52)
    ctrl.set_path(path)

    # Giả lập vật cản chắn giữa đường tại (1.0m, 0.0m)
    blocked_pt = (1.0, 0.0)
    detour = ctrl.generate_detour_splice(
        robot_x=0.2, robot_y=0.0, robot_theta=0.0,
        blocked_pt=blocked_pt,
        lateral_offset=0.52
    )

    assert detour is not None and len(detour) > 2, "❌ generate_detour_splice không sinh được đường né!"

    # Tính khoảng hở mép thực tế nhỏ nhất dọc toàn bộ đường né
    min_dist_to_obs = min(math.hypot(x - blocked_pt[0], y - blocked_pt[1]) for x, y in detour)
    robot_radius = 0.12
    margin = min_dist_to_obs - robot_radius

    print(f"-> Khoảng cách tâm quỹ đạo gần nhất tới tâm vật cản: {min_dist_to_obs:.2f} m")
    print(f"-> Khoảng hở mép an toàn (trừ bán kính thân xe {robot_radius}m): {margin:.2f} m")

    assert margin >= 0.22, f"❌ Khoảng hở mép {margin:.2f}m < 0.22m, nguy cơ va chạm hoặc dính phanh E-stop (0.20m)!"
    print(f"  ✅ PASS 3: Khoảng hở an toàn {margin:.2f}m >= 0.22m (vượt ngưỡng E-stop 0.20m)!")

    return detour


def test_4_curvature_velocity_scaling():
    print("\n" + "=" * 65)
    print(" [TEST 4/4] Điều Tốc Vào Cua Curvature-Aware Scaling (v = v_base / (1 + k|w|))")
    print("=" * 65)

    ctrl = PurePursuitController(max_linear_speed=0.25, curvature_scale_k=1.0)

    # 1. Chạy thẳng đường dài
    ctrl.set_path([(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)])
    v_straight, w_straight, info_s = ctrl.compute_command(0.0, 0.0, 0.0)

    # 2. Vào góc cua gắt vuông góc 90 độ
    ctrl.set_path([(0.0, 0.0), (0.0, 1.0)])
    v_curve, w_curve, info_c = ctrl.compute_command(0.0, 0.0, 0.0)

    print(f"-> Đường thẳng:   v = {v_straight:.2f} m/s, w = {w_straight:.2f} rad/s")
    print(f"-> Ôm cua 90 độ:  v = {v_curve:.2f} m/s, w = {w_curve:.2f} rad/s")

    assert v_curve < v_straight, f"❌ Bộ điều tốc chưa hãm vận tốc khi vào cua (v_curve {v_curve} >= v_straight {v_straight})!"
    assert v_curve >= 0.04, f"❌ Vận tốc ôm cua quá thấp gây chết máy: {v_curve} m/s!"

    diff_percent = ((v_straight - v_curve) / v_straight) * 100.0
    print(f"-> Tỉ lệ giảm tốc khi vào cua: {diff_percent:.1f}%")
    print("  ✅ PASS 4: Xe tự động hãm tốc mượt mà khi bẻ lái góc lớn, chống trượt lật bánh vi sai!")


def main():
    print("\n" + "#" * 65)
    print("   BỘ KIỂM THỬ THUẬT TOÁN ĐIỀU HƯỚNG & ĐIỀU KHIỂN (PC-FIRST)")
    print("   DỰ ÁN CS532 JETBOT — THƯ MỤC 04-hieu-MotionControl")
    print("#" * 65)

    grid = test_1_grid_update_and_clamping()
    path = test_2_clustering_and_astar(grid)
    detour = test_3_detour_plateau_clearance(path)
    test_4_curvature_velocity_scaling()

    print("\n" + "#" * 65)
    print("  🎉 XUẤT SẮC! TOÀN BỘ 4/4 TIÊU CHÍ KỸ THUẬT ĐÃ ĐẠT CHUẨN PASS!")
    print("  SẴN SÀNG ĐÓNG GÓI BÀN GIAO CHO TEAM LEAD NGHIỆM THU TRÊN JETBOT!")
    print("#" * 65 + "\n")


if __name__ == "__main__":
    main()
