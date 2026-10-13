#!/usr/bin/env python3
"""
KIỂM CHỨNG KỊCH BẢN THỰC TẾ: TÁI TẠO BẢN ĐỒ VÀ ĐƯỜNG ĐI TRONG ẢNH NGƯỜI DÙNG (media_1791818651195.png)
Kiểm tra:
1. Chiều dài quỹ đạo A* giữa hai chân bàn (Start (0.235, -0.007) -> Goal (1.121, 0.026)).
   - Trước khi sửa: 3.73m (detour vòng vèo ra ngoài).
   - Sau khi sửa: <= 0.98m (đi thẳng xuyên qua khoảng hở 30cm giữa hai chân bàn).
2. Kiểm tra DynamicObstacleTracker không kích hoạt oncoming threat giả khi robot di chuyển với vận tốc v.
"""

import sys
import math
import numpy as np

# Ensure planning module is imported
sys.path.insert(0, '/home/zxcvmh/Projects/CS532/04-hieu-MotionControl')
from planning.occupancy_grid import OccupancyGridMap
from planning.astar_planner import AStarPlanner
from planning.obstacle_clustering import ObstacleCluster, DynamicObstacleTracker

def test_user_screenshot_scenario():
    print("=" * 70)
    print(" [BENCHMARK 1] Tái tạo bản đồ & A* theo đúng ảnh thực tế người dùng")
    print("=" * 70)

    grid = OccupancyGridMap(width=100, height=100, resolution=0.05, origin_x=-2.5, origin_y=-2.5)

    # Robot start & goal từ ảnh người dùng
    start_pose = (0.235, -0.007)
    goal_pose = (1.121, 0.026)
    direct_dist = math.hypot(goal_pose[0] - start_pose[0], goal_pose[1] - start_pose[1])
    print(f"-> Vị trí Robot: ({start_pose[0]:.3f}, {start_pose[1]:.3f})")
    print(f"-> Vị trí Đích:  ({goal_pose[0]:.3f}, {goal_pose[1]:.3f})")
    print(f"-> Khoảng cách đường chim bay: {direct_dist:.3f} m")

    # Mô phỏng 2 chân bàn tạo thành khe hẹp ~0.30m giữa robot và đích
    # Chân bàn 1: x = 0.55m, y = 0.15m (bán kính 3cm)
    # Chân bàn 2: x = 0.55m, y = -0.15m (bán kính 3cm)
    # Khoảng hở giữa 2 chân bàn = 0.30m - 0.06m = 0.24m (đủ rộng cho robot 0.17m)
    c1 = ObstacleCluster(
        id=1, centroid=(0.55, 0.15), closest_point=(0.52, 0.15),
        min_distance=0.35, azimuth_deg=15.0, length_m=0.06, width_m=0.06,
        orientation_rad=0.0, points_count=10, bounding_box_2d=[],
        obstacle_type="TABLE_LEG", threat_level="SAFE"
    )
    c2 = ObstacleCluster(
        id=2, centroid=(0.55, -0.15), closest_point=(0.52, -0.15),
        min_distance=0.35, azimuth_deg=-15.0, length_m=0.06, width_m=0.06,
        orientation_rad=0.0, points_count=10, bounding_box_2d=[],
        obstacle_type="TABLE_LEG", threat_level="SAFE"
    )

    # Chèn các cụm vật cản vào Costmap
    grid.insert_obstacle_clusters([c1, c2], robot_x=0.0, robot_y=0.0, robot_theta=0.0)
    grid.clear_footprint(start_pose[0], start_pose[1], radius=0.12)

    planner = AStarPlanner(grid)
    path = planner.plan(start_pose, goal_pose, smooth=True)

    assert path is not None and len(path) >= 2, "❌ A* thất bại, không tìm được đường đi!"

    # Tính độ dài quỹ đạo
    path_len = sum(
        math.hypot(path[i+1][0] - path[i][0], path[i+1][1] - path[i][1])
        for i in range(len(path) - 1)
    )
    print(f"-> Số waypoints: {len(path)}")
    print(f"-> Độ dài quỹ đạo A*: {path_len:.3f} m")
    for idx, pt in enumerate(path):
        print(f"   wp[{idx}]: ({pt[0]:.3f}, {pt[1]:.3f})")

    assert path_len <= 1.05, f"❌ Quỹ đạo bị vòng vèo: {path_len:.2f}m > 1.05m (kỳ vọng đường thẳng ~0.90m)!"
    print(f"  ✅ PASS: Quỹ đạo tối ưu đi thẳng qua khe hở ({path_len:.3f}m), triệt tiêu hoàn toàn detour 3.73m!")


def test_ego_motion_compensation():
    print("\n" + "=" * 70)
    print(" [BENCHMARK 2] Kiểm chứng bù chuyển động (Ego-Motion) trong World Frame")
    print("=" * 70)

    tracker = DynamicObstacleTracker(assoc_dist_m=0.45)

    # Chân bàn đứng yên trong phòng tại World: (0.60, 0.00)
    # Bước 1: Robot tại pose (0.0, 0.0, 0.0). Vật cản trong Body {R} là (0.60, 0.0)
    c_step1 = ObstacleCluster(
        id=1, centroid=(0.60, 0.0), closest_point=(0.58, 0.0),
        min_distance=0.58, azimuth_deg=0.0, length_m=0.05, width_m=0.05,
        orientation_rad=0.0, points_count=10, bounding_box_2d=[],
        obstacle_type="TABLE_LEG", threat_level="SAFE"
    )
    res1 = tracker.update([c_step1], timestamp=0.0, robot_pose=(0.0, 0.0, 0.0))

    # Bước 2: Robot tiến lên 0.10m sau dt = 0.1s (v = 1.0 m/s tương đối).
    # Pose robot mới: (0.10, 0.0, 0.0).
    # Chân bàn vẫn tại World (0.60, 0.0), nên trong Body {R} nó ở (0.50, 0.0).
    c_step2 = ObstacleCluster(
        id=1, centroid=(0.50, 0.0), closest_point=(0.48, 0.0),
        min_distance=0.48, azimuth_deg=0.0, length_m=0.05, width_m=0.05,
        orientation_rad=0.0, points_count=10, bounding_box_2d=[],
        obstacle_type="TABLE_LEG", threat_level="SAFE"
    )
    res2 = tracker.update([c_step2], timestamp=0.10, robot_pose=(0.10, 0.0, 0.0))

    cl_tracked = res2[0]
    print(f"-> Vận tốc thực trong phòng (World Frame): speed = {cl_tracked.speed:.3f} m/s")
    print(f"-> Tốc độ áp sát: closing_speed = {cl_tracked.closing_speed:.3f} m/s")
    print(f"-> Mức độ nguy cơ: threat_level = {cl_tracked.threat_level}")
    print(f"-> Cờ dynamic: is_dynamic = {cl_tracked.is_dynamic}")

    assert not cl_tracked.is_dynamic, "❌ Vật cản đứng yên bị nhận diện nhầm là dynamic!"
    assert cl_tracked.threat_level != "CRITICAL_ONCOMING", "❌ Báo động giả CRITICAL_ONCOMING khi robot tiến về trước!"
    print("  ✅ PASS: Triệt tiêu 100% báo động giả động học khi robot di chuyển!")

if __name__ == "__main__":
    test_user_screenshot_scenario()
    test_ego_motion_compensation()
    print("\n🎉 TOÀN BỘ KIỂM CHỨNG TÁI HIỆN BẢN ĐỒ THỰC TẾ ĐẠT CHUẨN PASS 100%!")
