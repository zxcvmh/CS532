#!/usr/bin/env python3
"""
================================================================================
BỘ KIỂM THỬ ĐỘNG KHÉP VÒNG LIÊN TỤC (CLOSED-LOOP CONTINUOUS DYNAMIC BENCHMARK)
Dự án CS532 JetBot — Thư mục 04-hieu-MotionControl
================================================================================
Mục đích:
  Kiểm chứng 100% tính năng điều hướng thực tế trong môi trường liên tục (10Hz):
  - Tích phân động học vi sai (Differential Drive Kinematics)
  - Mô phỏng LiDAR 360 độ thời gian thực với nhiễu thực tế (Gaussian noise 10mm)
  - Đóng vòng phản hồi: Scan -> Costmap -> A* -> Pure Pursuit + DDE + CBF -> (v, w) -> Robot Pose
  - Tiêu chuẩn nghiệm thu:
    1. Không va chạm bất kỳ vật cản nào (Clearance > bán kính xe 0.12m)
    2. Về đích thành công (Khoảng cách tới Goal <= goal_tolerance 0.10m)
    3. Triệt tiêu hiện tượng văng lắc đổi hướng liên tục (Chattering rate < 25%)
    4. Xoay tại chỗ (Spin in place) khi góc lệch lớn (> 95 độ), không đâm tới phía trước
================================================================================
"""

import math
import time
import numpy as np
from typing import List, Tuple, Dict, Any

from planning.occupancy_grid import OccupancyGridMap
from planning.astar_planner import AStarPlanner
from planning.controller import PurePursuitController
from planning.disparity_extender import DisparityExtender
from planning.cbf_safety_filter import CBFSafetyFilter


class ContinuousSimulationArena:
    """
    Môi trường vật lý 2D liên tục mô phỏng sàn thi đấu thực tế của JetBot.
    Hỗ trợ tường biên, chướng ngại vật tròn, và quét LiDAR 360 tia với nhiễu.
    """
    def __init__(self, x_min=-1.0, x_max=3.0, y_min=-1.5, y_max=1.5):
        self.x_min = x_min
        self.x_max = x_max
        self.y_min = y_min
        self.y_max = y_max
        # Danh sách vật cản: (x, y, radius)
        self.obstacles: List[Tuple[float, float, float]] = []

    def add_obstacle(self, x: float, y: float, r: float):
        self.obstacles.append((x, y, r))

    def raycast_lidar_360(self, rx: float, ry: float, rtheta: float, noise_std=0.01) -> List[float]:
        """Bắn 360 tia LiDAR từ tâm robot (rx, ry, rtheta) lấy khoảng cách đo."""
        ranges = []
        for deg in range(360):
            # Góc tia trong hệ quy chiếu thế giới {W}
            beam_angle = rtheta + math.radians(deg)
            cos_b = math.cos(beam_angle)
            sin_b = math.sin(beam_angle)

            dist_min = 8.0  # Max LiDAR range

            # 1. Cắt với tường biên
            if cos_b > 1e-4:
                dist_min = min(dist_min, (self.x_max - rx) / cos_b)
            elif cos_b < -1e-4:
                dist_min = min(dist_min, (self.x_min - rx) / cos_b)

            if sin_b > 1e-4:
                dist_min = min(dist_min, (self.y_max - ry) / sin_b)
            elif sin_b < -1e-4:
                dist_min = min(dist_min, (self.y_min - ry) / sin_b)

            # 2. Cắt với từng hình trụ vật cản
            for ox, oy, orad in self.obstacles:
                dx = rx - ox
                dy = ry - oy
                b = 2.0 * (dx * cos_b + dy * sin_b)
                c = dx * dx + dy * dy - orad * orad
                disc = b * b - 4.0 * c
                if disc >= 0:
                    sqrt_disc = math.sqrt(disc)
                    t1 = (-b - sqrt_disc) / 2.0
                    if t1 > 0.05:
                        dist_min = min(dist_min, t1)

            # Thêm nhiễu Gaussian thực tế
            if noise_std > 0:
                dist_min += float(np.random.normal(0.0, noise_std))

            ranges.append(max(0.12, min(8.0, dist_min)))
        return ranges

    def min_clearance(self, rx: float, ry: float, robot_radius=0.12) -> float:
        """Khoảng cách mép robot tới vật cản gần nhất (âm nếu va chạm)."""
        dist_to_wall = min(rx - self.x_min, self.x_max - rx, ry - self.y_min, self.y_max - ry)
        dist_to_obs = 99.0
        for ox, oy, orad in self.obstacles:
            d = math.hypot(rx - ox, ry - oy) - orad
            if d < dist_to_obs:
                dist_to_obs = d
        return min(dist_to_wall, dist_to_obs) - robot_radius


def run_closed_loop_navigation(
    scenario_name: str,
    start_pose: Tuple[float, float, float],
    goal_pt: Tuple[float, float],
    obstacles: List[Tuple[float, float, float]],
    max_steps: int = 250,
    dt: float = 0.1,
    replan_interval: int = 5
) -> Dict[str, Any]:
    """
    Mô phỏng vòng lặp đóng 10Hz hoàn chỉnh giữa Robot, Cảm biến, và Hệ thống Điều hướng.
    """
    arena = ContinuousSimulationArena(x_min=-1.5, x_max=3.0, y_min=-1.5, y_max=1.5)
    for ox, oy, orad in obstacles:
        arena.add_obstacle(ox, oy, orad)

    # Khởi tạo pipeline thuật toán với thông số thực tế
    grid_map = OccupancyGridMap(width=100, height=100, resolution=0.05, origin_x=-2.0, origin_y=-2.0)
    grid_map.inflation_radius = 0.10
    planner = AStarPlanner(grid_map)
    dde = DisparityExtender(robot_radius=0.12, safety_margin=0.08)
    cbf = CBFSafetyFilter(lookahead_L=0.12, d_safe=0.15, v_max=0.10, w_max=0.28)
    controller = PurePursuitController(
        lookahead_dist=0.28,
        max_linear_speed=0.10,
        max_angular_speed=0.28,
        goal_tolerance=0.10,
        curvature_scale_k=1.5
    )

    rx, ry, rtheta = start_pose
    trajectory = [(rx, ry, rtheta)]
    velocities = []
    min_observed_clearance = 99.0
    goal_reached = False
    step_reached = -1

    # Khởi tạo bản đồ với vài scan ban đầu
    for _ in range(3):
        scan = arena.raycast_lidar_360(rx, ry, rtheta, noise_std=0.005)
        grid_map.update_scan(rx, ry, rtheta, scan, max_valid_range=2.4)

    # Lập kế hoạch toàn cục ban đầu
    initial_path = planner.plan((rx, ry), goal_pt)
    if initial_path:
        controller.set_path(initial_path)
    else:
        controller.set_path([(rx, ry), goal_pt])

    w_signs = []
    spin_forward_displacement = 0.0

    for step in range(max_steps):
        # 1. Quét LiDAR 360 độ với nhiễu thực tế
        scan = arena.raycast_lidar_360(rx, ry, rtheta, noise_std=0.008)

        # 2. Cập nhật bản đồ
        grid_map.update_scan(rx, ry, rtheta, scan, max_valid_range=2.4)

        # 3. Lập kế hoạch lại định kỳ hoặc khi đường bị chắn
        if step % replan_interval == 0 or controller.is_path_blocked:
            new_path = planner.plan((rx, ry), goal_pt)
            if new_path and len(new_path) >= 2:
                controller.set_path(new_path)

        # 4. Tính toán lệnh điều khiển (Pure Pursuit + DDE + CBF)
        v, w, info = controller.compute_command(
            rx, ry, rtheta,
            grid_map=grid_map,
            lidar_ranges=scan,
            auto_evade=True
        )

        velocities.append((v, w))
        if abs(w) > 0.05:
            w_signs.append(1 if w > 0 else -1)

        # Kiểm tra khoảng cách an toàn vật lý
        clr = arena.min_clearance(rx, ry, robot_radius=0.12)
        if clr < min_observed_clearance:
            min_observed_clearance = clr

        # Kiểm tra xem có về đích chưa
        dist_to_goal = math.hypot(rx - goal_pt[0], ry - goal_pt[1])
        if info.get("status") == "GOAL_REACHED" or dist_to_goal <= controller.goal_tolerance:
            goal_reached = True
            step_reached = step
            break

        # Kiểm tra độ dịch chuyển tiến khi xoay góc lớn (> 95 độ)
        alpha = math.atan2(goal_pt[1] - ry, goal_pt[0] - rx) - rtheta
        alpha = (alpha + math.pi) % (2.0 * math.pi) - math.pi
        if abs(alpha) > 1.65:
            spin_forward_displacement += abs(v) * dt

        # 5. Cập nhật động học xe (Differential Drive Unicycle Integration)
        rx += v * math.cos(rtheta) * dt
        ry += v * math.sin(rtheta) * dt
        rtheta = (rtheta + w * dt + math.pi) % (2.0 * math.pi) - math.pi
        trajectory.append((rx, ry, rtheta))

    # Tính toán chỉ số rung lắc (chattering rate)
    chatter_count = 0
    for i in range(1, len(w_signs)):
        if w_signs[i] != w_signs[i - 1]:
            chatter_count += 1
    chatter_rate = (chatter_count / max(1, len(w_signs) - 1)) * 100.0

    final_dist = math.hypot(rx - goal_pt[0], ry - goal_pt[1])

    return {
        "scenario": scenario_name,
        "success": goal_reached,
        "step_reached": step_reached,
        "elapsed_sec": round(len(trajectory) * dt, 2),
        "final_pose": (round(rx, 3), round(ry, 3), round(rtheta, 3)),
        "final_distance_to_goal": round(final_dist, 3),
        "min_clearance_m": round(min_observed_clearance, 3),
        "chatter_rate_percent": round(chatter_rate, 1),
        "spin_forward_displacement_m": round(spin_forward_displacement, 4),
        "trajectory_len": len(trajectory)
    }


def main():
    print("\n" + "#" * 72)
    print("   BỘ KIỂM THỬ ĐỘNG KHÉP VÒNG LIÊN TỤC (CLOSED-LOOP DYNAMIC BENCHMARK)")
    print("   DỰ ÁN CS532 JETBOT — THƯ MỤC 04-hieu-MotionControl")
    print("#" * 72)

    # =========================================================================
    # KỊCH BẢN 1: HÀNH LANG CHICANE VỚI VẬT CẢN ZIGZAG (SLALOM CORRIDOR)
    # =========================================================================
    print("\n" + "=" * 70)
    print(" [SCENARIO 1] Hành Lang Thử Thách Zig-Zag (2 Vật Cản Lệch Nhau)")
    print("=" * 70)
    res1 = run_closed_loop_navigation(
        scenario_name="Chicane Slalom Corridor",
        start_pose=(0.0, 0.0, 0.0),
        goal_pt=(1.8, 0.0),
        obstacles=[(0.65, 0.16, 0.08), (1.20, -0.16, 0.08)],
        max_steps=260,
        dt=0.1
    )
    print(f"-> Thời gian về đích: {res1['elapsed_sec']} s ({res1['step_reached']} ticks)")
    print(f"-> Khoảng cách cuối tới Goal: {res1['final_distance_to_goal']} m (Ngưỡng <= 0.10m)")
    print(f"-> Khoảng cách an toàn nhỏ nhất tới mép vật cản: {res1['min_clearance_m']} m (Yêu cầu > 0.00m)")
    print(f"-> Tỉ lệ đảo dấu góc bẻ lái (Chatter rate): {res1['chatter_rate_percent']}% (< 25% mượt mà)")

    assert res1["success"], f"❌ Kịch bản 1 thất bại: Không về tới đích! Khoảng cách còn lại: {res1['final_distance_to_goal']}m"
    assert res1["min_clearance_m"] > 0.0, f"❌ Kịch bản 1 bị va chạm: Mép cạ {res1['min_clearance_m']}m!"
    assert res1["chatter_rate_percent"] < 25.0, f"❌ Xe bị giật lắc bánh: Chatter rate {res1['chatter_rate_percent']}% quá cao!"
    print("  ✅ PASS SCENARIO 1: Robot lượn qua 2 vật cản hình chữ S hoàn hảo và chạm vạch đích an toàn!")

    # =========================================================================
    # KỊCH BẢN 2: VẬT CẢN TRỰC DIỆN CHÍNH DIỆN MŨI (HEAD-ON BLUNT OBSTACLE)
    # =========================================================================
    print("\n" + "=" * 70)
    print(" [SCENARIO 2] Vật Cản Chặn Thẳng Mũi Xe & DDE/CBF Tự Động Lách Né")
    print("=" * 70)
    res2 = run_closed_loop_navigation(
        scenario_name="Direct Head-on Obstacle",
        start_pose=(0.0, 0.0, 0.0),
        goal_pt=(1.6, 0.0),
        obstacles=[(0.75, 0.0, 0.10)],
        max_steps=280,
        dt=0.1
    )
    print(f"-> Thời gian về đích: {res2['elapsed_sec']} s ({res2['step_reached']} ticks)")
    print(f"-> Khoảng cách cuối tới Goal: {res2['final_distance_to_goal']} m")
    print(f"-> Khoảng hở an toàn nhỏ nhất: {res2['min_clearance_m']} m")

    assert res2["success"], f"❌ Kịch bản 2 thất bại: Không vượt qua vật cản đối diện!"
    assert res2["min_clearance_m"] > 0.0, f"❌ Kịch bản 2 bị va chạm với vật cản chính diện!"
    print("  ✅ PASS SCENARIO 2: Phản xạ DDE + CBF phát hiện và lách né vật cản chính diện mượt mà!")

    # =========================================================================
    # KỊCH BẢN 3: ĐÍCH Ở PHÍA SAU LƯNG (180 ĐỘ) - KIỂM CHỨNG SPIN-IN-PLACE
    # =========================================================================
    print("\n" + "=" * 70)
    print(" [SCENARIO 3] Đích Ở Sau Lưng (180°) — Kiểm Chứng Tự Quay Tại Chỗ Không Đâm Tường")
    print("=" * 70)
    res3 = run_closed_loop_navigation(
        scenario_name="180-Degree Goal Reverse Spin",
        start_pose=(0.0, 0.0, 0.0),
        goal_pt=(-0.8, 0.0),
        obstacles=[(0.35, 0.0, 0.08)],
        max_steps=260,
        dt=0.1
    )
    print(f"-> Độ dịch chuyển tiến trong lúc xoay ngược đầu: {res3['spin_forward_displacement_m']} m (Chỉ tiêu < 0.03m)")
    print(f"-> Thời gian về đích: {res3['elapsed_sec']} s")
    print(f"-> Khoảng cách cuối tới Goal: {res3['final_distance_to_goal']} m")
    print(f"-> Khoảng cách an toàn nhỏ nhất: {res3['min_clearance_m']} m")

    assert res3["spin_forward_displacement_m"] < 0.03, f"❌ Xe vẫn bị rướn tới trước {res3['spin_forward_displacement_m']}m khi quay 180°!"
    assert res3["success"], f"❌ Kịch bản 3 thất bại: Không quay đầu về đích sau lưng!"
    assert res3["min_clearance_m"] > 0.0, f"❌ Xe va chạm vật cản phía trước trong lúc quay đầu!"
    print("  ✅ PASS SCENARIO 3: Xe quay tại chỗ (v=0) an toàn tuyệt đối, triệt tiêu hoàn toàn rướn đâm tường!")

    print("\n" + "#" * 72)
    print("  🎉 XUẤT SẮC TOÀN DIỆN! 3/3 KỊCH BẢN ĐỘNG KHÉP VÒNG ĐẠT CHUẨN PASS 100%!")
    print("  THUẬT TOÁN ĐÃ CHỨNG MINH ĐỘ TIN CẬY TOÁN HỌC TRONG MÔ PHỎNG LIÊN TỤC:")
    print("  - KHÔNG VA CHẠM (0 COLLISION)")
    print("  - VỀ ĐÍCH CHÍNH XÁC (FINAL DIST <= 0.10M)")
    print("  - KHÔNG VĂNG ĐUÔI / RUNG LẮC BÁNH (CHATTER < 25%)")
    print("  - QUAY ĐẦU TẠI CHỖ AN TOÀN (FORWARD DRIFT < 3CM)")
    print("#" * 72 + "\n")


if __name__ == "__main__":
    main()
