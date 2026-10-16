#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
CS532 AUTONOMOUS JETBOT — BỘ KIỂM THỬ TÍCH HỢP TOÀN DIỆN MPPI (INTEGRATION TEST SUITE)
=====================================================================================
Mục tiêu kiểm thử:
  1. Kiểm tra tính tương thích giao diện (API Conformance): set_path, clear_path, telemetry.
  2. Kiểm tra động học vi sai, deadband motor (v >= 0.12 m/s), và độ trễ (< 5ms).
  3. Kiểm tra phanh phản xạ an toàn phần cứng (Hardware Safety Reflex < 0.16m).
  4. Kiểm tra xoay tại chỗ không trôi tiến (Spin-in-place heading recovery khi lệch > 70°).
  5. Kiểm tra đóng vòng phản hồi liên tục (Closed-Loop Continuous Navigation trên 3 kịch bản).
  6. Kiểm tra tích hợp trực tiếp với MockRobot và vòng lặp FastAPI State Manager.
=====================================================================================
"""

import os
import sys
import time
import math
import asyncio
import numpy as np
from typing import List, Tuple, Dict, Any

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
APP_DIR = os.path.join(SCRIPT_DIR, "app")
if APP_DIR not in sys.path:
    sys.path.insert(0, APP_DIR)

from app.config import Config
from app.planning.occupancy_grid import OccupancyGridMap
from app.planning.astar_planner import AStarPlanner
from app.planning.mppi_controller import MPPIController
from app.robot.mock_robot import MockRobot
from app.models.schemas import ModeEnum, NavigationStatusEnum


class ContinuousSimulationArena:
    """Môi trường 2D liên tục mô phỏng sàn thi đấu và quét LiDAR 360 tia."""
    def __init__(self, x_min=-1.5, x_max=3.0, y_min=-1.5, y_max=1.5):
        self.x_min = x_min
        self.x_max = x_max
        self.y_min = y_min
        self.y_max = y_max
        self.obstacles: List[Tuple[float, float, float]] = []

    def add_obstacle(self, x: float, y: float, r: float):
        self.obstacles.append((x, y, r))

    def raycast_lidar_360(self, rx: float, ry: float, rtheta: float, noise_std=0.005) -> List[float]:
        ranges = []
        for deg in range(360):
            beam_angle = rtheta + math.radians(deg)
            cos_b = math.cos(beam_angle)
            sin_b = math.sin(beam_angle)
            dist_min = 8.0

            if cos_b > 1e-4:
                dist_min = min(dist_min, (self.x_max - rx) / cos_b)
            elif cos_b < -1e-4:
                dist_min = min(dist_min, (self.x_min - rx) / cos_b)

            if sin_b > 1e-4:
                dist_min = min(dist_min, (self.y_max - ry) / sin_b)
            elif sin_b < -1e-4:
                dist_min = min(dist_min, (self.y_min - ry) / sin_b)

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

            if noise_std > 0:
                dist_min += float(np.random.normal(0.0, noise_std))
            ranges.append(max(0.10, min(8.0, dist_min)))
        return ranges

    def min_clearance(self, rx: float, ry: float, robot_radius=0.12) -> float:
        dist_to_wall = min(rx - self.x_min, self.x_max - rx, ry - self.y_min, self.y_max - ry)
        dist_to_obs = 99.0
        for ox, oy, orad in self.obstacles:
            d = math.hypot(rx - ox, ry - oy) - orad
            if d < dist_to_obs:
                dist_to_obs = d
        return min(dist_to_wall, dist_to_obs) - robot_radius


def test_part1_api_and_kinematics():
    print("\n" + "=" * 80)
    print(" [TEST 1] Kiểm tra API Conformance, Deadband & Động học Unicycle MPPI")
    print("=" * 80)

    controller = MPPIController(
        horizon_steps=15,
        num_samples=250,
        dt=0.10,
        max_linear_speed=0.14,
        min_drive_speed=0.12,
        max_angular_speed=0.60,
        goal_tolerance=0.10
    )

    path = [(0.0, 0.0), (0.5, 0.0), (1.0, 0.0), (2.0, 0.0)]
    controller.set_path(path)

    assert controller.global_goal == (2.0, 0.0), "❌ Global goal không khớp!"
    assert not controller.is_goal_reached, "❌ Goal reached bị gán sai!"

    # Test tick đầu tiên (cold start) và tick kế tiếp (steady state)
    _ = controller.compute_command(0.0, 0.0, 0.0)
    v, w, info = controller.compute_command(0.0, 0.0, 0.0)
    print(f"-> Tick điều khiển (steady-state): v={v:.3f} m/s, w={w:+.3f} rad/s, status={info['status']}, latency={info['latency_ms']:.2f}ms")

    assert v >= controller.min_drive_speed, f"❌ Lỗi Deadband: v={v} < {controller.min_drive_speed} m/s khi đang di chuyển tới trước!"
    assert v <= controller.max_linear_speed + 0.01, f"❌ Lỗi Max Speed: v={v} vượt quá {controller.max_linear_speed} m/s!"
    assert abs(w) <= controller.max_angular_speed + 0.01, f"❌ Lỗi Max Angular: |w|={abs(w)} > {controller.max_angular_speed} rad/s!"
    assert info["status"] == "NAVIGATING_MPPI", f"❌ Status không đúng: {info['status']}"
    assert info["latency_ms"] < 5.0, f"❌ Độ trễ ổn định vượt 5.0ms: {info['latency_ms']}ms!"
    assert len(info["best_rollout"]) == 15, f"❌ Best rollout horizon không đủ 15 bước!"

    # Test clear_path
    controller.clear_path()
    assert controller.global_goal is None, "❌ clear_path không dọn global_goal!"
    v_stop, w_stop, info_stop = controller.compute_command(0.0, 0.0, 0.0)
    assert v_stop == 0.0 and w_stop == 0.0, "❌ Xe không dừng khi không có goal!"
    assert info_stop["status"] == "NO_GOAL", "❌ Status không phải NO_GOAL!"

    print("  ✅ PASS TEST 1: API Conformance, Deadband & Ràng buộc động học đạt chuẩn 100%.")


def test_part2_safety_reflex():
    print("\n" + "=" * 80)
    print(" [TEST 2] Kiểm tra Phanh Phản Xạ Khẩn Cấp (Hardware Safety Reflex < 0.16m)")
    print("=" * 80)

    controller = MPPIController(reflex_dist_threshold=0.16)
    controller.set_path([(0.0, 0.0), (2.0, 0.0)])

    # Tạo scan giả lập: tia 0 (chính diện mũi xe) cách 0.11m (< 0.16m)
    mock_ranges = [1.5] * 360
    mock_ranges[0] = 0.11

    v, w, info = controller.compute_command(0.0, 0.0, 0.0, lidar_ranges=mock_ranges)
    print(f"-> Phản xạ khi vật cản cách 0.11m: v={v}, w={w}, Status={info['status']}, Reason={info.get('reason')}")

    assert v == 0.0 and w == 0.0, f"❌ Phanh phản xạ thất bại: xe vẫn ra lệnh v={v}, w={w}!"
    assert info["status"] == "EMERGENCY_STOP", f"❌ Status không phải EMERGENCY_STOP ({info['status']})!"
    assert controller.is_reflex_stopped, "❌ Flag is_reflex_stopped không được bật!"

    # Khi vật cản rút lui (> 0.50m)
    clear_ranges = [1.5] * 360
    v_rec, w_rec, info_rec = controller.compute_command(0.0, 0.0, 0.0, lidar_ranges=clear_ranges)
    print(f"-> Phục hồi sau khi vật cản biến mất: v={v_rec}, w={w_rec}, Status={info_rec['status']}")
    assert not controller.is_reflex_stopped, "❌ Flag is_reflex_stopped không được tự xóa!"
    assert v_rec >= controller.min_drive_speed, "❌ Xe không phục hồi vận tốc tiến sau khi đường thông!"

    print("  ✅ PASS TEST 2: Phanh phản xạ khẩn cấp 10Hz hoạt động chính xác tuyệt đối.")


def test_part3_spin_in_place_recovery():
    print("\n" + "=" * 80)
    print(" [TEST 3] Kiểm tra Xoay Tại Chỗ Khi Đích Ở Sau Lưng (180° Spin-in-Place)")
    print("=" * 80)

    controller = MPPIController(
        max_linear_speed=0.10,
        min_drive_speed=0.08,
        max_angular_speed=0.28
    )
    # Robot hướng +x (0 rad), đích ở (-2.0, 0.0) -> Lệch 180 độ
    controller.set_path([(0.0, 0.0), (-2.0, 0.0)])

    v, w, info = controller.compute_command(0.0, 0.0, 0.0)
    print(f"-> Lệnh điều khiển khi quay 180°: v={v:.3f} m/s, w={w:+.3f} rad/s")

    assert v == 0.0, f"❌ Lỗi rướn đâm tường: v={v} > 0 khi góc lệch 180°!"
    assert abs(w) >= 0.15, f"❌ Tốc độ xoay quá yếu (|w|={abs(w)} < 0.15 rad/s)!"
    assert abs(w) <= 0.28 + 0.01, f"❌ Tốc độ xoay vượt trần trượt bánh (|w|={abs(w)} > 0.28 rad/s)!"

    print("  ✅ PASS TEST 3: Xoay tại chỗ an toàn tuyệt đối, triệt tiêu hoàn toàn rướn tiến.")


def test_part4_closed_loop_continuous_scenarios():
    print("\n" + "=" * 80)
    print(" [TEST 4] Kiểm tra Đóng Vòng Điều Hướng Liên Tục (3 Kịch Bản Khép Vòng)")
    print("=" * 80)

    scenarios = [
        {
            "name": "Scenario 1: Chicane Slalom Corridor",
            "start": (0.0, 0.0, 0.0),
            "goal": (1.8, 0.0),
            "obstacles": [(0.65, 0.16, 0.08), (1.20, -0.16, 0.08)],
            "max_steps": 350
        },
        {
            "name": "Scenario 2: Direct Head-on Obstacle",
            "start": (0.0, 0.0, 0.0),
            "goal": (1.6, 0.0),
            "obstacles": [(0.75, 0.0, 0.10)],
            "max_steps": 350
        },
        {
            "name": "Scenario 3: 180-Degree Reverse Goal",
            "start": (0.0, 0.0, 0.0),
            "goal": (-0.8, 0.0),
            "obstacles": [(0.35, 0.0, 0.08)],
            "max_steps": 350
        }
    ]

    for sc in scenarios:
        print(f"\n--- Đang chạy {sc['name']} ---")
        arena = ContinuousSimulationArena(x_min=-1.5, x_max=3.0, y_min=-1.5, y_max=1.5)
        for ox, oy, orad in sc["obstacles"]:
            arena.add_obstacle(ox, oy, orad)

        grid_map = OccupancyGridMap(width=100, height=100, resolution=0.05, origin_x=-2.0, origin_y=-2.0)
        grid_map.inflation_radius = 0.10
        planner = AStarPlanner(grid_map)
        controller = MPPIController(
            horizon_steps=15,
            num_samples=250,
            dt=0.10,
            max_linear_speed=0.10,
            min_drive_speed=0.08,
            max_angular_speed=0.28,
            goal_tolerance=0.10
        )

        rx, ry, rtheta = sc["start"]
        dt = 0.10
        goal_reached = False
        min_clearance_observed = 99.0
        latencies = []

        # Warm-up scans
        for _ in range(3):
            scan = arena.raycast_lidar_360(rx, ry, rtheta)
            grid_map.update_scan(rx, ry, rtheta, scan, max_valid_range=2.4)

        init_path = planner.plan((rx, ry), sc["goal"])
        controller.set_path(init_path if init_path else [(rx, ry), sc["goal"]])

        for step in range(sc["max_steps"]):
            scan = arena.raycast_lidar_360(rx, ry, rtheta)
            grid_map.update_scan(rx, ry, rtheta, scan, max_valid_range=2.4)

            # Replan nếu cần
            if step % 5 == 0 or controller.is_path_blocked:
                new_path = planner.plan((rx, ry), sc["goal"])
                if new_path and len(new_path) >= 2:
                    controller.set_path(new_path)

            v, w, info = controller.compute_command(
                rx, ry, rtheta,
                grid_map=grid_map,
                lidar_ranges=scan
            )
            latencies.append(info["latency_ms"])

            # Đo khoảng hở an toàn
            clr = arena.min_clearance(rx, ry, robot_radius=0.12)
            if clr < min_clearance_observed:
                min_clearance_observed = clr

            dist_to_goal = math.hypot(rx - sc["goal"][0], ry - sc["goal"][1])
            if info["status"] == "GOAL_REACHED" or dist_to_goal <= controller.goal_tolerance:
                goal_reached = True
                break

            # Forward kinematics integration
            rx += v * math.cos(rtheta) * dt
            ry += v * math.sin(rtheta) * dt
            rtheta = (rtheta + w * dt + math.pi) % (2.0 * math.pi) - math.pi

        avg_lat = np.mean(latencies)
        p95_lat = np.percentile(latencies, 95)
        print(f"-> Kết quả: Reached={goal_reached} | Min Clearance={min_clearance_observed:.3f}m | Latency: Mean={avg_lat:.2f}ms, P95={p95_lat:.2f}ms")

        assert goal_reached, f"❌ {sc['name']} không về tới đích!"
        assert min_clearance_observed > 0.0, f"❌ {sc['name']} bị va chạm (Clearance={min_clearance_observed:.3f}m)!"
        assert p95_lat < 5.0, f"❌ {sc['name']} có P95 latency vượt quá 5.0ms ({p95_lat:.2f}ms)!"
        print(f"  ✅ PASS {sc['name']}")


async def test_part5_mock_robot_integration():
    print("\n" + "=" * 80)
    print(" [TEST 5] Kiểm tra Tích Hợp Trực Tiếp Với MockRobot & State Manager")
    print("=" * 80)

    # Đảm bảo Config dùng MPPI
    Config.NAVIGATION_CONTROLLER = "mppi"

    robot = MockRobot()
    assert isinstance(robot.controller, MPPIController), "❌ MockRobot không khởi tạo đúng MPPIController!"
    print(f"[*] MockRobot Controller Type: {type(robot.controller).__name__}")

    # Chuyển sang AUTONOMOUS mode và ra lệnh navigate_to
    await robot.set_mode(ModeEnum.AUTONOMOUS)
    assert robot.mode == ModeEnum.AUTONOMOUS, "❌ Chuyển mode AUTONOMOUS thất bại!"

    # Điểm đến cách robot 1.5m
    await robot.set_goal(1.5, 0.0)
    assert robot.nav_status.status == NavigationStatusEnum.NAVIGATING, f"❌ Trạng thái navigation không phải NAVIGATING ({robot.nav_status.status})!"
    assert len(robot.planned_path.points) >= 2, "❌ Planned path không có điểm nào!"

    # Bước thử 5 chu kỳ điều khiển
    for i in range(5):
        # Chạy 1 chu kỳ update của controller
        v, w, info = robot.controller.compute_command(
            robot.pose.x, robot.pose.y, robot.pose.theta,
            lidar_ranges=robot.latest_raw_ranges,
            clusters=robot.latest_clusters,
            grid_map=robot.grid_map,
            auto_evade=True
        )
        print(f"  Tick {i+1}: v={v:.3f} m/s, w={w:+.3f} rad/s, Status={info['status']}, Latency={info['latency_ms']:.2f}ms")
        assert v >= 0.08, f"❌ Tick {i+1}: v={v} < 0.08 m/s!"
        assert info["status"] == "NAVIGATING_MPPI", f"❌ Status không phải NAVIGATING_MPPI!"

    # Dừng robot
    await robot.cancel_goal()
    assert robot.nav_status.status == NavigationStatusEnum.CANCELLED, "❌ Hủy điều hướng thất bại!"
    print("  ✅ PASS TEST 5: Tích hợp MockRobot và luồng điều phối State Manager hoàn hảo.")


def main():
    print("\n" + "█" * 80)
    print("   CS532: BỘ KIỂM THỬ TÍCH HỢP TOÀN DIỆN THUẬT TOÁN MPPI VÀO BACKEND")
    print("█" * 80)

    test_part1_api_and_kinematics()
    test_part2_safety_reflex()
    test_part3_spin_in_place_recovery()
    test_part4_closed_loop_continuous_scenarios()
    asyncio.run(test_part5_mock_robot_integration())

    print("\n" + "█" * 80)
    print("  🎉 TẤT CẢ 5 BÀI TEST TÍCH HỢP MPPI ĐÃ VƯỢT QUA 100% (ALL TESTS PASSED)!")
    print("  - ĐỘNG HỌC & DEADBAND MOTOR: ĐẠT")
    print("  - PHANH PHẢN XẠ 10HZ: ĐẠT")
    print("  - XOAY TẠI CHỖ CHỐNG ĐÂM TƯỜNG: ĐẠT")
    print("  - ĐÓNG VÒNG NÉ VẬT CẢN (3/3 KỊCH BẢN): ĐẠT")
    print("  - TÍCH HỢP MOCKROBOT STATE MANAGER: ĐẠT")
    print("█" * 80 + "\n")


if __name__ == "__main__":
    main()
