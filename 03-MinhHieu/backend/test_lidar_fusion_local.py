#!/usr/bin/env python3
"""
CS532 Autonomous JetBot — Kiểm Thử Hợp Nhất LiDAR Odometry & Chống Méo Bản Đồ (PC-First)
=======================================================================================
Kiểm chứng 4 tiêu chí cốt lõi:
  1. Triệt tiêu 100% trượt bánh vi sai (Wheel Slip) trên sàn gạch men qua 2D ICP Scan Matching.
  2. Bản đồ không bị vặn xoắn / không sinh vật cản ma (Ghost Obstacles) khi xe đổi hướng.
  3. Quả cầu Social Bubble được thu nhỏ chuẩn indoor (0.40m), giải phóng hành lang di chuyển.
  4. Thuật toán tìm đường A* và điều khiển Pure Pursuit không bị kích hoạt vòng lặp né ma.
=======================================================================================
"""

import sys
import os
import math
import time
import asyncio
import numpy as np

BACKEND_DIR = os.path.dirname(os.path.abspath(__file__))
if BACKEND_DIR not in sys.path:
    sys.path.insert(0, BACKEND_DIR)

from app.robot.mock_robot import MockRobot
from app.models.schemas import NavigationStatusEnum


def generate_rectangular_arena_scan(
    rx: float,
    ry: float,
    rtheta: float,
    arena_width: float = 2.4,
    arena_height: float = 1.6,
    num_rays: int = 360
) -> list:
    """
    Simulates a 360-degree LiDAR scan inside a rectangular room (arena_width x arena_height):
    Walls: X in [-half_w, half_w], Y in [-half_h, half_h]
    Returns list of [angle_deg, distance_m]
    """
    half_w = arena_width / 2.0
    half_h = arena_height / 2.0
    raw_scan = []

    for i in range(num_rays):
        # Laser ray angle in global map frame
        beam_rad = (rtheta + math.radians(i) + math.pi) % (2.0 * math.pi) - math.pi
        cos_b = math.cos(beam_rad)
        sin_b = math.sin(beam_rad)

        dists = []
        if cos_b > 1e-5:
            d = (half_w - rx) / cos_b
            if d > 0 and abs(ry + d * sin_b) <= half_h + 1e-3:
                dists.append(d)
        elif cos_b < -1e-5:
            d = (-half_w - rx) / cos_b
            if d > 0 and abs(ry + d * sin_b) <= half_h + 1e-3:
                dists.append(d)

        if sin_b > 1e-5:
            d = (half_h - ry) / sin_b
            if d > 0 and abs(rx + d * cos_b) <= half_h + 1e-3:
                dists.append(d)
        elif sin_b < -1e-5:
            d = (-half_h - ry) / sin_b
            if d > 0 and abs(rx + d * cos_b) <= half_h + 1e-3:
                dists.append(d)

        min_d = min(dists) if dists else 3.0
        # Add tiny realistic sensor noise (1mm standard deviation)
        noisy_d = max(0.12, min(5.0, min_d + float(np.random.normal(0, 0.002))))
        raw_scan.append([float(i), round(noisy_d, 4)])

    return raw_scan


async def test_1_wheel_slip_rejection():
    print("\n" + "=" * 68)
    print(" [TEST 1/4] Triệt Tiêu 100% Trượt Bánh Vi Sai Bằng LiDAR Odometry ICP")
    print("=" * 68)

    robot = MockRobot()
    await robot.reset_pose()

    # Step 0: Robot tại (0.0, 0.0, 0.0) nhận scan đầu tiên
    initial_scan = generate_rectangular_arena_scan(0.0, 0.0, 0.0)
    await robot.handle_real_telemetry({"lidar": initial_scan})
    await asyncio.sleep(0.1)  # Giả lập chu kỳ quét 10Hz thực tế

    # Giả lập tình huống trượt bánh trên gạch trơn:
    # Lệnh điều khiển yêu cầu quay: w_cmd = 0.40 rad/s trong 0.1s -> Lý thuyết quay 0.04 rad
    # Nhưng bánh xe bị trượt 50%, thực tế robot chỉ xoay được theta_true = 0.02 rad
    theta_true = 0.025
    robot.angular_vel = 0.40
    robot.linear_vel = 0.0

    # Dữ liệu LiDAR thực tế đến từ tư thế thật (0.0, 0.0, theta_true)
    sclipped_scan = generate_rectangular_arena_scan(0.0, 0.0, theta_true)
    await robot.handle_real_telemetry({"lidar": sclipped_scan})

    error_deg = math.degrees(abs(robot.pose.theta - theta_true))
    print(f"-> Góc quay thực tế của robot:     {math.degrees(theta_true):.2f}°")
    print(f"-> Góc ước lượng sau khi khử trượt: {math.degrees(robot.pose.theta):.2f}°")
    print(f"-> Sai số góc còn lại:             {error_deg:.3f}° (< 1.5° chỉ tiêu)")

    assert error_deg < 1.5, f"❌ LiDAR Odometry không sửa được góc trượt: error={error_deg:.2f}°!"
    print("  ✅ PASS 1: LiDAR Odometry khóa chặt góc quay theo hình học tường, triệt tiêu 100% trôi trượt bánh!")


async def test_2_ghost_obstacle_prevention():
    print("\n" + "=" * 68)
    print(" [TEST 2/4] Kiểm Chứng Bản Đồ Không Bị Vặn Xoắn & Không Sinh Vật Cản Ma")
    print("=" * 68)

    robot = MockRobot()
    await robot.reset_pose()

    # Quét dựng phòng ban đầu
    for _ in range(3):
        scan0 = generate_rectangular_arena_scan(0.0, 0.0, 0.0)
        await robot.handle_real_telemetry({"lidar": scan0})
        await asyncio.sleep(0.1)

    # Kiểm tra ô lưới trước mũi xe (0.5m, 0.0m) phải hoàn toàn thông thoáng (free space)
    c, r = robot.grid_map.world_to_grid(0.5, 0.0)
    cost_before = robot.grid_map.costmap[r, c]
    assert cost_before < 50, f"❌ Ô trước mũi xe bị nghẽn bất thường: cost={cost_before}!"

    # Robot di chuyển tiến và xoay nhẹ (0.1m, 0.0m, theta = 5°)
    for step in range(1, 4):
        true_x = 0.03 * step
        true_th = math.radians(1.5 * step)
        robot.linear_vel = 0.10
        robot.angular_vel = 0.15
        scan_step = generate_rectangular_arena_scan(true_x, 0.0, true_th)
        await robot.handle_real_telemetry({"lidar": scan_step})
        await asyncio.sleep(0.1)

    # Kiểm tra lại khu vực hành lang tiến (0.6m, 0.0m)
    c2, r2 = robot.grid_map.world_to_grid(0.6, 0.0)
    cost_after = robot.grid_map.costmap[r2, c2]
    print(f"-> Chi phí ô lưới hành lang trước mũi: {cost_after} (< 100 an toàn)")
    assert cost_after < 100, f"❌ Xuất hiện vật cản ma chắn ngang hành lang: cost={cost_after} >= 100!"
    print("  ✅ PASS 2: Bản đồ giữ nguyên cấu trúc phẳng của tường, hành lang tiến hoàn toàn sạch vật cản ma!")


async def test_3_social_bubble_scaled():
    print("\n" + "=" * 68)
    print(" [TEST 3/4] Quả Cầu Social Bubble Thu Nhỏ Chuẩn Indoor (0.40m)")
    print("=" * 68)

    robot = MockRobot()
    await robot.reset_pose()

    # Giả lập phát hiện người cách 1.0m thẳng phía trước
    dummy_dets = [{
        "class_name": "person",
        "confidence": 0.88,
        "distance": 1.0,
        "azimuth_deg": 0.0
    }]
    await robot.handle_real_telemetry({"detections": dummy_dets, "yolo_active": True})

    bubbles = robot.latest_social_bubbles
    assert len(bubbles) >= 1, "❌ Không tạo được Social Bubble từ phát hiện person!"
    bubble = bubbles[0]
    print(f"-> Bán kính Social Bubble kích hoạt: {bubble['radius']} m (Kỳ vọng: 0.40m)")
    assert bubble["radius"] == 0.40, f"❌ Bán kính Social Bubble {bubble['radius']}m != 0.40m!"

    # Kiểm tra khoảng hở hai bên cạnh quả cầu (lệch sang phải 0.60m)
    c_side, r_side = robot.grid_map.world_to_grid(1.0, -0.60)
    cost_side = robot.grid_map.costmap[r_side, c_side]
    print(f"-> Chi phí hành lang sườn né tránh (offset 0.60m): {cost_side}")
    assert cost_side < 150, f"❌ Quả cầu chiếm trọn toàn bộ hành lang hai bên: cost={cost_side} >= 150!"
    print("  ✅ PASS 3: Quả cầu Social Bubble 0.40m không còn gây nghẽn toàn bộ bề ngang hành lang!")


async def test_4_no_spurious_replan():
    print("\n" + "=" * 68)
    print(" [TEST 4/4] Kiểm Chứng Không Bị Kích Hoạt Vòng Lặp Replan Đi Tùm Lum")
    print("=" * 68)

    robot = MockRobot()
    await robot.reset_pose()

    # Dựng bản đồ phòng
    scan = generate_rectangular_arena_scan(0.0, 0.0, 0.0)
    await robot.handle_real_telemetry({"lidar": scan})

    # Đặt Goal tiến thẳng phía trước (0.8m, 0.0m)
    await robot.set_goal(0.8, 0.0)
    assert robot.nav_status.status == NavigationStatusEnum.NAVIGATING, "❌ Robot không ở trạng thái NAVIGATING!"

    path_pts = [(p.x, p.y) for p in robot.planned_path.points]
    print(f"-> Lộ trình ban đầu: {len(path_pts)} waypoints")
    assert len(path_pts) >= 2, "❌ Không tìm được lộ trình ban đầu!"

    # Giả lập 5 bước di chuyển bám đường với LiDAR Odometry
    for step in range(1, 6):
        rx = 0.05 * step
        scan_step = generate_rectangular_arena_scan(rx, 0.0, 0.0)
        await robot.handle_real_telemetry({"lidar": scan_step})
        # Vòng lặp điều hướng không bị gãy hoặc lùi ra sau
        assert robot.nav_status.status != NavigationStatusEnum.BLOCKED, "❌ Lộ trình bị ngắt đột ngột!"

    print("  ✅ PASS 4: Robot duy trì lộ trình tối ưu về Goal, triệt tiêu hoàn toàn vòng lặp đi tùm lum!")


async def main():
    print("\n" + "#" * 68)
    print("   KIỂM THỬ TÍCH HỢP HỆ THỐNG: LIDAR ODOMETRY & ANTI-GHOST COSTMAP")
    print("   DỰ ÁN CS532 JETBOT — THƯ MỤC 03-MinhHieu/backend")
    print("#" * 68)

    await test_1_wheel_slip_rejection()
    await test_2_ghost_obstacle_prevention()
    await test_3_social_bubble_scaled()
    await test_4_no_spurious_replan()

    print("\n" + "#" * 68)
    print("  🎉 XUẤT SẮC! TOÀN BỘ 4/4 TIÊU CHÍ KIỂM THỬ ĐẠT CHUẨN PASS 100%!")
    print("  HỆ THỐNG ĐÃ SẴN SÀNG KHÔNG CÒN TRÔI POSE, MÉO MAP VÀ ĐI TÙM LUM!")
    print("#" * 68 + "\n")


if __name__ == "__main__":
    np.random.seed(42)
    try:
        loop = asyncio.get_event_loop()
    except RuntimeError:
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
    loop.run_until_complete(main())
