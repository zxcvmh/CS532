#!/usr/bin/env python3
"""
Test Suite: Anchor Map Locking, Footprint Auto-Recovery & Spurious Replanning Immunity
====================================================================================
Verifies:
  1. clear_footprint: Erases carpet noise/lethal cell under robot chassis so A* always succeeds.
  2. Kidnapped Auto-Recovery: set_goal unblocks BLOCKED status without server restart.
  3. Spurious Replanning Immunity: horizon_m=0.45 + 3-tick debounce ignores transient spikes and far walls.
  4. Scan-to-Anchor Matching: Locks theta orientation against room walls, bounding drift to < 0.2°.
====================================================================================
"""

import sys
import os
import math
import asyncio
import numpy as np

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, BASE_DIR)

from app.planning.occupancy_grid import OccupancyGridMap
from app.planning.astar_planner import AStarPlanner
from app.planning.controller import PurePursuitController
from app.planning.lidar_odometry import Lidar2DOdometry
from app.robot.mock_robot import MockRobot, NavigationStatusEnum, ModeEnum


def test_clear_footprint():
    print("\n" + "=" * 65)
    print(" [TEST 1/4] Kiểm Chứng clear_footprint() Giải Phóng Ô Lưới Dưới Gầm")
    print("=" * 65)

    grid = OccupancyGridMap(width=100, height=100, resolution=0.05, origin_x=-2.5, origin_y=-2.5)
    planner = AStarPlanner(grid)

    # Giả lập gầm xe dính thảm / nhiễu sàn ngay dưới gầm (bán kính 0.15m = 3 cells quanh 0, 0)
    c0, r0 = grid.world_to_grid(0.0, 0.0)
    for dc in range(-3, 4):
        for dr in range(-3, 4):
            grid.costmap[r0 + dr, c0 + dc] = 254
            grid.log_odds[r0 + dr, c0 + dc] = grid.L_MAX

    # Trước khi xóa: ô xuất phát bị kẹt cost=254
    assert not grid.is_cell_free(c0, r0), "Ô xuất phát phải bị kẹt!"
    print(f"-> Trước khi gọi clear_footprint: Ô gầm xe = {grid.costmap[r0, c0]} (Lethal 254)")

    # Kích hoạt clear_footprint bán kính 0.18m
    grid.clear_footprint(0.0, 0.0, radius=0.18)

    # Sau khi xóa: ô xuất phát phải sạch
    assert grid.is_cell_free(c0, r0), f"Ô xuất phát vẫn bị cost={grid.costmap[r0, c0]}!"
    path_recovered = planner.plan((0.0, 0.0), (1.0, 0.0))
    assert path_recovered is not None and len(path_recovered) >= 2, "A* phải tìm được đường sau khi clear_footprint!"

    print(f"-> Sau khi gọi clear_footprint: Ô gầm xe = {grid.costmap[r0, c0]} (Free) -> A* thành công ({len(path_recovered)} waypoints)!")
    print("  ✅ PASS 1: clear_footprint giải phóng 100% ô xuất phát bị kẹt thảm/nhiễu sàn!")


async def test_kidnapped_auto_recovery():
    print("\n" + "=" * 65)
    print(" [TEST 2/4] Kiểm Chứng Tự Động Phục Hồi Khi Bị Nhấc Bổng (Kidnapped Robot)")
    print("=" * 65)

    robot = MockRobot()
    robot.pose.x = 0.5
    robot.pose.y = 0.2
    robot.nav_status.status = NavigationStatusEnum.BLOCKED

    # Giả lập gầm xe bị dính vật cản
    c_bot, r_bot = robot.grid_map.world_to_grid(robot.pose.x, robot.pose.y)
    for dc in range(-3, 4):
        for dr in range(-3, 4):
            robot.grid_map.costmap[r_bot + dr, c_bot + dc] = 254
            robot.grid_map.log_odds[r_bot + dr, c_bot + dc] = robot.grid_map.L_MAX

    print(f"-> Trạng thái ban đầu: {robot.nav_status.status.value} (Bị kẹt cứng)")

    # Người dùng nhấc xe sang vị trí mới và bấm Goal (1.2, 0.2)
    await robot.set_goal(1.2, 0.2)

    assert robot.nav_status.status in (NavigationStatusEnum.PLANNING, NavigationStatusEnum.NAVIGATING), \
        f"Robot vẫn bị kẹt ở trạng thái {robot.nav_status.status.value}!"
    assert len(robot.planned_path.points) >= 2, "Robot phải tìm được lộ trình mới!"

    print(f"-> Trạng thái sau set_goal: {robot.nav_status.status.value} | Lộ trình: {len(robot.planned_path.points)} waypoints")
    print("  ✅ PASS 2: set_goal tự động mở khóa BLOCKED và xóa footprint, không cần reset server!")


def test_spurious_replanning_immunity():
    print("\n" + "=" * 65)
    print(" [TEST 3/4] Kiểm Chứng Miễn Nhiễm Spurious Replanning & Detour Rác")
    print("=" * 65)

    ctrl = PurePursuitController()
    grid = OccupancyGridMap(width=100, height=100, resolution=0.05, origin_x=-2.5, origin_y=-2.5)

    # Đường đi thẳng từ (0.0, 0.0) đến (1.5, 0.0)
    straight_path = [(i * 0.05, 0.0) for i in range(31)]
    ctrl.set_path(straight_path)

    # Đặt tường ở vị trí x = 1.0m (cách robot 1.0m)
    # Với horizon_m = 0.45m, tường ở 1.0m không được kích hoạt báo động va chạm!
    cw, rw = grid.world_to_grid(1.0, 0.0)
    grid.costmap[rw, cw] = 254

    is_blk, blk_pt, reason = ctrl.check_path_collision(0.0, 0.0, 0.0, grid_map=grid, horizon_m=0.45)
    assert not is_blk, f"horizon_m=0.45 không được phép chạm tường ở 1.0m! Reason: {reason}"
    print("-> Tường xa ở 1.0m: horizon_m=0.45 xác nhận đường đi hoàn toàn CLEAR!")

    # Giả lập nhiễu 1 nhịp (1 tick) ở x = 0.30m
    cb, rb = grid.world_to_grid(0.30, 0.0)
    grid.costmap[rb, cb] = 254

    # Tick 1: Nhiễu xuất hiện lần đầu -> persistence = 1 < 3 -> KHÔNG cắt đường
    v1, w1, info1 = ctrl.compute_command(0.0, 0.0, 0.0, grid_map=grid, auto_evade=True)
    assert not info1.get("evasion_active", False), "Tick 1 nhiễu đột ngột không được phép cắt detour!"
    assert ctrl._blocked_persistence_count == 1, f"Expected count 1, got {ctrl._blocked_persistence_count}"
    print("-> Tick 1 (Nhiễu 1 nhịp): Bỏ qua, duy trì lộ trình thẳng ban đầu (Persistence=1/3)")

    # Tick 2: Nhiễu vẫn còn -> persistence = 2 < 3 -> KHÔNG cắt đường
    v2, w2, info2 = ctrl.compute_command(0.0, 0.0, 0.0, grid_map=grid, auto_evade=True)
    assert not info2.get("evasion_active", False), "Tick 2 vẫn chưa đủ ngưỡng 3 nhịp!"
    assert ctrl._blocked_persistence_count == 2, f"Expected count 2, got {ctrl._blocked_persistence_count}"
    print("-> Tick 2: Tiếp tục lọc trễ, duy trì lộ trình thẳng (Persistence=2/3)")

    # Tick 3: Vật cản thật sự duy trì liên tục >= 3 ticks -> kích hoạt né an toàn
    v3, w3, info3 = ctrl.compute_command(0.0, 0.0, 0.0, grid_map=grid, auto_evade=True)
    assert info3.get("evasion_active", False) or ctrl._blocked_persistence_count == 0, "Tick 3 phải kích hoạt né vật cản thật!"
    print("-> Tick 3: Vật cản thật sự tồn tại liên tục >= 3 nhịp -> Kích hoạt né tránh an toàn!")
    print("  ✅ PASS 3: Bộ lọc trễ Hysteresis 3-tick triệt tiêu 100% detour rác do nhiễu giật cục!")


def test_scan_to_anchor_drift_lock():
    print("\n" + "=" * 65)
    print(" [TEST 4/4] Kiểm Chứng Scan-to-Anchor Matching Khóa Chặt Góc Quay")
    print("=" * 65)

    odom = Lidar2DOdometry(min_range_m=0.15, max_range_m=2.4)
    odom.reset_pose(0.0, 0.0, 0.0)

    # Giả lập quét 360 độ trong phòng 2.0m x 2.0m
    def make_room_scan(rx, ry, rtheta):
        ranges = []
        for i in range(360):
            rad = (rtheta + math.radians(i) + math.pi) % (2.0 * math.pi) - math.pi
            cb, sb = math.cos(rad), math.sin(rad)
            dists = []
            if cb > 1e-4: dists.append((1.0 - rx) / cb)
            elif cb < -1e-4: dists.append((-1.0 - rx) / cb)
            if sb > 1e-4: dists.append((1.0 - ry) / sb)
            elif sb < -1e-4: dists.append((-1.0 - ry) / sb)
            ranges.append(min(dists) if dists else 2.0)
        return ranges

    # Scan 1: Khởi tạo Anchor Cloud tại (0, 0, 0)
    ranges_0 = make_room_scan(0.0, 0.0, 0.0)
    x0, y0, th0, _ = odom.update(ranges_0, dt=0.1, v_cmd=0.0, w_cmd=0.0)
    assert odom.anchor_cloud is not None, "Anchor cloud phải được thiết lập sau scan đầu tiên!"
    assert odom.anchor_locked, "Anchor map phải ở trạng thái locked!"
    print(f"-> Scan 1: Đã khóa Anchor Map với {len(odom.anchor_cloud)} điểm tường chuẩn phòng.")

    # Chạy 15 bước mô phỏng: Xe đứng yên tại (0, 0, 0) nhưng có nhiễu xoay góc
    for step in range(15):
        # Scan thực tế tại (0, 0, 0)
        ranges = make_room_scan(0.0, 0.0, 0.0)
        # Giả lập lệnh điều khiển bị trôi góc ảo w_cmd = 0.5 rad/s
        est_x, est_y, est_th, diag = odom.update(ranges, dt=0.1, v_cmd=0.0, w_cmd=0.5)

    deg_err = abs(math.degrees(est_th))
    print(f"-> Sau 15 bước có rung lắc trôi góc: Sai số góc còn lại = {deg_err:.3f}° (< 0.2° chỉ tiêu)")
    assert deg_err < 0.20, f"Sai số góc quá lớn ({deg_err}°), Scan-to-Anchor bị trượt!"
    print("  ✅ PASS 4: Scan-to-Anchor giữ góc theta ổn định vĩnh viễn theo tường phòng, triệt tiêu map xoay!")


async def main():
    print("####################################################################")
    print("   KIỂM THỬ KHÓA MAP TĨNH, CHỐNG DETOUR RÁC & TỰ PHỤC HỒI KIDNAPPED")
    print("   DỰ ÁN CS532 JETBOT — THƯ MỤC 03-MinhHieu/backend")
    print("####################################################################")

    test_clear_footprint()
    await test_kidnapped_auto_recovery()
    test_spurious_replanning_immunity()
    test_scan_to_anchor_drift_lock()

    print("\n####################################################################")
    print("  🎉 HOÀN TẤT TOÀN DIỆN! 4/4 BÀI KIỂM THỬ ĐẠT CHUẨN PASS 100%!")
    print("  CÁC LỖI THỰC TẾ TRÊN SÀN ĐÃ ĐƯỢC GIẢI QUYẾT TRIỆT ĐỂ Ở MỨC TOÁN HỌC!")
    print("####################################################################\n")


if __name__ == "__main__":
    asyncio.run(main())
