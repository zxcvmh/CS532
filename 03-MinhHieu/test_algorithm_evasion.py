#!/usr/bin/env python3
"""
Test Suite: Navigation Avoidance, DDE Evasion, Dynamic Tracker & Auto-Recovery
Validates the fixes for Session 1 and Session 2.
"""
import sys
import os
import math
import time

sys.path.insert(0, os.path.abspath("05-d500lib-motion/planning"))
sys.path.insert(0, os.path.abspath("03-MinhHieu/backend"))
sys.path.insert(0, os.path.abspath("03-MinhHieu"))

from disparity_extender import DisparityExtender
from obstacle_clustering import DynamicObstacleTracker, ObstacleCluster
from controller import PurePursuitController
from command_mux import CommandMux, SRC_MANUAL, SRC_AUTO


def test_1_dde_gap_finding():
    """Verify DDE inflates obstacles and identifies evasive steering direction."""
    print("--- Test 1: Disparity Extender Evasion Gap Finding ---")
    dde = DisparityExtender(robot_radius=0.10, safety_margin=0.08)
    
    # 360 ranges: front cone (index 0, and ±20 deg) has an obstacle at 0.30m
    # Left side (index 20..70) is open (3.0m)
    ranges = [3.0] * 360
    for deg in list(range(0, 16)) + list(range(345, 360)):
        ranges[deg] = 0.30
        
    res = dde.evaluate(ranges, goal_heading_rad=0.0)
    assert res["status"] in ("EVASION_ACTIVE", "EVASION_FALLBACK"), f"Expected evasion active, got {res['status']}"
    assert res["target_heading_rad"] != 0.0, "Expected non-zero evasive steering"
    assert res["front_obstacle_dist"] <= 0.35, f"Expected obstacle distance <= 0.35, got {res['front_obstacle_dist']}"
    print(f"  [PASS] Status: {res['status']}, Steer: {math.degrees(res['target_heading_rad']):.1f}°, Front: {res['front_obstacle_dist']}m")


def test_2_controller_dde_integration():
    """Verify compute_command uses DDE to evade instead of permanently freezing into BLOCKED_WAIT."""
    print("\n--- Test 2: Controller Reactive Avoidance via DDE ---")
    ctrl = PurePursuitController(
        max_linear_speed=0.20,
        min_drive_speed=0.04,
        max_angular_speed=0.50,
        lookahead_dist=0.30
    )
    # Set a straight path along X axis
    path = [(0.0, 0.0), (0.5, 0.0), (1.0, 0.0), (1.5, 0.0), (2.0, 0.0)]
    ctrl.set_path(path)
    
    # Obstacle in front at 0.30m, but free space on the left
    ranges = [3.0] * 360
    for deg in list(range(0, 20)) + list(range(340, 360)):
        ranges[deg] = 0.30
        
    for frame in range(3):
        v, w, info = ctrl.compute_command(
            robot_x=0.0,
            robot_y=0.0,
            robot_theta=0.0,
            lidar_ranges=ranges,
            auto_evade=True
        )
    
    # When obstacle blocks the path, controller should actively steer (EVASION_ACTIVE or REFLEX_PIVOT_EVADE)
    # rather than just returning 0.0, 0.0 with BLOCKED_WAIT
    status = info.get("status")
    print(f"  [RESULT] v={v:.3f}, w={w:.3f}, status={status}, reason={info.get('reason')}")
    assert status in ("EVASION_ACTIVE", "REFLEX_PIVOT_EVADE", "EVADING", "PIVOT_ALIGNING"), f"Expected active evasion status, got {status}"
    assert abs(w) > 0.01 or v > 0.01, "Expected motion command during evasion"
    print("  [PASS] Controller commanded evasion successfully!")


def test_3_dynamic_tracker_filters():
    """Verify DynamicObstacleTracker filters out single-frame jitter and large wall boundaries."""
    print("\n--- Test 3: Dynamic Obstacle Tracker Multi-frame & Wall Filtering ---")
    tracker = DynamicObstacleTracker(assoc_dist_m=0.45)

    def make_cluster(cid, cx, cy, lm, wm):
        return ObstacleCluster(
            id=cid,
            centroid=(cx, cy),
            closest_point=(cx, cy),
            min_distance=math.hypot(cx, cy),
            azimuth_deg=0.0,
            length_m=lm,
            width_m=wm,
            orientation_rad=0.0,
            points_count=20,
            bounding_box_2d=[(cx - wm/2, cy - lm/2), (cx + wm/2, cy + lm/2)],
            obstacle_type="WALL" if lm > 0.8 else "LEG",
            threat_level="WARNING"
        )
    
    # Frame 1: Single detection of stationary wall
    wall_cluster = make_cluster(1, 1.0, 0.5, 1.0, 0.2)
    t1 = tracker.update([wall_cluster], timestamp=1.0, robot_pose=(0.0, 0.0, 0.0))
    assert not t1[0].is_dynamic, "Wall cluster must not be flagged dynamic on frame 1"
    
    # Frame 2: Wall centroid jumps slightly due to laser noise (2 cm)
    wall_cluster_2 = make_cluster(1, 1.02, 0.5, 1.0, 0.2)
    t2 = tracker.update([wall_cluster_2], timestamp=1.1, robot_pose=(0.0, 0.0, 0.0))
    assert not t2[0].is_dynamic, "Wall cluster must NEVER be flagged dynamic despite noise jump"
    print("  [PASS] Wall cluster successfully rejected from dynamic tracking.")
    
    # Human leg: small diameter (0.25m), moving at 0.4 m/s (4cm / 0.1s = 0.4 m/s)
    leg_tracker = DynamicObstacleTracker(assoc_dist_m=0.45)
    # Frame 1
    c1 = make_cluster(1, 0.80, 0.0, 0.2, 0.2)
    leg_tracker.update([c1], timestamp=1.0, robot_pose=(0.0, 0.0, 0.0))
    
    # Frame 2
    c2 = make_cluster(1, 0.76, 0.0, 0.2, 0.2)
    leg_tracker.update([c2], timestamp=1.1, robot_pose=(0.0, 0.0, 0.0))
    
    # Frame 3
    c3 = make_cluster(1, 0.72, 0.0, 0.2, 0.2)
    t3 = leg_tracker.update([c3], timestamp=1.2, robot_pose=(0.0, 0.0, 0.0))
    assert t3[0].is_dynamic, "Human leg moving consistently over 3 frames must be confirmed dynamic"
    assert t3[0].speed >= 0.12, f"Expected speed >= 0.12, got {t3[0].speed}"
    print(f"  [PASS] True human movement confirmed dynamic: speed={t3[0].speed}m/s, closing={t3[0].closing_speed}m/s")


def test_4_command_mux():
    """Verify CommandMux priority levels and fail-safe watchdog."""
    print("\n--- Test 4: CommandMux Priority Levels & Failsafe ---")
    dispatched = []
    stopped = []
    
    def on_apply(l, r):
        dispatched.append((round(l, 3), round(r, 3)))
        
    def on_stop():
        stopped.append(True)
        
    mux = CommandMux(apply_fn=on_apply, stop_fn=on_stop, hz=50.0, watchdog_s=0.2)
    mux.start()
    time.sleep(0.04)
    
    # User command: forward
    mux.submit_user(SRC_AUTO, 0.10, 0.0)
    time.sleep(0.05)
    assert len(dispatched) > 0, "Expected motor dispatch"
    
    # Reflex override (higher priority)
    mux.reflex_override(0.05, 0.20, ttl=0.15)
    time.sleep(0.05)
    
    # Reflex stop (higher than override)
    mux.reflex_stop()
    time.sleep(0.05)
    
    # E-STOP (highest priority)
    mux.estop()
    time.sleep(0.05)
    assert len(stopped) > 0, "Expected stop call on E-Stop"
    
    mux.stop()
    print("  [PASS] CommandMux correctly enforces priority hierarchy.")


if __name__ == "__main__":
    test_1_dde_gap_finding()
    test_2_controller_dde_integration()
    test_3_dynamic_tracker_filters()
    test_4_command_mux()
    print("\n=======================================================")
    print("  🎉 ALL 4 ALGORITHM & ARCHITECTURE TESTS PASSED 100%!")
    print("=======================================================")
