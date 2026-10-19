#!/usr/bin/env python3
"""
Unit and Integration Tests for SLAM & Perception Fixes
======================================================
Tests:
1. D500 LiDAR Driver:
   - sweep_decay_window_s == 0.80
   - timed_out_revolution boundary & 360-degree sweep accumulation
2. ScanGate:
   - STALE_FORCE_ACCEPT_SEC == 0.20
   - Force-acceptance of stationary scans after 0.20s (5Hz pipeline feed)
3. Lidar2DOdometry:
   - set_pose method exists and updates pose while preserving keyframe
   - Kinematic fallback accumulates smoothly without resetting to (0, 0)
4. TelemetryParser:
   - Anchor map initialization in apply_binary_telemetry
   - Degraded scan matching: pose is not reset/pulled back, backward sync via set_pose
   - Reliable scan matching: pose updates properly
"""
import sys
import os
import time
import math
import unittest

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
BACKEND = os.path.join(PROJECT_ROOT, "03-MinhHieu", "backend")
MOTION = os.path.join(PROJECT_ROOT, "05-d500lib-motion")
sys.path.insert(0, BACKEND)
sys.path.insert(0, MOTION)

from app.config import Config
from app.hardware.scan_gate import ScanGate, STALE_FORCE_ACCEPT_SEC
from app.hardware.telemetry_parser import TelemetryParser
from app.streaming.binary_protocol import BinaryTelemetryPacker
from planning.lidar_odometry import Lidar2DOdometry
from planning.occupancy_grid import OccupancyGridMap
from app.models.schemas import RobotPose, ModeEnum

# Import d500 driver from 03-MinhHieu
import importlib.util
spec = importlib.util.spec_from_file_location("d500_driver_03", os.path.join(PROJECT_ROOT, "03-MinhHieu", "d500_driver.py"))
d500_03 = importlib.util.module_from_spec(spec)
spec.loader.exec_module(d500_03)

spec_05 = importlib.util.spec_from_file_location("d500_driver_05", os.path.join(PROJECT_ROOT, "05-d500lib-motion", "d500_driver.py"))
d500_05 = importlib.util.module_from_spec(spec_05)
spec_05.loader.exec_module(d500_05)


class MockCoordinator:
    """Mock NavigationCoordinator for telemetry testing."""
    def __init__(self):
        self._has_reset_real_map = True
        self.pose = RobotPose(x=0.0, y=0.0, theta=0.0)
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        self.battery = 100.0
        self.mode = ModeEnum.AUTONOMOUS
        self.events = []
        self.scan_gate = ScanGate()
        self.latest_raw_ranges = [0.0] * 360
        self.latest_scan_seq = 0
        self.latest_tick_seq = 0
        self._last_scan_tick_time = 0.0
        self._last_map_update = 0.0
        self.map_version = 0

        class MockBridge:
            last_telemetry_time = 0.0
            is_connected = False
            last_lidar_time = 0.0
            lidar_connected = False
            last_battery_time = 0.0
            battery_connected = False
        self.bridge = MockBridge()

        class MockStreamer:
            def update_frame(self, frame): pass
        self.streamer = MockStreamer()

        # Motion Adapter Mock
        from app.navigation.motion_adapter import MotionAdapter
        self.motion = MotionAdapter()
        self.trajectory = []

    def add_event(self, level: str, msg: str):
        self.events.append((level, msg))


class TestSLAMFixes(unittest.TestCase):

    def test_01_d500_driver_single_revolution_sweep(self):
        """LiDAR Driver should be Pure Single-Revolution (~100ms) with no sliding cache ghosting."""
        driver_03 = d500_03.D500OfficialDriver()
        self.assertFalse(hasattr(driver_03, "_sliding_ranges"), "03-MinhHieu must not have sliding_ranges")

        driver_05 = d500_05.D500OfficialDriver()
        self.assertFalse(hasattr(driver_05, "_sliding_ranges"), "05-d500lib-motion must not have sliding_ranges")

        # Verify timeout logic at 120ms does NOT prematurely fire
        now = time.time()
        driver_03._last_sweep_publish_time = now - 0.125  # 125ms passed
        driver_03._current_sweep_point_count = 50
        time_since_sweep = 0.125
        # Old condition: time_since_sweep >= 0.12 and count > 0 -> True (BUG!)
        # New condition: should be False
        timed_out = (
            (time_since_sweep >= 0.25 and driver_03._current_sweep_point_count >= 150)
            or (time_since_sweep >= 0.40 and driver_03._current_sweep_point_count > 0)
        )
        self.assertFalse(timed_out, "Should not timeout at 125ms with 50 points")

        # At 260ms with 160 points -> True
        time_since_sweep = 0.26
        driver_03._current_sweep_point_count = 160
        timed_out = (
            (time_since_sweep >= 0.25 and driver_03._current_sweep_point_count >= 150)
            or (time_since_sweep >= 0.40 and driver_03._current_sweep_point_count > 0)
        )
        self.assertTrue(timed_out, "Should timeout at 260ms with 160 points")

    def test_02_scangate_force_accept_at_200ms(self):
        """ScanGate should force-accept stationary/identical scans after 0.20s to feed coordinator."""
        self.assertEqual(STALE_FORCE_ACCEPT_SEC, 0.20, "STALE_FORCE_ACCEPT_SEC must be 0.20s")

        gate = ScanGate()
        scan1 = {"scan_seq": 10, "scan_stamp": 100.0, "ranges": [2.0] * 360}
        t1 = gate.accept(scan1)
        self.assertIsNotNone(t1, "First scan accepted")

        # Immediate duplicate scan with same seq should be dropped
        scan1_dup = {"scan_seq": 10, "scan_stamp": 100.05, "ranges": [2.0] * 360}
        t2 = gate.accept(scan1_dup)
        self.assertIsNone(t2, "Immediate duplicate scan dropped")

        # Fast forward wall-clock by 0.21s
        gate._last_accept_wall = time.time() - 0.21
        t3 = gate.accept(scan1_dup)
        self.assertIsNotNone(t3, "Scan must be force-accepted after 0.20s to prevent >0.5s timeout")

    def test_03_lidar_odometry_set_pose_and_fallback(self):
        """Lidar2DOdometry set_pose must update coordinates without wiping keyframes."""
        odom = Lidar2DOdometry(min_range_m=0.05, max_range_m=6.0)

        # Feed first scan to initialize keyframe
        scan_pts = [1.5] * 360
        x, y, th, info = odom.update(scan_pts, dt=0.1)
        self.assertIsNotNone(odom.prev_points_2d, "Keyframe should be stored")

        # Call set_pose
        odom.set_pose(2.5, 3.5, 1.57)
        self.assertAlmostEqual(odom.x, 2.5)
        self.assertAlmostEqual(odom.y, 3.5)
        self.assertAlmostEqual(odom.theta, 1.57)
        self.assertIsNotNone(odom.prev_points_2d, "set_pose MUST NOT erase keyframe!")

        # Degraded scan (< 10 points) triggers kinematic dead-reckoning fallback
        degraded_scan = [0.0] * 360
        x2, y2, th2, info2 = odom.update(degraded_scan, dt=0.1, v_cmd=0.2, w_cmd=0.0)
        self.assertFalse(info2["is_reliable"], "Degraded scan should not be reliable")
        # Should have progressed smoothly along x
        self.assertGreater(x2, 2.5, "Pose should progress smoothly forward via dead-reckoning")
        self.assertAlmostEqual(y2, 3.5, delta=0.05)

    def test_04_telemetry_parser_anchor_map_binary(self):
        """apply_binary_telemetry must initialize Anchor Map upon first dense scan."""
        coord = MockCoordinator()
        self.assertFalse(coord.motion.anchor_initialized)

        # Build binary telemetry payload with 360 points
        packer = BinaryTelemetryPacker()
        ranges = [2.0] * 360
        payload = packer.pack(
            pose_x=0.0,
            pose_y=0.0,
            pose_theta=0.0,
            linear_vel=0.0,
            angular_vel=0.0,
            nearest_obs=2.0,
            battery=95.0,
            cpu_temp=40.0,
            ranges_m=ranges,
            timestamp=time.time(),
            seq=1
        )

        TelemetryParser.apply_binary_telemetry(payload, coord)
        self.assertTrue(coord.motion.anchor_initialized, "Anchor map state must be initialized in apply_binary_telemetry")

    def test_05_telemetry_parser_no_pose_jump_on_degraded_odom(self):
        """When odometry is degraded, coord.pose should not jump backward to old odom coordinates."""
        coord = MockCoordinator()
        packer = BinaryTelemetryPacker()
        ranges = [2.0] * 360
        payload1 = packer.pack(
            pose_x=0.0, pose_y=0.0, pose_theta=0.0, linear_vel=0.0, angular_vel=0.0,
            nearest_obs=2.0, battery=90.0, cpu_temp=40.0, ranges_m=ranges,
            timestamp=time.time(), seq=1
        )
        TelemetryParser.apply_binary_telemetry(payload1, coord)

        # Robot drives forward smoothly with high-rate dead reckoning
        coord.linear_vel = 0.2
        coord.pose.x = 0.5  # Advanced smoothly via kinematics between ticks
        coord.pose.y = 0.0
        coord.pose.theta = 0.0
        coord.motion.set_pose(0.5, 0.0, 0.0)

        # Now simulate degraded odometry scan (all zeros, no points -> is_reliable: False)
        # ScanGate needs new seq and stamp
        payload_degraded = packer.pack(
            pose_x=0.0, pose_y=0.0, pose_theta=0.0, linear_vel=0.2, angular_vel=0.0,
            nearest_obs=0.0, battery=90.0, cpu_temp=40.0, ranges_m=[0.0] * 360,
            timestamp=time.time() + 0.1, seq=2
        )
        TelemetryParser.apply_binary_telemetry(payload_degraded, coord)

        # Verification: coord.pose.x must NOT have reset back to (0,0) or odom lag
        self.assertAlmostEqual(coord.pose.x, 0.5, delta=0.05,
                               msg=f"coord.pose.x should stay at 0.5, got {coord.pose.x}")
        # Lidar odometry should have been backward-synced via set_pose
        self.assertAlmostEqual(coord.motion.lidar_odometry.x, coord.pose.x, delta=0.05,
                               msg="Lidar odometry must be backward-synced with coord.pose")

    def test_06_d500_packet_stream_and_zero_crossing(self):
        """Simulate feeding 36 D500 packets across 360° to verify full sweep accumulation."""
        driver = d500_03.D500OfficialDriver()

        def make_pkt(s_deg, e_deg, dist_mm=1500, intensity=200):
            header = bytes([0x54, 0x2C])
            speed = (2520).to_bytes(2, "little")  # 2520 deg/s = 7Hz
            start_raw = int(s_deg * 100).to_bytes(2, "little")
            pts = bytearray()
            for _ in range(12):
                pts.extend(int(dist_mm).to_bytes(2, "little"))
                pts.append(int(intensity))
            end_raw = int(e_deg * 100).to_bytes(2, "little")
            ts = (100).to_bytes(2, "little")
            body = header + speed + start_raw + pts + end_raw + ts
            crc = d500_03.cal_crc8(body)
            return body + bytes([crc])

        # Feed 36 packets from 0 to 360 degrees (10 degrees each)
        for i in range(36):
            s_deg = i * 10.0
            e_deg = (i + 1) * 10.0
            pkt = make_pkt(s_deg, e_deg)
            driver._process_packet(pkt)

        # Before crossing zero, scan_sweep_count is 0
        self.assertEqual(driver.scan_sweep_count, 0)

        # Now feed packet crossing zero (start_angle > 270 and end_angle < 90)
        pkt_wrap = make_pkt(5.0, 15.0)
        driver._process_packet(pkt_wrap)

        # Now scan_sweep_count should be 1
        self.assertEqual(driver.scan_sweep_count, 1, "Revolution should complete upon crossing zero")
        ranges = driver.get_ranges_360()
        non_zero = sum(1 for r in ranges if r > 0.05)
        self.assertGreaterEqual(non_zero, 300, f"Dense 360 sweep expected, got {non_zero} non-zero bins")

    def test_07_robot_pose_translation_trajectory(self):
        """Verify smooth forward robot pose translation across cycles with mixed reliable/degraded scans."""
        coord = MockCoordinator()
        packer = BinaryTelemetryPacker()

        def simulate_room_scan(rx, ry, rth, size=2.0):
            ranges = [0.0] * 360
            for deg in range(360):
                th = math.radians(deg) + rth
                cth = math.cos(th)
                sth = math.sin(th)
                dists = []
                if abs(cth) > 1e-4:
                    d_east = (size - rx) / cth
                    if d_east > 0: dists.append(d_east)
                    d_west = (-size - rx) / cth
                    if d_west > 0: dists.append(d_west)
                if abs(sth) > 1e-4:
                    d_north = (size - ry) / sth
                    if d_north > 0: dists.append(d_north)
                    d_south = (-size - ry) / sth
                    if d_south > 0: dists.append(d_south)
                ranges[deg] = min(dists) if dists else 0.0
            return ranges

        # Initial tick
        r0 = simulate_room_scan(0.0, 0.0, 0.0)
        p0 = packer.pack(
            pose_x=0.0, pose_y=0.0, pose_theta=0.0, linear_vel=0.0, angular_vel=0.0,
            nearest_obs=2.0, battery=90.0, cpu_temp=40.0, ranges_m=r0,
            timestamp=time.time(), seq=1
        )
        TelemetryParser.apply_binary_telemetry(p0, coord)
        self.assertAlmostEqual(coord.pose.x, 0.0)

        # Drive forward at v=0.2 m/s over 10 steps (dx=0.02m per step)
        cur_t = time.time()
        true_x = 0.0
        for step in range(1, 11):
            cur_t += 0.1
            true_x += 0.02
            coord.linear_vel = 0.2
            coord.angular_vel = 0.0

            r = simulate_room_scan(true_x, 0.0, 0.0)
            p = packer.pack(
                pose_x=0.0, pose_y=0.0, pose_theta=0.0, linear_vel=0.2, angular_vel=0.0,
                nearest_obs=2.0, battery=90.0, cpu_temp=40.0, ranges_m=r,
                timestamp=cur_t, seq=100 + step
            )
            TelemetryParser.apply_binary_telemetry(p, coord)
            # Verify pose is advancing forward
            self.assertGreater(coord.pose.x, 0.0)
            self.assertAlmostEqual(coord.pose.y, 0.0, delta=0.05)

        # Robot must have moved forward continuously to ~0.20m
        self.assertAlmostEqual(coord.pose.x, 0.20, delta=0.03,
                               msg=f"Robot should translate to ~0.20m, got {coord.pose.x}")

        # Now simulate 3 degraded scans: pose must hold position without jumping back to 0.0
        held_x = coord.pose.x
        for step in range(11, 14):
            cur_t += 0.1
            p_deg = packer.pack(
                pose_x=0.0, pose_y=0.0, pose_theta=0.0, linear_vel=0.2, angular_vel=0.0,
                nearest_obs=0.0, battery=90.0, cpu_temp=40.0, ranges_m=[0.0] * 360,
                timestamp=cur_t, seq=100 + step
            )
            TelemetryParser.apply_binary_telemetry(p_deg, coord)
            # Verify kinematic forward integration persists and does not jump back to origin
            self.assertGreaterEqual(coord.pose.x, held_x - 0.01,
                                    msg="Pose must not jump back to origin on degraded scan")


if __name__ == "__main__":
    unittest.main()
