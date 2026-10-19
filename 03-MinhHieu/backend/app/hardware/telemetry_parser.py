"""
Telemetry Parser for JetBot Hardware Bridge.
Extracts 360-ray LiDAR data, INA219 battery voltage, CSI camera frames, and YOLO detections.
"""
import base64
import math
import time
from typing import Dict, Any, List, Optional, Tuple
from app.config import Config
from app.models.schemas import LidarPoint, Detection, DetectionList, LidarScan
from app.streaming.binary_protocol import unpack_binary_telemetry, patch_binary_pose, BinaryTelemetryPacker


class TelemetryParser:
    """Parses raw hardware telemetry packets received from jetbot_bridge.py."""

    @staticmethod
    def parse_camera_frame(data: Dict[str, Any]) -> Optional[bytes]:
        """Decode base64 CSI camera frame."""
        cam_b64 = data.get("camera")
        if cam_b64 and isinstance(cam_b64, str) and len(cam_b64) > 100:
            try:
                return base64.b64decode(cam_b64)
            except Exception:
                return None
        return None

    @staticmethod
    def parse_battery(data: Dict[str, Any]) -> Optional[float]:
        """Extract battery percentage from INA219 sensor."""
        bat = data.get("battery")
        if bat is not None:
            try:
                val = float(bat)
                return max(0.0, min(100.0, val))
            except (ValueError, TypeError):
                return None
        return None

    @staticmethod
    def parse_lidar_ranges(
        data: Dict[str, Any], max_range: float = None
    ) -> Optional[List[float]]:
        """
        Extract the 360-bin range ring (distance in metres, 0.0 = no return).

        Prefers the dense ``lidar_ranges`` array emitted by the bridge (Phase 5.1): the
        legacy ``lidar`` list of [angle_deg, dist_m] pairs is sparse and lossy to 1 degree.
        Falls back to the pair list for older bridges.

        The near-field floor is ``Config.LIDAR_MIN_RANGE`` and the ceiling is
        ``Config.MAPPING_MAX_RANGE`` — one source of truth, no local literals.
        """
        if max_range is None:
            max_range = Config.MAPPING_MAX_RANGE
        min_range = Config.LIDAR_MIN_RANGE

        dense = data.get("lidar_ranges")
        if isinstance(dense, list) and len(dense) >= 15:
            ranges = [0.0] * 360
            for i, dist in enumerate(dense[:360]):
                try:
                    d = float(dist)
                except (ValueError, TypeError):
                    continue
                if d > 0.0 and min_range <= d <= max_range:
                    ranges[i] = d
            return ranges

        raw_lidar = data.get("lidar")
        if not raw_lidar or not isinstance(raw_lidar, list) or len(raw_lidar) < 15:
            return None
        ranges = [0.0] * 360
        for item in raw_lidar:
            if len(item) >= 2:
                try:
                    ang = float(item[0])
                    dist = float(item[1])
                    if min_range <= dist <= max_range:
                        ranges[int(round(ang)) % 360] = dist
                except (ValueError, TypeError):
                    continue
        return ranges

    @staticmethod
    def ranges_to_points(ranges: List[float], px: float, py: float, pth: float,
                         max_range: float = None) -> List[LidarPoint]:
        """Transform 360-ray ranges to 2D world coordinates with LiDAR lever arm offset (+0.08m)."""
        if max_range is None:
            max_range = Config.MAX_RANGE
        points = []
        lidar_x = px + 0.08 * math.cos(pth)
        lidar_y = py + 0.08 * math.sin(pth)
        for ang_deg, dist in enumerate(ranges):
            if Config.LIDAR_MIN_RANGE <= dist <= max_range:
                rad = math.radians(ang_deg) + pth
                points.append(LidarPoint(x=lidar_x + dist * math.cos(rad), y=lidar_y + dist * math.sin(rad)))
        return points

    @staticmethod
    def parse_detections(data: Dict[str, Any]) -> Tuple[DetectionList, bool]:
        """Extract YOLO detections and activity status."""
        raw_dets = data.get("detections")
        yolo_act = bool(data.get("yolo_active", False))
        if raw_dets is not None and isinstance(raw_dets, list):
            objs = []
            for d in raw_dets:
                try:
                    objs.append(Detection(
                        class_name=d.get("class_name") or d.get("class", "object"),
                        confidence=float(d.get("confidence", 0.0)),
                        bbox=d.get("bbox", [0, 0, 0, 0]),
                        distance=float(d.get("distance", 0.0)),
                        azimuth_deg=float(d.get("azimuth_deg", 0.0))
                    ))
                except Exception:
                    continue
            return DetectionList(objects=objs), True
        return DetectionList(objects=[]), yolo_act

    @classmethod
    def _log_lidar_surroundings(cls, coord: Any, ranges: List[float]) -> None:
        """Periodic 1Hz diagnostic log summarizing 360-degree LiDAR surroundings in 4 cardinal sectors."""
        now = time.time()
        if now - getattr(coord, "_last_lidar_sense_log_time", 0.0) < 1.0:
            return
        coord._last_lidar_sense_log_time = now

        if not ranges or len(ranges) < 180:
            return

        def sector_min(center_deg: int, window: int = 15) -> float:
            vals = []
            for d in range(center_deg - window, center_deg + window + 1):
                idx = (d + 360) % 360
                if idx < len(ranges):
                    r = ranges[idx]
                    if 0.05 <= r <= 6.0:
                        vals.append(r)
            return min(vals) if vals else 99.0

        f_d = sector_min(0)    # Front: 0 deg
        l_d = sector_min(90)   # Left: 90 deg
        b_d = sector_min(180)  # Back: 180 deg
        r_d = sector_min(270)  # Right: 270 deg

        all_valid = [r for r in ranges if 0.05 <= r <= 6.0]
        min_all = min(all_valid) if all_valid else 99.0
        min_idx = ranges.index(min_all) if all_valid and min_all in ranges else -1
        dir_label = "None"
        if 0 <= min_idx < 45 or 315 <= min_idx < 360:
            dir_label = "Trước"
        elif 45 <= min_idx < 135:
            dir_label = "Trái"
        elif 135 <= min_idx < 225:
            dir_label = "Sau"
        elif 225 <= min_idx < 315:
            dir_label = "Phải"

        f_str = f"{f_d:.2f}m" if f_d < 90.0 else "Trống"
        l_str = f"{l_d:.2f}m" if l_d < 90.0 else "Trống"
        b_str = f"{b_d:.2f}m" if b_d < 90.0 else "Trống"
        r_str = f"{r_d:.2f}m" if r_d < 90.0 else "Trống"
        min_str = f"{min_all:.2f}m ({dir_label})" if min_all < 90.0 else "Không có vật cản"

        # Telemetry latency: measures true network/processing delay beyond clock skew baseline
        scan_t = getattr(coord, "_last_odom_time", 0.0)
        lag_str = ""
        if scan_t > 0:
            raw_diff = now - scan_t
            # Adaptive Clock Skew Filter: tracks baseline offset between Jetson and Laptop clocks
            min_skew = getattr(coord, "_min_clock_skew", None)
            if min_skew is None or raw_diff < min_skew:
                coord._min_clock_skew = raw_diff
                min_skew = raw_diff
            elif (now - getattr(coord, "_last_skew_reset_time", now)) > 60.0:
                # Slowly adapt baseline every 60s against clock drift
                coord._min_clock_skew = raw_diff
                coord._last_skew_reset_time = now
                min_skew = raw_diff

            true_lag_ms = max(0.0, (raw_diff - min_skew) * 1000.0)
            if abs(min_skew) > 0.05:
                lag_str = f" | Độ trễ: {true_lag_ms:.0f}ms (lệch đồng hồ: {min_skew*1000.0:+.0f}ms)"
            else:
                lag_str = f" | Độ trễ: {true_lag_ms:.0f}ms"

        print(
            f"📡 [LiDAR Senses 360°{lag_str}] Trước (0°): {f_str} | Trái (90°): {l_str} | Sau (180°): {b_str} | Phải (270°): {r_str} | Hits: {len(all_valid)}/360 | Gần nhất: {min_str}",
            flush=True,
        )

    @classmethod
    def apply_telemetry(cls, data: Dict[str, Any], coord) -> None:
        """Applies parsed telemetry to the NavigationCoordinator instance."""
        if not coord._has_reset_real_map:
            coord.motion.reset()
            coord.map_data = coord.motion.grid_map.to_dashboard_data()
            coord.trajectory.clear()
            coord._has_reset_real_map = True
            coord.add_event("SUCCESS", "Switched to physical JetBot mapping.")

        coord.bridge.last_telemetry_time = time.time()
        coord.bridge.is_connected = True
        cam = cls.parse_camera_frame(data)
        if cam:
            coord.streamer.update_frame(cam)
            coord.bridge.last_camera_time = time.time()
            coord.bridge.camera_connected = True

        bat = cls.parse_battery(data)
        if bat is not None:
            coord.battery = bat
            coord.bridge.last_battery_time = time.time()
            coord.bridge.battery_connected = True

        # Deduplicate and synchronize LiDAR scans via ScanGate
        tick = None
        if hasattr(coord, "scan_gate") and coord.scan_gate:
            tick = coord.scan_gate.accept(data)

        # Phase 8: relay the sequence numbers and the driver's evidence counters so the
        # 1 Hz health probe can state them instead of the operator guessing.
        try:
            coord.latest_scan_seq = int(data.get("scan_seq") or 0)
        except (TypeError, ValueError):
            coord.latest_scan_seq = 0
        if data.get("lidar_diag"):
            coord.lidar_diag = data["lidar_diag"]

        ranges = cls.parse_lidar_ranges(data)
        if ranges:
            coord.latest_raw_ranges = ranges
            coord.bridge.last_lidar_time = time.time()
            coord.bridge.lidar_connected = True

        # High-rate kinematic odometry extrapolation between LiDAR sweeps (22-50 Hz smooth pose)
        if tick is None and (abs(coord.linear_vel) > 0.005 or abs(coord.angular_vel) > 0.01):
            now_inter = time.time()
            last_inter = getattr(coord, "_last_inter_tick_time", now_inter)
            dt_inter = max(0.001, min(0.1, now_inter - last_inter))
            coord._last_inter_tick_time = now_inter
            c_th = math.cos(coord.pose.theta)
            s_th = math.sin(coord.pose.theta)
            coord.pose.x += coord.linear_vel * c_th * dt_inter
            coord.pose.y += coord.linear_vel * s_th * dt_inter
            coord.pose.theta = (coord.pose.theta + coord.angular_vel * dt_inter + math.pi) % (2.0 * math.pi) - math.pi

        # Update SLAM scan match odometry, clusters, and occupancy grid on a FRESH scan tick
        if tick is not None:
            coord._last_inter_tick_time = time.time()
            ranges = tick.ranges
            dt_odom = tick.dt
            coord.latest_tick_seq = tick.seq
            coord._last_odom_time = tick.stamp
            ox, oy, oth, odom_info = coord.motion.update_odometry(
                ranges, dt_odom, coord.linear_vel, coord.angular_vel
            )
            # Always adopt integrated pose smoothly from odometry (dead-reckoning fallback is handled internally)
            coord.pose.x, coord.pose.y, coord.pose.theta = ox, oy, oth

            # Phase 4.4: the localizer fell back to pure dead-reckoning (no scan match).
            # Say so instead of silently presenting a drifting pose as measured.
            if isinstance(odom_info, dict) and odom_info.get("status") == "POSE_STALE":
                is_moving = abs(coord.linear_vel) >= 0.01 or abs(coord.angular_vel) >= 0.02
                is_auto = getattr(coord, "mode", None) and (getattr(coord.mode, "value", str(coord.mode)) == "AUTONOMOUS")
                if is_moving and is_auto:
                    now_w = time.time()
                    if now_w - getattr(coord, "_pose_stale_warn_time", 0.0) > 2.0:
                        coord._pose_stale_warn_time = now_w
                        coord.add_event(
                            "WARNING",
                            "⚠️ LiDAR odometry degraded (no scan match) — "
                            "pose is dead-reckoned",
                        )
            elif isinstance(odom_info, dict) and odom_info.get("is_reliable"):
                coord._pose_stale_warn_time = 0.0

            # Phase 4.5: wall-clock freshness stamp for the coordinator staleness guard.
            coord._last_scan_tick_time = time.time()

            pts = cls.ranges_to_points(ranges, ox, oy, oth)
            coord.current_scan = LidarScan(points=pts)
            cls._log_lidar_surroundings(coord, ranges)
            valid_cls, out_cls = coord.motion.process_clusters(ranges, ox, oy, oth)
            coord.latest_clusters, coord.latest_clusters_dict = valid_cls, out_cls

            # Disabled Pristine Anchor Locking (plan_setup_SLAM.md Section 2.3):
            # Maps must be dynamically constructed via accumulated Bayesian evidence,
            # avoiding locking transient initial startup clutter into permanent walls.
            if not coord.motion.anchor_initialized:
                coord.motion.anchor_initialized = True

            now_t = time.time()
            if now_t - coord._last_map_update >= 0.045:
                is_rotating = abs(getattr(coord, "angular_vel", 0.0)) > 0.15
                coord.motion.grid_map.update_scan(
                    ox,
                    oy,
                    oth,
                    ranges=ranges,
                    min_valid_range=Config.LIDAR_MIN_RANGE,
                    max_valid_range=Config.MAPPING_MAX_RANGE,
                    defer_costmap=True,
                    is_rotating=is_rotating,
                )
                # Phase 3.2/3.3: ONE mutation order, matching the controller's replan path:
                # clear the (real-radius) footprint, THEN insert the cluster set. The cluster
                # set is passed unconditionally (Phase 3.4) so feeding an empty list actually
                # clears stale clusters instead of leaving a ghost behind.
                coord.motion.grid_map.clear_footprint(
                    ox,
                    oy,
                    radius=Config.ROBOT_RADIUS,
                    evict_clusters=False,
                    defer_costmap=True,
                )
                if not is_rotating:
                    coord.motion.grid_map.insert_obstacle_clusters(
                        valid_cls, ox, oy, oth, defer_costmap=True
                    )

                # Phase 4.1/4.2: CV low-obstacle detections live in a short-TTL lethal
                # layer only — they can never harden into the belief map.
                raw_low = data.get("low_obstacles")
                if isinstance(raw_low, list):
                    coord.low_obstacles = raw_low[:32]
                    coord.motion.grid_map.insert_low_obstacles(
                        raw_low, ox, oy, oth, defer_costmap=True
                    )

                coord.motion.grid_map.update_costmap()
                coord.map_data = coord.motion.grid_map.to_dashboard_data()
                coord.map_data_b64 = coord.motion.grid_map.to_dashboard_b64()
                coord.map_version += 1
                coord._last_map_update = now_t

        raw_dets = data.get("detections")
        dets, yolo_act = cls.parse_detections(data)
        if dets.objects or yolo_act:
            coord.current_detections = dets
            coord.bridge.last_yolo_time = time.time()
            coord.bridge.yolo_connected = True
            if raw_dets and isinstance(raw_dets, list):
                coord.motion.grid_map.update_social_bubbles_from_detections(
                    detections=raw_dets,
                    robot_x=coord.pose.x,
                    robot_y=coord.pose.y,
                    robot_theta=coord.pose.theta,
                    bubble_radius_m=0.40
                )
                coord.latest_social_bubbles = [
                    {"x": round(b[0], 3), "y": round(b[1], 3), "radius": round(b[2], 2)}
                    for b in getattr(coord.motion.grid_map, "social_bubbles", [])
                ]

    @classmethod
    def apply_binary_telemetry(cls, raw_bytes: bytes, coord) -> None:
        """Applies ultra-fast 1,130-byte JB-Binary telemetry directly to NavigationCoordinator."""
        if not coord._has_reset_real_map:
            coord.motion.reset()
            coord.map_data = coord.motion.grid_map.to_dashboard_data()
            coord.trajectory.clear()
            coord._has_reset_real_map = True
            coord.add_event("SUCCESS", "Switched to physical JetBot mapping (JB-Binary Ultra-Fast Stream).")

        parsed = unpack_binary_telemetry(raw_bytes)
        if not parsed:
            return

        now_t = time.time()
        coord.bridge.last_telemetry_time = now_t
        coord.bridge.is_connected = True
        coord.bridge.last_lidar_time = now_t
        coord.bridge.lidar_connected = True

        if parsed["battery"] > 0:
            coord.battery = parsed["battery"]
            coord.bridge.last_battery_time = now_t
            coord.bridge.battery_connected = True

        if "linear_vel" in parsed and "angular_vel" in parsed:
            if coord.mode.value == "MANUAL" or (abs(parsed["linear_vel"]) > 0.001 or abs(parsed["angular_vel"]) > 0.001):
                coord.linear_vel = parsed["linear_vel"]
                coord.angular_vel = parsed["angular_vel"]

        ranges_m = parsed["ranges"]
        coord.latest_raw_ranges = ranges_m.tolist() if hasattr(ranges_m, "tolist") else list(ranges_m)
        coord.latest_scan_seq = parsed["seq"]

        fake_data = {
            "lidar_ranges": coord.latest_raw_ranges,
            "scan_seq": parsed["seq"],
            "scan_stamp": parsed["timestamp"]
        }
        tick = None
        if hasattr(coord, "scan_gate") and coord.scan_gate:
            tick = coord.scan_gate.accept(fake_data)

        if tick is None and (abs(coord.linear_vel) > 0.005 or abs(coord.angular_vel) > 0.01):
            now_inter = now_t
            last_inter = getattr(coord, "_last_inter_tick_time", now_inter)
            dt_inter = max(0.001, min(0.1, now_inter - last_inter))
            coord._last_inter_tick_time = now_inter
            c_th = math.cos(coord.pose.theta)
            s_th = math.sin(coord.pose.theta)
            coord.pose.x += coord.linear_vel * c_th * dt_inter
            coord.pose.y += coord.linear_vel * s_th * dt_inter
            coord.pose.theta = (coord.pose.theta + coord.angular_vel * dt_inter + math.pi) % (2.0 * math.pi) - math.pi

        if tick is not None:
            coord._last_inter_tick_time = now_t
            ranges = tick.ranges
            dt_odom = tick.dt
            coord.latest_tick_seq = tick.seq
            coord._last_odom_time = tick.stamp
            ox, oy, oth, odom_info = coord.motion.update_odometry(
                ranges, dt_odom, coord.linear_vel, coord.angular_vel
            )
            # Always adopt integrated pose smoothly from odometry (dead-reckoning fallback is handled internally)
            coord.pose.x, coord.pose.y, coord.pose.theta = ox, oy, oth

            if isinstance(odom_info, dict) and odom_info.get("status") == "POSE_STALE":
                is_moving = abs(coord.linear_vel) >= 0.01 or abs(coord.angular_vel) >= 0.02
                is_auto = getattr(coord, "mode", None) and (getattr(coord.mode, "value", str(coord.mode)) == "AUTONOMOUS")
                if is_moving and is_auto:
                    now_w = time.time()
                    if now_w - getattr(coord, "_pose_stale_warn_time", 0.0) > 2.0:
                        coord._pose_stale_warn_time = now_w
                        coord.add_event(
                            "WARNING",
                            "⚠️ LiDAR odometry degraded (no scan match) — pose is dead-reckoned",
                        )
            elif isinstance(odom_info, dict) and odom_info.get("is_reliable"):
                coord._pose_stale_warn_time = 0.0

            coord._last_scan_tick_time = time.time()
            pts = cls.ranges_to_points(ranges, ox, oy, oth)
            coord.current_scan = LidarScan(points=pts)
            cls._log_lidar_surroundings(coord, ranges)
            valid_cls, out_cls = coord.motion.process_clusters(ranges, ox, oy, oth)
            coord.latest_clusters, coord.latest_clusters_dict = valid_cls, out_cls

            # Disabled Pristine Anchor Locking (plan_setup_SLAM.md Section 2.3):
            # Maps must be dynamically constructed via accumulated Bayesian evidence,
            # avoiding locking transient initial startup clutter into permanent walls.
            if not coord.motion.anchor_initialized:
                coord.motion.anchor_initialized = True

            now_map = time.time()
            if now_map - coord._last_map_update >= 0.045:
                coord.motion.grid_map.update_scan(
                    ox,
                    oy,
                    oth,
                    ranges=ranges,
                    min_valid_range=Config.LIDAR_MIN_RANGE,
                    max_valid_range=Config.MAPPING_MAX_RANGE,
                    defer_costmap=True,
                    is_rotating=(abs(coord.angular_vel) > 0.15)
                )
                if abs(coord.angular_vel) <= 0.15:
                    coord.motion.grid_map.insert_obstacle_clusters(
                        valid_cls, ox, oy, oth, defer_costmap=True
                    )
                coord.motion.grid_map.update_costmap()
                coord.map_data = coord.motion.grid_map.to_dashboard_data()
                coord.map_data_b64 = coord.motion.grid_map.to_dashboard_b64()
                coord.map_version += 1
                coord._last_map_update = now_map

        # Patch SLAM world pose into the binary packet for instant forward to dashboard
        coord.latest_binary_telemetry = patch_binary_pose(raw_bytes, coord.pose.x, coord.pose.y, coord.pose.theta)

