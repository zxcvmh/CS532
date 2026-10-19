#!/usr/bin/env python3
"""
Official LDROBOT LD19 / D500 LiDAR High-Precision Python Driver
=============================================================================
CS532 Autonomous Robotics — Module 05-d500lib-motion
Based on Official LDROBOT C++ SDK (ldrobotSensorTeam/ldlidar_stl_sdk / lipkg)

Key Specifications:
  - Protocol: LD19 / D500 47-Byte Frame (Header: 0x54, VerLen: 0x2C)
  - Checksum: Table-driven CRC-8 with generator polynomial 0x4D
  - Coordinate Frame: Right-Handed Cartesian (REP-103) CCW standard
  - Noise Filter: Confidence/Intensity thresholding (removes tile reflections)
  - Revolution Sync: Frame-synchronized 360° sweeps without yaw smear
=============================================================================
"""

import os
import time
import math
import threading

try:
    import serial
except ImportError:
    serial = None


from typing import List, Tuple, Dict, Optional, Any
import numpy as np

# Official LDROBOT CRC-8 Lookup Table (Polynomial: 0x4D, derived from ldlidar_stl_sdk lipkg.cpp CrcTable)
CRC8_TABLE = [
    0x00, 0x4D, 0x9A, 0xD7, 0x79, 0x34, 0xE3, 0xAE, 0xF2, 0xBF, 0x68, 0x25, 0x8B, 0xC6, 0x11, 0x5C,
    0xA9, 0xE4, 0x33, 0x7E, 0xD0, 0x9D, 0x4A, 0x07, 0x5B, 0x16, 0xC1, 0x8C, 0x22, 0x6F, 0xB8, 0xF5,
    0x1F, 0x52, 0x85, 0xC8, 0x66, 0x2B, 0xFC, 0xB1, 0xED, 0xA0, 0x77, 0x3A, 0x94, 0xD9, 0x0E, 0x43,
    0xB6, 0xFB, 0x2C, 0x61, 0xCF, 0x82, 0x55, 0x18, 0x44, 0x09, 0xDE, 0x93, 0x3D, 0x70, 0xA7, 0xEA,
    0x3E, 0x73, 0xA4, 0xE9, 0x47, 0x0A, 0xDD, 0x90, 0xCC, 0x81, 0x56, 0x1B, 0xB5, 0xF8, 0x2F, 0x62,
    0x97, 0xDA, 0x0D, 0x40, 0xEE, 0xA3, 0x74, 0x39, 0x65, 0x28, 0xFF, 0xB2, 0x1C, 0x51, 0x86, 0xCB,
    0x21, 0x6C, 0xBB, 0xF6, 0x58, 0x15, 0xC2, 0x8F, 0xD3, 0x9E, 0x49, 0x04, 0xAA, 0xE7, 0x30, 0x7D,
    0x88, 0xC5, 0x12, 0x5F, 0xF1, 0xBC, 0x6B, 0x26, 0x7A, 0x37, 0xE0, 0xAD, 0x03, 0x4E, 0x99, 0xD4,
    0x7C, 0x31, 0xE6, 0xAB, 0x05, 0x48, 0x9F, 0xD2, 0x8E, 0xC3, 0x14, 0x59, 0xF7, 0xBA, 0x6D, 0x20,
    0xD5, 0x98, 0x4F, 0x02, 0xAC, 0xE1, 0x36, 0x7B, 0x27, 0x6A, 0xBD, 0xF0, 0x5E, 0x13, 0xC4, 0x89,
    0x63, 0x2E, 0xF9, 0xB4, 0x1A, 0x57, 0x80, 0xCD, 0x91, 0xDC, 0x0B, 0x46, 0xE8, 0xA5, 0x72, 0x3F,
    0xCA, 0x87, 0x50, 0x1D, 0xB3, 0xFE, 0x29, 0x64, 0x38, 0x75, 0xA2, 0xEF, 0x41, 0x0C, 0xDB, 0x96,
    0x42, 0x0F, 0xD8, 0x95, 0x3B, 0x76, 0xA1, 0xEC, 0xB0, 0xFD, 0x2A, 0x67, 0xC9, 0x84, 0x53, 0x1E,
    0xEB, 0xA6, 0x71, 0x3C, 0x92, 0xDF, 0x08, 0x45, 0x19, 0x54, 0x83, 0xCE, 0x60, 0x2D, 0xFA, 0xB7,
    0x5D, 0x10, 0xC7, 0x8A, 0x24, 0x69, 0xBE, 0xF3, 0xAF, 0xE2, 0x35, 0x78, 0xD6, 0x9B, 0x4C, 0x01,
    0xF4, 0xB9, 0x6E, 0x23, 0x8D, 0xC0, 0x17, 0x5A, 0x06, 0x4B, 0x9C, 0xD1, 0x7F, 0x32, 0xE5, 0xA8,
]


def cal_crc8(data: bytes) -> int:
    """Calculates official LDROBOT CRC-8 checksum."""
    crc = 0x00
    for byte in data:
        crc = CRC8_TABLE[(crc ^ byte) & 0xFF]
    return crc


class D500OfficialDriver:
    """
    Official Specification LDROBOT D500 Driver.
    Features:
      - 47-Byte hardware sync (0x54, 0x2C) with CRC-8 hardware verification.
      - Dynamic motor speed and timestamp extraction.
      - Confidence/intensity thresholding to suppress specular tile reflections.
      - Full single-revolution synchronization (eliminates rotational yaw smear).
    """

    def __init__(
        self,
        port: str = "/dev/ttyUSB0",
        baudrate: int = 230400,
        min_distance_m: float = 0.05,
        max_distance_m: float = 6.00,
        min_intensity: int = 2,
        min_intensity_near: int = 1,
        near_field_m: float = 1.0,
        yaw_offset_deg: float = 0.0,
    ):
        self.port = port
        self.baudrate = baudrate
        # Phase 1.2: datasheet floor is 0.03m; 0.05m = floor + 2cm mounting margin.
        self.min_dist = float(min_distance_m)
        self.max_dist = float(max_distance_m)
        # Phase 1.1: range-dependent confidence acceptance.
        # A weak SHORT return is real geometry (a black box at 0.4m comes back faint);
        # specular floor/tile reflections come back LONG, so the strict threshold stays
        # out there. Phase 1.5: both thresholds are constructor-overridable (no code edit).
        self.min_intensity = int(min_intensity)
        self.min_intensity_near = int(min_intensity_near)
        self.near_field_m = float(near_field_m)
        self.yaw_offset_deg = float(yaw_offset_deg)

        # Threading state (Must be RLock to support re-entrant queries from scan_cache & hardware reflex)
        self.running = False
        self.thread: Optional[threading.Thread] = None
        self.lock = threading.RLock()
        self._first_scan_logged: bool = False

        # Telemetry & Scan storage
        self.latest_ranges: List[float] = [0.0] * 360
        self.latest_intensities: List[int] = [0] * 360
        self.latest_scan_list: List[List[float]] = []  # [[angle_deg, dist_m], ...]
        self.latest_timestamp: float = 0.0
        self.motor_speed_deg_s: float = 0.0

        # Quality metrics
        self.packet_count: int = 0
        self.crc_error_count: int = 0
        self.scan_sweep_count: int = 0

        # Phase 1.3 — evidence counters. The old single predicate silently discarded the
        # black box's faint returns, so the backend could not tell "no return" from
        # "filtered by intensity" (NEW-03 / BUG-01).
        self.rejected_by_intensity: int = 0
        self.rejected_by_range: int = 0
        self.accepted_points: int = 0

        # Revolution accumulator & timeout protection
        self._current_sweep_ranges: List[float] = [0.0] * 360
        self._current_sweep_intensities: List[int] = [0] * 360
        self._current_sweep_raw_pts: List[List[float]] = []
        self._current_sweep_point_count: int = 0
        self._current_sweep_packet_count: int = 0
        self._last_pkg_start_angle: float = 0.0
        self._last_pkg_end_angle: float = 0.0
        self._last_sweep_publish_time: float = time.time()
        self.last_packet_timestamp: float = 0.0


    def start(self) -> bool:
        if self.running:
            return True
        self.running = True
        self.thread = threading.Thread(target=self._read_worker, daemon=True)
        self.thread.start()
        return True

    def stop(self):
        self.running = False
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=1.0)

    def _read_worker(self):
        buffer = bytearray()
        while self.running:
            ser = None
            # Prioritize existing ports, starting with self.port if it exists
            candidate_ports = []
            if self.port and os.path.exists(self.port):
                candidate_ports.append(self.port)
            for cp in ["/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyTHS1", "/dev/ttyACM0"]:
                if cp not in candidate_ports and os.path.exists(cp):
                    candidate_ports.append(cp)
            # If no ports exist on filesystem, fall back to self.port and standard ports to capture OS error
            if not candidate_ports:
                candidate_ports = [self.port]
                for cp in ["/dev/ttyUSB0", "/dev/ttyTHS1"]:
                    if cp not in candidate_ports:
                        candidate_ports.append(cp)

            active_port = self.port
            err_details = []
            for candidate in candidate_ports:
                try:
                    if serial is None:
                        raise RuntimeError(
                            "pyserial is not installed. LiDAR hardware driver cannot open serial port."
                        )
                    ser = serial.Serial(candidate, baudrate=self.baudrate, timeout=0.05)
                    ser.reset_input_buffer()
                    active_port = candidate
                    self.port = candidate
                    break
                except Exception as e:
                    err_details.append(f"{candidate} ({type(e).__name__}: {e})")
                    ser = None

            if ser is None:
                err_summary = (
                    "; ".join(err_details)
                    if err_details
                    else "No candidate ports available"
                )
                print(
                    f"[WARN] LiDAR serial connection error on candidate ports ({', '.join(candidate_ports)}): {err_summary}. Retrying in 1.0s..."
                )
                time.sleep(1.0)
                continue

            try:
                ser.reset_input_buffer()
                buffer.clear()
                print(
                    f"[OK] D500 LiDAR serial opened on {active_port} @ {self.baudrate} baud."
                )
                while self.running:
                    n_avail = ser.in_waiting
                    # Read available serial bytes (or block up to timeout 0.05s for at least 1 byte if empty)
                    chunk = ser.read(n_avail if n_avail > 0 else 1)
                    if not chunk:
                        continue
                    buffer.extend(chunk)

                    # Backlog safeguard: if buffer exceeds 8192 bytes (> 350ms of unparsed data),
                    # keep the freshest ~1.5 revolutions (~3000 bytes) aligned to 0x54, 0x2C header.
                    # Never call ser.reset_input_buffer() during scanning to prevent packet starvation!
                    if len(buffer) > 8192:
                        cut_idx = len(buffer) - 3000
                        latest_hdr = buffer.find(b"\x54\x2c", cut_idx)
                        if latest_hdr != -1:
                            del buffer[:latest_hdr]
                        else:
                            del buffer[:cut_idx]

                    # Zero-copy index stepping parser through linear buffer (microsecond throughput)
                    idx = 0
                    buf_len = len(buffer)
                    while idx + 47 <= buf_len:
                        if buffer[idx] != 0x54 or buffer[idx + 1] != 0x2C:
                            # Advance to next candidate 0x54, 0x2C frame header
                            next_hdr = buffer.find(b"\x54\x2c", idx + 1)
                            if next_hdr == -1:
                                idx = buf_len - 1 if buffer[-1] == 0x54 else buf_len
                                break
                            idx = next_hdr
                            continue

                        # Extract 47-byte packet and verify official CRC-8
                        pkt = bytes(buffer[idx:idx + 47])
                        expected_crc = pkt[46]
                        computed_crc = cal_crc8(pkt[:46])
                        if expected_crc == computed_crc:
                            self._process_packet(pkt)
                            idx += 47
                        else:
                            self.crc_error_count += 1
                            # Resynchronize by advancing 1 byte past false header
                            idx += 1

                    # Single batch slice of consumed bytes (replaces 40+ del calls per chunk)
                    if idx > 0:
                        del buffer[:idx]

            except Exception as e:
                print(
                    f"[WARN] LiDAR serial connection error on {active_port}: {e}. Retrying in 1.0s..."
                )
                time.sleep(1.0)
            finally:
                if ser and ser.is_open:
                    ser.close()

    def _process_packet(self, packet: bytes):
        self.packet_count += 1
        now = time.time()
        self.last_packet_timestamp = now

        # Radar speed (deg/s)
        speed = packet[2] | (packet[3] << 8)
        self.motor_speed_deg_s = float(speed)

        # Start and End angles (0.01 deg)
        start_angle = (packet[4] | (packet[5] << 8)) / 100.0
        end_angle = (packet[42] | (packet[43] << 8)) / 100.0

        if end_angle < start_angle:
            end_angle += 360.0

        step = (end_angle - start_angle) / 11.0

        # Robust 360-degree revolution boundary check (aligned with official LDROBOT C++ driver):
        # A true 360° revolution of D500 contains 30–50 packets (~360–600 raw points).
        time_since_sweep = now - getattr(self, "_last_sweep_publish_time", 0.0)
        angle_wrapped = (
            (start_angle < self._last_pkg_start_angle and (self._last_pkg_start_angle - start_angle) > 180.0)
            or (self._last_pkg_start_angle > 270.0 and start_angle < 90.0)
        )
        crossed_zero = angle_wrapped and (self._current_sweep_packet_count >= 15)
        timed_out_revolution = (
            time_since_sweep >= 0.20 and self._current_sweep_packet_count >= 20
        )
        # Stale sweep safeguard: If sensor stream was paused or disconnected > 1.5s, wipe accumulator
        if time_since_sweep >= 1.5:
            self._current_sweep_ranges = [0.0] * 360
            self._current_sweep_intensities = [0] * 360
            self._current_sweep_point_count = 0
            self._current_sweep_packet_count = 0
            self._last_sweep_publish_time = now

        if crossed_zero or timed_out_revolution:
            # Pure Single-Revolution Sweep (REP-103/REP-105 compliant):
            # Only export measurements collected during this specific revolution (~100-140ms).
            # Eliminates leaky sliding cache ghosting and synthetic gap-filling artifacts.
            with self.lock:
                self.latest_ranges = list(self._current_sweep_ranges)
                self.latest_intensities = list(self._current_sweep_intensities)
                self.latest_timestamp = now
                self.scan_sweep_count += 1
                self.latest_scan_list = [
                    [deg, self.latest_ranges[deg]]
                    for deg in range(360)
                    if self.latest_ranges[deg] > 0.05
                ]

            self._last_sweep_publish_time = now
            self._current_sweep_ranges = [0.0] * 360
            self._current_sweep_intensities = [0] * 360
            self._current_sweep_point_count = 0
            self._current_sweep_packet_count = 0
            if not self._first_scan_logged and len(self.latest_scan_list) > 10:
                self._first_scan_logged = True
                hz = (
                    (self.motor_speed_deg_s / 360.0)
                    if self.motor_speed_deg_s > 0
                    else 10.0
                )
                print(
                    f"[OK] D500 LiDAR active: receiving 360° sweeps ({len(self.latest_scan_list)} pts, spin: {hz:.1f}Hz / {self.motor_speed_deg_s:.0f}°/s)!"
                )
            self._current_sweep_raw_pts = []

        self._current_sweep_packet_count += 1
        self._last_pkg_start_angle = start_angle
        self._last_pkg_end_angle = end_angle % 360.0

        # Parse 12 measurement points
        for i in range(12):
            offset = 6 + (i * 3)
            dist_mm = packet[offset] | (packet[offset + 1] << 8)
            intensity = packet[offset + 2]
            raw_angle = (start_angle + (i * step)) % 360.0
            dist_m = dist_mm / 1000.0

            # Convert Clockwise sensor angle to Right-Handed Cartesian CCW frame with float sub-degree precision
            raw_angle_ccw = (360.0 - raw_angle + self.yaw_offset_deg) % 360.0
            angle_bin = int(round(raw_angle_ccw)) % 360

            # Phase 1.1 — range-dependent confidence acceptance, with evidence counters.
            # The old single predicate `intensity >= 15` threw away the very returns this
            # fix exists to keep (NEW-03), while the counters below make the drop provable
            # in `get_diagnostics()`.
            if dist_m <= 0.0 or not (self.min_dist <= dist_m <= self.max_dist):
                self.rejected_by_range += 1
                continue
            required_intensity = (
                self.min_intensity_near
                if dist_m <= self.near_field_m
                else self.min_intensity
            )
            if intensity < required_intensity:
                self.rejected_by_intensity += 1
                continue

            self.accepted_points += 1
            self._current_sweep_point_count += 1
            self._current_sweep_raw_pts.append(
                [round(raw_angle_ccw, 2), round(dist_m, 3)]
            )
            # Update Single-Sweep Revolution Accumulator (nearest return per degree bin)
            if (
                self._current_sweep_ranges[angle_bin] <= 0.0
                or dist_m < self._current_sweep_ranges[angle_bin]
            ):
                self._current_sweep_ranges[angle_bin] = round(dist_m, 3)
                self._current_sweep_intensities[angle_bin] = intensity


    def get_ranges_360(self) -> List[float]:
        """Returns thread-safe copy of latest 360-degree range array."""
        with self.lock:
            return list(self.latest_ranges)

    def get_scan_list(self) -> List[List[float]]:
        """Returns list of [angle_deg, dist_meters] for WebSocket telemetry."""
        with self.lock:
            return list(self.latest_scan_list)

    def get_scan_snapshot(self) -> Tuple[List[float], List[int]]:
        """Atomically returns (ranges_360, intensities_360) from the SAME sweep.

        Two separate getters could straddle a revolution boundary, pairing one sweep's
        ranges with the previous sweep's confidences (Phase 5.1).
        """
        with self.lock:
            return list(self.latest_ranges), list(self.latest_intensities)

    def get_intensities_360(self) -> List[int]:
        """Returns thread-safe copy of the latest 360-degree intensity ring.

        Phase 1.4/5.1: the per-bin confidences were already computed and then thrown away.
        Exposing them lets the intensity policy live downstream of the driver instead of
        inside it, and lets the operator see confidence in the dashboard/triage tools.
        """
        with self.lock:
            return list(self.latest_intensities)

    def get_point_cloud_2d(self) -> np.ndarray:
        """Returns filtered (N, 2) Cartesian point cloud in robot frame."""
        with self.lock:
            ranges = list(self.latest_ranges)
        pts = []
        for deg, r in enumerate(ranges):
            if r > 0.0:
                rad = math.radians(deg)
                pts.append([r * math.cos(rad), r * math.sin(rad)])
        if not pts:
            return np.empty((0, 2), dtype=np.float32)
        return np.array(pts, dtype=np.float32)

    def get_diagnostics(self) -> Dict[str, Any]:
        """Returns driver health metrics."""
        with self.lock:
            return {
                "port": self.port,
                "packet_count": self.packet_count,
                "crc_error_count": self.crc_error_count,
                "scan_sweep_count": self.scan_sweep_count,
                "motor_speed_deg_s": self.motor_speed_deg_s,
                "latest_timestamp": self.latest_timestamp,
                # Phase 1 policy + evidence (NEW-03)
                "min_dist": self.min_dist,
                "max_dist": self.max_dist,
                "min_intensity": self.min_intensity,
                "min_intensity_near": self.min_intensity_near,
                "near_field_m": self.near_field_m,
                "accepted_points": self.accepted_points,
                "rejected_by_intensity": self.rejected_by_intensity,
                "rejected_by_range": self.rejected_by_range,
            }
