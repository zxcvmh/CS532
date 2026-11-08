import time, math
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

def cal_crc8_bytes(data: bytes) -> int:
    crc = 0x00
    for byte in data:
        crc = CRC8_TABLE[(crc ^ byte) & 0xFF]
    return crc

class D500Parser:
    def __init__(self, offset_x: float = 0.04, offset_y: float = 0.0, offset_yaw_deg: float = 0.0, blind_radius_m: float = 0.06, min_intensity: int = 15):
        self.offset_x = float(offset_x)
        self.offset_y = float(offset_y)
        self.offset_yaw_rad = math.radians(float(offset_yaw_deg))
        self.blind_radius_m = float(blind_radius_m)
        self.min_intensity = int(min_intensity)
        self._cos_yaw = math.cos(self.offset_yaw_rad)
        self._sin_yaw = math.sin(self.offset_yaw_rad)
        self.scan_cache: Dict[int, Tuple[float, int, float]] = {}
        self.latest_ranges: List[float] = [0.0] * 360
        self.latest_intensities: List[int] = [0] * 360

    def parse_packet(self, packet: bytes, current_time: Optional[float] = None) -> bool:
        if len(packet) != 47 or packet[0] != 0x54 or packet[1] != 0x2C:
            return False
        # Official CRC-8 verification (rejects noise and corrupted serial frames)
        if cal_crc8_bytes(packet[:46]) != packet[46]:
            return False

        if current_time is None:
            current_time = time.time()
        start_angle = (packet[4] | (packet[5] << 8)) / 100.0
        end_angle = (packet[42] | (packet[43] << 8)) / 100.0
        if end_angle < start_angle:
            end_angle += 360.0
        step = (end_angle - start_angle) / 11.0
        for i in range(12):
            offset = 6 + (i * 3)
            dist_mm = packet[offset] | (packet[offset + 1] << 8)
            intensity = packet[offset + 2]
            raw_angle = (start_angle + (i * step)) % 360.0
            dist_m = dist_mm / 1000.0
            angle_ccw = (360 - int(round(raw_angle))) % 360
            if (0.02 <= dist_m <= 12.0) and (intensity >= self.min_intensity):
                self.scan_cache[angle_ccw] = (round(dist_m, 3), intensity, current_time)
        return True

    def get_scan_360(self, max_age_sec: float = 0.5) -> Tuple[List[float], List[int]]:
        now = time.time()
        ranges = [0.0] * 360
        intensities = [0] * 360
        for ang, (dist, inten, ts) in self.scan_cache.items():
            if now - ts <= max_age_sec:
                ranges[ang] = dist
                intensities[ang] = inten
        return ranges, intensities

    def get_point_cloud_robot_frame(self, max_age_sec: float = 0.5, filter_intensity: bool = True) -> np.ndarray:
        now = time.time()
        valid = [(a, d, i) for a, (d, i, ts) in self.scan_cache.items() if (now - ts <= max_age_sec) and (d > self.blind_radius_m)]
        if not valid:
            return np.empty((0, 3), dtype=np.float32)
        angles = np.array([v[0] for v in valid], dtype=np.float32)
        ranges = np.array([v[1] for v in valid], dtype=np.float32)
        intens = np.array([v[2] for v in valid], dtype=np.float32)
        if filter_intensity:
            m = intens >= self.min_intensity
            if not np.any(m): return np.empty((0, 3), dtype=np.float32)
            angles, ranges, intens = angles[m], ranges[m], intens[m]
        rads = np.radians(angles)
        xl, yl = ranges * np.cos(rads), ranges * np.sin(rads)
        xr = (xl * self._cos_yaw) - (yl * self._sin_yaw) + self.offset_x
        yr = (xl * self._sin_yaw) + (yl * self._cos_yaw) + self.offset_y
        return np.column_stack((xr, yr, intens))

    def get_compensated_scan_360(self, max_age_sec: float = 0.5, filter_intensity: bool = True) -> Tuple[List[float], List[int]]:
        pts = self.get_point_cloud_robot_frame(max_age_sec=max_age_sec, filter_intensity=filter_intensity)
        if len(pts) < 3: return self.get_scan_360(max_age_sec=max_age_sec)
        rr = np.hypot(pts[:, 0], pts[:, 1])
        th = np.degrees(np.arctan2(pts[:, 1], pts[:, 0])) % 360.0
        idx = np.argsort(th)
        sorted_th, sorted_r, sorted_i = th[idx], rr[idx], pts[:, 2][idx]
        angles_unwrapped = np.concatenate([sorted_th - 360.0, sorted_th, sorted_th + 360.0])
        r_ext = np.concatenate([sorted_r, sorted_r, sorted_r])
        i_ext = np.concatenate([sorted_i, sorted_i, sorted_i])
        bins = np.arange(360, dtype=np.float32)
        c_r = np.interp(bins, angles_unwrapped, r_ext)
        c_i = np.interp(bins, angles_unwrapped, i_ext).astype(int)
        return [round(float(v), 3) for v in c_r], [int(v) for v in c_i]

    def get_frontal_corridor_stats(self, corridor_width: float = 0.35, max_dist: float = 2.0, max_age_sec: float = 0.5) -> dict:
        pts = self.get_point_cloud_robot_frame(max_age_sec=max_age_sec, filter_intensity=True)
        if len(pts) == 0: return {"is_blocked": False, "min_distance": 99.0, "closest_point": (99.0, 0.0), "num_obstacle_points": 0}
        xr, yr = pts[:, 0], pts[:, 1]
        in_corr = (xr > 0.05) & (xr <= max_dist) & (np.abs(yr) <= corridor_width / 2.0)
        n = int(np.count_nonzero(in_corr))
        if n == 0: return {"is_blocked": False, "min_distance": 99.0, "closest_point": (99.0, 0.0), "num_obstacle_points": 0}
        dists = np.hypot(xr[in_corr], yr[in_corr])
        m_idx = np.argmin(dists)
        return {"is_blocked": True, "min_distance": float(round(dists[m_idx], 3)), "closest_point": (float(round(xr[in_corr][m_idx], 3)), float(round(yr[in_corr][m_idx], 3))), "num_obstacle_points": n}
