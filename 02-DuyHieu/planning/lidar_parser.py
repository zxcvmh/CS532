import time, math
from typing import List, Tuple, Dict, Optional, Any
import numpy as np

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
            if 0.02 <= dist_m <= 12.0:
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
