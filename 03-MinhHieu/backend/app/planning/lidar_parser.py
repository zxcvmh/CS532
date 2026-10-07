import time
import math
from typing import List, Tuple, Dict, Optional

class D500Parser:
    def __init__(self):
        self.scan_cache: Dict[int, Tuple[float, int, float]] = {}
        self.latest_ranges = [0.0] * 360
        self.latest_intensities = [0] * 360

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
            if 0.03 <= dist_m <= 12.0:
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
        self.latest_ranges = ranges
        self.latest_intensities = intensities
        return ranges, intensities
