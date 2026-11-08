"""
Mock Arena Simulation Module.
Contains geometric raycasting (line & circle intersection) and unicycle kinematics for simulation mode.
"""
import math
import random
from typing import List, Tuple, Optional, Dict, Any
from app.config import Config
from app.models.schemas import LidarPoint, Detection, DetectionList


def line_intersection(p1: Tuple[float, float], p2: Tuple[float, float],
                      p3: Tuple[float, float], p4: Tuple[float, float]) -> Optional[Tuple[float, float]]:
    """Find intersection of two 2D line segments."""
    x1, y1 = p1
    x2, y2 = p2
    x3, y3 = p3
    x4, y4 = p4
    den = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
    if abs(den) < 1e-10:
        return None
    t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / den
    u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / den
    if 0 <= t <= 1 and 0 <= u <= 1:
        return (x1 + t * (x2 - x1), y1 + t * (y2 - y1))
    return None


def circle_intersection(p1: Tuple[float, float], p2: Tuple[float, float],
                        cx: float, cy: float, r: float) -> Optional[Tuple[float, float]]:
    """Find nearest intersection of a line segment with a circle."""
    x1, y1 = p1
    x2, y2 = p2
    dx = x2 - x1
    dy = y2 - y1
    a = dx**2 + dy**2
    b = 2 * (dx * (x1 - cx) + dy * (y1 - cy))
    c = (x1 - cx)**2 + (y1 - cy)**2 - r**2
    det = b**2 - 4 * a * c
    if det < 0 or a == 0:
        return None
    t1 = (-b + math.sqrt(det)) / (2 * a)
    t2 = (-b - math.sqrt(det)) / (2 * a)
    intersections = []
    if 0 <= t1 <= 1:
        intersections.append((x1 + t1 * dx, y1 + t1 * dy))
    if 0 <= t2 <= 1:
        intersections.append((x1 + t2 * dx, y1 + t2 * dy))
    if not intersections:
        return None
    return min(intersections, key=lambda p: (p[0] - x1)**2 + (p[1] - y1)**2)


class MockArena:
    """Simulates 2D physical arena boundaries, obstacle raycasting, and unicycle kinematics."""

    def __init__(self, walls=None, obstacles=None, max_range: float = None):
        self.walls = walls if walls is not None else Config.WALLS
        self.obstacles = obstacles if obstacles is not None else Config.OBSTACLES
        self.max_range = max_range if max_range is not None else Config.MAX_RANGE

    def raycast(self, rx: float, ry: float, angle: float) -> float:
        """Cast single ray from robot position at given angle, return distance to nearest obstacle."""
        ray_end = (
            rx + math.cos(angle) * self.max_range,
            ry + math.sin(angle) * self.max_range
        )
        p1 = (rx, ry)
        min_dist = self.max_range

        for w in self.walls:
            hit = line_intersection(p1, ray_end, (w[0], w[1]), (w[2], w[3]))
            if hit:
                d = math.hypot(hit[0] - p1[0], hit[1] - p1[1])
                min_dist = min(min_dist, d)

        for obs in self.obstacles:
            hit = circle_intersection(p1, ray_end, obs[0], obs[1], obs[2])
            if hit:
                d = math.hypot(hit[0] - p1[0], hit[1] - p1[1])
                min_dist = min(min_dist, d)

        # Add Gaussian measurement noise (std = 1.5cm)
        return max(0.05, min_dist + random.gauss(0, 0.015))

    def simulate_scan(self, rx: float, ry: float, rtheta: float,
                      num_rays: int = None) -> Tuple[List[float], List[LidarPoint]]:
        """Generate full 360-degree simulated LiDAR scan."""
        if num_rays is None:
            num_rays = Config.NUM_RAYS

        ranges = [0.0] * 360
        points = []
        step = 360 / num_rays

        for i in range(num_rays):
            ang_deg = (i * step) % 360
            angle_rad = rtheta + math.radians(ang_deg)
            dist = self.raycast(rx, ry, angle_rad)
            ranges[int(round(ang_deg)) % 360] = dist

            if 0.15 <= dist <= self.max_range:
                hx = rx + math.cos(angle_rad) * dist
                hy = ry + math.sin(angle_rad) * dist
                points.append(LidarPoint(x=hx, y=hy))

        return ranges, points

    @staticmethod
    def update_kinematics(x: float, y: float, theta: float,
                          v: float, w: float, dt: float) -> Tuple[float, float, float]:
        """Integrate unicycle kinematic model for differential drive robot."""
        new_theta = (theta + w * dt + math.pi) % (2.0 * math.pi) - math.pi
        new_x = x + v * math.cos(new_theta) * dt
        new_y = y + v * math.sin(new_theta) * dt
        return new_x, new_y, new_theta

    @staticmethod
    def simulate_battery(current_bat: float, dt: float, drain_rate: float = 0.01) -> float:
        """Simulate realistic battery depletion."""
        return max(0.0, current_bat - drain_rate * dt)
