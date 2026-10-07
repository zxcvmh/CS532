import math
from typing import List, Tuple, Optional, Dict, Any

class PurePursuitController:
    def __init__(self, lookahead_dist: float = 0.30, max_linear_speed: float = 0.25, max_angular_speed: float = 1.0, track_width: float = 0.115, goal_tolerance: float = 0.12, reflex_dist_threshold: float = 0.20):
        self.lookahead_dist = lookahead_dist
        self.max_linear_speed = max_linear_speed
        self.max_angular_speed = max_angular_speed
        self.track_width = track_width
        self.goal_tolerance = goal_tolerance
        self.reflex_dist_threshold = reflex_dist_threshold
        self.current_path: List[Tuple[float, float]] = []
        self.path_index: int = 0
        self.is_goal_reached: bool = False
        self.is_reflex_stopped: bool = False

    def set_path(self, path: List[Tuple[float, float]]):
        self.current_path = path
        self.path_index = 0
        self.is_goal_reached = False
        self.is_reflex_stopped = False

    def clear_path(self):
        self.current_path = []
        self.path_index = 0
        self.is_goal_reached = False

    def check_safety_reflex(self, ranges: List[float]) -> bool:
        if not ranges or len(ranges) < 360:
            return False
        frontal_indices = list(range(0, 36)) + list(range(325, 360))
        for idx in frontal_indices:
            r = ranges[idx]
            if 0.05 < r < self.reflex_dist_threshold:
                self.is_reflex_stopped = True
                return True
        self.is_reflex_stopped = False
        return False

    def compute_command(self, robot_x: float, robot_y: float, robot_theta: float, lidar_ranges: Optional[List[float]] = None) -> Tuple[float, float, Dict[str, Any]]:
        if lidar_ranges and self.check_safety_reflex(lidar_ranges):
            return 0.0, 0.0, {"status": "EMERGENCY_STOP", "reason": "Obstacle < 20cm in front cone!", "distance_remaining": 0.0}
        if not self.current_path:
            return 0.0, 0.0, {"status": "IDLE", "distance_remaining": 0.0}
        goal_x, goal_y = self.current_path[-1]
        dist_to_final_goal = math.hypot(goal_x - robot_x, goal_y - robot_y)
        if dist_to_final_goal <= self.goal_tolerance:
            self.is_goal_reached = True
            self.clear_path()
            return 0.0, 0.0, {"status": "GOAL_REACHED", "distance_remaining": 0.0}
        while self.path_index < len(self.current_path) - 1:
            curr_pt = self.current_path[self.path_index]
            next_pt = self.current_path[self.path_index + 1]
            if math.hypot(next_pt[0] - robot_x, next_pt[1] - robot_y) < math.hypot(curr_pt[0] - robot_x, curr_pt[1] - robot_y):
                self.path_index += 1
            else:
                break
        target_pt = self.current_path[-1]
        for i in range(self.path_index, len(self.current_path)):
            if math.hypot(self.current_path[i][0] - robot_x, self.current_path[i][1] - robot_y) >= self.lookahead_dist:
                target_pt = self.current_path[i]
                break
        dx = target_pt[0] - robot_x
        dy = target_pt[1] - robot_y
        alpha = (math.atan2(dy, dx) - robot_theta + math.pi) % (2.0 * math.pi) - math.pi
        if abs(alpha) > 0.65:
            v = 0.03
            w = math.copysign(self.max_angular_speed * 0.8, alpha)
        else:
            kappa = (2.0 * math.sin(alpha)) / self.lookahead_dist
            v = min(self.max_linear_speed, max(0.08, dist_to_final_goal * 0.6)) * max(0.2, math.cos(alpha))
            w = max(-self.max_angular_speed, min(self.max_angular_speed, v * kappa))
        return round(v, 3), round(w, 3), {"status": "NAVIGATING", "distance_remaining": round(dist_to_final_goal, 3)}
