file_path = "/home/jetbot/mhieu/backend/app/robot/mock_robot.py"
with open(file_path, "r") as f:
    code = f.read()

# 1. Update handle_real_telemetry to use grid_map
old_lidar_block = """            if scan_points:
                self.current_scan = LidarScan(points=scan_points)"""

new_lidar_block = """            if scan_points:
                self.current_scan = LidarScan(points=scan_points)
                self.grid_map.update_scan(
                    self.pose.x, self.pose.y, self.pose.theta,
                    ranges=[p.dist if hasattr(p, 'dist') else 0.0 for p in self.current_scan.points] if hasattr(self.current_scan.points[0], 'dist') else [0.0]*360,
                    max_valid_range=Config.MAX_RANGE - 0.15
                )
                self.map_data = self.grid_map.to_dashboard_data()"""

if old_lidar_block in code:
    code = code.replace(old_lidar_block, new_lidar_block, 1)

# 2. Update navigation loop to Pure Pursuit + 10Hz Reflex Brake
old_nav_loop = """            # Autonomous navigation control
            if (self.mode == ModeEnum.AUTONOMOUS and 
                self.nav_status.status == NavigationStatusEnum.NAVIGATING):
                gx = self.nav_status.goal_x
                gy = self.nav_status.goal_y
                if gx is not None and gy is not None:
                    dx = gx - self.pose.x
                    dy = gy - self.pose.y
                    dist = math.hypot(dx, dy)
                    
                    if dist < 0.15:
                        self.nav_status.status = NavigationStatusEnum.REACHED
                        self.nav_status.progress = 100.0
                        self.nav_status.distance_remaining = 0.0
                        self.linear_vel = 0.0
                        self.angular_vel = 0.0
                        self.planned_path.points = []
                        self._add_event("SUCCESS", "Goal reached!")
                        await self.send_hardware_stop()
                    else:
                        target_angle = math.atan2(dy, dx)
                        angle_diff = (target_angle - self.pose.theta + math.pi) % (2 * math.pi) - math.pi
                        
                        if abs(angle_diff) > 0.1:
                            self.angular_vel = math.copysign(
                                min(abs(angle_diff) * 2.0, Config.MAX_ANGULAR_SPEED), angle_diff
                            )
                            self.linear_vel = 0.05  # Slow forward while turning
                        else:
                            self.angular_vel = angle_diff * 0.5  # Small correction
                            self.linear_vel = min(dist * 0.8, Config.MAX_SPEED)
                            
                        self.nav_status.distance_remaining = dist
                        self.nav_status.velocity = self.linear_vel
                        await self.send_hardware_cmd_vel(self.linear_vel, self.angular_vel)
                        
                        # Calculate progress
                        if self._initial_goal_distance > 0:
                            self.nav_status.progress = max(0, min(100,
                                (1.0 - dist / self._initial_goal_distance) * 100
                            ))"""

new_nav_loop = """            # Autonomous navigation control via Pure Pursuit + 10Hz Reflex Safety Brake
            if (self.mode == ModeEnum.AUTONOMOUS and 
                self.nav_status.status == NavigationStatusEnum.NAVIGATING):
                
                v, w, info = self.controller.compute_command(
                    self.pose.x, self.pose.y, self.pose.theta,
                    lidar_ranges=self.latest_raw_ranges
                )
                
                status_str = info.get("status")
                if status_str == "EMERGENCY_STOP":
                    self.linear_vel = 0.0
                    self.angular_vel = 0.0
                    await self.send_hardware_stop()
                    self.nav_status.status = NavigationStatusEnum.BLOCKED
                    self._add_event("ERROR", f"🚨 10Hz SAFETY REFLEX: {info.get('reason')}")
                elif status_str == "GOAL_REACHED":
                    self.linear_vel = 0.0
                    self.angular_vel = 0.0
                    await self.send_hardware_stop()
                    self.nav_status.status = NavigationStatusEnum.REACHED
                    self.nav_status.progress = 100.0
                    self.nav_status.distance_remaining = 0.0
                    self.planned_path.points = []
                    self._add_event("SUCCESS", "🎯 Target Goal Reached Successfully!")
                else:
                    self.linear_vel = v
                    self.angular_vel = w
                    self.nav_status.velocity = v
                    rem_dist = info.get("distance_remaining", 0.0)
                    self.nav_status.distance_remaining = rem_dist
                    if self._initial_goal_distance > 0:
                        self.nav_status.progress = max(0.0, min(100.0, (1.0 - rem_dist / self._initial_goal_distance) * 100.0))
                    await self.send_hardware_cmd_vel(v, w)"""

if old_nav_loop in code:
    code = code.replace(old_nav_loop, new_nav_loop, 1)

with open(file_path, "w") as f:
    f.write(code)

print("HOAN TAT: He thong Planning & Control da duoc tich hop 100% vao mock_robot.py!")
