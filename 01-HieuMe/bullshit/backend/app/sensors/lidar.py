"""
LiDAR sensor module.

For real JetBot integration, implement a ROS2LiDARProvider here:

Example:
    class ROS2LiDAR(SensorProvider):
        def __init__(self, node):
            self.node = node
            self.latest_scan = LidarScan(points=[])
            self.sub = node.create_subscription(
                LaserScan, '/scan', self._callback, 10
            )
        
        def _callback(self, msg: LaserScan):
            # Convert polar LaserScan to Cartesian LidarPoints
            points = []
            for i, r in enumerate(msg.ranges):
                if msg.range_min < r < msg.range_max:
                    angle = msg.angle_min + i * msg.angle_increment
                    points.append(LidarPoint(
                        x=r * math.cos(angle),
                        y=r * math.sin(angle)
                    ))
            self.latest_scan = LidarScan(points=points)
        
        async def get_lidar_scan(self) -> LidarScan:
            return self.latest_scan
"""
