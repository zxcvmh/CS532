"""
Package planning: Module Thuật toán Điều hướng & Điều khiển (CS532 JetBot)
Bao gồm:
- OccupancyGridMap: Lưới bản đồ chiếm chỗ thời gian thực, Ray Clamping, Costmap & Social Bubbles
- AStarPlanner: Thuật toán tìm đường tối ưu toàn cục kết hợp Costmap Penalty & Path Smoothing
- PurePursuitController: Bộ bám quỹ đạo Pure Pursuit, né hình thang Plateau, điều tốc vào cua Curvature Scaling
- LiDARClusterDetector: Thuật toán gom cụm 1D Range Jump O(N), ROI filtering & selective PCA
- D500Parser: Giải mã gói tin LiDAR D500 UART 360 độ
"""

from .occupancy_grid import OccupancyGridMap
from .astar_planner import AStarPlanner
from .controller import PurePursuitController
from .mppi_controller import MPPIController
from .obstacle_clustering import LiDARClusterDetector, ObstacleCluster
from .lidar_parser import D500Parser
from .disparity_extender import DisparityExtender
from .cbf_safety_filter import CBFSafetyFilter
from .lidar_odometry import Lidar2DOdometry
from .kinodynamic_rvo import KinodynamicRVOPlanner

__all__ = [
    "OccupancyGridMap",
    "AStarPlanner",
    "PurePursuitController",
    "MPPIController",
    "LiDARClusterDetector",
    "ObstacleCluster",
    "D500Parser",
    "DisparityExtender",
    "CBFSafetyFilter",
    "Lidar2DOdometry",
    "KinodynamicRVOPlanner",
]
