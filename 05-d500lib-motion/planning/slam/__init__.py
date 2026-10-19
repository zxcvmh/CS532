"""
Multi-Layer Decoupled SLAM Architecture.
CS532 Advanced AI Robotics — Module 05-d500lib-motion / slam

Exports clean, modular SLAM building blocks:
- GridCore: 2D geometry, arrays, and coordinate transforms.
- BayesianRaycaster: Fast continuous 360-ray log-odds updating and footprint clearing.
- CostmapInflation: EDT distance transform, C-space inflation, dynamic clusters, social bubbles.
- DashboardRasterizer: Solid high-contrast map export (restoring anh_lidar.png aesthetic).
- HectorMatcher: 2D Gaussian likelihood field scan matching for odometry.
- AnchorLayer: Static environmental reference boundaries.
- DynamicLayer: Rolling dynamic obstacle layer with TTL decay.
"""
from .grid_core import GridCore
from .bayesian_raycaster import BayesianRaycaster
from .costmap_inflation import CostmapInflation
from .dashboard_rasterizer import DashboardRasterizer
from .hector_matcher import HectorMatcher
from .anchor_layer import AnchorLayer
from .dynamic_layer import DynamicLayer

CostmapEDT = CostmapInflation

__all__ = [
    "GridCore",
    "BayesianRaycaster",
    "CostmapInflation",
    "CostmapEDT",
    "DashboardRasterizer",
    "HectorMatcher",
    "AnchorLayer",
    "DynamicLayer",
]
