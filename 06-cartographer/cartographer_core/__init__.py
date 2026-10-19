"""
Google Cartographer 2D Core Reference Library
=============================================
Reference implementation of Google Cartographer 2D SLAM algorithms
(Hess et al. 2016, IEEE ICRA):
- ProbabilityGrid2D (Odds representation, Bayesian hit/miss updates)
- RealTimeCorrelativeScanMatcher2D (CSM exhaustive candidate search)
- Submap2D & ActiveSubmaps2D (Dual-overlapping submap lifecycle)
- PoseGraph2D (Trajectory nodes, loop closure detection, SPA optimization)
"""
from .probability_grid import (
    ProbabilityGrid2D,
    probability_to_odds,
    odds_to_probability,
    P_MIN,
    P_MAX,
    P_HIT_DEFAULT,
    P_MISS_DEFAULT
)
from .correlative_scan_matcher import RealTimeCorrelativeScanMatcher2D
from .submap_2d import Submap2D, ActiveSubmaps2D
from .pose_graph_2d import PoseGraph2D, TrajectoryNode2D, Constraint2D

__all__ = [
    "ProbabilityGrid2D",
    "probability_to_odds",
    "odds_to_probability",
    "P_MIN",
    "P_MAX",
    "P_HIT_DEFAULT",
    "P_MISS_DEFAULT",
    "RealTimeCorrelativeScanMatcher2D",
    "Submap2D",
    "ActiveSubmaps2D",
    "PoseGraph2D",
    "TrajectoryNode2D",
    "Constraint2D"
]
