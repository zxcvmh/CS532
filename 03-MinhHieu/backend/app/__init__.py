import sys
import os

# Single Source of Truth for Motion Planning: 05-d500lib-motion/planning
_motion_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../05-d500lib-motion"))
if _motion_dir not in sys.path and os.path.exists(_motion_dir):
    sys.path.insert(0, _motion_dir)

try:
    import planning
    sys.modules["app.planning"] = planning
    for sub in [
        "occupancy_grid", "astar_planner", "controller", "mppi_controller",
        "obstacle_clustering", "lidar_odometry", "cbf_safety_filter",
        "disparity_extender", "kinodynamic_rvo", "lidar_parser"
    ]:
        try:
            mod = __import__(f"planning.{sub}", fromlist=[sub])
            sys.modules[f"app.planning.{sub}"] = mod
        except Exception:
            pass
except Exception:
    pass
