import os
import sys

# Expose unified workspace packages (jetbot_planning, jetbot_il, jetbot_hardware, jetbot_perception)
_PACKAGES_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../packages"))
if _PACKAGES_DIR not in sys.path and os.path.isdir(_PACKAGES_DIR):
    sys.path.insert(0, _PACKAGES_DIR)

try:
    import jetbot_planning

    sys.modules["planning"] = jetbot_planning
    sys.modules["app.planning"] = jetbot_planning
    for sub in [
        "occupancy_grid",
        "astar_planner",
        "controller",
        "mppi_controller",
        "obstacle_clustering",
        "lidar_odometry",
        "cbf_safety_filter",
        "disparity_extender",
        "kinodynamic_rvo",
        "lidar_parser",
    ]:
        try:
            mod = __import__(f"jetbot_planning.{sub}", fromlist=[sub])
            sys.modules[f"planning.{sub}"] = mod
            sys.modules[f"app.planning.{sub}"] = mod
        except Exception:
            pass
except Exception:
    pass
