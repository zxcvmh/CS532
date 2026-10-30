import os


class Config:
    # WebSocket broadcast rates (Hz) - Throttled strictly per Ticket TK-02 & CS532 specs
    WS_RATES = {
        "robot_pose": 25,  # 25Hz smooth real-time pose tracking
        "lidar_scan": 20,  # 20Hz responsive LiDAR points
        "map_update": 15,  # 15Hz fast synchronized map updates
        "detections": 10,  # 10Hz
        "navigation_status": 5,  # 5Hz
        "battery": 0.5,  # 0.5Hz (every 2s)
        "sensor_health": 1,  # 1Hz
        "camera_frame": 0,  # 0 = Disabled in WS (Using native MJPEG HTTP /video_feed to prevent lag)
        "trajectory": 2,  # 2Hz
        "planned_path": 2,  # 2Hz
        "motion_xai": 5,  # 5Hz Explainable Motion & Obstacle telemetry
    }

    # Environment Profile: "ROOM" (standard real room), "SEMINAR_5X5", or "ARENA_2X1"
    ENVIRONMENT_MODE = os.environ.get("ENVIRONMENT_MODE", "ROOM")

    # Virtual Arena Bounds (meters in World frame) — SINGLE SOURCE OF TRUTH.
    # Tuple layout: (x_min, x_max, y_min, y_max). ROOM is deliberately unbounded:
    # a stale 2m x 1m arena here block-filled 97% of the map AND clamped every
    # A*/goal coordinate (cs532-jetbot-final-fix-plan.md BUG-01/BUG-02).
    ARENA_BOUNDS_BY_PROFILE = {
        "ROOM": None,  # unbounded room: never fabricate arena walls
        "ARENA_2X1": (-0.25, 2.05, -0.65, 0.65),  # 2m x 1m testbed
        "SEMINAR_5X5": (-0.25, 5.25, -0.25, 5.25),  # 5m x 5m final demo arena
    }
    ARENA_BOUNDS = ARENA_BOUNDS_BY_PROFILE.get(ENVIRONMENT_MODE, None)
    ARENA_BOUNDS_TUPLE = ARENA_BOUNDS  # backwards-compatible alias for legacy callers
    ROOM_BOUNDS_TUPLE = (-5.0, 5.0, -5.0, 5.0)

    # Safety & sensor single-source-of-truth constants (govern every downstream default)
    ROBOT_RADIUS = 0.09  # m, 15 x 10 cm chassis circumscribed radius
    LIDAR_MIN_RANGE = 0.05  # m, datasheet floor 0.03m + 2cm mounting margin
    MAPPING_MAX_RANGE = 6.0  # m, mapping/odometry range ceiling
    REFLEX_STOP = 0.30  # m, DDE critical stop (>= the 300mm accuracy floor)

    # Load calibrated dynamics from setup/robot_dynamics.json (Single Source of Truth)
    _dynamics_paths = [
        os.path.join(os.path.dirname(__file__), "../../../setup/robot_dynamics.json"),
        os.path.join(os.path.dirname(__file__), "../../test-hardware/robot_dynamics.json"),
        "/home/zxcvmh/Projects/CS532/setup/robot_dynamics.json",
        "/home/jetbot/setup/robot_dynamics.json"
    ]
    _cfg_dyn = {}
    for _dp in _dynamics_paths:
        if os.path.exists(_dp):
            try:
                import json
                with open(_dp, "r", encoding="utf-8") as _f:
                    _cfg_dyn = json.load(_f)
                break
            except Exception:
                pass

    # Robot parameters (calibrated to setup/robot_dynamics.json & ceramic tile physics)
    MAX_SPEED = float(_cfg_dyn.get("max_safe_speed_mps", 0.14))  # m/s (matching calibrated max_safe_speed_mps)
    CRUISE_SPEED = float(_cfg_dyn.get("cruise_speed", 0.17))  # m/s (calibrated cruise_speed)
    MIN_DRIVE_SPEED = float(_cfg_dyn.get("min_drive_speed", 0.12))  # m/s (GA25 static deadband compensation)
    TURN_SPEED = float(_cfg_dyn.get("turn_speed", 0.14))  # normalized speed PWM for rotation
    TURN_TIME_90_DEG_SEC = 0.50  # 0.50s for 90 deg = 3.1416 rad/s
    TURN_TIME_180_DEG_SEC = 1.00  # 1.00s for 180 deg
    MAX_ANGULAR_SPEED = 3.1416  # rad/s: EXACT physical angular speed (90 deg in 0.50s)
    ACTUAL_SPEED_MPS = float(_cfg_dyn.get("actual_speed_mps", 0.244))  # m/s
    TRIM_FACTOR = float(_cfg_dyn.get("trim_factor", 1.024))
    LIDAR_YAW_OFFSET_DEG = float(_cfg_dyn.get("lidar_yaw_offset_deg", 0.0))
    CENTRIFUGAL_LIMIT_MPS2 = (
        0.176  # m/s^2 (|v*w| <= 0.176 prevents slip on glossy tile)
    )
    TACTICAL_YIELD_DIST = 0.30  # m (Tactical Yielding stop distance)
    PASSAGE_CLEARANCE_MIN = 0.18  # m (minimum passable corridor width for 0.15m JetBot, allows passing 18cm-25cm corridors)
    LATERAL_DETOUR_OFFSET = (
        0.15  # m (safe detour arc in 1m-wide arena, ensures >18cm wall clearance)
    )
    ROBOT_UPDATE_RATE = 25  # Hz
    NAVIGATION_CONTROLLER = "pure_pursuit"  # "pure_pursuit" (RPP active) or "mppi"

    # Map parameters
    MAP_RESOLUTION = 0.05  # meters per cell
    MAP_WIDTH = 200  # cells (10m)
    MAP_HEIGHT = 200  # cells (10m)
    MAP_ORIGIN_X = -5.0  # meters
    MAP_ORIGIN_Y = -5.0  # meters

    # LiDAR parameters
    NUM_RAYS = 360  # 1 degree resolution
    MAX_RANGE = (
        2.5 if ENVIRONMENT_MODE == "ARENA_2X1" else 6.0
    )  # 2.5m for 2x1m arena, 6.0m for 5x5m seminar room

    # Simulated environment
    # Walls: (x1, y1, x2, y2) line segments
    WALLS = [
        (-5, -4, 5, -4),  # Bottom wall
        (5, -4, 5, 4),  # Right wall
        (5, 4, -5, 4),  # Top wall
        (-5, 4, -5, -4),  # Left wall
        # Internal walls for more interesting map
        (-2, -4, -2, -1),  # Internal wall segment
        (2, 1, 2, 4),  # Internal wall segment
    ]

    # Obstacles: (center_x, center_y, radius)
    OBSTACLES = [
        (2.0, 2.0, 0.5),
        (-2.0, 1.0, 0.3),
        (1.0, -2.0, 0.4),
        (-3.0, -2.0, 0.6),
        (3.5, -1.5, 0.35),
    ]


def _validate_environment_profile():
    """
    Boot-time consistency guard (fail loudly, never silently mis-map).

    ROOM must never be bounded by a small artificial arena. That single mismatch
    is what produced the black map (97.1% lethal cells) and silently clamped the
    start, goal and every waypoint of A* (cs532-jetbot-final-fix-plan.md BUG-01/02).
    """
    mode = Config.ENVIRONMENT_MODE
    bounds = Config.ARENA_BOUNDS
    if mode == "ROOM" and bounds is not None:
        span_x = bounds[1] - bounds[0]
        span_y = bounds[3] - bounds[2]
        if min(span_x, span_y) < 3.0:
            raise RuntimeError(
                f"ENVIRONMENT_MODE='ROOM' must not be bounded by an artificial arena "
                f"{bounds} (min span {min(span_x, span_y):.2f}m < 3.0m). Fix "
                "ARENA_BOUNDS_BY_PROFILE or set ENVIRONMENT_MODE=ARENA_2X1/SEMINAR_5X5."
            )
    if mode not in Config.ARENA_BOUNDS_BY_PROFILE:
        raise RuntimeError(
            f"Unknown ENVIRONMENT_MODE={mode!r}; expected one of "
            f"{sorted(Config.ARENA_BOUNDS_BY_PROFILE)}"
        )


_validate_environment_profile()
