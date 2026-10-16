class Config:
    # WebSocket broadcast rates (Hz) - Throttled strictly per Ticket TK-02 & CS532 specs
    WS_RATES = {
        "robot_pose": 10,        # 10Hz synchronized with LiDAR
        "lidar_scan": 10,        # 10Hz
        "map_update": 2,         # 2Hz (Decoupled rate for map visualization, non-blocking)
        "detections": 10,        # 10Hz
        "navigation_status": 5,  # 5Hz
        "battery": 0.5,          # 0.5Hz (every 2s)
        "sensor_health": 1,      # 1Hz
        "camera_frame": 0,       # 0 = Disabled in WS (Using native MJPEG HTTP /video_feed to prevent lag)
        "trajectory": 2,         # 2Hz
        "planned_path": 2,       # 2Hz
        "motion_xai": 5,         # 5Hz Explainable Motion & Obstacle telemetry
    }

    # Environment Profile: "ARENA_2X1" (Sa ban test 2mx1m) or "SEMINAR_5X5" (Phong hoc 5mx5m)
    ENVIRONMENT_MODE = "ARENA_2X1"

    # Virtual Arena Bounds for 2m x 1m testbed (meters in World frame)
    # Strictly bounds A* search space so it never routes into infinite empty void outside arena
    ARENA_BOUNDS = {"x_min": -0.1, "x_max": 2.1, "y_min": -0.6, "y_max": 0.6}
    ARENA_BOUNDS_TUPLE = (-0.1, 2.1, -0.6, 0.6)

    # Robot parameters (calibrated to setup/robot_dynamics.json & ceramic tile physics)
    MAX_SPEED = 0.14               # m/s (matching calibrated max_safe_speed_mps)
    CRUISE_SPEED = 0.17            # m/s (calibrated cruise_speed)
    MIN_DRIVE_SPEED = 0.12         # m/s (GA25 static deadband compensation)
    TURN_SPEED = 0.14              # m/s
    MAX_ANGULAR_SPEED = 0.40       # rad/s (calibrated zero-slip rotation on glossy tile and stable ICP scan matching)
    CENTRIFUGAL_LIMIT_MPS2 = 0.176 # m/s^2 (|v*w| <= 0.176 prevents slip on glossy tile)
    TACTICAL_YIELD_DIST = 0.30     # m (Tactical Yielding stop distance)
    PASSAGE_CLEARANCE_MIN = 0.18   # m (minimum passable corridor width for 0.15m JetBot, allows passing 18cm-25cm corridors)
    ROBOT_UPDATE_RATE = 20         # Hz
    NAVIGATION_CONTROLLER = "pure_pursuit" # "pure_pursuit" (RPP active) or "mppi"

    # Map parameters
    MAP_RESOLUTION = 0.05      # meters per cell
    MAP_WIDTH = 200            # cells (10m)
    MAP_HEIGHT = 200           # cells (10m)
    MAP_ORIGIN_X = -5.0        # meters
    MAP_ORIGIN_Y = -5.0        # meters

    # LiDAR parameters
    NUM_RAYS = 360             # 1 degree resolution
    MAX_RANGE = 6.0            # meters (supporting both 2m x 1m arena and 5m x 5m seminar room)

    # Simulated environment
    # Walls: (x1, y1, x2, y2) line segments
    WALLS = [
        (-5, -4, 5, -4),      # Bottom wall
        (5, -4, 5, 4),        # Right wall
        (5, 4, -5, 4),        # Top wall
        (-5, 4, -5, -4),      # Left wall
        # Internal walls for more interesting map
        (-2, -4, -2, -1),     # Internal wall segment
        (2, 1, 2, 4),         # Internal wall segment
    ]

    # Obstacles: (center_x, center_y, radius)
    OBSTACLES = [
        (2.0, 2.0, 0.5),
        (-2.0, 1.0, 0.3),
        (1.0, -2.0, 0.4),
        (-3.0, -2.0, 0.6),
        (3.5, -1.5, 0.35),
    ]
