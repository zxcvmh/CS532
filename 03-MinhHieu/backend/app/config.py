class Config:
    # WebSocket broadcast rates (Hz) - Throttled strictly per Ticket TK-02 & CS532 specs
    WS_RATES = {
        "robot_pose": 10,        # 10Hz synchronized with LiDAR
        "lidar_scan": 10,        # 10Hz
        "map_update": 2,         # 2Hz (sufficient for grid map changes)
        "detections": 10,        # 10Hz
        "navigation_status": 5,  # 5Hz
        "battery": 0.5,          # 0.5Hz (every 2s)
        "sensor_health": 1,      # 1Hz
        "camera_frame": 0,       # 0 = Disabled in WS (Using native MJPEG HTTP /video_feed to prevent lag)
        "trajectory": 2,         # 2Hz
        "planned_path": 2,       # 2Hz
    }

    # Robot parameters
    MAX_SPEED = 0.5            # m/s
    MAX_ANGULAR_SPEED = 1.0    # rad/s
    ROBOT_UPDATE_RATE = 20     # Hz

    # Map parameters
    MAP_RESOLUTION = 0.05      # meters per cell
    MAP_WIDTH = 200            # cells (10m)
    MAP_HEIGHT = 200           # cells (10m)
    MAP_ORIGIN_X = -5.0        # meters
    MAP_ORIGIN_Y = -5.0        # meters

    # LiDAR parameters
    NUM_RAYS = 360             # 1 degree resolution
    MAX_RANGE = 8.0            # meters

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
