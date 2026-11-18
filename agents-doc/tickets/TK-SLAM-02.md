### TK-SLAM-02: LiDAR points and clusters ignore the lever arm - [x]
- Role: integration (both files are in `dashboard/backend/`)
- Status: DONE 2026-11-15 — auditor PASS. Lever arm = 0.01 m (human-measured, `lidar_lever_arm_m`).
  Contour test_02 passes; dashboard points and cluster poses agree to 1e-7. Ran while DRAFT and touched
  `config.py` (accepted 1-line scope extension). Follow-ups for the next integration ticket:
  `motion_adapter.py:205` use `Config.LIDAR_LEVER_ARM_M` (drop the getattr default);
  `telemetry_parser.py:88` docstring still says LiDAR is at the robot origin.
- Problem: `dashboard/backend/app/hardware/telemetry_parser.py:92-93` (`ranges_to_points`) and
  `dashboard/backend/app/navigation/motion_adapter.py:205` (`la = 0.0`) drop the 0.01 m axle→LiDAR
  offset that the raycaster and matcher apply → dashboard points/person boxes sit 1 cm off and
  the error rotates with heading, even with perfect SLAM.
