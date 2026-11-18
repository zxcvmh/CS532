### TK-SLAM-08: A heading the LiDAR has not confirmed is never published or used
- Role: slam
- Status: SUPERSEDED by TK-SLAM-09 (2026-11-15) for the heading; its consumer gating (clusters / tracker /
  theta extrapolation) is kept and its locked test remains a regression. Was: READY. Builds on TK-SLAM-07 attempt 2
  (working tree) — TK-SLAM-07 must be audited and closed first.
- Field symptom (human, after TK-SLAM-07 attempt 2): pressing A/D or the joystick makes the map rotate with
  the robot, then snap back; black/green trails and ghost legs/boxes stay behind.
- Measured on the real sweeps (field_turns_2026-11-15.npz): while the button is held the robot has NOT moved
  (motor start delay); it turns after release and coasts 0.3–0.6 s. TK-SLAM-07 attempt 2 still dead-reckons
  5.24 rad/s from the first command sweep → published heading leads the LiDAR by 110–165°, until the LiDAR
  says still and the re-anchor snaps it back (= "map quay rồi quay lại"). The occupancy map is NOT written
  then (0 writes at wrong heading), but every sweep still runs `process_clusters` → obstacle tracker
  (legs/boxes) and `insert_obstacle_clusters` with that heading: 14–42 wrong-heading writes per replay.
  Third source: `telemetry_parser.py:250-260` extrapolates `pose.theta += angular_vel * dt` between sweeps
  from the COMMAND, so the icon/map also turn the moment the key is pressed.
- Human proposal "wait < 1 s after the key press before updating": right principle (never trust the command
  before the motor moves), but a fixed timer does not fit the data — the robot turns mostly AFTER release
  and coasts, and start delay varies. Equivalent rule with no magic number: wait for the LiDAR.
- Required behaviour:
  1. From a pivot command until the LiDAR re-anchor, do NOT advance heading from w_cmd (hold the last
     confirmed heading, or track it from the LiDAR — either passes). Expose "pose confirmed" (e.g.
     `can_write_map`) in the info dict.
  2. `MotionAdapter.process_clusters` (tracker update) and `insert_obstacle_clusters` /
     `insert_low_obstacles` do nothing while the pose is not confirmed (gate on the LiDAR, not on w_cmd).
  3. `telemetry_parser.py`: no inter-sweep `theta` extrapolation from `angular_vel` while the pose is not
     confirmed (both JSON and binary paths). Not covered by the locked test — auditor reads the diff.
- Supervisor feasibility check (monkeypatch, not in repo): pivot rate 0 + gate consumers on `can_write_map`
  → pose_hold 3/3, field_turns 6/6, pivot_then_arc 7/7, stop_turn_go 2/2.
- Locked test: `tests/acceptance/test_slam_pose_hold.py` (replay of the real sweeps, 3 command timings,
  MotionAdapter called exactly as telemetry_parser does on each fresh sweep).
- Baseline (TK-SLAM-07 attempt 2 working tree): FAILED 3/3 — heading ahead of LiDAR 110.7 / 164.5 / 142.9°;
  tracker + cluster writes at wrong heading 26 / 42 / 14.
- Allowed files: `motion/planning/lidar_odometry.py`; `dashboard/backend/app/navigation/motion_adapter.py`;
  `dashboard/backend/app/hardware/telemetry_parser.py` (theta extrapolation gate only, ≤ 10 lines).
- Forbidden: `tests/`, `setup/robot_dynamics.json`, `dashboard/jetbot_bridge.py`, everything else. No fixed
  timers / delays keyed to the button.
- Acceptance command: `.venv/bin/python tests/acceptance/test_slam_pose_hold.py -v`
- Threshold: 3/3 — heading never more than 10° beyond the LiDAR-seen rotation; 0 map / tracker / cluster
  writes at sweeps whose heading is > 5° wrong; > 100 map and cluster writes per replay (layers stay live).
- Regression commands (all must stay green, paste RAW output): `test_slam_field_turns.py` (6/6),
  `test_slam_pivot_then_arc.py` (7/7), `test_slam_stop_turn_go.py` (2/2), `test_slam_inplace_turn.py` (2/2),
  `tests/test_hector_anchor_slam.py`; `cd dashboard/backend && ../../.venv/bin/python -c "import app.main"`;
  Python 3.6 syntax (`ast.parse(..., feature_version=(3,6))`) on all 3 files.
