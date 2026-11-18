### TK-SLAM-09: Track heading with a LiDAR-only wide-window correlative scan matcher (method change)
- Role: slam
- Status: READY — attempt 2 (2026-11-16). Locked test tightened after attempt 1: 13 tests, attempt 1's working tree
  fails 3 of them (11-13) for the reasons below; the other 10 pass on it.
- Why the method changes (human decision, 2026-11-15): TK-SLAM-04/07/08 all guessed the heading during a turn
  (dead-reckon from the key, then hold the pose). Field result: map turns with the robot, rays rotate while the
  icon stands still. The robot has no encoder or IMU; the D500 at 10 Hz sees ~30° of rotation per sweep at the
  pivot rate. The human chose: correlative scan matching (Olson 2009) with a wide search window, LiDAR only, plus
  de-skew later; slower pivots later (bridge/hardware).
- Old versions checked on the real data (Supervisor bench, 2026-11-15): d551f38 `Lidar2DOdometry` — 3.1° on small
  turns but 44.6° on big turns and 24.5 cm position error; repo `CartographerAdapter` CSM with the key as prior —
  98° on small turns; same CSM with NO key prior and a 60° window — passes both recordings (90 ms, too slow).
- Required behaviour:
  1. Pose comes from a correlative scan matcher against a local submap / probability grid, search window ≥ ±45°
     in heading and ≥ ±10 cm in x/y, coarse-to-fine (vectorised or branch-and-bound) — not a Python loop over
     candidates. Prior = the matcher's own previous motion (constant velocity) or zero. `w_cmd`/`v_cmd` must not
     change the result (test runs recorded / zero / inverted commands).
  2. Remove the pivot dead-reckoning (`calibrated_pivot_yaw_rate` × dt) and the pose-hold from the pose path. The
     heading is TRACKED during the turn (moving-sweep error ≤ 20°).
  3. Only LiDAR-still sweeps (or translating sweeps that matched well) are written to the matcher's submap and to
     `grid_map`; do not re-insert an unchanged view every sweep (`ProbabilityGrid2D.insert_scan` is a Python loop,
     ~9 ms per call — the Supervisor prototype inserts when the pose moved > 1.5 cm / 1° or every 5th still sweep).
  4. Keep TK-SLAM-08's consumer gating (clusters, tracker, low obstacles, theta extrapolation) keyed on the same
     "pose confirmed" flag.
  5. De-skew is NOT part of this ticket (next ticket, measured against the moving-sweep error).
- Supervisor feasibility prototype (scratchpad, not in repo): passes this test 6/6 (still ≤ 4.5°, moving ≤ 14.5°,
  pos err ≤ 1.6 cm, 5.7 ms) and field_turns 6/6, pivot_then_arc 7/7, stop_turn_go 2/2, inplace_turn 2/2.
  It reuses `cartographer/cartographer_core` (`ActiveSubmaps2D`, `ProbabilityGrid2D`) — reuse is allowed.
- Locked test: `tests/acceptance/test_slam_csm_turns.py` + `tests/acceptance/data/big_turns_2026-11-15.npz`
  (new real recording: 4 large turns, left and right) + existing `field_turns_2026-11-15.npz`.
- Baseline (TK-SLAM-07 att 2 + TK-SLAM-08 att 1 working tree): FAILED 6/6 — moving-sweep heading error 62–179°;
  with w_cmd = 0 still error 24.5° and position error 10 cm.
- Allowed files: `motion/planning/lidar_odometry.py`; `dashboard/backend/app/navigation/motion_adapter.py`;
  `dashboard/backend/app/hardware/telemetry_parser.py` (gating only); one NEW file
  `motion/planning/slam/csm_tracker.py` if the matcher is kept separate. ≈ 250 changed lines max.
- Forbidden: `tests/`, `setup/robot_dynamics.json`, `dashboard/jetbot_bridge.py`, `cartographer/` (read/import
  only), everything else. No timers keyed to the button, no use of w_cmd in the pose.
- Acceptance command: `.venv/bin/python tests/acceptance/test_slam_csm_turns.py -v`
- Threshold: 13/13 — still heading max ≤ 5.0°, p90 ≤ 4.0°; moving heading worst ≤ 20°, p90 ≤ 10°; position error vs
  LiDAR truth ≤ 3 cm at each still segment; occupied between 0.60 and 1.25 × oracle; median update ≤ 15 ms; the same 6 checks under 4 more
  command variants (forward `v_cmd` while turning / on every sweep: the pose must not care); pose x, y, theta are plain
  Python `float`; `set_pose` re-anchors (below).
- Attempt 2 requirements (Supervisor, from the audit findings of attempt 1; tests `test_11..13` in the locked file):
  (a) `update_odometry` returns `theta` as `numpy.float32` (the CSM tracker returns numpy values) — x, y, theta must be
  plain `float`, also right after `set_pose`. (b) `MotionAdapter.set_pose(x, y, th)` mid-recording must re-anchor the
  world frame: attempt 1 snaps (7 cm / 7 deg right after `set_pose(0,0,0)` at truth heading 150 deg) and tracks worse
  afterwards (moving worst 30.6 deg with a non-zero anchor) because the CSM submap stays in the old frame.
  Supervisor-verified reference (in memory, not in the repo): keep the submap AND the internal pose, store a rigid
  offset (rotation + translation) between the internal and the published frame in `CSMTracker.set_pose`, apply it to
  every output -> right after 0 cm / 0.1 deg, later still 2.3 deg, moving 11.9 deg. Clearing the submap instead fails
  the later-moving bound (21.8 / 39.4 deg) — do not do that. New guards that attempt 1 already passes: `v_turn` /
  `v_always` variants, occupancy floor 0.60 (attempt 1 measures 0.82 / 0.86).
- `test_slam_pose_hold.py` is TK-SLAM-08's gate: its envelope tolerance was aligned 10 -> 20 deg by the Supervisor
  (ledgered 2026-11-16, matches the 20 deg moving bound here); writes-at-wrong-heading stay 0. Coders must not edit it.
- Regression commands (all green, paste RAW output of every one): `test_slam_field_turns.py` (6/6),
  `test_slam_pose_hold.py` (3/3), `test_slam_pivot_then_arc.py` (7/7), `test_slam_stop_turn_go.py` (2/2),
  `test_slam_inplace_turn.py` (2/2), `tests/test_hector_anchor_slam.py`;
  `cd dashboard/backend && ../../.venv/bin/python -c "import app.main"`; Python 3.6 syntax
  (`ast.parse(..., feature_version=(3,6))`) on every changed file.
- References for `/researcher` to verify before they drive further design (not yet opened by the Supervisor this
  session): E. Olson, "Real-Time Correlative Scan Matching", ICRA 2009; W. Hess et al., "Real-Time Loop Closure in
  2D LIDAR SLAM", ICRA 2016 (branch-and-bound).
