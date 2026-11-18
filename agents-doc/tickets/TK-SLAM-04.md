### TK-SLAM-04: Stop-then-scan — never map smeared sweeps, re-anchor pose after each turn (direction C)
- Role: slam
- Status: DONE (attempt 2, auditor PASS 2026-11-15) — needs field check on the real robot.
  Accepted deviation: motion_adapter.py is +8/−5 (13 changed lines) vs "≤ 10"; the minimum required
  behaviour (re-indent update_scan, move the scan counter) already needs ~12. Supervisor accepts.
  Non-blocking follow-ups (→ TK-SLAM-06 cleanup): pivot branch returns early, so its diag uses `dtheta`
  not `dtheta_rad` and lacks match_info/inliers (no consumer today: telemetry_parser reads only
  status/is_reliable); match_count not incremented; keyframe not initialised if the robot spins before
  the first scan; thresholds 0.05/0.02 duplicated in lidar_odometry.py and motion_adapter.py.
  `test_slam_inplace_turn.py` 0.35 rad/s cases removed (impossible on the robot; passed by 15x coincidence).
- History: READY — ATTEMPT 2 (2026-11-15). TK-SLAM-03 committed (fc02870).
  Attempt 1 (WIP saved in `docs/plans/tk-slam-04-attempt1-wip.patch`, removed from the tree) passed the
  first test version but is REJECTED:
  (a) heading prediction used the MAGNITUDE of w_cmd (`eff_w = copysign(min(|w_cmd|, pivot_rate), w_cmd)`).
      w_cmd is the planner's command, not the physical rate — the bridge spins at ONE rate whenever
      |w| >= 0.02. With the hardened test: heading error 78.4° at stop 3, 180° at stop 4;
  (b) `motion_adapter.py` changed 16 lines (limit 10);
  (c) added a second hypothesis (map writes only on Hector SUCCESS, "R2 defense") — out of scope.
- Attempt 2 instructions (Supervisor checked feasibility: a ~14-line change passes both tests and keeps
  `test_slam_inplace_turn.py` 4/4):
  1. While pivoting, predict heading with the SIGN of w_cmd and the rate from json:
     `copysign(self.calibrated_pivot_yaw_rate, w_cmd)`. Never use |w_cmd| as the spin rate.
  2. While pivoting, skip scan-to-map matching (dead-reckon heading, translation 0).
  3. `motion_adapter.py`: skip `grid_map.update_scan` and anchor locking while spinning. ≤ 10 lines.
     No Hector-status gate.
- Decision (human, 2026-11-15): direction C. The robot can only spin at ~5.24 rad/s (`turn_speed` =
  `min_drive_speed` = 0.19 PWM, 90 deg in 0.3 s measured with `setup/calibrate_motors.py`), so a 10 Hz
  D500 sweep is smeared by ~30 deg while spinning. Slowing down (A) is not possible without pulsing;
  software deskew (B) is deferred.
- Required behaviour:
  1. While the commanded |w| is above a small threshold (robot spinning in place): do NOT write the
     sweep into the occupancy grid / anchor, and do not trust scan-to-map matching on the smeared sweep;
     predict heading from the commanded w.
  2. On the first stationary sweeps after a turn: match the (unsmeared) sweep against the map, seeded
     with the predicted heading, with a search window wide enough to absorb a ±10 % turn error
     (±9 deg on a 90 deg turn); only then resume map writes.
- Locked test: `tests/acceptance/test_slam_stop_turn_go.py` — 4 x (90 deg spin at the physical rate with
  smeared sweeps, actual turn x1.00/x0.90/x1.10/x0.95, then 4 stationary sweeps), anchor ON and production
  (anchor OFF). Hardened 2026-11-15: w_cmd magnitude during the spin ramps 0.6/1.2/1.8 rad/s (NOT the
  physical 5.24 rad/s) — only its sign is trustworthy.
- Baseline 2026-11-15 (on top of the uncommitted TK-SLAM-03 tree):
  anchor ON: drift 1.3–3.7 cm, heading 0.0°, **76 ghost cells** → FAIL;
  production: heading error 0.0 → 2.5 → **5.1 → 14.0°**, drift up to 8.4 cm, **129 ghost cells** → FAIL.
  Control run with unsmeared sweeps (same scenario): 1.6 cm / 0.0° / 0 ghost cells at every stop — the
  failure is caused by the smear only.
- Allowed files: `motion/planning/lidar_odometry.py`; `dashboard/backend/app/navigation/motion_adapter.py`
  **only** to skip `grid_map.update_scan` / anchor locking while spinning (≤ 10 changed lines there;
  Supervisor-approved exception to Integration ownership).
- Acceptance command: `.venv/bin/python tests/acceptance/test_slam_stop_turn_go.py -v`
- Threshold: both tests pass — at every stop heading error ≤ 3°, drift ≤ 5 cm; ghost wall cells ≤ 10;
  map writes (`grid_map.update_scan` calls) = 0 during every spin and ≥ 1 during every stop
  (test hardened 2026-11-15: baseline spin=[3,3,3,3] stop=[4,4,4,4]; "freeze the map forever" now fails).
- Regression commands: `.venv/bin/python tests/acceptance/test_slam_inplace_turn.py -v` (must stay 4/4);
  `.venv/bin/python tests/test_hector_anchor_slam.py`;
  `cd dashboard/backend && ../../.venv/bin/python -c "import app.main"`
- Depends on: TK-SLAM-03 (DONE fc02870), TK-INT-07 (bridge spin PWM step, Supervisor).
  The earlier "field dependency TK-INT-04" was wrong: on the binary path `telemetry_parser.py:406` checks
  keys `linear_vel`/`angular_vel` but `binary_protocol.py:117-118` returns `linear_speed`/`angular_speed`,
  so telemetry velocities are never applied; w_cmd is the planner's command (`coordinator.py:391,422`),
  which `stop()` resets to 0.
