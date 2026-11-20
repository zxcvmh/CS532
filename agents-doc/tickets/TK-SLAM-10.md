### TK-SLAM-10: Scan-match quality score and TRACKING / WEAK / LOST status (flag only, never replace the pose)
- Role: slam
- Status: READY (2026-11-20) — locked test `tests/acceptance/test_slam_csm_status.py` (4 tests, ~30 s): 3 RED with `KeyError: 'match_status'` (interface missing) on HEAD, `test_03_pose_unchanged` is a regression guard that PASSES on HEAD (golden `tests/acceptance/data/slam09_golden.npz`). Supervisor prototype (scratchpad) confirms the score: clean still 0.85, white noise 0.01-0.05.
- ATTEMPT 2 (2026-11-20, after the coder's needs-human): three Supervisor decisions, prototyped in the scratchpad on 0.64 s dynamics: 4/4 status, 13/13 SLAM-09, 4/4 hector. (a) LOST uses a separate `SCORE_LOST` = 0.10: LOST after `LOST_AFTER` = 5 consecutive sweeps with score < 0.10 (white noise 0.01-0.05, real turns >= 0.17); WEAK is still score < `SCORE_WEAK` 0.64. CORRECTION of this ticket: "longest bad run 3" counted sweeps with heading error > 6 deg; sweeps with score < 0.64 run 6-8 long in `big`. (b) Warmup: while `submaps[0].num_range_data < 2` the status is TRACKING and `can_write_map` is not gated by the score. (c) Scope extension: `lidar_odometry.py` must NOT derive `CSMTracker.max_dth_step_rad` from `robot_dynamics.json`; pin it to the audited value `(pi/2)/0.53*1.25*0.1` = 0.37047 (named constant, exact expression, not a rounded 0.3705: rounding already changed poses by 0.2). Locked test for (c): `tests/acceptance/test_slam_csm_dynamics_independent.py` (red now: clamp 0.3068 at 0.64 s, pose off golden by 0.6). `setup/robot_dynamics.json` stays at the human's 0.64 s (TK-INT-12); do not touch it.
- Interface (fixed by the test): `MotionAdapter.update_odometry(...)[3]` gains `match_status` ('TRACKING'/'WEAK'/'LOST'), `match_score` (float), `reinit_requested` (bool, cleared by `set_pose`/reset); `can_write_map` False while WEAK/LOST. Existing `status` ('STILL'/'TRACKING') unchanged. The first sweep has no score: report TRACKING.
- Problem: during fast pivots the CSM tracker (TK-SLAM-09) gives 1-3 sweeps with heading errors of 6-14 deg and no
  signal tells the map writer, the dashboard or the IL layer that those sweeps are weak. Inspired by
  `lidar_localization_ros2` (score-gated matching, status Tracking / Weak / Lost, reinitialization request); that repo is
  map-based 3D localization on ROS 2 and cannot be copied, only the idea.
  Supervisor prototype (scratchpad, not in the repo: a copy of `csm_tracker.py` that also stores
  `best_fine_score / len(pts_match)` after each update), on the two turn recordings (488 sweeps with a score):
  - median score: sweeps with err > 6 deg 0.45 (n = 10) vs sweeps with err <= 3 deg 0.86 (n = 444);
  - gate score < 0.64: flags 8/10 bad sweeps (80 %), false alarm 30/444 (6.8 %); gate < 0.79: 10/10 but 17 % false alarms;
  - the score falls with rotation per sweep: 0.86 (<= 8 deg), 0.57 (8-17 deg), 0.44 (> 17 deg);
  - the three recordings with people near a still robot (`docs/measurements/2026-11-16/`, 702 / 603 / 598 sweeps):
    p5 score 0.83-0.85, below 0.64 in 0.7 % / 0.0 % / 0.0 % of sweeps, so people alone do not trigger the gate.
  CORRECTION to the first idea ("keep the predicted pose when a match is rejected", as that repo does): tried in the
  prototype (hold the last published heading, or extrapolate it) and it is MUCH WORSE: moving heading max 62 deg
  (field) and 179 deg (big) instead of 12 / 14 deg. That repo bridges rejected scans with external odometry or an IMU;
  this robot has neither. So the pose is never replaced here: status only.
- Required: (1) `CSMTracker` stores a per-point match score after each `update` (`last_score`, float); (2) a status
  `TRACKING` / `WEAK` / `LOST`: WEAK when the score is below `SCORE_WEAK`; LOST after `LOST_AFTER` consecutive WEAK
  sweeps, which also sets `reinit_requested` = True (cleared by `reset_pose` / `set_pose`); (3) the published
  x, y, theta are bit-identical to today's (the status is extra output, not a correction); (4) `can_write_map` is also
  False while the status is WEAK or LOST; (5) the status is reachable from `Lidar2DOdometry` and `MotionAdapter`
  (attribute or return info; telemetry/dashboard wiring is OUT of scope). Provisional constants, named, not physics:
  `SCORE_WEAK` = 0.64, `LOST_AFTER` = 5 (the longest run of bad sweeps in the recordings was 3).
- Allowed files (attempt 2 unchanged; the clamp pin is a one-line edit in `lidar_odometry.py`): `motion/planning/slam/csm_tracker.py`; `motion/planning/lidar_odometry.py`;
  `dashboard/backend/app/navigation/motion_adapter.py`. About 150 changed lines max.
- Forbidden files: `tests/`, `setup/robot_dynamics.json`, `dashboard/jetbot_bridge.py`, `cartographer/`, everything else +
  `agents-doc/LOCKED_PATHS.txt`. Python 3.6 compatible; no new dependency.
- Acceptance command: `.venv/bin/python tests/acceptance/test_slam_csm_status.py -v` AND `.venv/bin/python tests/acceptance/test_slam_csm_dynamics_independent.py -v`. Assertions on the real recordings:
  recall >= 80 % of sweeps with err > 6 deg flagged WEAK or LOST; false alarm <= 10 % of sweeps with err <= 3 deg; on
  the three person recordings TRACKING in >= 98 % of sweeps and never LOST; no LOST on the two turn recordings;
  published heading equal to the SLAM-09 tracker within 1e-9.
- Threshold (numbers): recall >= 80 %, false alarm <= 10 %, person recordings TRACKING >= 98 %, LOST count = 0,
  median update time still <= the limit already asserted in `test_slam_csm_turns.py`.
- Regression commands: `.venv/bin/python tests/acceptance/test_slam_csm_turns.py -v` (13 OK, unchanged);
  `.venv/bin/python tests/test_hector_anchor_slam.py` (4 OK).
- Depends on: TK-SLAM-09 (audited and closed). Independent of TK-INT-12; together they should leave fewer WEAK sweeps.
- Assumptions: `SCORE_WEAK` 0.64 and `LOST_AFTER` 5 come from 10 bad sweeps on two recordings, so they are provisional;
  the status is for consumers (map writer, dashboard, IL, reflex) and no consumer is changed here; the per-point score
  is comparable between rooms (the demo room is not recorded yet).
  Human-approved: 2026-11-20
- Needs-human: none open. (Dashboard display of WEAK/LOST would be a separate INT ticket.)
