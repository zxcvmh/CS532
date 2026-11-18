### TK-SLAM-03: Map follows the robot during in-place turns — three different lever arms in SLAM
- Role: slam
- Status: CLOSED — auditor PASS 2026-11-15 (acceptance 4/4, 1.6 cm / 0.0° / 0 ghost; hector 4 OK;
  path_following 7 OK 1 skip; backend boots). Report omitted the regression section (auditor ran it).
  Contour test_01 still fails and is NOT caused by the lever arm: the 0.20 m probe cell is the cell next to
  the 0.28 m hit, which the raycaster skips by design — fails identically before and after. Supervisor's
  "should pass" note below was wrong. Stress (sim): noise 1–2 cm, wheel slip 0.7–1.3×, drive+turn routes,
  person crossing → all ≤ 5 cm / 3°. Still FAILS with intra-sweep smear at 5.24 rad/s (29° / 103 ghost
  cells) → TK-SLAM-04 is the next blocker for the real robot. Non-blocking: lever-arm json loader is
  copy-pasted in 3 files (+config.py); `* 1.25` yaw margin is unnamed.
- Hypothesis (ONE): the matcher, the map raycaster and the anchor layer each place the LiDAR at a
  different distance from the axle, so during a pivot the scan used for matching and the map it is
  matched against disagree by a vector that rotates with heading, and Hector "fixes" that by moving
  the robot. Evidence:
  - `motion/planning/lidar_odometry.py:69,181` builds the matching cloud at axle **+0.08 m**
    (`:603,613` hardcode 0.08 again for the gates).
  - `motion/planning/occupancy_grid.py:73` builds the raycaster with **`lever_arm=0.0`** (map drawn as
    if the LiDAR sat on the axle); the raycaster's own default `slam/bayesian_raycaster.py:18` is 0.08.
  - `motion/planning/occupancy_grid.py:488-494` calls `anchor.set_anchor_from_scan` without a lever
    arm, so `slam/anchor_layer.py:44` uses its default **0.08**.
  - Measured value (human, TK-SLAM-02): `lidar_lever_arm_m = 0.01` in `setup/robot_dynamics.json`.
  - Supervisor experiment (runtime patch only, no source change): setting the matcher and raycaster
    to 0.01 m takes all 4 locked scenarios from 10.6–16.3 cm drift / 2–160 ghost cells to
    1.6 cm / 0.0° / 0 ghost cells, WITHOUT touching any gate or the raycaster update rule.
- Also fix in the same file (same physics-from-json rule): `lidar_odometry.py:117-119` overwrites the
  json with 0.50 s / 3.1416 rad/s / 5.0. Derive the pivot yaw rate from `turn_time_90_deg_sec`.
- Allowed files (4, each a few lines — accepted because the bug is the disagreement between them):
  `motion/planning/lidar_odometry.py`, `motion/planning/occupancy_grid.py`,
  `motion/planning/slam/bayesian_raycaster.py`, `motion/planning/slam/anchor_layer.py`
- Forbidden files: everything else + agents-doc/LOCKED_PATHS.txt. In particular
  `dashboard/backend/app/navigation/motion_adapter.py` (Integration) and `tests/acceptance/`.
- Rules for this fix:
  - One source: read `lidar_lever_arm_m` and `turn_time_90_deg_sec` from `setup/robot_dynamics.json`.
    No numeric fallback that repeats a json value (no `.get("lidar_lever_arm_m", 0.01)`, no `= 0.08`
    defaults left behind). Missing key = raise with a clear message.
  - Do NOT tighten the pivot gates (`:614-620`, `:650-654`) and do NOT add an `is_rotating` map freeze in
    this ticket. That is a separate hypothesis (TK-SLAM-04) and would hide this bug instead of fixing it.
- Acceptance command: `.venv/bin/python tests/acceptance/test_slam_inplace_turn.py -v`
- Threshold (numbers, all 4 tests): drift after 360° ≤ 5 cm AND ≤ 3°; ghost wall cells ≤ 10;
  median SLAM time ≤ 15 ms/scan. Baseline 2026-11-15: 10.7 / 15.0 / 16.3 / 10.6 cm → FAILED (failures=4).
- Regression commands:
  - `.venv/bin/python tests/test_hector_anchor_slam.py` (baseline: 4 OK)
  - `cd dashboard && PYTHONPATH=../motion:backend ../.venv/bin/python test_path_following_and_modular_architecture.py` (baseline: 7 OK, 1 skipped)
  - `cd dashboard && PYTHONPATH=../motion:backend ../.venv/bin/python test_slam_lidar_contour_restoration.py`
    (baseline: test_01 FAIL `100 != 0 : Space between robot and box must be cleared` — report its result; it should
    pass after this fix, but it is not the acceptance gate)
  - `cd dashboard/backend && ../../.venv/bin/python -c "import app.main"` (must still boot)
- Rejected design (docs/research/tk_slam_03_in_place_drift_design.md, 2026-11-15): its call trace hops 1 and 5
  point at the wrong code (`coordinator.py:465` is the simulator loop, `:512` is the health log; real scans
  enter through `telemetry_parser.py` `apply_binary_telemetry` → `motion_adapter.update_odometry`); it missed
  `occupancy_grid.py:73` (`lever_arm=0.0`); its 7 mm gate would keep rejecting Hector corrections (the
  lock-in found in the audit); its "wall drift ≤ 2 cm" target is below what a 5 cm grid can measure.
- Depends on: TK-SLAM-02 (DONE)
