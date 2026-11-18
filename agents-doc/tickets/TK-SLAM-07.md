### TK-SLAM-07: Heading after a turn must come from the LiDAR, not from how long the button was held
- Role: slam
- Status: SUPERSEDED by TK-SLAM-09 (2026-11-15). Attempt 2 code (unaudited, uncommitted) stays in the working
  tree as TK-SLAM-09's starting point; its locked tests remain regressions. Was: READY — attempt 2.
- Attempt 1 rejection (reproduced by the Supervisor; HEAD 3f6870a vs attempt 1, pivot ×0.9 then an
  arc with no still sweep between):
  * The pivot exit only cleared when the robot was still, or translating with inter-sweep rotation ≤ 5°.
    In an arc the robot really turns > 5°/sweep (w = 1.0 rad/s at 10 Hz = 5.7°), so every arc sweep stayed
    in the pivot branch: heading dead-reckoned at copysign(5.24 rad/s, w_cmd), dx = dy = 0, no map write.
    v=0.14 w=1.0/1.5/2.0 → 10–11 cm, 157–180° during the arc, 0/8 map writes; v=0.294 w=1.0 → 23 cm, 179°.
    HEAD: ≤ 2.4 cm, ≤ 5.6°, 8/8 writes. MANUAL mode allows this (v ≤ 0.14, |w| up to 5.24 rad/s).
  * Even w = 0.5 (2.9°/sweep, below the 5° rule) gives 35.9° heading error on the first arc sweep.
  * Fix direction: the first sweep with forward command after a pivot ENDS the pivot: re-anchor once
    (full-circle search vs the last still keyframe), then hand back to Hector. Remove the ≤ 5° step-rotation
    condition. Never apply the 5.24 rad/s pivot rate to a sweep with |v_cmd| ≥ 0.02 (the bridge already
    leaves the spin branch as soon as v > 0.01). Name the magic numbers (5.0°, 0.05/6.0 m, 200, 360).
- Field evidence (backend telemetry recorded by the Supervisor, 4 manual left turns, robot + LiDAR correct):
  real turns +60 / +28 / +97 / +32 deg (LiDAR sweep alignment); SLAM after each stop +110 / +95 /
  +152 / +116 deg, then drifts a further 30–50 deg while standing still → map rotates with the robot.
  Cause: TK-SLAM-04 dead-reckons 5.24 rad/s × (time w_cmd != 0). The motor starts late and coasts, so
  command time ≠ motion; the error (50–85 deg) is far outside Hector's capture range.
- Required behaviour (human, 2026-11-15: "map phải nhờ vào LiDAR, không nhờ vào tôi nhấn"):
  1. Decide "robot is still" from the LiDAR (consecutive sweeps agree), not only from w_cmd. Never write
     a sweep to the map unless the robot is still by that test.
  2. After motion, re-anchor heading by a FULL-CIRCLE (±180°) rotational search of the first still sweep
     against the last still keyframe, then refine with the existing Hector match. w_cmd may be used as a
     hint (sign) but the answer must not depend on its timing or magnitude.
  3. Must stay fast: median update ≤ 15 ms on the laptop (Jetson loop ≤ 50 ms) — use an O(N log N)
     circular correlation (FFT over the 360 bins) or equivalent, not a Python loop over 360 shifts.
- Supervisor feasibility check (prototype in scratchpad, NOT in the repo): scan-to-scan stillness test +
  full-circle re-anchor reached max 3.0° / p90 2.2° and 382–453 occupied cells (oracle 466) on all 6
  cases; it was too slow (Python loop), so speed is the coder's real work.
- Locked test: `tests/acceptance/test_slam_field_turns.py` + `tests/acceptance/data/field_turns_2026-11-15.npz`
  (replay of the real sweeps; 3 command timings: recorded / widened ±3 sweeps / leads by 3 sweeps; anchor
  ON and production).
- Baseline 2026-11-15 (HEAD 869509b): FAILED 6/6 — heading error max 136–162°, median 48–72°;
  occupied 1.97–2.47 × oracle; median 3.8 ms.
- Allowed files: `motion/planning/lidar_odometry.py`; `dashboard/backend/app/navigation/motion_adapter.py`
  (map-write gate only, ≤ 15 changed lines).
- Forbidden: `tests/`, `setup/robot_dynamics.json`, `dashboard/jetbot_bridge.py`, everything else.
- Acceptance commands (BOTH required):
  `.venv/bin/python tests/acceptance/test_slam_field_turns.py -v`
  `.venv/bin/python tests/acceptance/test_slam_pivot_then_arc.py -v`  (added attempt 2; HEAD 7/7, attempt 1 0/7)
- Threshold: field_turns 6/6 — max heading error ≤ 4.0° and p90 ≤ 3.0° at still sweeps; occupied cells ≤
  1.25 × oracle; median update ≤ 15 ms. pivot_then_arc 7/7 — during the arc pos ≤ 5 cm, heading ≤ 8°, map
  written at least once; after the stop pos ≤ 5 cm, heading ≤ 3°.
- Report must paste the RAW output of every acceptance and regression command (attempt 1 omitted it).
- Regression commands: `.venv/bin/python tests/acceptance/test_slam_stop_turn_go.py -v` (2/2);
  `.venv/bin/python tests/acceptance/test_slam_inplace_turn.py -v` (2/2);
  `.venv/bin/python tests/test_hector_anchor_slam.py`; `cd dashboard/backend && ../../.venv/bin/python -c "import app.main"`;
  Python 3.6 syntax (`ast.parse(..., feature_version=(3,6))`) on both files.
- Depends on: TK-SLAM-04 (DONE 0ce172e).
