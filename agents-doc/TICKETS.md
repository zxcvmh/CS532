# TICKETS — CS532 JetBot (v2, from 2026-11-15)

Status flow: `DRAFT` (Supervisor still writing the locked test) → `READY` (Antigravity may start) →
`IN REVIEW` (report submitted, auditor running) → `DONE` (auditor PASS + committed) | `REJECTED`.
Old v1 tickets: `agents-doc/archive/agents-v1/TICKETS-v1.md` (paths there are dead).

## Template (copy for every new ticket)

```
### TK-<AREA>-<NN>: <title>
- Role: researcher | perception | slam | il-policy | integration | supervisor
- Status: DRAFT | READY | IN REVIEW | DONE | REJECTED
- Problem (with evidence, file:line or command output):
- Allowed files:
- Forbidden files: everything else + agents-doc/LOCKED_PATHS.txt
- Acceptance command:
- Threshold (numbers):
- Regression commands:
- Depends on:
```

---

### TK-SLAM-01: Backend cannot start — `arena_bounds` interface mismatch
- Role: slam
- Status: DONE 2026-11-15 — auditor PASS (session "CartographerAdapter arena_bounds parameter").
  Fix landed in `cartographer_adapter.py:41-43,52` but was swept into Supervisor commit d589c4a
  before the audit (process violation, see ledger). Follow-up: drop the catch-all `**kwargs`.
- Problem: `cd dashboard/backend && ../../.venv/bin/python -c "import app.main"` →
  `TypeError: CartographerAdapter.__init__() got an unexpected keyword argument 'arena_bounds'`.
  Caller: `dashboard/backend/app/navigation/motion_adapter.py:22-29`.
  Callee: `dashboard/backend/app/navigation/cartographer_adapter.py:32-42`.
  Introduced by the unfinished Hector/anchor refactor (commits 5719eba, a5879cf).
- Allowed files: `dashboard/backend/app/navigation/cartographer_adapter.py`
- Acceptance command: `cd dashboard/backend && ../../.venv/bin/python -c "import app.main"`
- Threshold: no `arena_bounds` TypeError. Report the NEXT error verbatim if one appears (do not fix it).
- Regression commands: `.venv/bin/python tests/test_hector_anchor_slam.py`;
  `cd dashboard && PYTHONPATH=../motion:backend ../.venv/bin/python test_path_following_and_modular_architecture.py`
  (baseline 2026-11-15: 7 ok, 1 skipped)
- Depends on: —

### TK-IL-01: Runtime policy must pass the crossing-pedestrian acceptance test
- Role: il-policy
- Status: PAUSED 2026-11-15 — sim dynamics are wrong (see TK-IL-04); re-run as regression after TK-IL-06.
  Plan: `docs/plans/2026-11-15-il-roadmap.md`
- Locked test: `tests/acceptance/test_il_sim_success.py` (evaluates exactly what
  `il_onnx_bridge.get_il_policy_runner()` loads, 100 seeds, 1 crossing pedestrian + 0-2 statics).
- Baseline 2026-11-15 (Supervisor run):
  - runtime policy: success **89.0 %**, collision **2.0 %**, timeout 9.0 % — failed seeds
    `[17, 34, 38, 44, 48, 49, 54, 62, 69, 83, 86]`
  - expert (`GeometricExpertPolicy`, the teacher): success **48.0 %**, collision **40.0 %**
  - the neural net drives only **40.2 %** of control steps; the other ~60 % are hand-written rules in
    `imitation_learning/il_policy_runner.py` (terminal approach, yield, escape pivot, bypass latch,
    flank follow, pivot-to-goal).
- Decision (human → Supervisor, 2026-11-15): goal is a robot that runs, so the rules+NN hybrid is
  accepted for now. The NN-driven share of control steps must be REPORTED (baseline 40.2 %) and must
  not drop below 30 %; a later ticket fixes the expert so the NN can take over more.
- Allowed files: `imitation_learning/il_policy_runner.py`
- Hint: start from the failed seeds; run one seed and log the step where it times out or collides.
- Acceptance command: `.venv/bin/python tests/acceptance/test_il_sim_success.py -v`
- Threshold: `test_runtime_policy_avoids_crossing_pedestrians` passes (success ≥ 90 %, collision ≤ 2 %, NN-driven steps ≥ 30 %).
- Depends on: —

### TK-IL-02: IL action caps hardcoded instead of read from robot_dynamics.json
- Role: il-policy
- Status: SUPERSEDED by TK-IL-04 (2026-11-15)
- Problem: `imitation_learning/config.py` hardcodes `MAX_LINEAR_SPEED=0.16`, `MIN_DRIVE_SPEED=0.10`,
  `MAX_ANGULAR_SPEED=0.35`; field file says `min_drive_speed=0.19`, `max_safe_speed_mps=0.14`,
  `turn_time_90_deg_sec=0.3`. The sim deadband therefore does not match the real robot.
  Changing caps requires recollecting data and retraining.
- Acceptance: `test_sim_physics_matches_field_dynamics` in the locked test (currently FAIL: 0.1 != 0.19),
  and `test_runtime_policy_avoids_crossing_pedestrians` must still pass afterwards.
- Depends on: TK-IL-01

### TK-IL-03: `test_il_pipeline.py` fakes PASS and overwrites the runtime model
- Role: il-policy
- Status: DONE 2026-11-16 (human approved closing after audit 1: code PASS, scope REJECT on docs only — the coder
  edited TICKETS.md + created docs/plans/2026-11-15-il-roadmap.md; Supervisor corrected and owns them now).
  Evidence: `Ran 6 tests ... OK`, `test_artifacts/` sha256 unchanged, `git status imitation_learning/` clean of
  artifacts; IL sim regression 89.0 % / 2.0 % unchanged. Follow-up: DAgger success is unseeded
  (16.7 % vs 66.7 % on two runs) → the new `> 0` assert can flake.
- Problem: `imitation_learning/test_il_pipeline.py` exits 0 and prints
  `[PASS] DAgger aggregation verified!` while printing `Final Success Rate: 0.0%`. It writes into
  `imitation_learning/test_artifacts/models/`, which is the FIRST path
  `dashboard/backend/app/navigation/il_onnx_bridge.py:44` loads in production.
- Allowed files: `imitation_learning/test_il_pipeline.py`
- Acceptance: (1) the pipeline test writes only into a temp dir (`tempfile.mkdtemp()`), proven by
  `git status --short imitation_learning/` showing no modified tracked files after a run;
  (2) it no longer prints `[PASS]` for a stage whose printed success rate is 0.
- Regression: `.venv/bin/python tests/acceptance/test_il_sim_success.py -v` numbers unchanged.
- Depends on: —

### TK-CV-01: Two diverged floor IPM implementations
- Role: perception
- Status: DEFERRED 2026-11-16 (human approved the cut: floor IPM / low obstacles are out of the demo scope)
- Problem: `perception/core/floor_ipm.py` ≠ `dashboard/floor_ipm.py` (bridge imports the latter).
- Depends on: —

### TK-CV-02: Camera detections never reach navigation
- Role: perception (+ interface note to integration)
- Status: DRAFT — REWRITTEN 2026-11-16 as the person-aware LiDAR layer (below); the original premise was half wrong: `coordinator.py:272` already feeds person detections into 0.40 m social bubbles.
- Problem: `dashboard/backend/app/navigation/coordinator.py:266-270` only uses detections to set
  `yolo_connected`; people are not added to any costmap layer or IL observation.
- Depends on: TK-SLAM-01

### TK-INT-01: Deploy the single dynamics file to the Jetson
- Role: supervisor (hardware)
- Status: DONE 2026-11-15 — Jetson is `jetbot@192.168.1.18` (`~/.ssh/config` alias still says .16).
  Copied to `/home/jetbot/setup/robot_dynamics.json` (sha256 matches repo). Backups:
  `~/setup/robot_dynamics.json.bak-20261115`, `~/03-MinhHieu/test-hardware/robot_dynamics.json.bak-20261115`.
  All 3 bridges on the Jetson now log `Loaded robot dynamics from /home/jetbot/setup/robot_dynamics.json`.

### TK-INT-03: Jetson runs stale bridges that ignore turn_time_90_deg_sec
- Role: supervisor (deploy) + integration (Python 3.6 check)
- Status: DONE 2026-11-15 — human runs only `~/03-MinhHieu/run_bridge.sh`. Deployed repo
  `dashboard/{jetbot_bridge,command_mux,d500_driver,floor_ipm}.py` + `motion/planning/disparity_extender.py`
  into `~/03-MinhHieu/` (sha256 match, py_compile OK on Python 3.6.9). Backups: `*.bak-20261115`.
  Loader check: `min=0.19 cruise=0.2 trim=1.045 turn=0.19 T90=0.3 maxW=5.236`. Other old bridge copies
  (`~/`, `~/05-d500lib-motion/`, `~/01-HieuMe/`) left untouched — not launched. Not yet run on motors.
- Problem: the Jetson has 5 `jetbot_bridge.py` copies (`~/`, `~/03-MinhHieu/`, `~/03-MinhHieu/backend/`,
  `~/05-d500lib-motion/`, `~/01-HieuMe/`) and an empty `~/dashboard/`. With the new config they compute
  max angular speed as 3.142 / 1.400 / 0.350 rad/s; the repo bridge computes 5.236 from the measured 0.3 s.
- Plan: deploy `dashboard/jetbot_bridge.py` + its imports into `~/dashboard/`, verify it imports under
  Python 3.6.9, then archive the old copies on the Jetson (rename, not delete).
- Problem: bridge now reads only `<repo>/setup/robot_dynamics.json` or
  `/home/jetbot/setup/robot_dynamics.json`; the old `dashboard/test-hardware/` copy is no longer read.
- Acceptance: bridge log line `[BRIDGE] Loaded robot dynamics from .../setup/robot_dynamics.json`.

### TK-INT-02: Legacy tests still point at dead `04-hieu-MotionControl` paths
- Role: integration
- Status: DRAFT
- Problem: `motion/test_*.py` reference `04-hieu-MotionControl/` (no such folder).
- Depends on: TK-SLAM-01

---

## SLAM findings from the TK-SLAM-01 audit (2026-11-15)

Simulated 2×1 m arena, real `MotionAdapter` code, in-place 360° turn (threshold ≤ 5 cm / 3°):
ideal scan @5.24 rad/s 0.0°/1.6 cm ✅ · distorted scan −22.8°/8.5 cm ❌ · distorted + anchor off
(= production) −37.0°/0.5 cm ❌ · ideal scan @0.35 rad/s −10.0°/16.6 cm ❌ · slow + distorted
+4.7°/36.5 cm ❌. Repro scripts (auditor scratchpad, to be turned into a locked test by the Supervisor):
`repro_slam.py`, `repro4.py`.

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

### TK-INT-09: Bridge spin PWM inverted once turn_speed < min_drive_speed (Supervisor hotfix)
- Role: supervisor (hardware)
- Status: DONE in repo, NOT deployed (Jetson offline 2026-11-15; Jetson runs ~/03-MinhHieu/jetbot_bridge.py, not
  the repo bridge — TK-INT-07 is not deployed either)
- Trigger: human wants slower pivots: turn_speed 0.14, turn_time_90_deg_sec 0.54, turn_time_180_deg_sec 1.08
  (human edits setup/robot_dynamics.json; Claude never does).
- Bug: spin PWM = min_drive_speed + (turn_speed - min_drive_speed) × |w|/w_max. With 0.14 < 0.19 a partial
  joystick / slider command (ratio 0.1) spun at 0.187 PWM — FASTER than a full command (0.14) — and at a rate
  nobody calibrated. Only one spin PWM is calibrated (calibrate_motors.py test 4).
- Fix: spin PWM = turn_speed for any |w| ≥ 0.02 (single-rate pivot). Test `tests/test_bridge_spin_step.py`
  (RED 0.187 on the old bridge → GREEN). Regression `dashboard/test_path_following_and_modular_architecture.py` 8 OK.
- Field check (human): after editing robot_dynamics.json and deploying, confirm the robot still starts turning from
  rest at 0.14 PWM (below the straight-line min_drive_speed) and that 90° ≈ 0.54 s.

### TK-INT-08: MotionAdapter.insert_low_obstacles raises TypeError on the cartographer path
- Role: integration
- Status: DRAFT (found by the Supervisor while writing TK-SLAM-08's test, 2026-11-15)
- Evidence: `motion_adapter.py:175-180` calls `self.cartographer.insert_low_obstacles(..., ttl_sec=ttl_sec)`;
  `cartographer_adapter.py:275` has no `ttl_sec` parameter → TypeError whenever the camera sends
  `low_obstacles` (`telemetry_parser.py:316-320`). Whether the Jetson currently sends them is unverified.

### TK-SLAM-05: Production runs anchor OFF while the test runs it ON; test cannot see heading error
- Role: slam
- Status: DEFERRED 2026-11-16 (human approved the cut)
- Problem: `dashboard/backend/app/hardware/telemetry_parser.py:465-466` disables the anchor;
  `tests/test_hector_anchor_slam.py:31-44` uses a circular room (rotation-invariant scan) for 1 step.

### TK-INT-04: Velocity latch in AUTO mode makes the robot icon spin between scans
- Role: integration
- Status: CANCELLED 2026-11-16 (premise was wrong; human approved the cut)
  binary path: `unpack_binary_telemetry` returns `linear_speed`/`angular_speed`, the parser looks for
  `linear_vel`/`angular_vel`. coord velocities come only from the planner command. Re-scope before READY:
  decide whether the backend should use bridge-reported w at all (it is slewed, not physical either).
  Not a blocker for TK-SLAM-04.
- Problem: `dashboard/backend/app/hardware/telemetry_parser.py:406-409` never accepts v=0, so the
  interpolation at `:424-433` keeps rotating with a stale ω, then snaps back (perceived lag/jerk).

### TK-INT-07: Bridge ramps spin PWM 0 -> turn_speed over 0.24 s; calibration used a step
- Role: supervisor (hardware)
- Status: IN PROGRESS — code + test done, not deployed / not field-checked
- Evidence: `setup/calibrate_motors.py:333` measured `turn_time_90_deg_sec` with `drive_raw` (PWM step);
  `dashboard/jetbot_bridge.py` spin branch slewed PWM at 0.8/s (0.19 / 0.8 = 0.24 s). turn_speed ==
  min_drive_speed, so the ramp is below the measured drive threshold; whether the motor moves during it
  is UNVERIFIED. Removing the ramp makes the bridge equal to the calibration by construction.
- Fix: spin PWM applied as a step. Test `tests/test_bridge_spin_step.py` (RED 0.04 PWM -> GREEN 0.19).
- Field check (human): command a 90° turn from the dashboard, confirm ≈ 90°.

### TK-INT-05: Robot veers right — trim applied to the wrong wheel (Supervisor hotfix)
- Role: supervisor (hotfix during first field run, 2026-11-15)
- Status: DONE (needs field confirmation)
- Root cause: `setup/calibrate_motors.py:217-218` measures trim on the RIGHT wheel
  (`r_spd = base_spd * trim`), but `dashboard/jetbot_bridge.py:902` and
  `motion/planning/control/motor_dynamics.py:91` applied it to the LEFT wheel → ~9 % left-over-right
  speed difference with trim 1.045 → constant right drift.
- Fix: trim now scales the right wheel in both files; PWM tests updated (7 ok, 1 skip).
  Deployed to `~/03-MinhHieu/jetbot_bridge.py` (sha256 fe334e3db1d5).
- Also fixed during deploy: Supervisor had copied `motion/planning/disparity_extender.py` to the Jetson;
  the bridge needs `dashboard/disparity_extender.py` (`evaluate(..., cmd_v)`), which caused
  `[ERROR in hardware_reflex_task] evaluate() got an unexpected keyword argument 'cmd_v'` —
  the reflex stop was inactive during that run. Restored (sha256 45c501e6bcc2).

### TK-INT-06: Motion is jerky (unexplained)
- Role: supervisor (needs field log) → integration
- Status: DEFERRED 2026-11-16 (human approved the cut; reopen only with a captured bridge log)
- Candidates (unverified): reflex-task exception every loop (fixed in TK-INT-05),
  0 → min_drive_speed=0.19 PWM jump at start, backend velocity latch (TK-INT-04).

### TK-IL-04: IL sim must use the field dynamics and a stop-turn-go action set
- Role: il-policy
- Status: READY (2026-11-16) — locked test `tests/acceptance/test_il_sim_dynamics.py` FAILS 14/14 on HEAD for the
  ticket's reasons (forward 0.152 m/s vs field 0.294 after deadband + 5 % slip, v = 0.02 -> 0 m/s, w steers a moving
  robot, pivot turns at the commanded 0.5 rad/s instead of the field rate, no ACTION_* / step_action) and PASSES 7/7
  on a Supervisor prototype (scratchpad, not in repo) for both the real file and a synthetic one (0.41 m/s, 0.54 s).
- Problem: `imitation_learning/config.py` hardcodes MIN_DRIVE_SPEED 0.10 / MAX_LINEAR_SPEED 0.16 / MAX_ANGULAR_SPEED
  0.35; `mock_expert_simulator.py` `step()` has a 0.10 deadband, a 5 % slip on top of a field speed that already
  includes slip, and lets |w| steer while moving. The real robot (dashboard/jetbot_bridge.py `set_motor_speeds`,
  TK-INT-09) has ONE forward speed (`actual_speed_mps`), ONE pivot rate ((pi/2)/`turn_time_90_deg_sec`) and no arc
  steering -> a sim success rate says nothing about the robot.
- Contract (the test enforces it): `config.py` reads the dynamics file from env var `ROBOT_DYNAMICS_FILE` if set, else
  `<repo>/setup/robot_dynamics.json`; `ACTION_STOP/FORWARD/PIVOT_LEFT/PIVOT_RIGHT` = 0/1/2/3;
  `MAX_LINEAR_SPEED` = `actual_speed_mps`; `MAX_ANGULAR_SPEED` = (pi/2)/`turn_time_90_deg_sec`;
  `MIN_DRIVE_SPEED` = `min_drive_speed` (names stay importable — model.py, il_policy_runner.py etc. import them);
  `SimulationEnvironment.step(cmd_v, cmd_w, dt)` mirrors the bridge: |v| < 0.01 and |w| < 0.02 -> stop;
  |v| < 0.01 -> pivot at +-MAX_ANGULAR_SPEED by the sign of w, no translation; |v| >= 0.01 -> forward at
  MAX_LINEAR_SPEED, w ignored; new `step_action(action, dt)` maps the 4 actions onto `step`.
  Do not add slip on top of `actual_speed_mps`. Python 3.6 compatible syntax (this config may reach the Jetson).
- Allowed files: `imitation_learning/config.py`, `imitation_learning/mock_expert_simulator.py`
- Forbidden: `tests/`, `setup/robot_dynamics.json`, `imitation_learning/test_il_pipeline.py`, the policy / model / runner
  files, everything else. No dynamics number literal (0.294, 0.3, 5.236, 0.19, ...) in the two files.
- Acceptance command: `.venv/bin/python tests/acceptance/test_il_sim_dynamics.py -v`
- Threshold: 7/7 tests x 2 dynamics files (real, synthetic): forward speed = `actual_speed_mps` +-3 % (also for
  v = 0.02 / 0.05 / 0.10 / 0.30, +-3 %); heading change while driving <= 2 deg/s with w = +-1; pivot rate =
  +-(pi/2)/`turn_time_90_deg_sec` +-3 % for |w| = 0.5 and 3.0 and for the two pivot actions; pivot translation <= 5 cm
  over 2 s; stop creep <= 1 cm and <= 1 deg.
- Regression: `tests/acceptance/test_il_sim_success.py::test_sim_physics_matches_field_dynamics` passes (Supervisor
  removed its `MAX_LINEAR_SPEED <= max_safe_speed_mps` line, which contradicted the field speed);
  `.venv/bin/python imitation_learning/test_il_pipeline.py`: tests 1-4 and 6 stay green.
  **Accepted consequence (decided by the Supervisor, 2026-11-16):** `test_05_dagger_session_execution` goes RED
  (DAgger final success 0.0 %, measured on the prototype; expert 16.7 % vs ~50 % before) because the old continuous
  expert cannot drive stop-turn-go. That is the IL-03 guard doing its job. Do NOT edit that test or its assert to
  make it green; TK-IL-05 (new expert) and TK-IL-06 (retrain) turn it green. State it in the REPORT.
  `test_il_sim_success.py` (TK-IL-01) is PAUSED, its other numbers are not a regression gate until TK-IL-06.
- Decided by the human (2026-11-16): the sim follows `actual_speed_mps` (what the robot physically does, measured at
  cruise_speed 0.2) and IGNORES `max_safe_speed_mps` (0.14). The human re-measured at the chassis instead of the
  camera nose: 0.294 -> 0.286 m/s; pivot at turn_speed 0.14: 90 deg in 0.53 s, 180 deg 1.06 s (human to apply to the file; the test reads
  the file, so no code or test change is needed when it is).
- Depends on: —

### TK-IL-05: Privileged expert ≥ 95 % on the new sim
- Role: il-policy
- Status: DRAFT — locked `tests/acceptance/test_il_expert.py` written after TK-IL-04 is DONE.
- Problem: teacher success 48 %, collision 40 % on the locked seeds.
- Idea: expert may read the pedestrian's true pose and velocity (privileged teacher); student sees LiDAR only.
- Allowed files: `imitation_learning/mock_expert_simulator.py` (expert class only)
- Threshold: expert success ≥ 95 %, collision ≤ 1 % on the 100 locked seeds.
- Depends on: TK-IL-04

### TK-IL-06: Retrain BC → DAgger as a discrete classifier, NN drives ≥ 90 %
- Role: il-policy
- Status: DRAFT — Supervisor raises MIN_NN_SHARE_PCT to 90 in `test_il_sim_success.py` first.
- Allowed files: `imitation_learning/{model.py,train_bc.py,dagger_session.py,il_policy_runner.py}`
  (split into two attempts if > 150 changed lines)
- Acceptance command: `.venv/bin/python tests/acceptance/test_il_sim_success.py -v`
- Threshold: success ≥ 90 %, collision ≤ 2 %, NN-driven steps ≥ 90 %; only an emergency-stop rule may override.
- Depends on: TK-IL-05, TK-IL-03

### TK-IL-07: Sim-to-real (LiDAR noise from real recordings, Jetson latency)
- Role: il-policy + supervisor (hardware)
- Status: DRAFT
- Threshold: Jetson loop ≤ 50 ms, RAM ≤ 1.5 GB; real robot 10/10 goals, no contact.
- Depends on: TK-IL-06, TK-SLAM-09

---

## Week-1 plan tickets (2026-11-16, from the confirmed project plan in STATE.md)

### TK-INT-10: Bridge forward drive must be ONE PWM (cruise_speed), not a v-dependent window
- Role: integration
- Status: READY (2026-11-16) — locked test `tests/acceptance/test_bridge_forward_single_pwm.py` fails on HEAD for the
  ticket's reasons (steady PWM 0.112 / 0.122 / 0.140 for v = 0.02 / 0.05 / 0.10 instead of cruise 0.2; w steers the
  wheels, w = 3 even reverses the left one; after a pivot the forward PWM sticks at 0.14) and a Supervisor reference
  (in memory, not in the repo) passes 6/6, and keeps `tests/test_bridge_spin_step.py` 4/4.
- Problem: `set_motor_speeds` section "3. Linear drive" maps wheel speeds through a window around
  `max_safe_speed_mps` onto PWM [`min_drive_speed`, `cruise_speed`]. With the field file of 2026-11-16
  (min_drive 0.08, max_safe 0.1, cruise 0.2) the robot drives at PWM 0.14-0.175 for most commands, so
  `actual_speed_mps` (0.284, measured at cruise 0.2), the IL sim (TK-IL-04) and every SLAM assumption describe a
  robot that does not exist. Same fix pattern as TK-INT-09 for the pivot.
- Required: stop-turn-go. |v| >= 0.01 and no pivot -> both logical wheel PWMs = sign(v) x `cruise_speed` after the soft
  start, for any |v| and any w (w ignored); soft start = a PWM ramp 0 -> cruise over `soft_start_ramp_sec`
  (non-decreasing, starts below cruise, never above it), which mirrors `setup/calibrate_motors.py`
  `soft_start_drive`; backward symmetric; a pivot resets the forward ramp; stop = 0. Trim stays in
  `set_raw_motor_speeds` (do not apply it here). `min_drive_speed` / `max_safe_speed_mps` must not influence the
  forward PWM. Hint (Supervisor reference, ~6 lines): one module-level magnitude `_fwd_pwm_mag` raised by
  `CRUISE_SPEED / ramp * dt` per call and capped at `CRUISE_SPEED`, zeroed in the full-stop and spin branches.
- Allowed files: `dashboard/jetbot_bridge.py` (only `set_motor_speeds` and the new state variable)
- Forbidden: `tests/`, `setup/robot_dynamics.json`, `dashboard/backend/`, everything else. Python 3.6 compatible.
- Acceptance command: `.venv/bin/python tests/acceptance/test_bridge_forward_single_pwm.py -v`
- Threshold: 6/6 over four dynamics sets (field file, old file, two synthetic): steady PWM = cruise_speed +-1e-6.
- Regression: `.venv/bin/python tests/test_bridge_spin_step.py` (4 OK);
  `.venv/bin/python dashboard/test_path_following_and_modular_architecture.py`; py3.6 `ast.parse(..., feature_version=(3,6))`.
- NOT in this ticket: deployment. The Supervisor diffs against `~/03-MinhHieu/jetbot_bridge.py` and deploys the bridge
  TOGETHER with the new dynamics file once the auditor passes. The human must not copy the new dynamics file to the
  Jetson before that, or its speed becomes command-dependent (0.122-0.2 PWM).
- Depends on: —

### TK-INT-11: Emergency-stop paths are unproven (demo safety)
- Role: integration (Supervisor measures first)
- Status: DRAFT — needs a measurement before any code: (1) dashboard / API `emergency_stop` (laptop -> WiFi -> bridge)
  and (2) `teleop_gamepad.py` button A, which calls `robot.stop()` in its OWN process, while the bridge is running.
  Nothing shows that either pre-empts an active automatic command or how long motors take to reach 0.
- Plan: Supervisor writes a measurement protocol (trigger -> motor PWM 0, 20 trials, p95) and, if the gamepad path
  does not pre-empt the bridge, a locked test for a gamepad -> bridge stop that wins over every other source.
- Threshold (proposal, to confirm after the Jetson measurement): trigger -> PWM 0 p95 <= 300 ms for the gamepad path.
- Depends on: Jetson reachable from the home network.

### TK-CV-03: Camera calibration (intrinsics + distortion)
- Role: perception (human takes the images; no code can start before)
- Status: DRAFT — no intrinsics exist in the repo (`find` for calib/intrinsic found only `setup/calibrate_motors.py`).
- Plan: human photographs a printed checkerboard (about 20 poses, week-1 field session); a script writes
  `setup/camera_intrinsics.json`. Threshold: mean reprojection error <= 0.5 px, image size and focal length stored.
  Needed by TK-CV-05 (tags) and for projecting detections onto LiDAR azimuth.

### TK-CV-04: Person detection measured on the real room and the Jetson
- Role: perception (Supervisor writes the locked evaluation script from the human's labelled frames)
- Status: DRAFT — only a detector exists (`perception/models/yolo26n`, engine in the bridge); there is no measurement.
- Plan: about 200 frames from the real robot and room with a person at 0.5-3 m, labelled by the human.
  Proposed thresholds (to confirm after the first measurement): recall >= 0.90 and precision >= 0.90 for persons in
  the camera's field of view at 0.5-3 m; Jetson detection loop >= 8 FPS with the bridge running; whole-bridge RAM
  <= 1.5 GB. Depends on: TK-CV-03, Jetson measurement.

### TK-CV-02 (rewritten): Person-aware LiDAR layer — label clusters, mask person beams out of scan matching
- Role: perception + slam (split into two tickets when it starts: labelling, then CSM masking)
- Status: DRAFT (see the Status line above) — needs a real recording with a person walking (TK-CV-04 session).
- Required behaviour: a LiDAR cluster whose azimuth falls inside a detection box is a PERSON (else static/unknown);
  persons keep the 0.40 m bubble; the beams of persons are removed before `CSMTracker.update` so a crossing person
  does not pull the pose; IL observations are NOT changed (stay 41-D, no retrain).
- Measurement for the report (ablation): pose error with a person walking, masking OFF vs ON, same recording.
- Depends on: TK-SLAM-09, TK-CV-04, a recording with people.

### TK-CV-05: AprilTag / ArUco drift anchor and optional goal (should-have)
- Role: perception
- Status: DRAFT — no tag code in the repo; `cv2.aruco` availability on the Jetson (OpenCV from JetPack) is unchecked.
- Plan: one tag on a stand in the room; pose of the tag in the map frame corrects accumulated drift when seen.
  Threshold: tag position error <= 5 cm vs a ruler at 1-2 m; drift after 5 minutes of driving with vs without the anchor.
  Cut first if week-2 field time runs short. Depends on: TK-CV-03.

### TK-CV-06: People counter overlay (only after the IL gate of 2026-12-07)
- Role: perception
- Status: DRAFT / OPTIONAL — IDs on top of the person detector, count shown on the dashboard. Dropped if IL is late.

### TK-MEAS-01: Measure the network, the Jetson budget and the LiDAR with people (decides "real time" and where the loop runs)
- Role: supervisor-assistant (laptop side only) + Supervisor / human for everything that touches the Jetson or the room
- Status: READY (2026-11-16) — locked validator `tests/acceptance/test_measurements_valid.py` is red on HEAD (no
  `docs/measurements/<date>/`), green on synthetic data (proved, then deleted).
- Problem: no number exists for laptop<->Jetson latency, Jetson RAM/CPU headroom, or how a walking person looks in the
  LiDAR scan. Without them the control-loop location and the CV mask (TK-CV-02) cannot be decided.
- Division of work (Rule 8: the assistant never SSHes to the Jetson, never starts motors):
  1. Assistant writes `scripts/measure_summary.py`: reads the raw files in the date directory and writes `results.json`
     (schema in the validator docstring). It computes; it never types a number.
  2. M1 ping: Supervisor/human runs `ping -c 200 <jetson> | tee docs/measurements/<date>/ping_home_wifi.txt` (home WiFi).
  3. M2 inventory (read-only): Supervisor runs `free -m`, `uptime`, `python3 -c "import cv2;print(cv2.__version__,hasattr(cv2,'aruco'))"`,
     `ls` of the engine/model dirs, `ps` of the bridge; output saved raw as `jetson_inventory.txt`. No kill, no install.
  4. M4 LiDAR with people: human stands/walks, robot stationary, 3 x 60 s: `static_person`, `crossing`, `head_on`.
     Recorder = whatever produced `tests/acceptance/data/field_turns_2026-11-15.npz` (keys `t`, `ranges` (n,360));
     if none is in the repo the assistant writes `scripts/record_lidar.py` that flushes every 5 s.
  5. Assistant writes `scripts/analyze_lidar_people.py` (reads the three .npz): per recording, number of person-like
     clusters per scan, their speed estimate and range, wall-point density; prints raw numbers into `analysis.txt`.
     It makes NO claim about SLAM pose (that is TK-CV-02's ablation).
- Allowed files: `scripts/measure_summary.py`, `scripts/analyze_lidar_people.py`, `scripts/record_lidar.py` (only if
  no recorder exists), `docs/measurements/**`.
- Forbidden files: everything else + agents-doc/LOCKED_PATHS.txt (tests/, setup/robot_dynamics.json, dashboard/, motion/).
- Acceptance command: `.venv/bin/python tests/acceptance/test_measurements_valid.py -v`
- Threshold: 3/3 tests pass on the real date directory; ping >= 200 packets; each recording >= 55 s; every figure in
  results.json recomputes from its raw file within 1 %. Anything not measured is written `CHƯA ĐO`, never estimated.
- Decision rule (Supervisor applies, not the assistant): ping p99 < 30 ms and loss < 1 % -> loop may run on the laptop;
  otherwise reflex + local policy run on the Jetson.
- Regression commands: `.venv/bin/python tests/acceptance/test_slam_csm_turns.py -v` (unchanged: no source file is touched).
- Out of scope: YOLO FPS on the Jetson (needs the detector chosen; TK-CV-04), motors, deployment, any threshold change.
- Depends on: the human on the home WiFi (not 172.30.x) for M1/M2; a free 2 h field slot for M4.
