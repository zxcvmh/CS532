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
- Status: READY
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
- Status: DRAFT
- Problem: `imitation_learning/config.py` hardcodes `MAX_LINEAR_SPEED=0.16`, `MIN_DRIVE_SPEED=0.10`,
  `MAX_ANGULAR_SPEED=0.35`; field file says `min_drive_speed=0.19`, `max_safe_speed_mps=0.14`,
  `turn_time_90_deg_sec=0.3`. The sim deadband therefore does not match the real robot.
  Changing caps requires recollecting data and retraining.
- Acceptance: `test_sim_physics_matches_field_dynamics` in the locked test (currently FAIL: 0.1 != 0.19),
  and `test_runtime_policy_avoids_crossing_pedestrians` must still pass afterwards.
- Depends on: TK-IL-01

### TK-IL-03: `test_il_pipeline.py` fakes PASS and overwrites the runtime model
- Role: il-policy
- Status: READY
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
- Status: DRAFT
- Problem: `perception/core/floor_ipm.py` ≠ `dashboard/floor_ipm.py` (bridge imports the latter).
- Depends on: —

### TK-CV-02: Camera detections never reach navigation
- Role: perception (+ interface note to integration)
- Status: DRAFT
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

### TK-SLAM-02: LiDAR points and clusters ignore the lever arm
- Role: integration (both files are in `dashboard/backend/`)
- Status: DONE 2026-11-15 — auditor PASS. Lever arm = 0.01 m (human-measured, `lidar_lever_arm_m`).
  Contour test_02 passes; dashboard points and cluster poses agree to 1e-7. Ran while DRAFT and touched
  `config.py` (accepted 1-line scope extension). Follow-ups for the next integration ticket:
  `motion_adapter.py:205` use `Config.LIDAR_LEVER_ARM_M` (drop the getattr default);
  `telemetry_parser.py:88` docstring still says LiDAR is at the robot origin.
- Problem: `dashboard/backend/app/hardware/telemetry_parser.py:92-93` (`ranges_to_points`) and
  `dashboard/backend/app/navigation/motion_adapter.py:205` (`la = 0.0`) drop the 0.08 m axle→LiDAR
  offset that the raycaster and matcher apply → dashboard points/person boxes sit 8 cm off and
  the error rotates with heading, even with perfect SLAM.

### TK-SLAM-03: Map follows the robot during in-place turns (self-reinforcing drift)
- Role: slam
- Status: DRAFT (locked test pending)
- Problem: `motion/planning/lidar_odometry.py:614-617` lets the Hector stage translate up to 6 cm per
  scan while pivoting (ICP stage clamps 2 mm at `:689-694`); `motion/planning/slam/bayesian_raycaster.py:131-132`
  writes hits at full weight immediately, so the map shifts with the wrong pose; the gate at `:620`
  then rejects corrections > 6 cm. `occupancy_grid.update_scan(is_rotating=...)` is unused.
  Also `lidar_odometry.py:117-119` hardcodes 0.50 s / 3.1416 rad/s / 5.0 (fix together).
- Lever arm (added 2026-11-15): the human measured axle-centre → LiDAR-centre = **0.01 m**, stored as
  `lidar_lever_arm_m` in `setup/robot_dynamics.json` and exposed as `Config.LIDAR_LEVER_ARM_M`. SLAM still
  hardcodes other values: `motion/planning/slam/bayesian_raycaster.py:18` (0.08),
  `motion/planning/slam/anchor_layer.py:44` (0.08), `motion/planning/lidar_odometry.py:69,603,613` (0.08),
  `motion/planning/occupancy_grid.py:73` (0.0). All must read the json value.
  Evidence: `dashboard/test_slam_lidar_contour_restoration.py` test_01 fails
  (`Space between robot and box must be cleared navigable free space`) — must pass after the fix.

### TK-SLAM-04: No scan deskew while rotating
- Role: slam
- Status: DRAFT
- Problem: D500 at 10 Hz + 5.24 rad/s turn = ~30° smear inside one scan. Options: deskew with the
  commanded ω, or cap ω while mapping. Needs a decision after TK-SLAM-03.

### TK-SLAM-05: Production runs anchor OFF while the test runs it ON; test cannot see heading error
- Role: slam
- Status: DRAFT
- Problem: `dashboard/backend/app/hardware/telemetry_parser.py:465-466` disables the anchor;
  `tests/test_hector_anchor_slam.py:31-44` uses a circular room (rotation-invariant scan) for 1 step.

### TK-INT-04: Velocity latch in AUTO mode makes the robot icon spin between scans
- Role: integration
- Status: DRAFT
- Problem: `dashboard/backend/app/hardware/telemetry_parser.py:406-409` never accepts v=0, so the
  interpolation at `:424-433` keeps rotating with a stale ω, then snaps back (perceived lag/jerk).

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
- Status: DRAFT — waiting for a captured bridge log with `[MOTOR RAW]` lines after TK-INT-05
- Candidates (unverified): reflex-task exception every loop (fixed in TK-INT-05),
  0 → min_drive_speed=0.19 PWM jump at start, backend velocity latch (TK-INT-04).
