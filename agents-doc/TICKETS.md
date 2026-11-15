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
- Status: READY
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

### TK-IL-01: IL test reports PASS with 0 % success, and overwrites the deployed model
- Role: il-policy
- Status: DRAFT (Supervisor writes `tests/acceptance/test_il_sim_success.py` first)
- Problem: `imitation_learning/test_il_pipeline.py` exits 0 and prints
  `[PASS] DAgger aggregation verified!` while printing `Final Success Rate: 0.0%`,
  `Final Collision Rate: 16.7%`. It writes into `imitation_learning/test_artifacts/models/`, which
  is exactly where `il_onnx_bridge.py:45` loads the runtime model from.
- Allowed files: decided when READY.
- Threshold: 100 episodes, fixed seeds, crossing pedestrians: success ≥ 90 %, collision ≤ 2 %;
  expert success reported alongside.
- Depends on: —

### TK-IL-02: IL action caps hardcoded instead of read from robot_dynamics.json
- Role: il-policy
- Status: DRAFT
- Problem: `imitation_learning/config.py` hardcodes `MAX_LINEAR_SPEED=0.16`, `MIN_DRIVE_SPEED=0.10`,
  `MAX_ANGULAR_SPEED=0.35`; field file says `min_drive_speed=0.19`, `max_safe_speed_mps=0.14`,
  `turn_time_90_deg_sec=0.3`. Changing caps requires retraining.
- Depends on: TK-IL-01

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
- Status: READY — needs Jetson host/user from the human
- Problem: bridge now reads only `<repo>/setup/robot_dynamics.json` or
  `/home/jetbot/setup/robot_dynamics.json`; the old `dashboard/test-hardware/` copy is no longer read.
- Acceptance: bridge log line `[BRIDGE] Loaded robot dynamics from .../setup/robot_dynamics.json`.

### TK-INT-02: Legacy tests still point at dead `04-hieu-MotionControl` paths
- Role: integration
- Status: DRAFT
- Problem: `motion/test_*.py` reference `04-hieu-MotionControl/` (no such folder).
- Depends on: TK-SLAM-01
