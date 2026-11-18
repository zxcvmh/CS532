# TK-SLAM-04 Stop-Then-Scan Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Eliminate scan smear mapping during in-place turns by pausing map updates and scan-to-map matching during spin, then re-anchoring pose on the first stationary sweep via Stage 1 CSM.

**Architecture:** Direction C ("Stop-then-scan"). In `motion_adapter.py`, skip `grid_map.update_scan` and anchor locking when spinning in place ($|\omega_{\text{cmd}}| > 0.05, |v_{\text{cmd}}| < 0.02$). In `lidar_odometry.py`, bypass `match_scan_to_map` and ICP during spin, dead-reckon $\theta$ with zero translation, and do not taint keyframes with smeared scans; on stop, Stage 1 CSM re-anchors heading.

**Tech Stack:** Python 3.11, NumPy, Hector Gauss-Newton SLAM, CSM.

**Spec:** [`docs/research/tk_slam_04_stop_turn_go_design.md`](file:///home/zxcvmh/Projects/CS532/docs/research/tk_slam_04_stop_turn_go_design.md)

## Global Constraints
- `motion_adapter.py` changes must be strictly $\le 10$ lines (Supervisor-approved exception).
- `setup/robot_dynamics.json` is LOCKED (never edit).
- Do not commit; the Supervisor commits after Auditor review.
- Reply to human in Vietnamese.

---

### Task 1: Gated Map Updates in `motion_adapter.py`

**Files:**
- Modify: `dashboard/backend/app/navigation/motion_adapter.py:110-130`
- Test: `tests/acceptance/test_slam_stop_turn_go.py`

**Interfaces:**
- Consumes: `v_cmd`, `w_cmd` in `MotionAdapter.update_odometry`.
- Produces: `grid_map.update_scan` called only when `not is_spinning`.

- [ ] **Step 1: Check baseline failure on map_writes**
  Run: `.venv/bin/python tests/acceptance/test_slam_stop_turn_go.py -v`
  Expected: Fails with `spin=[3, 3, 3, 3]` smeared sweeps written to map.

- [ ] **Step 2: Add `is_spinning` gate in `motion_adapter.py`**
  In `update_odometry`:
  ```python
  is_spinning = (abs(w_cmd) > 0.05 and abs(v_cmd) < 0.02)
  if not is_spinning:
      # Step 2: anchor initialization
      ...
      # Step 3: update_scan
      self.grid_map.update_scan(...)
  ```

- [ ] **Step 3: Verify git diff line count on `motion_adapter.py`**
  Run: `git diff dashboard/backend/app/navigation/motion_adapter.py`
  Expected: $\le 10$ modified lines.

---

### Task 2: Kinematic Dead-Reckoning During Spin in `lidar_odometry.py`

**Files:**
- Modify: `motion/planning/lidar_odometry.py:538-650`
- Test: `tests/acceptance/test_slam_stop_turn_go.py`

**Interfaces:**
- Consumes: `is_pivot`, `prior_dx`, `prior_dy`, `prior_dtheta`.
- Produces: `LidarOdometry.update` skips `match_scan_to_map` during spin, sets `self.last_status = "PIVOT_DEAD_RECKONING"`, and leaves `self.prev_points_2d` intact.

- [ ] **Step 1: Implement kinematic bypass for `is_pivot`**
  In `lidar_odometry.py:update()`:
  If `is_pivot`:
  - `dx_res = 0.0`
  - `dy_res = 0.0`
  - `dth_res = prior_dtheta`
  - `self.theta = (self.theta + dth_res + math.pi) % (2.0 * math.pi) - math.pi`
  - `self.last_status = "PIVOT_DEAD_RECKONING"`
  - `is_reliable = True`
  - `match_info = {"method": "KINEMATIC_PIVOT"}`
  - Do NOT call `match_scan_to_map` or `match_scans`.
  - Do NOT overwrite `self.prev_points_2d` with smeared `curr_cloud`.

- [ ] **Step 2: Verify `test_slam_stop_turn_go.py` passes**
  Run: `.venv/bin/python tests/acceptance/test_slam_stop_turn_go.py -v`
  Expected: Both `anchor=ON` and `anchor=OFF(prod)` PASS.
  Metrics: drift $\le 5.0\text{ cm}$, heading $\le 3.0^\circ$, ghost cells $\le 10$, map_writes spin=[0,0,0,0], stop=[$\ge 1$, $\ge 1$, $\ge 1$, $\ge 1$].

---

### Task 3: Regression Suite & Verification Gates

**Files:**
- Test: `tests/acceptance/test_slam_inplace_turn.py`
- Test: `tests/test_hector_anchor_slam.py`
- Test: `dashboard/backend/app/main.py`

- [ ] **Step 1: Run TK-SLAM-03 acceptance test**
  Run: `.venv/bin/python tests/acceptance/test_slam_inplace_turn.py -v`
  Expected: 4/4 PASS.

- [ ] **Step 2: Run anchor slam unit tests**
  Run: `.venv/bin/python tests/test_hector_anchor_slam.py`
  Expected: 4/4 PASS.

- [ ] **Step 3: Run backend import test**
  Run: `cd dashboard/backend && ../../.venv/bin/python -c "import app.main"`
  Expected: Exit code 0.

- [ ] **Step 4: Check allowed files diff**
  Run: `git diff --stat`
  Expected: Only `motion/planning/lidar_odometry.py` and `dashboard/backend/app/navigation/motion_adapter.py` modified (beyond prior ticket files).

---

### Task 4: Reporting
- [ ] Format final report block strictly matching `AGENTS.md` section 3 (`## REPORT TK-SLAM-04`).
