# TK-SLAM-03: In-Place Pivot Drift Elimination Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Eliminate self-reinforcing SLAM map and pose drift during robot in-place pivot turns by locking translational degrees of freedom to $\le 5\text{ mm}$ and pausing static wall log-odds overwrites while rotating.

**Architecture:** Enforce non-holonomic differential drive invariants during pivot ($v = 0 \implies \Delta p \approx 0$) in Tier 1 Hector SLAM odometry; plumb `is_rotating` through `OccupancyGridMap` into `BayesianRaycaster` to prevent intra-scan angular smear from corrupting the static map; read calibrated turn dynamics from `setup/robot_dynamics.json`.

**Tech Stack:** Python 3.11 (host) / Python 3.6 compatible backend, NumPy, SciPy, unittest.

**Spec:** [tk_slam_03_in_place_drift_design.md](file:///home/zxcvmh/Projects/CS532/docs/research/tk_slam_03_in_place_drift_design.md)

## Global Constraints
- Preserve robot dynamics single source of truth: read `turn_time_90_deg_sec` and `lidar_lever_arm_m` from `setup/robot_dynamics.json` (LOCKED).
- Python 3.6 compatibility for files imported by bridge: no dataclasses, no walrus operator, no postponed annotations.
- Zero weakening of existing tests: `tests/acceptance/` and `tests/test_hector_anchor_slam.py` must stay 100% passing.
- Changes must be bounded strictly to `motion/planning/` and `dashboard/backend/app/navigation/motion_adapter.py`.

## Review Focus
1. Pure in-place pivot ($v = 0, \omega > 0$): axle center $(x, y)$ must stay within $\le 0.01\text{ m}$ of origin.
2. Rapid turn ($5.24\text{ rad/s}$): heading $\theta$ must track CCW turn without flipping sign or accumulating angle lag $> 5^\circ$.
3. Static walls during pivot: anchor wall positions and log-odds must not shift or diffuse across a 360° turn.
4. Transition from pivot to straight motion ($v > 0$): translation lock must release immediately when $v_{\text{cmd}} > 0.02\text{ m/s}$.
5. Low-speed rotation while creeping forward ($v = 0.05, \omega = 0.2$): normal SLAM scan-to-map translation must remain active.

---

### Task 1: Reproduce Pivot Drift with a Deterministic Acceptance Test

**Files:**
- Create: `tests/test_hector_pivot_drift.py`
- Test: `tests/test_hector_pivot_drift.py`

**Interfaces:**
- Consumes: `app.navigation.motion_adapter.MotionAdapter`
- Produces: Failing test proving the 6 cm Hector translation jump and map shift during pivot.

- [ ] **Step 1: Write the failing acceptance test in `tests/test_hector_pivot_drift.py`**

```python
import sys
import os
import unittest
import math
import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "dashboard", "backend")))
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "motion")))

from app.navigation.motion_adapter import MotionAdapter

def generate_rectangular_arena_scan(rx: float, ry: float, rth: float, x_min=-0.5, x_max=1.5, y_min=-0.5, y_max=0.5):
    """Simulate exact 360-degree ray range intersections with a 2m x 1m rectangle."""
    ranges = []
    for deg in range(360):
        rad = math.radians(deg) + rth
        c = math.cos(rad)
        s = math.sin(rad)
        t_candidates = []
        if abs(c) > 1e-6:
            t1 = (x_max - rx) / c
            t2 = (x_min - rx) / c
            if t1 > 0: t_candidates.append(t1)
            if t2 > 0: t_candidates.append(t2)
        if abs(s) > 1e-6:
            t3 = (y_max - ry) / s
            t4 = (y_min - ry) / s
            if t3 > 0: t_candidates.append(t3)
            if t4 > 0: t_candidates.append(t4)
        valid_t = [t for t in t_candidates if (x_min - 0.01 <= rx + t * c <= x_max + 0.01) and (y_min - 0.01 <= ry + t * s <= y_max + 0.01)]
        ranges.append(min(valid_t) if valid_t else 2.5)
    return ranges

class TestHectorPivotDrift(unittest.TestCase):
    def test_pivot_turn_translation_lock_and_map_stability(self):
        adapter = MotionAdapter()
        # Initialize map at origin
        initial_scan = generate_rectangular_arena_scan(0.0, 0.0, 0.0)
        for _ in range(6):
            ox, oy, oth, _ = adapter.update_odometry(initial_scan, dt=0.1, v_cmd=0.0, w_cmd=0.0)
        
        # Snapshot initial occupied wall cell count
        walls_init = np.count_nonzero(adapter.grid_map.log_odds > 0.62)
        self.assertGreaterEqual(walls_init, 15)

        # Simulate 90-degree CCW pivot turn at 0.5 rad/s (10 steps of dt=0.18s)
        curr_th = 0.0
        w_cmd = 0.87
        dt = 0.18
        for step in range(5):
            curr_th += w_cmd * dt
            scan = generate_rectangular_arena_scan(0.0, 0.0, curr_th)
            ox, oy, oth, info = adapter.update_odometry(scan, dt=dt, v_cmd=0.0, w_cmd=w_cmd)

        # Axle center translation MUST be strictly clamped <= 0.01m (1 cm)
        trans_drift = math.hypot(ox, oy)
        self.assertLessEqual(trans_drift, 0.01, f"Axle drifted {trans_drift*100:.1f} cm during pure pivot (threshold <= 1.0 cm)!")
```

- [ ] **Step 2: Run test to verify it fails on baseline code**

Run: `.venv/bin/python -m unittest tests/test_hector_pivot_drift.py -v`  
Expected: FAIL (translation drift $> 0.01\text{ m}$).

---

### Task 2: Implement Strict Kinematic Pivot Lock in `lidar_odometry.py`

**Files:**
- Modify: `motion/planning/lidar_odometry.py:114-126, 612-628`
- Test: `tests/test_hector_pivot_drift.py`

**Interfaces:**
- Consumes: `setup/robot_dynamics.json` keys `turn_time_90_deg_sec` and `lidar_lever_arm_m`.
- Produces: `Lidar2DOdometry.update()` with bounded pivot translation $\le 5\text{ mm}$.

- [ ] **Step 1: Update `load_dynamics_config` in `motion/planning/lidar_odometry.py`**

Replace lines 117-124:
Read `turn_time_90_deg_sec` from `cfg` (fallback 0.30s). Compute:
`self.calibrated_pivot_yaw_rate = (math.pi / 2.0) / self.turn_time_90_deg_sec`
`self.max_physical_yaw_rate = self.calibrated_pivot_yaw_rate * 1.25`
`self.lever_arm_x = float(cfg.get("lidar_lever_arm_m", 0.01))`

- [ ] **Step 2: Update Tier 1 Hector SLAM Pivot translation bound in `lidar_odometry.py`**

Replace lines 612-617:
```python
lever_arm = getattr(self, "lever_arm_x", 0.01)
if is_pivot or abs(v_cmd) < 0.02:
    max_allowed_p = max(0.003, lever_arm * abs(delta_th) + 0.002)
else:
    max_allowed_p = max(0.08, self.max_physical_speed * dt * 2.2)
```

- [ ] **Step 3: Clamp Tier 1 output translation during pivot**

Right after line 624:
```python
if is_pivot or abs(v_cmd) < 0.02:
    step_gn_p = math.hypot(gn_x - self.x, gn_y - self.y)
    max_p = max(0.002, abs(v_cmd) * dt + 0.001)
    if step_gn_p > max_p:
        scale_p = max_p / step_gn_p
        gn_x = self.x + (gn_x - self.x) * scale_p
        gn_y = self.y + (gn_y - self.y) * scale_p
```

- [ ] **Step 4: Verify `test_hector_pivot_drift.py` passes translation check**

Run: `.venv/bin/python -m unittest tests/test_hector_pivot_drift.py -v`

---

### Task 3: Plumb `is_rotating` and Freeze Static Map Updating in `BayesianRaycaster`

**Files:**
- Modify: `motion/planning/slam/bayesian_raycaster.py:28-48, 126-138`
- Modify: `motion/planning/occupancy_grid.py:320-338`
- Modify: `dashboard/backend/app/navigation/motion_adapter.py:126-135`
- Test: `tests/test_hector_pivot_drift.py`

**Interfaces:**
- Consumes: `is_rotating` flag from `MotionAdapter.update_odometry()`.
- Produces: `BayesianRaycaster.update_scan(is_rotating=...)` where static log-odds overwrite is skipped during rotation.

- [ ] **Step 1: Add `is_rotating: bool = False` to `BayesianRaycaster.update_scan`**

In `motion/planning/slam/bayesian_raycaster.py:28`:
Add parameter `is_rotating: bool = False`.
Around line 130:
```python
# Only write static wall log-odds when NOT in fast pure rotation
if not is_rotating and len(unique_occ) > 0:
    flat[unique_occ] = np.maximum(flat[unique_occ] + self.grid.l_occ, self.grid.l_occ)
```

- [ ] **Step 2: Pass `is_rotating` from `OccupancyGridMap.update_scan`**

In `motion/planning/occupancy_grid.py:328-337`:
Pass `is_rotating=is_rotating` to `self.raycaster.update_scan(...)`.

- [ ] **Step 3: Compute and pass `is_rotating` from `MotionAdapter.update_odometry`**

In `dashboard/backend/app/navigation/motion_adapter.py:126-128`:
```python
is_rot = (abs(w_cmd) > 0.05 or abs(oth - getattr(self, "_prev_oth", oth)) > 0.02) and abs(v_cmd) < 0.02
self._prev_oth = oth
self.grid_map.update_scan(
    ox, oy, oth, ranges=ranges, max_valid_range=min(6.0, Config.MAX_RANGE), defer_costmap=False, is_rotating=is_rot
)
```

- [ ] **Step 4: Run full acceptance and regression tests**

Run:
1. `.venv/bin/python -m unittest tests/test_hector_pivot_drift.py -v`
2. `.venv/bin/python tests/test_hector_anchor_slam.py`
3. `cd dashboard && PYTHONPATH=../motion:backend ../.venv/bin/python test_path_following_and_modular_architecture.py`

Expected: ALL PASS.
