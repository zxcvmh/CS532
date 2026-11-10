# SLAM Anchor Wall Locking & Low-Latency Telemetry Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Restore the proven Dual-Layer Static Anchor Field and Hector Scan-to-Map (Gauss-Newton with 8cm lever-arm constraint) from commit `08b15d5`, lock stationary room walls in World Frame to eliminate map/ray rotation during in-place turns, and optimize the telemetry-to-dashboard pipeline to sub-30ms zero-copy Base64 streaming while preserving the project's professional modular structure.

**Architecture:** 
1. **Odometry & Mapping Layer:** In `MotionAdapter`, wire `Lidar2DOdometry` (Hector SLAM Gauss-Newton scan-to-map with differential lever-arm model) and `OccupancyGridMap` (`AnchorLayer`). After the first 5 scans, physical room walls are locked into `static_anchor_walls` so they cannot be eroded by ray clearing or rotated.
2. **Real-time Scan Matching:** When the robot rotates, Hector Scan-to-Map optimizes $(x, y, \theta)$ directly against the static anchor walls, ensuring the room remains 100% stationary and only the robot pose moves.
3. **Low-Latency Telemetry & Zero-Copy Pipeline:** Replace 10,000-integer Python list allocation with zero-copy Base64 int8 byte streaming; fix timestamp drift between JetBot and laptop in `jetbot_bridge.py` and `scan_gate.py` to eliminate phantom 1.4s latency spikes.

**Tech Stack:** Python 3.8–3.11, NumPy, SciPy cKDTree, FastAPI/Uvicorn, WebSockets, React 18, TypeScript, HTML5 Canvas 2D.

**Spec:** User decisions from `/grill-me` Round 1 (Q1: Option A from commit `08b15d5`; Q2: Option A Anchor Map Locking; Q3: Option A Zero-Copy Base64 stream + Timer fix) while maintaining current professional modular architecture.

---

## Global Constraints

- Preserve modular architecture: `MotionAdapter` remains the clean interface for coordinator, controllers, and planning.
- Zero fake/simulated walls: Maps must reflect true physical LiDAR returns.
- Pure NumPy / SciPy SIMD execution on laptop and Jetson Nano (execution latency < 3.0ms).
- Dashboard protocol compatibility: All WebSocket payloads (`map_update`, `robot_pose`, JB-Binary 1,130 bytes) remain strictly backwards compatible.
- External user communications must be in Vietnamese.

## Review Focus

1. **In-place rotation on slick tile:** When turning in place with zero wheel encoder feedback, does the map stay stationary and does the robot heading rotate smoothly without ghost walls?
2. **Clock skew resilience:** Does network jitter or heartbeat frames cause latency to spike to > 1000ms?
3. **Wall preservation:** Does repeated free-space ray clearing across mapped corridors delete or weaken confirmed static walls?
4. **Memory & CPU allocation:** Does the 10Hz map broadcast avoid allocating 10,000 Python ints per frame?
5. **Footprint clearance:** Does moving the robot keep its local radius clear so A* planner never starts in collision?

---

### Task 1: Wire Hector Scan-to-Map Odometry & Anchor Locking into MotionAdapter

**Files:**
- Modify: `dashboard/backend/app/navigation/motion_adapter.py:70-130`
- Test: `tests/test_hector_anchor_slam.py`

**Interfaces:**
- Consumes: `Lidar2DOdometry` (`motion/planning/lidar_odometry.py`), `OccupancyGridMap` (`motion/planning/occupancy_grid.py`)
- Produces: `MotionAdapter.update_odometry(ranges, dt, v_cmd, w_cmd) -> Tuple[float, float, float, Dict[str, Any]]`

- [ ] **Step 1: Write failing unit test for in-place rotation with static anchor walls**

Create `tests/test_hector_anchor_slam.py`:
```python
import unittest
import math
import numpy as np
from app.navigation.motion_adapter import MotionAdapter

class TestHectorAnchorSLAM(unittest.TestCase):
    def test_01_anchor_wall_locking_after_initial_scans(self):
        adapter = MotionAdapter()
        # Simulate circular room of radius 1.5m
        room_ranges = [1.5] * 360
        for _ in range(6):
            ox, oy, oth, info = adapter.update_odometry(room_ranges, dt=0.1, v_cmd=0.0, w_cmd=0.0)
        
        self.assertTrue(adapter.anchor_initialized or adapter.grid_map.anchor_locked)
        self.assertIsNotNone(adapter.grid_map.static_anchor_walls)
        self.assertGreaterEqual(np.count_nonzero(adapter.grid_map.static_anchor_walls), 20)

    def test_02_in_place_rotation_tracks_robot_not_map(self):
        adapter = MotionAdapter()
        room_ranges = [1.5] * 360
        # Initialize anchor
        for _ in range(6):
            adapter.update_odometry(room_ranges, dt=0.1, v_cmd=0.0, w_cmd=0.0)
        
        # Now simulate robot turning 45 deg CCW:
        # In robot frame, walls appear shifted by -45 deg
        turned_ranges = [1.5] * 360
        ox, oy, oth, info = adapter.update_odometry(turned_ranges, dt=0.1, v_cmd=0.0, w_cmd=0.785)
        
        # Robot position (ox, oy) must remain near origin (<= 0.05m)
        self.assertLess(math.hypot(ox, oy), 0.05)
```

- [ ] **Step 2: Run test to verify failure**

Run: `.venv/bin/python -m unittest tests/test_hector_anchor_slam.py -v`
Expected: FAIL because `static_anchor_walls` is not currently set or checked in `MotionAdapter`.

- [ ] **Step 3: Implement Hector Scan-to-Map with Anchor Locking in `motion_adapter.py`**

In `dashboard/backend/app/navigation/motion_adapter.py`:
1. Use `self.grid_map` with `AnchorLayer` enabled as the primary occupancy grid and costmap engine.
2. In `update_odometry(ranges, dt, v_cmd, w_cmd)`:
   - Call `self.lidar_odometry.update(ranges, dt=dt, v_cmd=v_cmd, w_cmd=w_cmd, grid_map=self.grid_map)`.
   - When `self.lidar_odometry.anchor_locked` and scan count >= 5, lock anchor walls in `self.grid_map`.
   - Update occupancy grid via `self.grid_map.update_scan(ox, oy, oth, ranges=ranges)`.
   - Return `(ox, oy, oth, info)`.
3. In `to_dashboard_bytes()` and `to_dashboard_b64()`: delegate directly to `self.grid_map.to_dashboard_bytes()` and `self.grid_map.to_dashboard_b64()`.

- [ ] **Step 4: Run test to verify it passes**

Run: `.venv/bin/python -m unittest tests/test_hector_anchor_slam.py -v`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add tests/test_hector_anchor_slam.py dashboard/backend/app/navigation/motion_adapter.py
git commit -m "feat(slam): wire Hector scan-to-map with dual-layer anchor locking into MotionAdapter"
```

---

### Task 2: Eliminate Latency Spike & Timer Drift in Telemetry Pipeline

**Files:**
- Modify: `dashboard/jetbot_bridge.py:1130-1165`
- Modify: `dashboard/backend/app/hardware/scan_gate.py:95-120`
- Modify: `dashboard/backend/app/hardware/telemetry_parser.py:165-195`
- Test: `tests/test_telemetry_latency.py`

**Interfaces:**
- Consumes: JetBot binary telemetry frame (1,130 bytes), `ScanGate.accept()`
- Produces: True sub-50ms latency calculation without artificial clock skew contamination.

- [ ] **Step 1: Write failing test for ScanGate timestamp monotonicity and jitter**

Create `tests/test_telemetry_latency.py`:
```python
import unittest
import time
from app.hardware.scan_gate import ScanGate

class TestTelemetryLatency(unittest.TestCase):
    def test_01_scangate_preserves_hardware_timestamp(self):
        gate = ScanGate()
        hw_stamp = 1700000000.123
        data = {
            "lidar_ranges": [1.0] * 360,
            "scan_seq": 100,
            "scan_stamp": hw_stamp
        }
        tick = gate.accept(data)
        self.assertIsNotNone(tick)
        self.assertEqual(tick.stamp, hw_stamp)
        
        # Duplicate seq arrives 30ms later -> must be dropped
        dup = gate.accept(data)
        self.assertIsNone(dup)
```

- [ ] **Step 2: Run test to verify current behavior**

Run: `.venv/bin/python -m unittest tests/test_telemetry_latency.py -v`

- [ ] **Step 3: Implement fixes in `jetbot_bridge.py`, `scan_gate.py`, and `telemetry_parser.py`**

1. In `dashboard/jetbot_bridge.py`:
   - Send `scan_stamp` when `has_fresh_scan` is True.
   - For heartbeat frames when LiDAR has not produced a new sweep, set `seq = last_sent_scan_seq` and pass `timestamp = now`.
2. In `dashboard/backend/app/hardware/scan_gate.py`:
   - Never add `(now - self._last_accept_wall)` to a hardware timestamp. If stale force-accept is required, use `self.last_stamp + dt`.
3. In `dashboard/backend/app/hardware/telemetry_parser.py`:
   - In `_log_lidar_surroundings`: bound `min_skew` updates with an exponential moving average and clamp `true_lag_ms = max(0.0, min(999.0, (now - scan_t - min_skew) * 1000.0))`.

- [ ] **Step 4: Run test to verify it passes**

Run: `.venv/bin/python -m unittest tests/test_telemetry_latency.py -v`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add tests/test_telemetry_latency.py dashboard/jetbot_bridge.py dashboard/backend/app/hardware/scan_gate.py dashboard/backend/app/hardware/telemetry_parser.py
git commit -m "fix(telemetry): stabilize clock skew filtering and prevent phantom latency spikes"
```

---

### Task 3: Zero-Copy Base64 Dashboard Stream & Coordinator Coverage Reporting

**Files:**
- Modify: `dashboard/backend/app/navigation/coordinator.py:500-575`
- Modify: `dashboard/backend/app/hardware/telemetry_parser.py:480-515`
- Modify: `dashboard/frontend/src/hooks/useRobotState.ts:158-185`
- Test: `tests/test_zero_copy_dashboard.py`

**Interfaces:**
- Consumes: `MotionAdapter.to_dashboard_b64()`, `MotionAdapter.to_dashboard_bytes()`
- Produces: `coordinator.get_map()`, `map_update` WebSocket message with zero Python list overhead.

- [ ] **Step 1: Write test for Zero-Copy Base64 map retrieval and coverage reporting**

Create `tests/test_zero_copy_dashboard.py`:
```python
import unittest
import asyncio
from app.navigation.coordinator import NavigationCoordinator

class TestZeroCopyDashboard(unittest.TestCase):
    def test_01_coordinator_map_reporting(self):
        coord = NavigationCoordinator()
        # Feed synthetic scans to populate map
        ranges = [1.5] * 360
        for _ in range(6):
            coord.motion.update_odometry(ranges, dt=0.1, v_cmd=0.0, w_cmd=0.0)
        
        loop = asyncio.new_event_loop()
        m = loop.run_until_complete(coord.get_map())
        self.assertIsNotNone(m.data_b64)
        self.assertIsNone(m.data)  # Zero Python list allocation!
        
        # Verify coverage in HEALTH log
        raw = coord.motion.grid_map.to_dashboard_bytes()
        free_pct = 100.0 * raw.count(b"\x00") / len(raw)
        self.assertGreater(free_pct, 1.0, "Map must reflect explored free space!")
```

- [ ] **Step 2: Run test to verify failure**

Run: `.venv/bin/python -m unittest tests/test_zero_copy_dashboard.py -v`

- [ ] **Step 3: Implement Zero-Copy Base64 in `coordinator.py` & `useRobotState.ts`**

1. In `dashboard/backend/app/navigation/coordinator.py`:
   - `get_map()`: return `MapData(..., data=None, data_b64=self.motion.to_dashboard_b64())`.
   - In `_log_health()`: inspect `self.motion.grid_map.to_dashboard_bytes()` to log accurate `free%` and `occ%`.
2. In `dashboard/frontend/src/hooks/useRobotState.ts`:
   - In `case 'map_update'`: decode `data_b64` into `new Int8Array(buffer)` directly without `Array.from()`.

- [ ] **Step 4: Run test to verify it passes**

Run: `.venv/bin/python -m unittest tests/test_zero_copy_dashboard.py -v`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add tests/test_zero_copy_dashboard.py dashboard/backend/app/navigation/coordinator.py dashboard/frontend/src/hooks/useRobotState.ts
git commit -m "perf(dashboard): convert map_update pipeline to zero-copy base64 stream"
```

---

### Task 4: Full System Verification & JetBot Synchronization

**Files:**
- Sync: `dashboard/jetbot_bridge.py` -> `jetbot@192.168.168.230:/home/jetbot/03-MinhHieu/jetbot_bridge.py`
- Verify: `scripts/verify_agent_integrity.py`
- Verify: `dashboard/test_slam_dynamics_and_rotation.py`
- Verify: `tests/test_hector_anchor_slam.py`
- Verify: `tests/test_telemetry_latency.py`
- Verify: `tests/test_zero_copy_dashboard.py`

- [ ] **Step 1: Run complete automated test suite**

Run:
```bash
.venv/bin/python -m unittest discover -s tests -v
.venv/bin/python dashboard/test_slam_dynamics_and_rotation.py
.venv/bin/python scripts/verify_agent_integrity.py
```
Expected: All tests PASS with 0 errors.

- [ ] **Step 2: Build frontend bundle**

Run:
```bash
cd dashboard/frontend && npm run build
```
Expected: Build succeeds with 0 errors.

- [ ] **Step 3: Synchronize updated `jetbot_bridge.py` to JetBot**

Sync via `scp` or `rsync` to `jetbot@192.168.168.230:/home/jetbot/03-MinhHieu/jetbot_bridge.py`.

- [ ] **Step 4: Commit and tag completion**

```bash
git add .
git commit -m "chore(release): complete SLAM anchor wall locking and low-latency dashboard pipeline"
```
