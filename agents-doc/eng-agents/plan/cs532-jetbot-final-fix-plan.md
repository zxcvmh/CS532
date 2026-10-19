# CS532 JetBot SLAM — Audit Findings & Final Fix Plan

**Role:** `quality-auditor` (per `Projects/CS532/agents-doc/eng-agents/quality-auditor.md`)
**Status:** plan only — **no code has been modified**
**Line numbers** are from the revision audited. The repo is unchanged, but re-locate every
target **by symbol** when patching.

> **Requested output path was `Projects/CS532/agents-doc/eng-agents/plan/`.**
> That path is not writable in Plan mode. This file was written to the only directory
> permitted while Plan mode is active: `/home/zxcvmh/.opencode/plan/`.
> After switching agents it can be copied to the requested location verbatim.

---

## 0. Reported symptoms → proven causes

| Symptom | Root cause | Status |
|---|---|---|
| Auto mode: robot does not update / does not move | stale 2 m × 1 m arena in `setup/robot_dynamics.json` propagating into `AStarPlanner` bounds → start, goal and every waypoint clamped | **PROVEN** |
| Map renders black | *same* stale arena → `_update_costmap()` block-fills the exterior lethal → 97.1 % of cells exported as obstacle | **PROVEN** |
| Ghost obstacles | `if valid_cls:` guard makes the only clearing path unreachable | **PROVEN** |
| Box in front not recognised (physical bump) | near-field blind spot + `min_intensity=15` discarding the black box's weak returns + unwired CV low-obstacle path + float-equality bug + cluster eviction by over-large `clear_footprint` | **4 mechanisms proven; the dominant one pending Phase 0** |

**Key insight:** the black map and the "Auto does nothing" symptom share **one** root cause
(the stale arena). Fixing it once fixes both.

**Second key insight:** `threat_level` / `WALL_SURFACE` is **cosmetic only**.
`controller.py#L334-L345` gates the static-cluster check on `elif grid_map is None:`, and
`MotionAdapter` always supplies a grid map — so **the costmap is the ONLY channel by which a
static obstacle influences motion**. The cluster-lifecycle bugs are therefore
safety-relevant, not cosmetic.

---

## 1. Which copy of each file actually runs

```
03-MinhHieu/                       NO planning/ directory exists here
  d500_driver.py            <- live driver (hand-written 0x54/0x2C + CRC-8, no vendor lib)
  jetbot_bridge.py          <- live bridge (instantiates the driver, DDE, reflex task)
  disparity_extender.py     <- live DDE (imported first at jetbot_bridge.py#L434)
  floor_ipm.py, command_mux.py, health.py, backend/, frontend/

05-d500lib-motion/planning/        <- imported by the backend; the ONLY live SLAM tree
  occupancy_grid.py  lidar_odometry.py  obstacle_clustering.py
  astar_planner.py   controller.py      disparity_extender.py (mirror)  lidar_parser.py

02-DuyHieu/planning/               <- divergent third copy; NOT imported by anything
```

Evidence — `03-MinhHieu/backend/app/__init__.py`:

```python
# Single Source of Truth for Motion Planning: 05-d500lib-motion/planning
_motion_dir = .../"05-d500lib-motion"
sys.path.insert(0, _motion_dir)
for sub in ["occupancy_grid", "astar_planner", "controller", "mppi_controller",
            "obstacle_clustering", "lidar_odometry", "cbf_safety_filter",
            "disparity_extender", "kinodynamic_rvo", "lidar_parser"]:
    sys.modules[f"app.planning.{sub}"] = __import__(f"planning.{sub}", fromlist=[sub])
```

**Therefore "don't touch `05`" cannot apply to the SLAM fixes** — `occupancy_grid.py`,
`lidar_odometry.py` and `obstacle_clustering.py` (three of the seven files originally under
review) exist **only** in `05-d500lib-motion/planning/`. User has since authorised touching
`05` where it is the only live copy.

Also note: the driver is **not** covered by the `sys.path` rule — that gap is how
`min_intensity` could drift between copies.

---

## 2. Proven findings

Every item below was reproduced by running the real modules with the repo venv, or by
control-flow proof.

### BUG-01 — CRITICAL — black map
`setup/robot_dynamics.json` contains `"tested_arena": "2m_x_1m"` →
`OccupancyGridMap.arena_bounds = (-0.25, 2.05, -0.65, 0.65)` (loaded via
`../..//setup/robot_dynamics.json`) → `_update_costmap()` block-fills everything outside as
lethal `254` → `to_dashboard_bytes()` maps `costmap >= 250` → `100`.

Measured: **38 859 / 40 000 cells = 97.1 % occupied, 2.8 % free, 0.0 % unknown.**
`MapViewer.tsx` paints `val >= 50` as `rgb(9,13,22)` in the default light theme → near-black
rectangle.

### BUG-02 — CRITICAL — Auto mode produces no motion
`AStarPlanner(self.grid_map)` → `arena_bounds=None` → falls back to the stale arena. Start,
goal **and every waypoint** are clamped (`astar_planner.py#L176-L200`, `#L269-L275`).

Measured:
- `plan((0,0) → (4.0,4.0))` → 2 points, last waypoint **(1.97, 0.57)** in 0.24 ms.
- `plan((0,0) → (0.10,0.05))` → degenerate start≈goal path → instant `GOAL_REACHED`, **no motion**.

Note: arena enforcement is redundant for planning — `astar_planner.py#L239-L243` already
re-masks outside the arena, so removing the block-fill costs A* nothing.

### BUG-03 — HIGH — ghost obstacles
`telemetry_parser.py#L150-L151` guards `insert_obstacle_clusters` with `if valid_cls:`, so the
only clearing path (`occupancy_grid.py#L509-L527`) can never run.
Reproduced: `active_clusters == [(0.9, 0.0, 0.4)]` persisted after the source vanished.

### BUG-04 — HIGH — phantom lethal disc
`occupancy_grid.py#L550-L559` gives a 1.5 m wall segment `obs_radius = 0.40` → an 0.8 m
diameter lethal disc at its centroid.

### BUG-05 — CRITICAL — the bump (4 mechanisms)
1. Near-field erasure by `clear_footprint(radius=0.15)` — wider than the 0.09 m chassis.
2. CV low-obstacle path never wired: `insert_low_obstacles` has **zero callers**; the
   `low_obstacles` telemetry key is ignored; `coordinator.motion_xai_info` lacks it →
   `MapViewer.tsx#L248` layer is dead.
3. Arena filter drops clusters outside ±0.15 m of the stale arena — reproduced: a cluster at
   world `(10, 10)` was dropped.
4. Sub-16 cm blind spot: the scan plane sits 16 cm above the floor and cannot be lowered
   (user constraint, consistent with the datasheet — no mandated mounting height).

### BUG-06 — MEDIUM — YOLO false positive becomes a solid obstacle
`add_social_bubble` writes cost `254` for `d <= 0.20` → exported as obstacle `100` for
`social_bubble_ttl_sec = 1.5 s`.

### BUG-07 — MEDIUM — silent pose freeze
`lidar_odometry.py#L504-L511` reports `dx_res/dy_res/dth_res` but never integrates
`self.x/y/theta`.

### BUG-08 — MEDIUM — dead TTL layer
`_rolling_cells` (declared `occupancy_grid.py#L86-L89`) is only popped/cleared, **never
written** → the claimed 250 ms "zero ghost obstacles" layer is dead code.

### BUG-09 — LOW/MEDIUM
`insert_obstacle_clusters` drops clusters whose centroid is exactly `(0.0, 0.0)`;
`get_probability_map()` performs a full 200×200 `np.exp` per GN iteration (10×/scan).

### BUG-10 — MEDIUM (latent) — A* has no deadline and blocks the loop
- `coordinator.py#L185` / `#L210` call `motion.plan(...)` **synchronously** on the event loop
  (`#L130` correctly uses `to_thread`).
- `motion_adapter.py#L131-L134` passes `timeout_sec` / `max_iterations` / `start_theta`, which
  do not exist on `AStarPlanner.plan` → swallowed by `**kwargs`.
- `max_time_sec` / `max_expansions` are declared but **never referenced again**.

### BUG-11 — LOW — two different clouds from one frame
Driver confirmed sweeping (`[OK] D500 LiDAR active: receiving 360° sweeps`), so downgraded to
defensive + instrumentation. Sub-item is real: `ScanGate` accepts `dist <= 12.0` while
`parse_lidar_ranges` uses `Config.MAX_RANGE` (6.0).

### NEW-01 — two call sites mutate the costmap in opposite orders
`telemetry_parser.py#L150-L153` inserts clusters **then** clears footprint (evicting near
clusters); `controller.py#L923-L925` clears **then** inserts (keeping them) → flicker.

### NEW-02 — six inconsistent near-field constants
`min_range_m=0.15`, parser `0.15`, `ScanGate 0.15`, controller `0.12`,
`clear_footprint(0.15)`, `reflex_dist_threshold=0.16`.

### NEW-03 — HIGH diagnosability
`d500_driver.py#L270` drops intensity-rejected bins **before** appending → `latest_scan_list`
has no entry → the backend sees `0.0` and cannot distinguish "no return" from "filtered by
intensity".

### NEW-04 — CRITICAL architectural
No contact / bumper / IR safety layer exists anywhere (only a comment in
`disparity_extender.py#L234`). Accepted as residual risk by user decision (R3-Q3 option (a),
software only).

---

## 3. Hypotheses raised and closed

| Hypothesis | Verdict |
|---|---|
| `WALL_SURFACE → threat="SAFE"` suppresses avoidance | **REFUTED** — the controller ignores static clusters whenever a grid map exists; `threat_level` is unused |
| Stale bitmap in the frontend | **CLEARED** — `useRobotState.ts#L106` increments `version` locally |
| `mapToScreen` origin bug | **CLEARED** — origin anchoring is deliberate and consistent |
| Vendor/prebuilt D500 library in use | **CLEARED** — `03-MinhHieu/d500_driver.py` is hand-written |
| The bump is a SLAM bug or the sub-16 cm blind spot | **UNLIKELY** — the box was 30 cm tall, i.e. **3× the 16 cm scan plane**, so the beam hits its side face; the datasheet guarantees 8 m on 4 % black targets. The CV floor-IPM path is calibrated for "<16 cm" and would not have prevented it. |
| **H2 (leading):** `min_intensity=15` silently discards the black canvas box's low-confidence returns | **OPEN — Phase 0 falsifies** |
| H3: `lidar_yaw_offset_deg` is wrong (currently `0.0`, suggesting the orientation check never ran on this mounting) | **OPEN — Phase 0 falsifies** |
| Geometric only | **OPEN — Phase 0 falsifies** |

**Falsifiable prediction for Phase 0:** box bins present with confidence ~5–15, straddling the
threshold. Bins absent *even at `min_intensity=0`* ⇒ H2 dead ⇒ cause is geometric or H3.

---

## 4. Patch surface & authorisation

| Tree | Files | Note |
|---|---|---|
| `03-MinhHieu/` | `d500_driver.py`, `jetbot_bridge.py`, `disparity_extender.py`, `floor_ipm.py`, `backend/app/**` | live copies; authorised |
| `05-d500lib-motion/planning/` | `occupancy_grid.py`, `lidar_odometry.py`, `obstacle_clustering.py`, `astar_planner.py`, `controller.py` | the imported tree; authorised |
| `03-MinhHieu/frontend/src/components/MapViewer.tsx` | render + legends | authorised |
| `setup/test_lidar.py` | triage tool | authorised |
| **untouched** | `05-*/d500_driver.py`, `05-*/jetbot_bridge.py`, `05-*/disparity_extender.py`, `02-DuyHieu/**` | mirrors + teammate's work; drift accepted |

---

## 5. Hardware & sensor constraints (govern every constant below)

- LiDAR: **LDROBOT D500 / STL-19P** — DTOF 905 nm, 5000 Hz, 10 Hz, 0.72°, 60 Klux.
- Range: **0.03–12 m** on white 80 % target; **0.03–8 m** on black 4 % target.
- Accuracy table starts at **300 mm**; LD19 manual: *"when the ranging is lower than 0.3 m,
  some data are output"*. Features include **glass-wall detection** (§5.1 requires an
  unoccluded optical window; no mandated mounting height).
- Chassis: **15 cm × 10 cm** → circumscribed radius **9.0 cm**.
- Scan plane: **16 cm above the floor, cannot be lowered**.
- Reflex thresholds today (`0.13` / `0.16 m`) sit **below the sensor's guaranteed accuracy
  floor (300 mm)** — the DDE brakes inside its own unspecified zone.
- `min_intensity=15` also defeats the D500's documented glass-wall-detection feature.
- `Config.ENVIRONMENT_MODE="ROOM"` sets `MAX_RANGE=6.0` while the JSON arena disagrees.
- Environment: free space now (home testing); final demo is a **5 m × 5 m arena**.

---

## Phase 0 — Triage (hardware, run by the operator) · BLOCKS Phase 1's constants

Extend `setup/test_lidar.py` (it already has the port finder, live compass, orientation check
and reflex test):

| Addition | Output |
|---|---|
| `--raw-dump` | re-read the serial stream with `min_intensity=0`: `angle, dist_mm, confidence` for the frontal ±25° sector, plus `rejected_by_intensity` / `rejected_by_range` counters |
| `--box-sweep` | operator enters the box's true distance; prints the frontal-sector minimum and the valid-ray count within 0.6 m, at 10 / 16 / 25 / 40 / 60 cm |
| orientation | the 0° verdict and the `lidar_yaw_offset_deg` it writes |

**Decides:** H2 vs H3 vs pure geometry. This step can invalidate Phase 1's parameter values, so
it runs first (per sequencing decision R3-Q5).

---

## Phase 1 — Intensity policy + observability → *fixes the bump*

**Files:** `03-MinhHieu/d500_driver.py`, `03-MinhHieu/jetbot_bridge.py`

1. **Range-dependent acceptance**, replacing the single predicate at `d500_driver.py#L270`:
   accept `intensity >= 1` when `dist_m <= 1.0 m`; require `intensity >= 15` beyond.
   Weak short returns are real geometry; specular floor returns come back *long*.
2. `min_distance_m` **0.12 → 0.05** (datasheet floor 0.03 + 2 cm mounting margin).
3. **Stop destroying evidence:** `rejected_by_intensity` / `rejected_by_range` counters exposed
   via `get_diagnostics()`.
4. Keep the per-bin `_current_sweep_intensities` (already computed but discarded) so the policy
   can live downstream instead of in the driver.
5. `min_intensity` overridable at construction — tuning without a code edit.

**Verify:** rerun `--box-sweep`; the box must appear as a bin at its bearing → in the `/ws`
cluster payload → as a `100` cell at the box's world coordinate. **And** the empty-room
`100`-cell census must not rise (tile-ghost regression gate).

---

## Phase 2 — Geometry, rendering, A* honesty → *fixes the black map + Auto no-motion*

**Files:** `05-.../occupancy_grid.py`, `05-.../astar_planner.py`, `03-.../backend/app/config.py`,
`coordinator.py`, `motion_adapter.py`, `frontend/src/components/MapViewer.tsx`

> **Ordering constraint: 2.4 before 2.3.** Removing the arena inflates the A* search space
> ~30×. The missing deadline must be restored first or this phase reintroduces the event-loop
> stall.

1. `config.py` — **one source of truth**: `ARENA_BOUNDS` per profile
   (`ROOM = None`, `ARENA_2X1 = (-0.25, 2.05, -0.65, 0.65)`,
   `SEMINAR_5X5 = (-0.25, 5.25, -0.25, 5.25)`), plus `LIDAR_MIN_RANGE = 0.05`,
   `MAPPING_MAX_RANGE = 6.0`, `ROBOT_RADIUS = 0.09`, `REFLEX_STOP = 0.30`.
   Add a **boot-time assert** rejecting `ROOM` + a sub-3 m arena — the single line that would
   have caught this entire class of failure.
2. `occupancy_grid.py` ~`#L110-L120` — stop deriving `arena_bounds` from `tested_arena`; read
   `Config`.
3. `occupancy_grid.py` ~`#L667-L679` — mark only the **1-cell perimeter ring** lethal, never
   block-fill the exterior. A* already re-masks outside its own bounds
   (`astar_planner.py#L239-L243`), so the fill buys nothing for planning and costs the map.
4. `astar_planner.py` ~`#L176-L200`, `#L269-L275` — **never silently relocate a goal.**
   Out-of-operating-area goal → return `None` + reason `GOAL_OUTSIDE_OPERATING_AREA`. Keep the
   clamp only for the **start** (defensive, with a `WARNING`). Honour a real `max_time_sec` /
   `max_expansions` inside `_py_search`.
5. `motion_adapter.py` ~`#L131-L134` — pass the **real** argument names (`max_time_sec=`,
   `max_expansions=`, `robot_theta=`); today `timeout_sec` / `max_iterations` / `start_theta`
   are swallowed by `**kwargs`.
6. `coordinator.py` ~`#L121-L142` — surface a rejected goal as an `ERROR` event instead of
   silently degrading into `GOAL_REACHED`.
7. `MapViewer.tsx` — unknown space renders as a **distinct slate tone in dark mode** (not
   transparent-black) + a small free/unknown/occupied legend. This ends the ambiguity that made
   the bug expensive.

**Verify:** map census `occupied` drops from 97.1 % to approximately the real wall area;
`plan((0,0) → (4.0,4.0))` fails **loudly** instead of returning `(1.97, 0.57)`; in
`SEMINAR_5X5`, an in-arena goal is reached within tolerance.

---

## HARDWARE VERIFICATION GATE — after Phase 2

The robot drives in `ROOM`, the map renders, goal rejection is visible in the UI.
**Do not proceed to Phase 3 until this passes on the JetBot** (decision R2-Q6).

---

## Phase 3 — Cluster lifecycle & ghost obstacles

**Files:** `telemetry_parser.py`, `occupancy_grid.py`, `obstacle_clustering.py`, `controller.py`

1. `telemetry_parser.py` ~`#L150-L152` — call `insert_obstacle_clusters(valid_cls, …)`
   **unconditionally**; the `if valid_cls:` guard is why `active_clusters` can never be cleared.
2. `telemetry_parser.py` ~`#L152` — `clear_footprint(radius=0.15)` → `Config.ROBOT_RADIUS`
   (0.09); **stop evicting clusters** there. Rely on `AStarPlanner.find_nearest_free_cell(
   max_radius=15)` for start-cell rescue (decision R2-Q4).
3. Unify the costmap mutation order across **both** call sites (`telemetry_parser.py#L150-L153`
   vs `controller.py#L923-L925` — currently opposite).
4. `occupancy_grid.py` ~`#L550-L559` — cluster footprint = rasterise the real `bounding_box_2d`
   (OBB), capped ≤ 0.25 m; **skip `WALL_SURFACE` clusters entirely** (walls already come from
   `log_odds`); `BOX_OBSTACLE` → OBB, `TABLE_LEG` / `THIN_OBJECT` → capped 0.11 m disc.
   Kills the 0.8 m phantom disc.
5. `occupancy_grid.py` ~`#L534-L536` — replace the `cx == 0.0 and cy == 0.0` float compare with
   a proper `points_count > 0` validity check.
6. `occupancy_grid.py` ~`#L686-L729` — social-bubble core cost `254 → 230` (non-lethal) so a
   YOLO false positive cannot materialise a solid obstacle; keep the comfort ring soft.
7. `occupancy_grid.py` ~`#L610` — `inscribed_m` `0.065 → 0.09` (the 15 × 10 cm chassis
   circumscribes at 9.0 cm; 0.065 under-protects the corners by 2.5 cm).
8. `obstacle_clustering.py` ~`#L393-L403` — stop labelling a 0.5 m box `WALL_SURFACE`; walls
   require thickness + linearity, not length alone. Cosmetic for motion (the controller ignores
   static clusters when a grid map exists) but it is the label the operator reads in the XAI
   panel.
9. **Dead code out** (verified by grep): `lock_anchor_map` / `unlock_anchor_map`,
   `static_anchor_walls`, `anchor_scan_count`, `_rolling_cells` / `rolling_ttl_sec`
   (`occupancy_grid.py`); `predict_position` / `predict_world_position` / `to_dict_list`
   (`obstacle_clustering.py`); the never-queried `anchor_kdtree` / `anchor_locked` tier
   (`lidar_odometry.py`). Correct the docstrings that promise non-existent behaviour
   ("zero ghost obstacles in 250 ms", "Scan-to-Anchor").

**Verify:** lifecycle test (feed clusters → `len == n`; feed `[]` through the **same** call site
→ `len == 0` — **fails today**); box acceptance test at 0.6 m and 0.12 m; ghost test (remove the
box → the `100` cell must vanish within one map cycle).

---

## Phase 4 — CV low-obstacle fusion + near-field safety

**Files:** `telemetry_parser.py`, `coordinator.py`, `occupancy_grid.py`,
`disparity_extender.py`, `lidar_odometry.py`, `MapViewer.tsx`

1. `telemetry_parser.py` — read `data["low_obstacles"]` (the bridge already sends it,
   `jetbot_bridge.py#L974`) → the existing `insert_low_obstacles()` helper → a **short-TTL
   (0.3 s) costmap layer**, never into `log_odds` (avoids a new ghost source).
2. `coordinator.py` ~`#L48` / `get_motion_xai()` — add `low_obstacles` so the amber diamonds in
   `MapViewer.tsx#L248` finally render.
3. `disparity_extender.py` (**live `03` copy**) — add the **"front unverified ⇒ crawl"** rule
   (decision R4-Q2): zero valid returns in the frontal ±25° cone while `cmd_v > 0` → cap to
   crawl speed and report `FRONT_UNVERIFIED`. Move `critical_stop_m` **0.16 → 0.30** and the
   boxed-in rule **0.13 → 0.22**.
4. `lidar_odometry.py` ~`#L504-L511` — integrate the kinematic prior (dead-reckon) **and** flag
   `is_reliable=False` + a `POSE_STALE` status + a visible `WARNING` event, instead of the
   current silent freeze that reports deltas it never applied.
5. `coordinator.py` — `_step_navigation` / `_step_recovery` → `await asyncio.to_thread(
   self.motion.plan, …)`; on > 0.5 s without a fresh `ScanTick`, zero `cmd_vel` and raise a
   `WARNING`.

**Division of authority (decided):** the DDE on the Jetson owns the near-field guard at 25 Hz;
the backend's `d_stop` is secondary; the **backend remains the sole planner / primary
navigation authority** and the DDE is a 0.3 s safety veto via `mux.reflex_override`.

**Verify:** diamonds appear for a < 16 cm object; `FRONT_UNVERIFIED` swallows the black box with
the intensity filter artificially restored; A* with a room-wide goal completes within the
deadline; a > 0.5 s telemetry gap produces a warning and a stop.

---

## Phase 5 — Wire-format simplification · *only after Phases 1–4 verify*

**Files:** `jetbot_bridge.py`, `telemetry_parser.py`, `scan_gate.py`

Send the dense `ranges_360` + `intensities_360` arrays the driver **already computes**
(`get_ranges_360()`) instead of the 360-pair list. `ScanGate` dedups on `scan_seq` with a
staleness fallback (`seq == 0/absent → accept`; `> 0.5 s → force accept`) and uses **one** range
filter from `Config`. Collapses BUG-11 + NEW-02 + the range-constant spread (R2-Q7) in a single
move, and deletes the duplicate binning in both `ScanGate` and the parser.

**Verify:** packet size drops; the dashboard cloud and the odometry cloud are provably
identical; `scan_gate.py`'s 5 self-tests still pass.

---

## Phase 6 — Probabilistic tuning · *gated on Phases 1–5*

`L_OCC` / `L_FREE`, the likelihood-field sigma, any cluster TTL. **Nothing here before the
geometry and the intensity policy are verified** — tuning against a wrong arena is how the
previous week was lost.

---

## 7. Ordering constraints (hard requirements)

| # | Constraint |
|---|---|
| 1 | Phase 0 triage **before** Phase 1 — it can change Phase 1's parameter values |
| 2 | Phase 2 item **2.4 before 2.3** — restore the A* deadline before removing the arena |
| 3 | **Hardware gate after Phase 2** — do not start Phase 3 until the robot drives and renders |
| 4 | Phase 5 (wire format) **only after** Phases 1–4 verify |
| 5 | Phase 6 tuning **only after** the geometry and intensity policy are verified |
| 6 | Phase 1's tile-ghost regression check must pass before any Phase 6 tuning |

---

## 8. Cross-cutting: make the next failure provable

Fold into Phase 2 — **highest value per line in this entire plan**:

- Backend logs to a **rotating file by default**. The operator was unsure `server.log` existed,
  which is what made the Auto-mode session unprovable.
- A **1 Hz health probe**, one line: `scan_seq`, last accepted `ScanTick.seq`, pose, nav status,
  `arena_bounds`, map census (unknown/free/occupied %), `rejected_by_intensity`,
  `rejected_by_range`.
- `verify_fixes.py` in `03-MinhHieu/`: prints the census, arena bounds, goal fidelity, cluster
  lifecycle and the near-field constants in one shot; runnable on the Jetson.

---

## 9. Residual risks — recorded, not fixed

| Risk | Status |
|---|---|
| No contact / bumper / IR layer. Any obstacle both LiDAR and CV miss still results in a bump. | user's R3-Q3(a) call, software-only, documented |
| `05-*` mirrors of `d500_driver.py` / `jetbot_bridge.py` / `disparity_extender.py` will drift from the patched `03-MinhHieu` copies | accepted (R4-Q4) |
| `02-DuyHieu/planning/` divergent copies remain | untouched — teammate's work |
| If Phase 0 shows bins absent at `min_intensity=0`, Phase 1's values change and H3 (yaw offset) becomes primary | open until triage |
| No regression test suite exists; module-level `if __name__ == "__main__"` self-tests are the convention | follow the convention per phase |

---

## 10. Verification harness sketch

| Check | Command / method | Pass condition |
|---|---|---|
| Map census | `verify_fixes.py` | occupied ≈ real wall area, not 97 % |
| Arena bounds | `verify_fixes.py` | matches `Config.ARENA_BOUNDS` for the active profile |
| Goal fidelity | `AStarPlanner.plan((0,0) → (4,4))` | loud `GOAL_OUTSIDE_OPERATING_AREA`, not a clamped path |
| Cluster lifecycle | feed clusters then `[]` via `insert_obstacle_clusters` | `len` returns to 0 |
| Intensity policy | `setup/test_lidar.py --box-sweep` | box bin present at low intensity, ≤1 m |
| Ghost regression | remove the box, one map cycle | `100` cell disappears |
| Near-field constants | `verify_fixes.py` | exactly one value per concept, from `Config` |
| Reflex veto | place a 0.4 m obstacle, command forward | stop/crawl, `FRONT_UNVERIFIED` when unverified |
| Pose staleness | kill telemetry 0.5 s | warning + zero `cmd_vel` |

---

## 11. Status

- **No files modified.** Session was in Plan mode throughout.
- Implementation requires switching to a code agent.
- Open: Phase 0 triage result (decides the bump's dominant mechanism and Phase 1's values).
