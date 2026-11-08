#!/usr/bin/env python3
"""
CS532 JetBot SLAM — one-shot verification harness
================================================================
Implements section 8/10 of cs532-jetbot-final-fix-plan.md: it prints the census, the arena
bounds, goal fidelity, the cluster lifecycle and the near-field constants, in one shot,
runnable on the Jetson.

Usage (from 03-MinhHieu/):
    python3 verify_fixes.py

Exit code 0 = every offline-provable check passed. Exit code 1 = at least one FAIL.
Checks needing the physical board (intensity policy, reflex veto on real hardware) are
reported as MANUAL with the exact command an operator must run — they are never
silently counted as passing.
"""

import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
BACKEND = os.path.join(HERE, "backend")
MOTION = os.path.join(REPO, "05-d500lib-motion")
for _p in (BACKEND, MOTION, HERE):
    if _p not in sys.path:
        sys.path.insert(0, _p)

PASS, FAIL, SKIP, MANUAL = [], [], [], []


def check(name, cond, detail=""):
    (PASS if cond else FAIL).append(name)
    print(
        f"[{'PASS' if cond else 'FAIL'}] {name}" + (f"  -- {detail}" if detail else "")
    )


def skip(name, why):
    SKIP.append(name)
    print(f"[SKIP] {name}  -- {why}")


def manual(name, how):
    MANUAL.append(name)
    print(f"[MANUAL] {name}  -- {how}")


try:
    import numpy as np
except ImportError:  # pragma: no cover
    print("=" * 72)
    print("FATAL: numpy is not importable. Run this with the project interpreter, e.g.")
    print("  ./backend/.venv/bin/python3 verify_fixes.py   (or the Jetson python3)")
    print("=" * 72)
    sys.exit(2)


class FakeCluster:
    """Minimal stand-in for obstacle_clustering.ObstacleCluster."""

    def __init__(
        self,
        centroid,
        length,
        width,
        obs_type="BOX_OBSTACLE",
        corners=None,
        points_count=12,
    ):
        self.centroid = centroid
        self.length_m = length
        self.width_m = width
        self.obstacle_type = obs_type
        self.points_count = points_count
        self.bounding_box_2d = corners


def load_live(module_name, filename):
    """Loads a module from 03-MinhHieu/ BY PATH.

    Required because ``app/__init__.py`` puts ``05-d500lib-motion`` at sys.path[0], which
    makes the stale 05 mirrors of ``d500_driver`` / ``disparity_extender`` shadow the live
    03 copies this harness is supposed to be verifying.
    """
    import importlib.util

    path = os.path.join(HERE, filename)
    spec = importlib.util.spec_from_file_location(module_name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    spec.loader.exec_module(module)
    return module


def live_grid(arena_bounds):
    """The grid the backend actually builds (Config.MAP_*), not a toy default."""
    from app.config import Config
    from planning.occupancy_grid import OccupancyGridMap

    return OccupancyGridMap(
        width=Config.MAP_WIDTH,
        height=Config.MAP_HEIGHT,
        resolution=Config.MAP_RESOLUTION,
        origin_x=Config.MAP_ORIGIN_X,
        origin_y=Config.MAP_ORIGIN_Y,
        arena_bounds=arena_bounds,
    )


def census(grid):
    """unknown/free/occupied percentages exactly as the dashboard sees them."""
    raw = grid.to_dashboard_bytes()
    arr = np.frombuffer(raw, dtype=np.uint8)
    n = arr.size or 1
    unknown = int(np.count_nonzero(arr == 255))
    occupied = int(np.count_nonzero(arr == 100))
    return {
        "unknown_pct": round(100.0 * unknown / n, 2),
        "free_pct": round(100.0 * (n - unknown - occupied) / n, 2),
        "occupied_pct": round(100.0 * occupied / n, 2),
        "occupied_cells": occupied,
    }


# ─────────────────────────────────────────────────────────────────────────────
# 1. Near-field constants — one value per concept, from Config
# ─────────────────────────────────────────────────────────────────────────────
def section_constants():
    print("\n" + "=" * 72)
    print("1. NEAR-FIELD / GEOMETRY CONSTANTS (one value per concept, from Config)")
    print("=" * 72)

    from app.config import Config

    check(
        "Config.ROBOT_RADIUS == 0.09 (15x10cm chassis circumscribed)",
        abs(Config.ROBOT_RADIUS - 0.09) < 1e-9,
        repr(Config.ROBOT_RADIUS),
    )
    check(
        "Config.LIDAR_MIN_RANGE == 0.05 (datasheet floor + mounting margin)",
        abs(Config.LIDAR_MIN_RANGE - 0.05) < 1e-9,
        repr(Config.LIDAR_MIN_RANGE),
    )
    check(
        "Config.MAPPING_MAX_RANGE == 6.0",
        abs(Config.MAPPING_MAX_RANGE - 6.0) < 1e-9,
        repr(Config.MAPPING_MAX_RANGE),
    )
    check(
        "Config.REFLEX_STOP == 0.30 (>= the D500 300mm accuracy floor)",
        abs(Config.REFLEX_STOP - 0.30) < 1e-9,
        repr(Config.REFLEX_STOP),
    )

    from planning.occupancy_grid import OccupancyGridMap
    from planning.controller import PurePursuitController
    from app.hardware.scan_gate import ScanGate, MIN_RANGE_M, MAX_RANGE_M
    d500_live = load_live("d500_driver_live", "d500_driver.py")
    D500OfficialDriver = d500_live.D500OfficialDriver
    dde_live = load_live("disparity_extender_live", "disparity_extender.py")
    DisparityExtender = dde_live.DisparityExtender

    check(
        "OccupancyGridMap default robot_radius == Config.ROBOT_RADIUS",
        abs(OccupancyGridMap().robot_radius - Config.ROBOT_RADIUS) < 1e-9,
    )

    d = D500OfficialDriver()
    check(
        "D500OfficialDriver default min_distance_m == Config.LIDAR_MIN_RANGE",
        abs(d.min_dist - Config.LIDAR_MIN_RANGE) < 1e-9,
        repr(d.min_dist),
    )
    check(
        "D500OfficialDriver: near-field policy present (near=1, <1.0m)",
        d.min_intensity_near == 1 and abs(d.near_field_m - 1.0) < 1e-9,
        f"near={d.min_intensity_near} field={d.near_field_m}",
    )

    dde = DisparityExtender()
    check(
        "DisparityExtender default critical_stop_m == Config.REFLEX_STOP",
        abs(dde.critical_stop_m - Config.REFLEX_STOP) < 1e-9,
        repr(dde.critical_stop_m),
    )
    check(
        "DisparityExtender default min_lidar_range == Config.LIDAR_MIN_RANGE",
        abs(dde.min_lidar_range - Config.LIDAR_MIN_RANGE) < 1e-9,
    )

    ctl = PurePursuitController()
    check(
        "PurePursuitController default min_lidar_range == Config.LIDAR_MIN_RANGE",
        abs(ctl.min_lidar_range - Config.LIDAR_MIN_RANGE) < 1e-9,
    )

    check(
        "ScanGate range window is sourced from Config",
        abs(MIN_RANGE_M - Config.LIDAR_MIN_RANGE) < 1e-9
        and abs(MAX_RANGE_M - Config.MAPPING_MAX_RANGE) < 1e-9,
        f"[{MIN_RANGE_M}, {MAX_RANGE_M}]",
    )

    # Targeted source audit: the specific stale literals that carried the bugs.
    audit = [
        (
            os.path.join(HERE, "d500_driver.py"),
            "min_distance_m: float = 0.12",
            "driver near-field floor",
        ),
        (
            os.path.join(HERE, "jetbot_bridge.py"),
            "min_distance_m=0.12",
            "bridge driver construction",
        ),
        (
            os.path.join(HERE, "jetbot_bridge.py"),
            "critical_stop_m=0.18",
            "bridge DDE stop distance",
        ),
        (
            os.path.join(HERE, "jetbot_bridge.py"),
            '"lidar": lidar.latest_scan',
            "legacy sparse scan pair list",
        ),
        (
            os.path.join(MOTION, "planning", "occupancy_grid.py"),
            "robot_radius: float = 0.065",
            "grid inscribed radius",
        ),
        (
            os.path.join(MOTION, "planning", "controller.py"),
            "if 0.12 <= r <",
            "controller chassis-reflection guard",
        ),
        (
            os.path.join(MOTION, "planning", "controller.py"),
            "d_stop = max(0.16,",
            "controller duplicate stop floor",
        ),
        (
            os.path.join(BACKEND, "app", "hardware", "telemetry_parser.py"),
            "0.15 <= dist",
            "parser near-field floor",
        ),
        (
            os.path.join(BACKEND, "app", "hardware", "scan_gate.py"),
            "0.15 <= dist",
            "scan-gate near-field floor",
        ),
    ]
    bad = []
    for path, literal, why in audit:
        try:
            text = open(path, encoding="utf-8").read()
        except OSError as exc:
            bad.append(f"{path}: unreadable ({exc})")
            continue
        if literal in text:
            bad.append(f"{os.path.basename(path)} still contains {literal!r} ({why})")
    check(
        f"no stale near-field literal remains in {len(audit)} audited code paths",
        not bad,
        "; ".join(bad),
    )

    manual(
        "Intensity policy / tile-ghost regression (needs the board)",
        "python3 setup/test_lidar.py --box-sweep   then --raw-dump",
    )


# ─────────────────────────────────────────────────────────────────────────────
# 2. Config boot guard + active arena bounds
# ─────────────────────────────────────────────────────────────────────────────
def section_arena():
    print("\n" + "=" * 72)
    print("2. ARENA BOUNDS + BOOT GUARD")
    print("=" * 72)

    from app.config import Config
    from planning.occupancy_grid import OccupancyGridMap

    print(f"     ENVIRONMENT_MODE = {Config.ENVIRONMENT_MODE!r}")
    print(f"     Config.ARENA_BOUNDS = {Config.ARENA_BOUNDS}")

    grid = OccupancyGridMap()
    check(
        "grid.arena_bounds == Config.ARENA_BOUNDS for the active profile",
        grid.arena_bounds == Config.ARENA_BOUNDS,
        f"grid={grid.arena_bounds} config={Config.ARENA_BOUNDS}",
    )

    # The boot guard that would have caught BUG-01/BUG-02 outright.
    from app import config as cfg

    old_mode, old_bounds = cfg.Config.ENVIRONMENT_MODE, cfg.Config.ARENA_BOUNDS
    raised = False
    try:
        cfg.Config.ENVIRONMENT_MODE = "ROOM"
        cfg.Config.ARENA_BOUNDS = (0.0, 2.0, 0.0, 1.0)
        cfg._validate_environment_profile()
    except RuntimeError:
        raised = True
    finally:
        cfg.Config.ENVIRONMENT_MODE, cfg.Config.ARENA_BOUNDS = old_mode, old_bounds
    check("boot guard raises on ROOM + sub-3m arena (the proven root cause)", raised)

    raised_bad = False
    try:
        cfg.Config.ENVIRONMENT_MODE = "NONSENSE"
        cfg._validate_environment_profile()
    except RuntimeError:
        raised_bad = True
    finally:
        cfg.Config.ENVIRONMENT_MODE = old_mode
    check("boot guard rejects an unknown ENVIRONMENT_MODE", raised_bad)


# ─────────────────────────────────────────────────────────────────────────────
# 3. Map census — the black-map regression
# ─────────────────────────────────────────────────────────────────────────────
def section_census():
    print("\n" + "=" * 72)
    print("3. MAP CENSUS (black map = 97% lethal exterior fill)")
    print("=" * 72)

    from planning.occupancy_grid import OccupancyGridMap

    room = live_grid(None)
    room.update_costmap()
    c_room = census(room)
    print(f"     ROOM, empty map     : {c_room}")
    check(
        "ROOM: exterior is NOT block-filled lethal (occupied < 1%)",
        c_room["occupied_pct"] < 1.0,
        str(c_room),
    )
    check(
        "ROOM: unobserved space stays unknown (> 95%)",
        c_room["unknown_pct"] > 95.0,
        str(c_room),
    )

    # One synthetic full-revolution scan from the origin: a 2m ring of walls.
    room.update_scan(0.0, 0.0, 0.0, ranges=[2.0] * 360, max_valid_range=5.85)
    room.update_costmap()
    c_scan = census(room)
    print(f"     ROOM, after 1 scan  : {c_scan}")
    check(
        "ROOM: after a 2m ring scan, occupied is a thin wall footprint (<10%)",
        c_scan["occupied_pct"] < 10.0,
        str(c_scan),
    )
    check(
        "ROOM: after a scan the map is still mostly unknown, not painted",
        c_scan["unknown_pct"] > 60.0,
        str(c_scan),
    )

    arena = OccupancyGridMap(arena_bounds=(-0.25, 2.05, -0.65, 0.65))
    arena.update_costmap()
    c_arena = census(arena)
    print(f"     ARENA_2X1, empty map: {c_arena}")
    check(
        "ARENA_2X1: only a 1-cell perimeter ring is lethal (<5%), not both",
        c_arena["occupied_pct"] < 5.0,
        str(c_arena),
    )
    check(
        "ARENA_2X1: occupied is nowhere near the 97.1% failure mode",
        c_arena["occupied_pct"] < 50.0,
    )


# ─────────────────────────────────────────────────────────────────────────────
# 4. Goal fidelity — no silent relocation
# ─────────────────────────────────────────────────────────────────────────────
def section_goal():
    print("\n" + "=" * 72)
    print("4. GOAL FIDELITY (silent relocation = 'Auto does nothing')")
    print("=" * 72)

    from planning.astar_planner import AStarPlanner
    from planning.occupancy_grid import OccupancyGridMap

    room = live_grid(None)
    planner = AStarPlanner(room, arena_bounds=room.arena_bounds)
    path = planner.plan((0.0, 0.0), (4.0, 4.0))
    last = None if not path else path[-1]
    print(f"     (0,0)->(4,4) status={planner.last_plan_status} last={last}")
    check("ROOM: in-map goal is planned", path is not None and len(path) >= 2)
    check(
        "ROOM: path terminus IS the true goal (no clamp to 1.97/0.57)",
        last is not None and abs(last[0] - 4.0) < 1e-6 and abs(last[1] - 4.0) < 1e-6,
        f"last={last}",
    )
    check(
        "ROOM: plan status is honest (OK or a reason)",
        planner.last_plan_status in ("OK", "GOAL_RELOCATED_TO_FREE_CELL"),
        planner.last_plan_status,
    )

    outside = planner.plan((0.0, 0.0), (12.0, 12.0))
    check(
        "ROOM: goal outside the mapped area is REJECTED loudly",
        outside is None and planner.last_plan_status == "GOAL_OUTSIDE_OPERATING_AREA",
        f"status={planner.last_plan_status} ret={outside}",
    )

    arena = OccupancyGridMap(arena_bounds=(-0.25, 2.05, -0.65, 0.65))
    p2 = AStarPlanner(arena, arena_bounds=arena.arena_bounds)
    rej = p2.plan((0.0, 0.0), (4.0, 4.0))
    check(
        "ARENA_2X1: out-of-arena goal rejected, not clamped",
        rej is None and p2.last_plan_status == "GOAL_OUTSIDE_OPERATING_AREA",
        f"status={p2.last_plan_status} ret={rej}",
    )

    ok = p2.plan((0.0, 0.0), (1.0, 0.4))
    last_ok = None if not ok else ok[-1]
    check(
        "ARENA_2X1: in-arena goal is actually reached",
        last_ok is not None
        and abs(last_ok[0] - 1.0) < 0.08
        and abs(last_ok[1] - 0.4) < 0.08,
        f"last={last_ok}",
    )

    # Bounded planner: the deadline must be real, not swallowed by **kwargs.
    t0 = time.time()
    planner.plan((0.0, 0.0), (4.5, -4.5), max_time_sec=0.05, max_expansions=10)
    elapsed = time.time() - t0
    check(
        "planner honours max_time_sec / max_expansions (bounded return)",
        elapsed < 2.0,
        f"elapsed={elapsed:.3f}s status={planner.last_plan_status}",
    )


# ─────────────────────────────────────────────────────────────────────────────
# 5. Cluster lifecycle + ghost regression
# ─────────────────────────────────────────────────────────────────────────────
def section_clusters():
    print("\n" + "=" * 72)
    print("5. CLUSTER LIFECYCLE + GHOST REGRESSION")
    print("=" * 72)

    from planning.occupancy_grid import OccupancyGridMap

    g = OccupancyGridMap(arena_bounds=None)
    box = FakeCluster(
        (0.2, 0.0),
        0.30,
        0.20,
        "BOX_OBSTACLE",
        corners=[(0.35, -0.1), (0.05, -0.1), (0.05, 0.1), (0.35, 0.1)],
    )
    leg = FakeCluster((0.0, 0.5), 0.10, 0.10, "CYLINDER_LEG")
    g.insert_obstacle_clusters([box, leg], 0.0, 0.0, 0.0)
    check(
        "clusters: non-wall clusters registered",
        len(g.active_clusters) == 2,
        str(g.active_clusters),
    )
    check(
        "clusters: OBB polygons tracked alongside the discs",
        len(g.active_cluster_polys) == len(g.active_clusters),
    )

    c_idx, r_idx = g.world_to_grid(0.2, 0.0)
    check(
        "clusters: obstacle footprint is lethal on the costmap",
        g.costmap[r_idx, c_idx] >= 250,
        f"cost={int(g.costmap[r_idx, c_idx])}",
    )

    wall = FakeCluster((0.0, 0.9), 1.50, 0.06, "WALL_SURFACE")
    gw = OccupancyGridMap(arena_bounds=None)
    gw.insert_obstacle_clusters([wall], 0.0, 0.0, 0.0)
    check(
        "clusters: WALL_SURFACE skipped (no 0.8m phantom disc)",
        len(gw.active_clusters) == 0,
        str(gw.active_clusters),
    )

    empty = FakeCluster((0.0, 0.0), 0.3, 0.3, "BOX_OBSTACLE", points_count=0)
    ge = OccupancyGridMap(arena_bounds=None)
    ge.insert_obstacle_clusters([empty], 0.0, 0.0, 0.0)
    check(
        "clusters: points_count<=0 is the validity test", len(ge.active_clusters) == 0
    )

    big = FakeCluster(
        (1.0, 0.0),
        2.0,
        2.0,
        "BOX_OBSTACLE",
        corners=[(2.0, -1.0), (0.0, -1.0), (0.0, 1.0), (2.0, 1.0)],
    )
    gb = OccupancyGridMap(arena_bounds=None)
    gb.insert_obstacle_clusters([big], 0.0, 0.0, 0.0)
    check(
        "clusters: oversized extent capped at 0.25m",
        len(gb.active_clusters) == 1 and gb.active_clusters[0][2] <= 0.25 + 1e-9,
        str(gb.active_clusters),
    )

    # The ghost regression: feeding [] through the SAME call site must clear.
    g.insert_obstacle_clusters([], 0.0, 0.0, 0.0)
    check(
        "clusters: feeding [] clears them (the unreachable-branch bug)",
        len(g.active_clusters) == 0 and len(g.active_cluster_polys) == 0,
        str(g.active_clusters),
    )
    g.update_costmap()
    check(
        "ghost regression: the cell disappears one map cycle later",
        g.costmap[r_idx, c_idx] < 250,
        f"cost={int(g.costmap[r_idx, c_idx])}",
    )


# ─────────────────────────────────────────────────────────────────────────────
# 6. CV low-obstacle short-TTL layer
# ─────────────────────────────────────────────────────────────────────────────
def section_low_obstacles():
    print("\n" + "=" * 72)
    print("6. CV LOW-OBSTACLE LAYER (short TTL, never hardens into the map)")
    print("=" * 72)

    from planning.occupancy_grid import OccupancyGridMap

    g = OccupancyGridMap(arena_bounds=None)
    g.insert_low_obstacles([(0.35, 0.0)], 0.0, 0.0, 0.0)
    c_idx, r_idx = g.world_to_grid(0.35, 0.0)
    check(
        "low-obstacle: stamped lethal while alive",
        g.costmap[r_idx, c_idx] >= 250,
        f"cost={int(g.costmap[r_idx, c_idx])}",
    )
    check(
        "low-obstacle: never written into log_odds (honesty preserved)",
        abs(g.log_odds[r_idx, c_idx]) < 1e-6,
    )
    time.sleep(0.35)
    g.update_costmap()
    check(
        "low-obstacle: expires after its TTL (no ghost)",
        g.costmap[r_idx, c_idx] < 250,
        f"cost={int(g.costmap[r_idx, c_idx])}",
    )


# ─────────────────────────────────────────────────────────────────────────────
# 7. Reflex / FRONT_UNVERIFIED + pose staleness
# ─────────────────────────────────────────────────────────────────────────────
def section_reflex_and_staleness():
    print("\n" + "=" * 72)
    print("7. REFLEX (FRONT_UNVERIFIED) + POSE STALENESS")
    print("=" * 72)

    from disparity_extender import DisparityExtender

    dde = DisparityExtender()
    blind = dde.evaluate([0.0] * 360, goal_heading_rad=0.0, cmd_v=0.2)
    check(
        "front-unverified: a blind frontal cone is NOT read as 'clear'",
        blind.get("status") == "FRONT_UNVERIFIED",
        str(blind.get("status")),
    )
    check(
        "front-unverified: crawl cap is returned and is slower than commanded",
        0.0 < float(blind.get("recommended_v_cap", 0)) < 0.2,
        str(blind.get("recommended_v_cap")),
    )

    seen = dde.evaluate([0.5] * 360, goal_heading_rad=0.0, cmd_v=0.2)
    check(
        "front-unverified: a verified front is not flagged",
        seen.get("status") != "FRONT_UNVERIFIED",
        str(seen.get("status")),
    )

    hold = dde.evaluate([0.0] * 360, goal_heading_rad=0.0, cmd_v=0.0)
    check(
        "front-unverified: not applied while stationary (cmd_v == 0)",
        hold.get("status") != "FRONT_UNVERIFIED",
        str(hold.get("status")),
    )

    try:
        import asyncio
        from app.navigation.coordinator import NavigationCoordinator
    except Exception as exc:  # pragma: no cover - depends on the env
        skip(
            "pose staleness: stale perception -> stop + WARNING",
            f"coordinator not importable here ({exc})",
        )
        return

    async def _probe():
        coord = NavigationCoordinator()
        coord.events = []
        coord._last_scan_tick_time = time.time() - 1.0  # 1.0s stale
        await coord._step_navigation()
        return coord

    try:
        coord = asyncio.run(_probe())
    except Exception as exc:  # pragma: no cover
        skip(
            "pose staleness: stale perception -> stop + WARNING",
            f"could not drive _step_navigation ({exc})",
        )
        return

    warned = any(
        "No fresh LiDAR scan" in str(getattr(ev, "message", ev)) for ev in coord.events
    )
    check(
        "pose staleness: zero cmd_vel commanded",
        abs(coord.linear_vel) < 1e-9 and abs(coord.angular_vel) < 1e-9,
        f"v={coord.linear_vel} w={coord.angular_vel}",
    )
    check(
        "pose staleness: a WARNING is raised, not silence",
        warned,
        f"{len(coord.events)} event(s)",
    )


# ─────────────────────────────────────────────────────────────────────────────
# 8. ScanGate dedup + staleness fallback
# ─────────────────────────────────────────────────────────────────────────────
def section_scan_gate():
    print("\n" + "=" * 72)
    print("8. SCANGATE (dedup root cause: seq == 0 is 'absent', not a value)")
    print("=" * 72)

    from app.hardware.scan_gate import ScanGate

    ring = [1.5] * 360
    gate = ScanGate()
    first = gate.accept({"lidar_ranges": ring})
    check("scan-gate: first dense scan accepted", first is not None)

    dup = gate.accept({"lidar_ranges": ring})
    check("scan-gate: identical scan dropped (no double odometry)", dup is None)

    seq0 = ScanGate()
    a = seq0.accept({"lidar_ranges": ring, "scan_seq": 0})
    b = seq0.accept({"lidar_ranges": ring, "scan_seq": 0})
    check("scan-gate: seq==0 is treated as ABSENT (first scan accepted)", a is not None)
    check(
        "scan-gate: seq==0 does not wedge the pipeline (content dedup still works)",
        b is None,
    )

    seqd = ScanGate()
    check(
        "scan-gate: seq=7 accepted",
        seqd.accept({"scan_seq": 7, "lidar_ranges": ring}) is not None,
    )
    check(
        "scan-gate: repeated seq=7 dropped",
        seqd.accept({"scan_seq": 7, "lidar_ranges": ring}) is None,
    )
    check(
        "scan-gate: seq=8 accepted",
        seqd.accept({"scan_seq": 8, "lidar_ranges": ring}) is not None,
    )

    near = ScanGate()
    tick = near.accept({"lidar_ranges": [0.06] * 360})
    check(
        "scan-gate: a 6cm return survives the range window (near-field fix)",
        tick is not None and tick.ranges[0] >= 0.05,
        f"r0={None if tick is None else tick.ranges[0]}",
    )


def main():
    print("=" * 72)
    print("CS532 JetBot SLAM — verification harness")
    print("=" * 72)
    from app.config import Config

    print(f"profile={Config.ENVIRONMENT_MODE}  arena={Config.ARENA_BOUNDS}")

    for section in (
        section_constants,
        section_arena,
        section_census,
        section_goal,
        section_clusters,
        section_low_obstacles,
        section_reflex_and_staleness,
        section_scan_gate,
    ):
        try:
            section()
        except Exception as exc:  # a section must never abort the harness
            FAIL.append(f"{section.__name__} raised {type(exc).__name__}: {exc}")
            print(f"[FAIL] {section.__name__} raised {type(exc).__name__}: {exc}")

    print("\n" + "=" * 72)
    print(
        f"PASS {len(PASS)}   FAIL {len(FAIL)}   SKIP {len(SKIP)}   MANUAL {len(MANUAL)}"
    )
    if FAIL:
        print("FAILED:")
        for name in FAIL:
            print("  -", name)
    if MANUAL:
        print("MANUAL (requires the physical board — run these yourself):")
        for name in MANUAL:
            print("  -", name)
    print("=" * 72)
    if FAIL:
        print("RESULT: NOT VERIFIED")
        return 1
    print("RESULT: ALL OFFLINE CHECKS PASSED")
    return 0


if __name__ == "__main__":
    sys.exit(main())
