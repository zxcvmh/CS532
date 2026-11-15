---
trigger: glob
globs: "motion/planning/slam/**, cartographer/**, dashboard/backend/app/navigation/cartographer_adapter.py, tests/test_hector_anchor_slam.py"
description: "SLAM, scan matching and costmap rules."
---

# SLAM rules

- Two SLAM paths exist: Cartographer 2D (`cartographer/`, wrapped by
  `dashboard/backend/app/navigation/cartographer_adapter.py`) and Hector + anchor/dynamic layers
  (`motion/planning/slam/`). Before editing, write down which one the runtime actually calls from
  `dashboard/backend/app/navigation/motion_adapter.py`.
- Known break (2026-11-15): `MotionAdapter` passes `arena_bounds=` to `CartographerAdapter`, whose
  `__init__` does not accept it, so the backend cannot start. Fix the interface on one side only.
- The static map (walls) and the dynamic layer (people, short TTL) must stay separate. A moving
  person must never be burned into the static map; a wall must never decay.
- Acceptance numbers: pose drift after an in-place 360° turn ≤ 5 cm and ≤ 3°; map update must not
  wipe existing walls; report scan-matching time per scan (ms).
- LiDAR yaw offset comes from `lidar_yaw_offset_deg` in `setup/robot_dynamics.json`. Never add
  another offset constant.
