# TK-CV-02B: CSM Person-Beam Masking Ablation (2026-11-28)

- **Implementation tested:** `CSMTracker.update(ranges, person_mask=...)` (`motion/planning/slam/csm_tracker.py`) and `Lidar2DOdometry.update(..., person_mask=...)` (`motion/planning/lidar_odometry.py`).
- **Recordings:** stationary JetBot with a person in the room from `docs/measurements/2026-11-16/lidar_{crossing,head_on,static_person}.npz` (ground truth: stationary robot `(0, 0, 0)`).
- **Mask provenance (important):** Synchronized camera+YOLO+LiDAR recordings are pending the field session (`TK-CV-04`). For this offline ablation on LiDAR-only recordings, a transparent **LiDAR foreground-difference proxy mask** was computed (`bg_90pct - r >= 0.25 m`, `r <= 3.5 m`, `cluster_lidar_360` with `width <= 1.2 m`). Once the synchronized field session is recorded, re-run with real `PersonLidarFuser` masks.

## Results (Mask OFF vs Mask ON on identical sweeps)

| Recording (sweeps) | Mode | Max pos drift (cm) | RMS pos drift (cm) | Max yaw err (deg) | Mean match score | WEAK / LOST | Mean masked rays | Median step (ms) |
|---|---|---:|---:|---:|---:|---:|---:|---:|
| `lidar_crossing.npz` (603) | **OFF** | 2.00 | 0.92 | 1.80 | 0.862 | 0 / 0 | 0.0 (0 sweeps) | 8.08 |
| `lidar_crossing.npz` (603) | **ON** | 2.24 | 1.15 | 1.78 | 0.880 | 0 / 0 | 7.5 (530 sweeps) | 8.09 |
| `lidar_head_on.npz` (598) | **OFF** | 2.24 | 1.26 | 0.54 | 0.854 | 0 / 0 | 0.0 (0 sweeps) | 8.01 |
| `lidar_head_on.npz` (598) | **ON** | 2.24 | 1.35 | 1.24 | 0.875 | 0 / 0 | 8.9 (543 sweeps) | 7.99 |
| `lidar_static_person.npz` (702) | **OFF** | 3.16 | 1.52 | 1.51 | 0.873 | 0 / 0 | 0.0 (0 sweeps) | 8.16 |
| `lidar_static_person.npz` (702) | **ON** | 3.16 | 1.57 | 1.26 | 0.873 | 0 / 0 | 0.4 (112 sweeps) | 8.50 |
