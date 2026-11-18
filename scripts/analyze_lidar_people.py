#!/usr/bin/env python3
"""
scripts/analyze_lidar_people.py — Analyze pedestrian characteristics in LiDAR sweeps for TK-MEAS-01.

Reads .npz recordings for scenarios:
  1. static_person (human standing stationary)
  2. crossing (human walking perpendicular to robot heading)
  3. head_on (human walking directly towards robot)

For each recording, computes:
  - Number of person-like clusters per scan (mean, min, max, p50, p95)
  - Estimated speed (m/s) and range (m) of person clusters
  - Wall-point count and density (ratio of wall points to valid returns)
  - Valid beam ratio (percentage of 360 beams returning valid range)

Outputs raw tables and summary into analysis.txt.
Explicit constraint: Makes NO claim about SLAM pose (reserved for TK-CV-02).
"""

import argparse
import datetime
import glob
import math
import os
import re
import sys
from typing import Dict, Any, List, Optional, Tuple

import numpy as np

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
SCENARIOS = ["static_person", "crossing", "head_on"]


def find_latest_measurement_dir() -> Optional[str]:
    dirs = sorted(
        d for d in glob.glob(os.path.join(ROOT, "docs", "measurements", "*"))
        if re.search(r"\d{4}-\d{2}-\d{2}$", d) and os.path.isdir(d)
    )
    return dirs[-1] if dirs else None


def cluster_scan_1d(ranges: np.ndarray, angles_rad: np.ndarray) -> List[Dict[str, Any]]:
    """
    1D radial range-jump clustering for 360-degree LiDAR scan.
    Returns list of clusters with geometric and classification attributes.
    """
    valid_idx = np.where((ranges >= 0.12) & (ranges <= 6.0) & np.isfinite(ranges))[0]
    if len(valid_idx) == 0:
        return []

    clusters_indices = []
    current_cluster = [valid_idx[0]]

    for i in range(1, len(valid_idx)):
        idx_prev = valid_idx[i - 1]
        idx_curr = valid_idx[i]

        r_prev = ranges[idx_prev]
        r_curr = ranges[idx_curr]
        d_theta = (idx_curr - idx_prev) * (2 * math.pi / 360.0)

        # Distance between adjacent rays (Law of Cosines)
        dist_sq = r_prev * r_prev + r_curr * r_curr - 2 * r_prev * r_curr * math.cos(d_theta)
        dist = math.sqrt(max(0.0, dist_sq))

        # Adaptive tolerance threshold eps(r)
        eps = max(0.12, r_curr * math.tan(math.radians(1.0)) + 0.05)
        # If angle gap is large (e.g. > 3 degrees), break cluster
        if dist <= eps and (idx_curr - idx_prev) <= 3:
            current_cluster.append(idx_curr)
        else:
            clusters_indices.append(current_cluster)
            current_cluster = [idx_curr]

    if current_cluster:
        clusters_indices.append(current_cluster)

    # Check wrap-around cluster between 359° and 0°
    if len(clusters_indices) > 1:
        first = clusters_indices[0]
        last = clusters_indices[-1]
        if first[0] == 0 and last[-1] == 359:
            r0 = ranges[0]
            r359 = ranges[359]
            d_theta = 2 * math.pi / 360.0
            dist_sq = r0 * r0 + r359 * r359 - 2 * r0 * r359 * math.cos(d_theta)
            dist = math.sqrt(max(0.0, dist_sq))
            if dist <= max(0.12, r0 * math.tan(math.radians(1.0)) + 0.05):
                first.extend(last)
                clusters_indices.pop()

    cluster_objects = []
    for c_idx in clusters_indices:
        pts_r = ranges[c_idx]
        pts_th = angles_rad[c_idx]
        xs = pts_r * np.cos(pts_th)
        ys = pts_r * np.sin(pts_th)

        cx = float(np.mean(xs))
        cy = float(np.mean(ys))
        r_dist = float(math.hypot(cx, cy))
        count = len(c_idx)

        # Geometry
        pts = np.column_stack((xs, ys))
        if count >= 3:
            centered = pts - np.array([cx, cy])
            cov = np.cov(centered, rowvar=False)
            eigvals, eigvecs = np.linalg.eigh(cov)
            sort_idx = np.argsort(eigvals)[::-1]
            eigvals = eigvals[sort_idx]
            eigvecs = eigvecs[:, sort_idx]

            proj = np.dot(centered, eigvecs)
            length_m = float(np.ptp(proj[:, 0]))
            width_m = float(np.ptp(proj[:, 1]))
            l1, l2 = max(1e-5, eigvals[0]), max(1e-5, eigvals[1])
            linearity = float((l1 - l2) / (l1 + l2))
        else:
            length_m = float(np.ptp(xs)) if count > 1 else 0.04
            width_m = float(np.ptp(ys)) if count > 1 else 0.04
            linearity = 0.5

        # Classification (aligned with obstacle_clustering.py Phase 3.5 standard)
        is_thin = min(length_m, width_m) <= 0.18
        is_long = max(length_m, width_m) >= 0.60
        is_wall = (is_long and is_thin and linearity > 0.85) or (length_m > 1.0)

        # Person-like cluster: human leg or torso signature
        # Typical 2D leg/torso: length 0.08m - 0.45m, width 0.08m - 0.45m, not a long wall
        is_person_like = (
            not is_wall
            and (0.05 <= max(length_m, width_m) <= 0.55)
            and (count >= 2)
            and (0.25 <= r_dist <= 5.0)
        )

        cluster_objects.append({
            "indices": c_idx,
            "centroid": (cx, cy),
            "range": r_dist,
            "count": count,
            "length_m": max(0.04, length_m),
            "width_m": max(0.04, width_m),
            "is_wall": is_wall,
            "is_person_like": is_person_like,
        })

    return cluster_objects


def analyze_recording(npz_path: str, scenario: str) -> Dict[str, Any]:
    data = np.load(npz_path)
    ranges = data["ranges"].astype(np.float32)  # (N, 360)
    t = data["t"].astype(np.float64)            # (N,)

    n_scans = ranges.shape[0]
    duration_s = float(t[-1] - t[0]) if n_scans > 1 else 0.0
    hz = (n_scans - 1) / max(0.001, duration_s)

    angles_rad = np.linspace(0, 2 * math.pi, 360, endpoint=False)

    person_cluster_counts = []
    person_ranges = []
    person_speeds = []
    wall_point_counts = []
    wall_point_ratios = []
    valid_beam_ratios = []

    prev_person_clusters = []
    prev_t = None

    for i in range(n_scans):
        scan_r = ranges[i]
        curr_t = t[i]
        clusters = cluster_scan_1d(scan_r, angles_rad)

        valid_beams = int(np.sum((scan_r >= 0.12) & (scan_r <= 6.0) & np.isfinite(scan_r)))
        valid_beam_ratios.append(valid_beams / 360.0)

        person_cls = [c for c in clusters if c["is_person_like"]]
        wall_cls = [c for c in clusters if c["is_wall"]]

        person_cluster_counts.append(len(person_cls))
        for pc in person_cls:
            person_ranges.append(pc["range"])

        wall_pts = sum(c["count"] for c in wall_cls)
        wall_point_counts.append(wall_pts)
        wall_point_ratios.append(wall_pts / max(1, valid_beams))

        # Speed estimation via nearest centroid tracking between scans
        if prev_person_clusters and prev_t is not None and (curr_t - prev_t) > 0.02:
            dt = curr_t - prev_t
            if dt < 0.5:  # Valid inter-scan interval
                for curr_p in person_cls:
                    cx, cy = curr_p["centroid"]
                    # Find closest cluster in previous scan within 0.6m
                    dists = [
                        math.hypot(cx - prev_p["centroid"][0], cy - prev_p["centroid"][1])
                        for prev_p in prev_person_clusters
                    ]
                    if dists:
                        min_d = min(dists)
                        if min_d <= 0.6:  # Max 0.6m displacement between consecutive frames
                            speed = min_d / dt
                            person_speeds.append(speed)

        prev_person_clusters = person_cls
        prev_t = curr_t

    def stats(arr):
        if not arr:
            return {"mean": 0.0, "min": 0.0, "max": 0.0, "p50": 0.0, "p95": 0.0}
        a = np.array(arr)
        return {
            "mean": float(np.mean(a)),
            "min": float(np.min(a)),
            "max": float(np.max(a)),
            "p50": float(np.percentile(a, 50)),
            "p95": float(np.percentile(a, 95)),
        }

    return {
        "scenario": scenario,
        "file": os.path.basename(npz_path),
        "n_scans": n_scans,
        "duration_s": round(duration_s, 2),
        "mean_hz": round(hz, 2),
        "person_clusters_per_scan": stats(person_cluster_counts),
        "person_range_m": stats(person_ranges),
        "person_speed_mps": stats(person_speeds),
        "wall_points_per_scan": stats(wall_point_counts),
        "wall_point_ratio": stats(wall_point_ratios),
        "valid_beam_ratio": stats(valid_beam_ratios),
    }


def format_report(results: List[Dict[str, Any]], unmeasured: List[str]) -> str:
    lines = []
    lines.append("=" * 80)
    lines.append("TK-MEAS-01: LiDAR Pedestrian Scan Characteristics Analysis")
    lines.append("Generated: %s" % datetime.datetime.now().isoformat())
    lines.append("=" * 80)
    lines.append("")
    lines.append("IMPORTANT GOVERNANCE INVARIANT:")
    lines.append("  This analysis measures raw LiDAR cluster and wall properties on the stationary robot.")
    lines.append("  It makes NO claim about SLAM pose estimation or trajectory accuracy.")
    lines.append("  (SLAM pose ablation with pedestrian beam-masking is strictly reserved for TK-CV-02).")
    lines.append("")

    if unmeasured:
        lines.append("-" * 80)
        lines.append("UNMEASURED SCENARIOS (CHƯA ĐO):")
        for sc in unmeasured:
            lines.append("  - %s: CHƯA ĐO (chưa có tệp .npz thực tế từ robot)" % sc)
        lines.append("-" * 80)
        lines.append("")

    for r in results:
        lines.append("-" * 80)
        lines.append(f"Scenario: {r['scenario']} ({r['file']})")
        lines.append(f"Scans: {r['n_scans']} | Duration: {r['duration_s']} s | Frequency: {r['mean_hz']} Hz")
        lines.append("-" * 80)

        pc = r["person_clusters_per_scan"]
        lines.append(f"Person-like clusters / scan:  mean={pc['mean']:.2f} (p50={pc['p50']:.1f}, p95={pc['p95']:.1f}, min={pc['min']:.0f}, max={pc['max']:.0f})")

        pr = r["person_range_m"]
        lines.append(f"Person range (m):             mean={pr['mean']:.2f} m (p50={pr['p50']:.2f} m, p95={pr['p95']:.2f} m, min={pr['min']:.2f} m, max={pr['max']:.2f} m)")

        ps = r["person_speed_mps"]
        lines.append(f"Estimated speed (m/s):        mean={ps['mean']:.2f} m/s (p50={ps['p50']:.2f} m/s, p95={ps['p95']:.2f} m/s, max={ps['max']:.2f} m/s)")

        wp = r["wall_points_per_scan"]
        wr = r["wall_point_ratio"]
        lines.append(f"Wall points / scan:           mean={wp['mean']:.1f} pts (ratio to valid: {wr['mean']*100:.1f}%)")

        vb = r["valid_beam_ratio"]
        lines.append(f"Valid beam ratio / scan:      mean={vb['mean']*100:.1f}% (360-ray ring)")
        lines.append("")

    if results:
        lines.append("=" * 80)
        lines.append("AGGREGATE MEASURED METRICS (COMPUTED):")
        mean_clusters = float(np.mean([r["person_clusters_per_scan"]["mean"] for r in results]))
        mean_wall_ratio = float(np.mean([r["wall_point_ratio"]["mean"] for r in results]))
        lines.append(f"  - Mean pedestrian clusters per scan across scenarios: {mean_clusters:.2f}")
        lines.append(f"  - Mean wall points ratio across scenarios: {mean_wall_ratio*100:.1f}%")
        lines.append("=" * 80)
    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser(description="Analyze pedestrian characteristics in LiDAR recordings")
    parser.add_argument("--dir", type=str, default=None, help="Target measurement directory (default: latest)")
    parser.add_argument("--output", type=str, default=None, help="Output analysis.txt path")
    args = parser.parse_args()

    if args.dir:
        meas_dir = os.path.abspath(args.dir)
    else:
        meas_dir = find_latest_measurement_dir()
        if not meas_dir:
            today_str = datetime.date.today().isoformat()
            meas_dir = os.path.join(ROOT, "docs", "measurements", today_str)

    os.makedirs(meas_dir, exist_ok=True)
    out_file = args.output if args.output else os.path.join(meas_dir, "analysis.txt")

    results = []
    unmeasured = []

    for sc in SCENARIOS:
        candidates = glob.glob(os.path.join(meas_dir, f"*{sc}*.npz"))
        if not candidates:
            unmeasured.append(sc)
            continue
        try:
            res = analyze_recording(candidates[0], sc)
            results.append(res)
        except Exception as e:
            print(f"[ERROR] Failed to analyze {candidates[0]}: {e}", file=sys.stderr)
            unmeasured.append(f"{sc} (error: {e})")

    report_text = format_report(results, unmeasured)
    with open(out_file, "w", encoding="utf-8") as f:
        f.write(report_text)

    print(report_text)
    print(f"\nReport written to: {out_file}\n")


if __name__ == "__main__":
    main()
