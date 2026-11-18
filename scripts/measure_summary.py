#!/usr/bin/env python3
"""
scripts/measure_summary.py — Compute results.json for TK-MEAS-01.

Reads raw measurement files in docs/measurements/<YYYY-MM-DD>/ and computes results.json
according to the locked schema of tests/acceptance/test_measurements_valid.py:
  - ping: {file, n_sent, n_lost, mean_ms, p95_ms, p99_ms}
  - lidar: [{file, scenario, n_scans, duration_s}, ...]
  - jetson_inventory: {file}

It computes every number directly from the raw files; it never hardcodes or estimates.
Any item without raw measurements is marked "CHƯA ĐO".
"""

import argparse
import datetime
import glob
import json
import os
import re
import sys
from typing import Dict, Any, Optional, List

import numpy as np

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
SCENARIOS = ["static_person", "crossing", "head_on"]


def find_latest_measurement_dir() -> Optional[str]:
    dirs = sorted(
        d for d in glob.glob(os.path.join(ROOT, "docs", "measurements", "*"))
        if re.search(r"\d{4}-\d{2}-\d{2}$", d) and os.path.isdir(d)
    )
    return dirs[-1] if dirs else None


def compute_ping(meas_dir: str) -> Dict[str, Any]:
    target_name = "ping_home_wifi.txt"
    ping_files = glob.glob(os.path.join(meas_dir, "*ping*.txt"))
    if not ping_files:
        return {"file": target_name, "status": "CHƯA ĐO"}

    ping_path = ping_files[0]
    fname = os.path.basename(ping_path)
    text = open(ping_path, encoding="utf-8", errors="replace").read()

    samples = [float(x) for x in re.findall(r"time[=<]([\d.]+)\s*ms", text)]
    m = re.search(r"(\d+) packets transmitted, (\d+) (?:packets )?received", text)
    if not m or not samples:
        return {"file": fname, "status": "CHƯA ĐO (ping output invalid/incomplete)"}

    sent, got = int(m.group(1)), int(m.group(2))
    n_lost = sent - got
    mean_ms = float(np.mean(samples))
    p95_ms = float(np.percentile(samples, 95))
    p99_ms = float(np.percentile(samples, 99))

    return {
        "file": fname,
        "n_sent": sent,
        "n_lost": n_lost,
        "mean_ms": round(mean_ms, 3),
        "p95_ms": round(p95_ms, 3),
        "p99_ms": round(p99_ms, 3),
    }


def compute_inventory(meas_dir: str) -> Dict[str, Any]:
    target_name = "jetson_inventory.txt"
    inv_files = glob.glob(os.path.join(meas_dir, "*inventory*.txt"))
    if not inv_files:
        return {"file": target_name, "status": "CHƯA ĐO"}

    inv_path = inv_files[0]
    if os.path.getsize(inv_path) == 0:
        return {"file": os.path.basename(inv_path), "status": "CHƯA ĐO (inventory file is empty)"}

    return {"file": os.path.basename(inv_path)}


def compute_lidar(meas_dir: str) -> List[Dict[str, Any]]:
    entries = []
    for sc in SCENARIOS:
        target_name = f"lidar_{sc}.npz"
        candidates = glob.glob(os.path.join(meas_dir, f"*{sc}*.npz"))
        if not candidates:
            entries.append({
                "file": target_name,
                "scenario": sc,
                "status": "CHƯA ĐO",
            })
            continue

        npz_path = candidates[0]
        try:
            data = np.load(npz_path)
            if "ranges" not in data or "t" not in data:
                entries.append({
                    "file": os.path.basename(npz_path),
                    "scenario": sc,
                    "status": "CHƯA ĐO (missing keys t/ranges)",
                })
                continue

            n_scans = int(data["ranges"].shape[0])
            dur = float(data["t"][-1] - data["t"][0])
            entries.append({
                "file": os.path.basename(npz_path),
                "scenario": sc,
                "n_scans": n_scans,
                "duration_s": round(dur, 3),
            })
        except Exception as e:
            entries.append({
                "file": os.path.basename(npz_path),
                "scenario": sc,
                "status": f"CHƯA ĐO (corrupted: {e})",
            })

    return entries


def main():
    parser = argparse.ArgumentParser(description="Generate results.json for TK-MEAS-01")
    parser.add_argument("--dir", type=str, default=None, help="Target measurement directory (default: latest)")
    args = parser.parse_args()

    if args.dir:
        meas_dir = os.path.abspath(args.dir)
    else:
        meas_dir = find_latest_measurement_dir()
        if not meas_dir:
            today_str = datetime.date.today().isoformat()
            meas_dir = os.path.join(ROOT, "docs", "measurements", today_str)

    os.makedirs(meas_dir, exist_ok=True)
    print(f"=== TK-MEAS-01 Measurement Summary ===")
    print(f"Target directory: {meas_dir}\n")

    results: Dict[str, Any] = {}

    # 1. Ping
    ping_res = compute_ping(meas_dir)
    results["ping"] = ping_res
    print(f"[M1 Ping] {ping_res if isinstance(ping_res, str) else json.dumps(ping_res)}")

    # 2. Inventory
    inv_res = compute_inventory(meas_dir)
    results["jetson_inventory"] = inv_res
    print(f"[M2 Inventory] {inv_res if isinstance(inv_res, str) else json.dumps(inv_res)}")

    # 3. LiDAR recordings
    lidar_res = compute_lidar(meas_dir)
    results["lidar"] = lidar_res
    print(f"[M4 LiDAR] {lidar_res if isinstance(lidar_res, str) else json.dumps(lidar_res)}")

    out_file = os.path.join(meas_dir, "results.json")
    with open(out_file, "w", encoding="utf-8") as f:
        json.dump(results, f, indent=2, ensure_ascii=False)

    print(f"\nWritten: {out_file}\n")


if __name__ == "__main__":
    main()
