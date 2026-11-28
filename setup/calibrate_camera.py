#!/usr/bin/env python3
"""
setup/calibrate_camera.py — Laptop-side checkerboard camera calibration tool (TK-CV-03).

Two subcommands:
  1. capture   : Fetch JPEG snapshots from the JetBot HTTP bridge (/snapshot) and check
                 live checkerboard detection while saving poses for calibration.
  2. calibrate : Run cv2.calibrateCamera on saved checkerboard images, validate
                 reprojection error, and export setup/camera_intrinsics.json.
"""

import argparse
import datetime
import glob
import json
import math
import os
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np


def _find_corners(
    gray: np.ndarray, cols: int, rows: int
) -> Tuple[bool, Optional[np.ndarray]]:
    """Detect and sub-pixel refine inner checkerboard corners."""
    pattern_size = (cols, rows)
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
    found, corners = cv2.findChessboardCorners(gray, pattern_size, flags)
    if not found or corners is None:
        return False, None
    criteria = (
        cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
        30,
        1e-4,
    )
    refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
    return True, refined


def calibrate_from_images(
    image_paths: Sequence[str],
    cols: int = 9,
    rows: int = 6,
    square_mm: float = 25.0,
) -> Dict[str, Any]:
    """Calibrate pinhole camera intrinsics from a sequence of checkerboard image paths."""
    if not image_paths:
        raise ValueError("No images provided for calibration.")

    objp = np.zeros((rows * cols, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * float(square_mm)

    objpoints: List[np.ndarray] = []
    imgpoints: List[np.ndarray] = []
    used_files: List[str] = []
    image_size: Optional[Tuple[int, int]] = None

    for path in sorted(image_paths):
        img = cv2.imread(str(path), cv2.IMREAD_COLOR)
        if img is None:
            continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        h, w = gray.shape[:2]
        if image_size is None:
            image_size = (w, h)
        elif image_size != (w, h):
            raise ValueError(
                f"Inconsistent image dimensions in {path}: {(w, h)} != {image_size}"
            )
        found, corners = _find_corners(gray, cols, rows)
        if found and corners is not None:
            objpoints.append(objp)
            imgpoints.append(corners)
            used_files.append(str(path))

    if image_size is None or not objpoints:
        raise RuntimeError(
            f"No valid {cols}x{rows} checkerboard corners detected in {len(image_paths)} images."
        )

    w, h = image_size
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, image_size, None, None
    )
    if not ret:
        raise RuntimeError("cv2.calibrateCamera failed to converge.")

    point_errors: List[float] = []
    view_errors: List[float] = []
    for objp_i, imgp_i, rvec, tvec in zip(objpoints, imgpoints, rvecs, tvecs):
        proj, _ = cv2.projectPoints(objp_i, rvec, tvec, mtx, dist)
        diff = imgp_i.reshape(-1, 2) - proj.reshape(-1, 2)
        dists = np.linalg.norm(diff, axis=1)
        point_errors.extend(dists.tolist())
        view_errors.append(float(np.mean(dists)))

    mean_err = float(np.mean(point_errors))
    max_err = float(np.max(view_errors))

    fx = float(mtx[0, 0])
    fy = float(mtx[1, 1])
    cx = float(mtx[0, 2])
    cy = float(mtx[1, 2])
    hfov_deg = 2.0 * math.degrees(math.atan(w / (2.0 * fx)))
    vfov_deg = 2.0 * math.degrees(math.atan(h / (2.0 * fy)))

    return {
        "image_width": int(w),
        "image_height": int(h),
        "camera_matrix": [[float(v) for v in row] for row in mtx.tolist()],
        "dist_coeffs": [float(v) for v in dist.ravel().tolist()],
        "fx": fx,
        "fy": fy,
        "cx": cx,
        "cy": cy,
        "hfov_deg": round(hfov_deg, 4),
        "vfov_deg": round(vfov_deg, 4),
        "mean_reprojection_error_px": round(mean_err, 6),
        "max_reprojection_error_px": round(max_err, 6),
        "num_images_used": len(used_files),
        "num_images_total": len(image_paths),
        "checkerboard": {
            "cols": int(cols),
            "rows": int(rows),
            "square_mm": float(square_mm),
        },
        "calibrated_at": datetime.datetime.now(datetime.timezone.utc).isoformat(),
    }


def _fetch_snapshot(url: str, timeout: float) -> bytes:
    req = urllib.request.Request(url, headers={"User-Agent": "cs532-calib/1.0"})
    with urllib.request.urlopen(req, timeout=timeout) as resp:
        return resp.read()


def cmd_capture(args: argparse.Namespace) -> int:
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    saved = 0
    interactive = args.interval is None or args.interval <= 0.0
    print(
        f"[CAPTURE] Source={args.url} | Target={args.count} poses | "
        f"Pattern={args.cols}x{args.rows} | Out={out_dir}"
    )

    while saved < args.count:
        if interactive:
            try:
                user_in = input(
                    f"Pose {saved + 1}/{args.count}: press [Enter] to snap (or 'q' to finish): "
                )
            except EOFError:
                break
            if user_in.strip().lower() == "q":
                break
        else:
            if saved > 0:
                time.sleep(args.interval)

        try:
            raw = _fetch_snapshot(args.url, timeout=args.timeout)
        except (urllib.error.URLError, TimeoutError, OSError) as exc:
            print(f"[WARN] Snapshot fetch failed: {exc}", file=sys.stderr)
            if not interactive:
                time.sleep(0.5)
            continue

        arr = np.frombuffer(raw, dtype=np.uint8)
        img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if img is None:
            print("[WARN] Failed to decode JPEG snapshot.", file=sys.stderr)
            continue

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        found, _ = _find_corners(gray, args.cols, args.rows)
        status = "CORNERS_OK" if found else "NO_CORNERS"
        if not found and args.require_corners:
            print(f"[SKIP] {status} — adjust checkerboard pose and retry.")
            continue

        fname = out_dir / f"pose_{saved:03d}.jpg"
        fname.write_bytes(raw)
        saved += 1
        print(f"[SAVED {saved}/{args.count}] {fname.name} ({img.shape[1]}x{img.shape[0]}) [{status}]")

    print(f"[DONE] Saved {saved} snapshots to {out_dir}")
    return 0 if saved > 0 else 1


def cmd_calibrate(args: argparse.Namespace) -> int:
    patterns = ("*.jpg", "*.jpeg", "*.png", "*.bmp")
    image_paths: List[str] = []
    for pat in patterns:
        image_paths.extend(glob.glob(os.path.join(args.images, pat)))
    image_paths = sorted(set(image_paths))

    if not image_paths:
        print(f"[FAIL] No images found in {args.images}", file=sys.stderr)
        return 1

    result = calibrate_from_images(
        image_paths,
        cols=args.cols,
        rows=args.rows,
        square_mm=args.square_mm,
    )

    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")

    used = result["num_images_used"]
    mean_err = result["mean_reprojection_error_px"]
    print(
        f"[CALIB] used={used}/{result['num_images_total']} | "
        f"size={result['image_width']}x{result['image_height']} | "
        f"fx={result['fx']:.2f} fy={result['fy']:.2f} "
        f"cx={result['cx']:.2f} cy={result['cy']:.2f} | "
        f"hfov={result['hfov_deg']:.2f} deg | "
        f"mean_err={mean_err:.4f} px (max={result['max_reprojection_error_px']:.4f} px)"
    )
    print(f"[CALIB] Wrote {out_path}")

    ok = (used >= args.min_images) and (mean_err <= args.max_error_px)
    if not ok:
        print(
            f"[FAIL] Threshold missed: need used >= {args.min_images} (got {used}) "
            f"and mean_err <= {args.max_error_px:.3f} px (got {mean_err:.4f} px)",
            file=sys.stderr,
        )
        return 0 if args.allow_fail else 1

    print("[PASS] Calibration satisfies TK-CV-03 thresholds.")
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Checkerboard camera calibration utility for JetBot CSI camera (TK-CV-03)."
    )
    sub = parser.add_subparsers(dest="command", required=True)

    p_cap = sub.add_parser("capture", help="Capture checkerboard snapshots from JetBot HTTP bridge.")
    p_cap.add_argument("--url", default="http://192.168.1.18:8080/snapshot", help="Bridge /snapshot URL")
    p_cap.add_argument("--out-dir", default="docs/measurements/checkerboard", help="Output directory")
    p_cap.add_argument("--count", type=int, default=20, help="Number of poses to capture")
    p_cap.add_argument(
        "--interval",
        type=float,
        default=0.0,
        help="Auto-capture interval in seconds (<= 0 uses interactive Enter prompt)",
    )
    p_cap.add_argument("--cols", type=int, default=9, help="Inner checkerboard corners horizontally")
    p_cap.add_argument("--rows", type=int, default=6, help="Inner checkerboard corners vertically")
    p_cap.add_argument("--timeout", type=float, default=3.0, help="HTTP request timeout (s)")
    p_cap.add_argument(
        "--require-corners",
        action="store_true",
        help="Only save snapshots where checkerboard corners are detected",
    )
    p_cap.set_defaults(func=cmd_capture)

    p_cal = sub.add_parser("calibrate", help="Calibrate camera intrinsics from saved images.")
    p_cal.add_argument("--images", required=True, help="Directory with checkerboard images")
    p_cal.add_argument("--cols", type=int, default=9, help="Inner checkerboard corners horizontally")
    p_cal.add_argument("--rows", type=int, default=6, help="Inner checkerboard corners vertically")
    p_cal.add_argument("--square-mm", type=float, default=25.0, help="Checkerboard square size in mm")
    p_cal.add_argument("--min-images", type=int, default=10, help="Minimum valid poses required")
    p_cal.add_argument("--max-error-px", type=float, default=0.5, help="Max allowed mean reprojection error (px)")
    p_cal.add_argument("--out", default="setup/camera_intrinsics.json", help="Output JSON path")
    p_cal.add_argument("--allow-fail", action="store_true", help="Exit 0 even if threshold fails")
    p_cal.set_defaults(func=cmd_calibrate)

    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    return int(args.func(args))


if __name__ == "__main__":
    sys.exit(main())
