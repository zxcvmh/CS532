#!/usr/bin/env python3
"""
scripts/eval_cv04.py — Evaluator for real-room YOLO person detection & Jetson metrics (TK-CV-04).

Evaluates a session recorded by scripts/record_cv_session.py against a ground-truth
manifest (gt_labels.json), computing Precision, Recall, F1 (0.5-3.0 m), YOLO update
FPS, and optional Jetson tegrastats RAM/SWAP statistics.
"""

import argparse
import json
import os
import re
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np


def compute_iou(box_a: Sequence[float], box_b: Sequence[float]) -> float:
    """Compute Intersection-over-Union for two [x1, y1, x2, y2] boxes."""
    ax1, ay1, ax2, ay2 = map(float, box_a[:4])
    bx1, by1, bx2, by2 = map(float, box_b[:4])
    ix1 = max(ax1, bx1)
    iy1 = max(ay1, by1)
    ix2 = min(ax2, bx2)
    iy2 = min(ay2, by2)
    iw = max(0.0, ix2 - ix1)
    ih = max(0.0, iy2 - iy1)
    inter = iw * ih
    if inter <= 0.0:
        return 0.0
    area_a = max(0.0, ax2 - ax1) * max(0.0, ay2 - ay1)
    area_b = max(0.0, bx2 - bx1) * max(0.0, by2 - by1)
    union = area_a + area_b - inter
    return inter / union if union > 0.0 else 0.0


def match_frame_detections(
    gt_boxes: Sequence[Sequence[float]],
    pred_boxes_with_conf: Sequence[Tuple[Sequence[float], float]],
    iou_thresh: float = 0.5,
) -> Tuple[int, int, int]:
    """Greedy 1-to-1 matching sorted by confidence descending. Returns (tp, fp, fn)."""
    preds_sorted = sorted(pred_boxes_with_conf, key=lambda item: float(item[1]), reverse=True)
    matched_gt = set()
    tp = 0
    fp = 0
    for pbox, _conf in preds_sorted:
        best_idx = -1
        best_iou = float(iou_thresh) - 1e-12
        for gi, gbox in enumerate(gt_boxes):
            if gi in matched_gt:
                continue
            iou = compute_iou(pbox, gbox)
            if iou >= iou_thresh and iou > best_iou:
                best_iou = iou
                best_idx = gi
        if best_idx >= 0:
            matched_gt.add(best_idx)
            tp += 1
        else:
            fp += 1
    fn = len(gt_boxes) - len(matched_gt)
    return tp, fp, fn


def compute_detection_fps(detections_jsonl_path: Path) -> Dict[str, Any]:
    """Compute YOLO metadata packet rate from detections.jsonl."""
    if not detections_jsonl_path.is_file():
        return {"packet_count": 0, "duration_s": 0.0, "mean_fps": 0.0, "median_fps": 0.0}

    ts: List[float] = []
    for line in detections_jsonl_path.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if not line:
            continue
        obj = json.loads(line)
        t_val = obj.get("host_t")
        if t_val is None:
            t_val = obj.get("bridge_timestamp")
        if t_val is not None:
            ts.append(float(t_val))

    if len(ts) < 2:
        return {"packet_count": len(ts), "duration_s": 0.0, "mean_fps": 0.0, "median_fps": 0.0}

    arr = np.sort(np.asarray(ts, dtype=np.float64))
    dts = np.diff(arr)
    pos_dts = dts[dts > 1e-6]
    dur = float(arr[-1] - arr[0])
    mean_fps = (len(arr) - 1) / dur if dur > 0.0 else 0.0
    median_fps = float(1.0 / np.median(pos_dts)) if pos_dts.size > 0 else 0.0
    return {
        "packet_count": int(len(arr)),
        "duration_s": round(dur, 3),
        "mean_fps": round(mean_fps, 2),
        "median_fps": round(median_fps, 2),
    }


def parse_tegrastats(path: Path) -> Dict[str, Any]:
    """Parse Jetson tegrastats log for RAM and SWAP statistics in MB."""
    ram_used: List[int] = []
    ram_total: List[int] = []
    swap_used: List[int] = []

    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        m_ram = re.search(r"RAM\s+(\d+)/(\d+)MB", line)
        if m_ram:
            ram_used.append(int(m_ram.group(1)))
            ram_total.append(int(m_ram.group(2)))
        m_swap = re.search(r"SWAP\s+(\d+)/(\d+)MB", line)
        if m_swap:
            swap_used.append(int(m_swap.group(1)))

    if not ram_used:
        return {"samples": 0, "ram_mean_mb": 0.0, "ram_max_mb": 0, "ram_total_mb": 0, "swap_max_mb": 0}

    return {
        "samples": len(ram_used),
        "ram_mean_mb": round(float(np.mean(ram_used)), 1),
        "ram_max_mb": int(np.max(ram_used)),
        "ram_total_mb": int(ram_total[-1]),
        "swap_max_mb": int(np.max(swap_used)) if swap_used else 0,
    }


def init_gt_manifest_from_session(
    session_dir: Path, max_frames: int = 200
) -> Dict[str, Any]:
    """Evenly sample up to max_frames entries from frames_index.jsonl into an empty GT manifest."""
    idx_path = session_dir / "frames_index.jsonl"
    if not idx_path.is_file():
        raise FileNotFoundError(f"Missing {idx_path}")

    entries = [
        json.loads(ln)
        for ln in idx_path.read_text(encoding="utf-8").splitlines()
        if ln.strip()
    ]
    if not entries:
        raise ValueError(f"No frame entries in {idx_path}")

    if len(entries) > max_frames:
        indices = np.linspace(0, len(entries) - 1, num=max_frames, dtype=int)
        sampled = [entries[int(i)] for i in indices]
    else:
        sampled = entries

    return {
        "session_dir": str(session_dir),
        "frames": [
            {
                "filename": e["filename"],
                "scan_seq": int(e.get("scan_seq", -1)),
                "persons": [],
            }
            for e in sampled
        ],
    }


def evaluate_session(
    session_dir: Path,
    gt_path: Path,
    iou_thresh: float = 0.5,
    conf_thresh: float = 0.4,
    min_dist_m: float = 0.5,
    max_dist_m: float = 3.0,
    tegrastats_path: Optional[Path] = None,
) -> Dict[str, Any]:
    """Evaluate recorded detections against ground-truth person bounding boxes."""
    gt_data = json.loads(gt_path.read_text(encoding="utf-8"))
    gt_frames = gt_data.get("frames", [])

    by_filename: Dict[str, Dict[str, Any]] = {}
    by_seq: Dict[int, Dict[str, Any]] = {}

    frames_idx_path = session_dir / "frames_index.jsonl"
    if frames_idx_path.is_file():
        for ln in frames_idx_path.read_text(encoding="utf-8").splitlines():
            if not ln.strip():
                continue
            row = json.loads(ln)
            by_filename[row["filename"]] = row

    dets_path = session_dir / "detections.jsonl"
    if dets_path.is_file():
        for ln in dets_path.read_text(encoding="utf-8").splitlines():
            if not ln.strip():
                continue
            row = json.loads(ln)
            seq = int(row.get("scan_seq", -1))
            if seq >= 0:
                by_seq[seq] = row

    total_tp = 0
    total_fp = 0
    total_fn = 0
    frames_evaluated = 0

    for gf in gt_frames:
        fname = gf.get("filename", "")
        seq = int(gf.get("scan_seq", -1))
        src = by_filename.get(fname) or by_seq.get(seq)
        if src is None:
            continue
        frames_evaluated += 1

        gt_boxes: List[List[float]] = []
        for p in gf.get("persons", []):
            dist = p.get("distance_m")
            if dist is not None and not (min_dist_m <= float(dist) <= max_dist_m):
                continue
            bbox = p.get("bbox")
            if bbox and len(bbox) == 4:
                gt_boxes.append([float(v) for v in bbox])

        preds: List[Tuple[List[float], float]] = []
        for d in src.get("detections", []):
            cname = str(d.get("class_name") or d.get("class") or "").strip().lower()
            conf = float(d.get("confidence", 0.0))
            bbox = d.get("bbox")
            if cname == "person" and conf >= conf_thresh and bbox and len(bbox) == 4:
                preds.append(([float(v) for v in bbox], conf))

        tp, fp, fn = match_frame_detections(gt_boxes, preds, iou_thresh=iou_thresh)
        total_tp += tp
        total_fp += fp
        total_fn += fn

    precision = total_tp / (total_tp + total_fp) if (total_tp + total_fp) > 0 else 0.0
    recall = total_tp / (total_tp + total_fn) if (total_tp + total_fn) > 0 else 0.0
    f1 = (2.0 * precision * recall / (precision + recall)) if (precision + recall) > 0.0 else 0.0

    fps_stats = compute_detection_fps(dets_path)
    miss_rate = (1.0 - recall) if (total_tp + total_fn) > 0 else 0.0
    fp_per_frame = (total_fp / frames_evaluated) if frames_evaluated > 0 else 0.0
    sim_noise_params = {
        "miss_rate": round(miss_rate, 4),
        "fp_per_frame": round(fp_per_frame, 4),
        "false_alarm_rate_est": round(min(1.0, fp_per_frame), 4),
    }
    out: Dict[str, Any] = {
        "session_dir": str(session_dir),
        "gt_path": str(gt_path),
        "frames_in_gt": len(gt_frames),
        "frames_evaluated": frames_evaluated,
        "iou_thresh": float(iou_thresh),
        "conf_thresh": float(conf_thresh),
        "distance_range_m": [float(min_dist_m), float(max_dist_m)],
        "tp": total_tp,
        "fp": total_fp,
        "fn": total_fn,
        "precision": round(precision, 4),
        "recall": round(recall, 4),
        "f1": round(f1, 4),
        "fps": fps_stats,
        "sim_noise_params": sim_noise_params,
    }
    if tegrastats_path is not None and tegrastats_path.is_file():
        out["tegrastats"] = parse_tegrastats(tegrastats_path)
    return out


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="Evaluate recorded CV session for TK-CV-04.")
    parser.add_argument("--session-dir", required=True, help="Directory from scripts/record_cv_session.py")
    parser.add_argument("--gt", default=None, help="Path to ground-truth gt_labels.json")
    parser.add_argument("--init-gt", default=None, help="Create initial gt_labels.json from session and exit")
    parser.add_argument("--max-frames", type=int, default=200, help="Max frames to sample for --init-gt")
    parser.add_argument("--iou-thresh", type=float, default=0.5)
    parser.add_argument("--conf-thresh", type=float, default=0.4)
    parser.add_argument("--min-dist", type=float, default=0.5)
    parser.add_argument("--max-dist", type=float, default=3.0)
    parser.add_argument("--tegrastats", default=None, help="Optional tegrastats log path")
    parser.add_argument("--min-recall", type=float, default=0.90)
    parser.add_argument("--min-precision", type=float, default=0.90)
    parser.add_argument("--min-fps", type=float, default=8.0)
    parser.add_argument("--max-ram-mb", type=float, default=None)
    parser.add_argument("--out", default=None, help="Output results JSON (default <session_dir>/cv04_results.json)")
    parser.add_argument("--no-gate", action="store_true", help="Exit 0 even if thresholds miss")
    args = parser.parse_args(argv)

    session_dir = Path(args.session_dir)

    if args.init_gt:
        manifest = init_gt_manifest_from_session(session_dir, max_frames=args.max_frames)
        out_gt = Path(args.init_gt)
        out_gt.parent.mkdir(parents=True, exist_ok=True)
        tmp = str(out_gt) + ".tmp"
        Path(tmp).write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
        os.replace(tmp, out_gt)
        print(f"[INIT-GT] Wrote {len(manifest['frames'])} frames to {out_gt}")
        return 0

    if not args.gt:
        print("[FAIL] Either --gt or --init-gt is required.", file=sys.stderr)
        return 2

    tegra_path = Path(args.tegrastats) if args.tegrastats else None
    res = evaluate_session(
        session_dir=session_dir,
        gt_path=Path(args.gt),
        iou_thresh=args.iou_thresh,
        conf_thresh=args.conf_thresh,
        min_dist_m=args.min_dist,
        max_dist_m=args.max_dist,
        tegrastats_path=tegra_path,
    )

    out_path = Path(args.out) if args.out else (session_dir / "cv04_results.json")
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(json.dumps(res, indent=2) + "\n", encoding="utf-8")

    print(
        f"[METRIC] cv04: frames={res['frames_evaluated']}/{res['frames_in_gt']} | "
        f"TP={res['tp']} FP={res['fp']} FN={res['fn']} | "
        f"precision={res['precision']:.4f} recall={res['recall']:.4f} f1={res['f1']:.4f} | "
        f"yolo_fps_mean={res['fps']['mean_fps']:.2f} median={res['fps']['median_fps']:.2f} | "
        f"sim_miss_rate={res['sim_noise_params']['miss_rate']:.4f} "
        f"sim_fa_est={res['sim_noise_params']['false_alarm_rate_est']:.4f}"
    )
    if "tegrastats" in res:
        tg = res["tegrastats"]
        print(
            f"[METRIC] tegrastats: ram_mean={tg['ram_mean_mb']} MB, "
            f"ram_max={tg['ram_max_mb']}/{tg['ram_total_mb']} MB, swap_max={tg['swap_max_mb']} MB"
        )

    passed = (
        res["frames_evaluated"] > 0
        and res["recall"] >= args.min_recall
        and res["precision"] >= args.min_precision
        and res["fps"]["mean_fps"] >= args.min_fps
    )
    if args.max_ram_mb is not None and "tegrastats" in res:
        passed = passed and (res["tegrastats"]["ram_max_mb"] <= args.max_ram_mb)

    res["passed"] = passed
    out_path.write_text(json.dumps(res, indent=2) + "\n", encoding="utf-8")

    if not passed and not args.no_gate:
        print("[FAIL] One or more TK-CV-04 thresholds were missed.", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
