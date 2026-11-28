#!/usr/bin/env python3
"""
scripts/label_cv04.py — Fast OpenCV bounding-box annotator for TK-CV-04 sessions.

Loads frames from a scripts/record_cv_session.py directory, overlays YOLO person
predictions in cyan, and lets the user draw/accept/edit ground-truth person boxes:
  - Left mouse drag : add person box
  - a               : accept YOLO person predictions for current frame
  - z               : undo last box
  - c               : clear all boxes on current frame
  - n / Space       : save & next frame
  - p               : previous frame
  - s               : save now
  - q / Esc         : save & quit
"""

import argparse
import json
import os
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence

import cv2
import numpy as np


def init_gt_manifest(frames_index_path: Path, max_frames: int = 200) -> Dict[str, Any]:
    """Evenly sample up to max_frames entries from frames_index.jsonl."""
    if not frames_index_path.is_file():
        raise FileNotFoundError(f"Missing frames index: {frames_index_path}")

    entries = [
        json.loads(ln)
        for ln in frames_index_path.read_text(encoding="utf-8").splitlines()
        if ln.strip()
    ]
    if not entries:
        raise ValueError(f"No entries found in {frames_index_path}")

    if len(entries) > max_frames:
        idxs = np.linspace(0, len(entries) - 1, num=max_frames, dtype=int)
        sampled = [entries[int(i)] for i in idxs]
    else:
        sampled = entries

    return {
        "session_dir": str(frames_index_path.parent),
        "frames": [
            {
                "filename": e["filename"],
                "scan_seq": int(e.get("scan_seq", -1)),
                "persons": [],
            }
            for e in sampled
        ],
    }


def accept_yolo_persons(
    yolo_detections: Optional[Sequence[Dict[str, Any]]],
    min_conf: float = 0.25,
) -> List[Dict[str, Any]]:
    """Extract person boxes from raw YOLO detections for fast pre-labeling."""
    if not yolo_detections:
        return []
    out: List[Dict[str, Any]] = []
    for d in yolo_detections:
        cname = str(d.get("class_name") or d.get("class") or "").strip().lower()
        conf = float(d.get("confidence", 0.0))
        bbox = d.get("bbox")
        if cname == "person" and conf >= min_conf and bbox and len(bbox) == 4:
            x1, y1, x2, y2 = [int(round(float(v))) for v in bbox]
            if x2 > x1 and y2 > y1:
                out.append({"bbox": [x1, y1, x2, y2]})
    return out


def save_gt_labels(gt_data: Dict[str, Any], out_path: Path) -> None:
    """Atomically write ground-truth labels JSON to disk."""
    out_path.parent.mkdir(parents=True, exist_ok=True)
    tmp_path = out_path.with_suffix(out_path.suffix + ".tmp")
    tmp_path.write_text(json.dumps(gt_data, indent=2) + "\n", encoding="utf-8")
    os.replace(tmp_path, out_path)


def load_gt_labels(path: Path) -> Dict[str, Any]:
    """Load ground-truth labels JSON from disk."""
    return json.loads(path.read_text(encoding="utf-8"))


def run_labeler(session_dir: Path, gt_path: Path, max_frames: int = 200) -> int:
    """Launch interactive OpenCV window for labeling session frames."""
    frames_idx_path = session_dir / "frames_index.jsonl"
    yolo_by_file: Dict[str, List[Dict[str, Any]]] = {}
    if frames_idx_path.is_file():
        for ln in frames_idx_path.read_text(encoding="utf-8").splitlines():
            if not ln.strip():
                continue
            row = json.loads(ln)
            yolo_by_file[row["filename"]] = row.get("detections", [])

    if gt_path.is_file():
        gt_data = load_gt_labels(gt_path)
    else:
        gt_data = init_gt_manifest(frames_idx_path, max_frames=max_frames)
        save_gt_labels(gt_data, gt_path)

    frames = gt_data.get("frames", [])
    if not frames:
        print("[WARN] Nothing to label.")
        return 1

    state = {"drawing": False, "x0": 0, "y0": 0, "x1": 0, "y1": 0, "idx": 0}
    win = "TK-CV-04 Person Labeler"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)

    def _on_mouse(event, x, y, _flags, _param):
        if event == cv2.EVENT_LBUTTONDOWN:
            state["drawing"] = True
            state["x0"], state["y0"] = x, y
            state["x1"], state["y1"] = x, y
        elif event == cv2.EVENT_MOUSEMOVE and state["drawing"]:
            state["x1"], state["y1"] = x, y
        elif event == cv2.EVENT_LBUTTONUP and state["drawing"]:
            state["drawing"] = False
            x1, x2 = sorted((state["x0"], x))
            y1, y2 = sorted((state["y0"], y))
            if (x2 - x1) >= 6 and (y2 - y1) >= 6:
                frames[state["idx"]]["persons"].append({"bbox": [int(x1), int(y1), int(x2), int(y2)]})

    cv2.setMouseCallback(win, _on_mouse)

    while True:
        cur = frames[state["idx"]]
        img_path = session_dir / cur["filename"]
        img = cv2.imread(str(img_path), cv2.IMREAD_COLOR)
        if img is None:
            img = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(img, f"Missing: {cur['filename']}", (20, 240), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        canvas = img.copy()
        yolo_dets = yolo_by_file.get(cur["filename"], [])
        for yd in accept_yolo_persons(yolo_dets, min_conf=0.25):
            bx1, by1, bx2, by2 = yd["bbox"]
            cv2.rectangle(canvas, (bx1, by1), (bx2, by2), (255, 255, 0), 1)

        for gp in cur["persons"]:
            gx1, gy1, gx2, gy2 = [int(v) for v in gp["bbox"]]
            cv2.rectangle(canvas, (gx1, gy1), (gx2, gy2), (0, 220, 0), 2)

        if state["drawing"]:
            cv2.rectangle(canvas, (state["x0"], state["y0"]), (state["x1"], state["y1"]), (0, 165, 255), 2)

        hud = (
            f"Frame {state['idx'] + 1}/{len(frames)} | GT={len(cur['persons'])} | "
            "[a]ccept YOLO [z]undo [c]lear [n/Space]next [p]rev [s]ave [q]uit"
        )
        cv2.rectangle(canvas, (0, 0), (canvas.shape[1], 28), (20, 20, 20), -1)
        cv2.putText(canvas, hud, (8, 19), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
        cv2.imshow(win, canvas)

        key = cv2.waitKey(30) & 0xFF
        if key in (ord("q"), 27):
            save_gt_labels(gt_data, gt_path)
            break
        elif key == ord("a"):
            cur["persons"] = accept_yolo_persons(yolo_dets, min_conf=0.25)
        elif key == ord("z") and cur["persons"]:
            cur["persons"].pop()
        elif key == ord("c"):
            cur["persons"].clear()
        elif key == ord("s"):
            save_gt_labels(gt_data, gt_path)
        elif key in (ord("n"), 32):
            save_gt_labels(gt_data, gt_path)
            state["idx"] = min(len(frames) - 1, state["idx"] + 1)
        elif key == ord("p"):
            state["idx"] = max(0, state["idx"] - 1)

    cv2.destroyAllWindows()
    print(f"[SAVED] {gt_path}")
    return 0


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="Interactive ground-truth labeler for TK-CV-04.")
    parser.add_argument("--session-dir", required=True, help="Session directory from record_cv_session.py")
    parser.add_argument("--gt", default=None, help="Output gt_labels.json path (default <session_dir>/gt_labels.json)")
    parser.add_argument("--max-frames", type=int, default=200, help="Max sampled frames")
    args = parser.parse_args(argv)

    session_dir = Path(args.session_dir)
    gt_path = Path(args.gt) if args.gt else (session_dir / "gt_labels.json")
    return run_labeler(session_dir, gt_path, max_frames=args.max_frames)


if __name__ == "__main__":
    sys.exit(main())
