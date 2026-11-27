#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Unit tests for scripts/eval_cv04.py and scripts/label_cv04.py (TK-CV-04)."""

import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "scripts"))

import eval_cv04  # noqa: E402
import label_cv04  # noqa: E402


class TestCV04Tools(unittest.TestCase):
    def test_01_iou_and_greedy_matching(self):
        self.assertAlmostEqual(eval_cv04.compute_iou([0, 0, 10, 10], [0, 0, 10, 10]), 1.0)
        self.assertAlmostEqual(eval_cv04.compute_iou([0, 0, 10, 10], [20, 20, 30, 30]), 0.0)
        self.assertAlmostEqual(eval_cv04.compute_iou([0, 0, 10, 10], [5, 0, 15, 10]), 50.0 / 150.0)

        gt = [[10, 10, 50, 100]]
        preds = [
            ([12, 12, 50, 98], 0.90),  # TP
            ([14, 14, 48, 96], 0.70),  # duplicate -> FP
            ([200, 10, 250, 100], 0.80),  # FP
        ]
        tp, fp, fn = eval_cv04.match_frame_detections(gt, preds, iou_thresh=0.5)
        self.assertEqual((tp, fp, fn), (1, 2, 0))

    def test_02_evaluate_session_end_to_end_pass_and_fail(self):
        with tempfile.TemporaryDirectory() as td:
            sess = Path(td)
            frames_idx = []
            dets_lines = []
            gt_frames = []
            for i in range(10):
                t = 1000.0 + i * 0.1  # 10 Hz
                fname = f"frames/frame_{i:05d}_seq_{i + 1}.jpg"
                pred_dets = [
                    {"class_name": "person", "confidence": 0.90, "bbox": [100, 50, 200, 300]},
                    {"class_name": "chair", "confidence": 0.95, "bbox": [300, 100, 380, 250]},
                    {"class_name": "person", "confidence": 0.20, "bbox": [400, 50, 460, 200]},  # below 0.40 conf
                ]
                frames_idx.append(
                    {
                        "frame_idx": i,
                        "filename": fname,
                        "host_t": t,
                        "scan_seq": i + 1,
                        "detections": pred_dets,
                    }
                )
                dets_lines.append(
                    json.dumps(
                        {
                            "host_t": t,
                            "scan_seq": i + 1,
                            "yolo_active": True,
                            "detections": pred_dets,
                        }
                    )
                )
                gt_frames.append(
                    {
                        "filename": fname,
                        "scan_seq": i + 1,
                        "persons": [
                            {"bbox": [102, 52, 198, 298], "distance_m": 1.5},
                            {"bbox": [500, 50, 550, 250], "distance_m": 4.2},  # outside 0.5-3.0m -> ignored
                        ],
                    }
                )

            (sess / "frames_index.jsonl").write_text(
                "\n".join(json.dumps(r) for r in frames_idx) + "\n", encoding="utf-8"
            )
            (sess / "detections.jsonl").write_text("\n".join(dets_lines) + "\n", encoding="utf-8")
            gt_path = sess / "gt_labels.json"
            gt_path.write_text(json.dumps({"frames": gt_frames}), encoding="utf-8")

            tegra_path = sess / "tegrastats.txt"
            tegra_path.write_text(
                "RAM 1500/3964MB (lfb 100x4MB) SWAP 0/6078MB (cached 0MB)\n"
                "RAM 1600/3964MB (lfb 100x4MB) SWAP 0/6078MB (cached 0MB)\n",
                encoding="utf-8",
            )

            res = eval_cv04.evaluate_session(sess, gt_path, tegrastats_path=tegra_path)
            self.assertEqual(res["tp"], 10)
            self.assertEqual(res["fp"], 0)
            self.assertEqual(res["fn"], 0)
            self.assertAlmostEqual(res["precision"], 1.0)
            self.assertAlmostEqual(res["recall"], 1.0)
            self.assertAlmostEqual(res["fps"]["mean_fps"], 10.0)
            self.assertEqual(res["tegrastats"]["ram_max_mb"], 1600)
            self.assertEqual(res["tegrastats"]["swap_max_mb"], 0)

            out_json = sess / "out_res.json"
            proc_pass = subprocess.run(
                [
                    sys.executable,
                    str(ROOT / "scripts" / "eval_cv04.py"),
                    "--session-dir",
                    str(sess),
                    "--gt",
                    str(gt_path),
                    "--tegrastats",
                    str(tegra_path),
                    "--max-ram-mb",
                    "1800",
                    "--out",
                    str(out_json),
                ],
                capture_output=True,
                text=True,
            )
            self.assertEqual(proc_pass.returncode, 0, proc_pass.stdout + proc_pass.stderr)
            self.assertTrue(json.loads(out_json.read_text(encoding="utf-8"))["passed"])

            proc_fail = subprocess.run(
                [
                    sys.executable,
                    str(ROOT / "scripts" / "eval_cv04.py"),
                    "--session-dir",
                    str(sess),
                    "--gt",
                    str(gt_path),
                    "--tegrastats",
                    str(tegra_path),
                    "--max-ram-mb",
                    "1500",
                    "--out",
                    str(out_json),
                ],
                capture_output=True,
                text=True,
            )
            self.assertEqual(proc_fail.returncode, 1)

    def test_03_label_cv04_manifest_and_accept_yolo(self):
        with tempfile.TemporaryDirectory() as td:
            sess = Path(td)
            idx_path = sess / "frames_index.jsonl"
            rows = [
                {"frame_idx": i, "filename": f"frames/frame_{i:05d}_seq_{i}.jpg", "scan_seq": i}
                for i in range(500)
            ]
            idx_path.write_text("\n".join(json.dumps(r) for r in rows) + "\n", encoding="utf-8")

            manifest = label_cv04.init_gt_manifest(idx_path, max_frames=200)
            self.assertEqual(len(manifest["frames"]), 200)
            self.assertEqual(manifest["frames"][0]["scan_seq"], 0)
            self.assertEqual(manifest["frames"][-1]["scan_seq"], 499)

            accepted = label_cv04.accept_yolo_persons(
                [
                    {"class_name": "person", "confidence": 0.82, "bbox": [10.2, 20.6, 80.1, 200.4]},
                    {"class_name": "chair", "confidence": 0.99, "bbox": [5, 5, 50, 50]},
                    {"class_name": "person", "confidence": 0.10, "bbox": [1, 1, 20, 20]},
                ],
                min_conf=0.25,
            )
            self.assertEqual(accepted, [{"bbox": [10, 21, 80, 200]}])

            gt_file = sess / "gt_labels.json"
            manifest["frames"][0]["persons"] = accepted
            label_cv04.save_gt_labels(manifest, gt_file)
            reloaded = label_cv04.load_gt_labels(gt_file)
            self.assertEqual(reloaded["frames"][0]["persons"], accepted)


if __name__ == "__main__":
    unittest.main()
