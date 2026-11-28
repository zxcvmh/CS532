### TK-CV-04: Person detection measured on the real room and the Jetson
- Role: perception (Supervisor writes the locked evaluation script from the human's labelled frames)
- Status: DRAFT — synchronized recorder `scripts/record_cv_session.py` prepared 2026-11-27; blocked on real-room recording (~200 labelled frames at 0.5-3 m), Jetson FPS/RAM measurement, and human approval of Assumptions.
- Problem: detector models exist (`perception/models/yolo26n.pt` and `perception/models/yolo26n.onnx` in repo; `/home/jetbot/yolo26n.engine` on Jetson per `docs/measurements/2026-11-16/jetson_inventory.txt`), but real-room person precision/recall and Jetson detection FPS under bridge load have never been measured.
- Allowed files: `scripts/record_cv_session.py`, `docs/measurements/<date>/` (raw session + `cv04_results.json`), evaluation script (TBD by Supervisor when locking acceptance test).
- Forbidden files: everything else + `agents-doc/LOCKED_PATHS.txt`.
- Acceptance command: TBD (Supervisor locks evaluator against the human-labelled ~200-frame dataset recorded with `scripts/record_cv_session.py`).
- Threshold (numbers): proposed (to confirm after the first measurement): `recall >= 0.90` and `precision >= 0.90` for persons in the camera FOV at `0.5-3.0 m`; Jetson detection loop `>= 8 FPS` with the bridge running; bridge RAM `<= 2.0 GB` (`2048 MB`) with `swap_max == 0 MB`.
- Regression commands: none (measurement-only ticket).
- Depends on: `TK-CV-03` (same field session can capture both checkerboard poses and the person recording).
- Assumptions: (a) one synchronized session (`scripts/record_cv_session.py`) captures JPEG frames + YOLO detections + LiDAR sweeps so the same recording serves both `TK-CV-04` and `TK-CV-02-rewritten` ablation; (b) YOLO inference stays on Jetson TensorRT (`yolo26n.engine`); (c) Jetson bridge RAM cap is set to `<= 2.0 GB` (`2048 MB`) with `swap == 0` (superseding the obsolete `1.5 GB` draft figure).
  Human-approved: 2026-11-28
- Needs-human: schedule the field session to record ~200 frames + checkerboard.
