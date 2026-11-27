### TK-CV-04: Person detection measured on the real room and the Jetson
- Role: perception (Supervisor writes the locked evaluation script from the human's labelled frames)
- Status: DRAFT — detector models exist (`perception/models/yolo26n.pt` and `perception/models/yolo26n.onnx` in repo; `/home/jetbot/yolo26n.engine` on Jetson per `docs/measurements/2026-11-16/jetson_inventory.txt`), but there is no accuracy/latency measurement yet.
- Plan: about 200 frames from the real robot and room with a person at 0.5-3 m, labelled by the human.
  Proposed thresholds (to confirm after the first measurement): recall >= 0.90 and precision >= 0.90 for persons in
  the camera's field of view at 0.5-3 m; Jetson detection loop >= 8 FPS with the bridge running; whole-bridge RAM
  <= 1.5 GB (NOTE: baseline Jetson bridge RSS is already ~1.55 GB on 2026-11-21 and ~1.9 GB streaming on 2026-11-16 per `agents-doc/STATE.md`, currently FROZEN — re-baseline RAM budget before locking the acceptance test). Depends on: TK-CV-03, Jetson measurement.
