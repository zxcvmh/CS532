### TK-CV-04: Person detection measured on the real room and the Jetson
- Role: perception (Supervisor writes the locked evaluation script from the human's labelled frames)
- Status: DRAFT — only a detector exists (`perception/models/yolo26n`, engine in the bridge); there is no measurement.
- Plan: about 200 frames from the real robot and room with a person at 0.5-3 m, labelled by the human.
  Proposed thresholds (to confirm after the first measurement): recall >= 0.90 and precision >= 0.90 for persons in
  the camera's field of view at 0.5-3 m; Jetson detection loop >= 8 FPS with the bridge running; whole-bridge RAM
  <= 1.5 GB. Depends on: TK-CV-03, Jetson measurement.
