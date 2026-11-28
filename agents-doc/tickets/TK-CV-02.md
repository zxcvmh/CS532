### TK-CV-02: Camera detections never reach navigation
- Role: perception (+ interface note to integration)
- Status: SUPERSEDED by TK-CV-02-rewritten (2026-11-16; synced 2026-11-27) — the original premise was half wrong: `coordinator.py:272` already feeds person detections into 0.40 m social bubbles.
- Problem: `dashboard/backend/app/navigation/coordinator.py:266-270` only uses detections to set
  `yolo_connected`; people are not added to any costmap layer or IL observation.
- Depends on: TK-SLAM-01
