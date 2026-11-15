---
trigger: glob
globs: "perception/**, dashboard/floor_ipm.py, dashboard/backend/app/cv/**"
description: "Camera, YOLO, floor IPM and proxemics rules."
---

# Perception rules

- Today the camera contributes **nothing** to avoidance: the coordinator only uses detections to set
  `yolo_connected`. The job is to turn detections into obstacles the planner/policy actually sees.
- There are two different floor IPM files: `perception/core/floor_ipm.py` and
  `dashboard/floor_ipm.py` (the one the bridge imports). Do not create a third. Unifying them is a
  ticket of its own.
- Every detection handed to navigation must be in the robot frame (REP-103, metres, x forward,
  y left) with a timestamp. State the camera→robot transform you used and where it is configured
  (`perception/config/`).
- YOLO runs on the Jetson inside `dashboard/jetbot_bridge.py`. Model size and FPS matter more than
  mAP: report FPS and latency, and mark Jetson numbers `UNVERIFIED` until measured on the robot.
- People are dynamic obstacles: they go to a short-TTL layer, never into the static SLAM map.
