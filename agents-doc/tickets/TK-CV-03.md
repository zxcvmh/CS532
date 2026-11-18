### TK-CV-03: Camera calibration (intrinsics + distortion)
- Role: perception (human takes the images; no code can start before)
- Status: DRAFT — no intrinsics exist in the repo (`find` for calib/intrinsic found only `setup/calibrate_motors.py`).
- Plan: human photographs a printed checkerboard (about 20 poses, week-1 field session); a script writes
  `setup/camera_intrinsics.json`. Threshold: mean reprojection error <= 0.5 px, image size and focal length stored.
  Needed by TK-CV-05 (tags) and for projecting detections onto LiDAR azimuth.
