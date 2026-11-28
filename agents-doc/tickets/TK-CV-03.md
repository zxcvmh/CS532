### TK-CV-03: Camera calibration (intrinsics + distortion)
- Role: perception (human takes the images; no code can start before)
- Status: DRAFT — tool `setup/calibrate_camera.py` prepared 2026-11-27; blocked on field checkerboard photos (~20 poses) and human approval of Assumptions.
- Problem: no camera intrinsics exist in `setup/` (`find` for calib/intrinsic found only `setup/calibrate_motors.py`). `perception/config/camera_config.json` only stores nominal `fov_deg: 160.0` (no calibrated focal length, principal point, or distortion), which is needed to project YOLO detections onto LiDAR azimuth in `TK-CV-02-rewritten`.
- Allowed files: `setup/calibrate_camera.py`, `setup/camera_intrinsics.json`, `docs/measurements/<date>/checkerboard/`.
- Forbidden files: everything else + `agents-doc/LOCKED_PATHS.txt`.
- Acceptance command: `.venv/bin/python setup/calibrate_camera.py calibrate --images docs/measurements/<date>/checkerboard --cols <C> --rows <R> --square-mm <S> --out setup/camera_intrinsics.json`
- Threshold (numbers): `num_images_used >= 10`, `mean_reprojection_error_px <= 0.5`, valid finite `camera_matrix`, `dist_coeffs`, `image_width`, `image_height`, `fx`, `fy` stored in `setup/camera_intrinsics.json`.
- Regression commands: `.venv/bin/python -m unittest discover -s perception -p "test_*.py"`
- Depends on: — (human prints checkerboard and captures ~20 poses with `setup/calibrate_camera.py capture`).
- Assumptions: (a) calibration runs on the laptop over snapshots captured at the exact bridge camera resolution (640x480); (b) standard pinhole + radial/tangential distortion (`cv2.calibrateCamera`) suffices (switch to `cv2.fisheye` if the 160-deg lens cannot achieve <= 0.5 px mean reprojection error); (c) `setup/camera_intrinsics.json` is the single source of truth for camera-to-LiDAR azimuth projection (`TK-CV-02-rewritten`; `TK-CV-05` tags cancelled 2026-11-27); (d) printed A4 checkerboard has 9x6 inner corners (10x7 squares, ~25 mm square_mm, measured with ruler after printing).
  Human-approved: 2026-11-28
- Needs-human: measure exact `square_mm` after printing and capture ~20 checkerboard poses on the field session.
