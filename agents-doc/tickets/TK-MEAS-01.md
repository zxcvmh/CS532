### TK-MEAS-01: Measure the network, the Jetson budget and the LiDAR with people (decides "real time" and where the loop runs)
- Role: supervisor-assistant (laptop side only) + Supervisor / human for everything that touches the Jetson or the room
- Status: READY (2026-11-16) — locked validator `tests/acceptance/test_measurements_valid.py` is red on HEAD (no
  `docs/measurements/<date>/`), green on synthetic data (proved, then deleted).
- Problem: no number exists for laptop<->Jetson latency, Jetson RAM/CPU headroom, or how a walking person looks in the
  LiDAR scan. Without them the control-loop location and the CV mask (TK-CV-02) cannot be decided.
- Division of work (Rule 8: the assistant never SSHes to the Jetson, never starts motors):
  1. Assistant writes `scripts/measure_summary.py`: reads the raw files in the date directory and writes `results.json`
     (schema in the validator docstring). It computes; it never types a number.
  2. M1 ping: Supervisor/human runs `ping -c 200 <jetson> | tee docs/measurements/<date>/ping_home_wifi.txt` (home WiFi).
  3. M2 inventory (read-only): Supervisor runs `free -m`, `uptime`, `python3 -c "import cv2;print(cv2.__version__,hasattr(cv2,'aruco'))"`,
     `ls` of the engine/model dirs, `ps` of the bridge; output saved raw as `jetson_inventory.txt`. No kill, no install.
  4. M4 LiDAR with people: human stands/walks, robot stationary, 3 x 60 s: `static_person`, `crossing`, `head_on`.
     Recorder = whatever produced `tests/acceptance/data/field_turns_2026-11-15.npz` (keys `t`, `ranges` (n,360));
     if none is in the repo the assistant writes `scripts/record_lidar.py` that flushes every 5 s.
  5. Assistant writes `scripts/analyze_lidar_people.py` (reads the three .npz): per recording, number of person-like
     clusters per scan, their speed estimate and range, wall-point density; prints raw numbers into `analysis.txt`.
     It makes NO claim about SLAM pose (that is TK-CV-02's ablation).
- Allowed files: `scripts/measure_summary.py`, `scripts/analyze_lidar_people.py`, `scripts/record_lidar.py` (only if
  no recorder exists), `docs/measurements/**`.
- Forbidden files: everything else + agents-doc/LOCKED_PATHS.txt (tests/, setup/robot_dynamics.json, dashboard/, motion/).
- Acceptance command: `.venv/bin/python tests/acceptance/test_measurements_valid.py -v`
- Threshold: 3/3 tests pass on the real date directory; ping >= 200 packets; each recording >= 55 s; every figure in
  results.json recomputes from its raw file within 1 %. Anything not measured is written `CHƯA ĐO`, never estimated.
- Decision rule (Supervisor applies, not the assistant): ping p99 < 30 ms and loss < 1 % -> loop may run on the laptop;
  otherwise reflex + local policy run on the Jetson.
- Regression commands: `.venv/bin/python tests/acceptance/test_slam_csm_turns.py -v` (unchanged: no source file is touched).
- Out of scope: YOLO FPS on the Jetson (needs the detector chosen; TK-CV-04), motors, deployment, any threshold change.
- Depends on: the human on the home WiFi (not 172.30.x) for M1/M2; a free 2 h field slot for M4.
