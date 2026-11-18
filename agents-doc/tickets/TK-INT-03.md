### TK-INT-03: Jetson runs stale bridges that ignore turn_time_90_deg_sec
- Role: supervisor (deploy) + integration (Python 3.6 check)
- Status: DONE 2026-11-15 — human runs only `~/03-MinhHieu/run_bridge.sh`. Deployed repo
  `dashboard/{jetbot_bridge,command_mux,d500_driver,floor_ipm}.py` + `motion/planning/disparity_extender.py`
  into `~/03-MinhHieu/` (sha256 match, py_compile OK on Python 3.6.9). Backups: `*.bak-20261115`.
  Loader check: `min=0.19 cruise=0.2 trim=1.045 turn=0.19 T90=0.3 maxW=5.236`. Other old bridge copies
  (`~/`, `~/05-d500lib-motion/`, `~/01-HieuMe/`) left untouched — not launched. Not yet run on motors.
- Problem: the Jetson has 5 `jetbot_bridge.py` copies (`~/`, `~/03-MinhHieu/`, `~/03-MinhHieu/backend/`,
  `~/05-d500lib-motion/`, `~/01-HieuMe/`) and an empty `~/dashboard/`. With the new config they compute
  max angular speed as 3.142 / 1.400 / 0.350 rad/s; the repo bridge computes 5.236 from the measured 0.3 s.
- Plan: deploy `dashboard/jetbot_bridge.py` + its imports into `~/dashboard/`, verify it imports under
  Python 3.6.9, then archive the old copies on the Jetson (rename, not delete).
- Problem: bridge now reads only `<repo>/setup/robot_dynamics.json` or
  `/home/jetbot/setup/robot_dynamics.json`; the old `dashboard/test-hardware/` copy is no longer read.
- Acceptance: bridge log line `[BRIDGE] Loaded robot dynamics from .../setup/robot_dynamics.json`.
