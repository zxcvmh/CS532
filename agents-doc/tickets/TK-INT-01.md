### TK-INT-01: Deploy the single dynamics file to the Jetson
- Role: supervisor (hardware)
- Status: DONE 2026-11-15 — Jetson is `jetbot@192.168.1.18` (`~/.ssh/config` alias still says .16).
  Copied to `/home/jetbot/setup/robot_dynamics.json` (sha256 matches repo). Backups:
  `~/setup/robot_dynamics.json.bak-20261115`, `~/03-MinhHieu/test-hardware/robot_dynamics.json.bak-20261115`.
  All 3 bridges on the Jetson now log `Loaded robot dynamics from /home/jetbot/setup/robot_dynamics.json`.
