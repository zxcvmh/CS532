### TK-INT-09: Bridge spin PWM inverted once turn_speed < min_drive_speed (Supervisor hotfix)
- Role: supervisor (hardware)
- Status: DONE in repo, NOT deployed (Jetson offline 2026-11-15; Jetson runs ~/03-MinhHieu/jetbot_bridge.py, not
  the repo bridge — TK-INT-07 is not deployed either)
- Trigger: human wants slower pivots: turn_speed 0.14, turn_time_90_deg_sec 0.54, turn_time_180_deg_sec 1.08
  (human edits setup/robot_dynamics.json; Claude never does).
- Bug: spin PWM = min_drive_speed + (turn_speed - min_drive_speed) × |w|/w_max. With 0.14 < 0.19 a partial
  joystick / slider command (ratio 0.1) spun at 0.187 PWM — FASTER than a full command (0.14) — and at a rate
  nobody calibrated. Only one spin PWM is calibrated (calibrate_motors.py test 4).
- Fix: spin PWM = turn_speed for any |w| ≥ 0.02 (single-rate pivot). Test `tests/test_bridge_spin_step.py`
  (RED 0.187 on the old bridge → GREEN). Regression `dashboard/test_path_following_and_modular_architecture.py` 8 OK.
- Field check (human): after editing robot_dynamics.json and deploying, confirm the robot still starts turning from
  rest at 0.14 PWM (below the straight-line min_drive_speed) and that 90° ≈ 0.54 s.
