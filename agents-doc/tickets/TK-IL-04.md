### TK-IL-04: IL sim must use the field dynamics and a stop-turn-go action set
- Role: il-policy
- Status: DONE 2026-11-20 — auditor PASS reported by the human (re-run by the Supervisor 2026-11-20: acceptance green on HEAD e15a8e1). Earlier note: READY (2026-11-16) — locked test `tests/acceptance/test_il_sim_dynamics.py` FAILS 14/14 on HEAD for the
  ticket's reasons (forward 0.152 m/s vs field 0.294 after deadband + 5 % slip, v = 0.02 -> 0 m/s, w steers a moving
  robot, pivot turns at the commanded 0.5 rad/s instead of the field rate, no ACTION_* / step_action) and PASSES 7/7
  on a Supervisor prototype (scratchpad, not in repo) for both the real file and a synthetic one (0.41 m/s, 0.54 s).
- Problem: `imitation_learning/config.py` hardcodes MIN_DRIVE_SPEED 0.10 / MAX_LINEAR_SPEED 0.16 / MAX_ANGULAR_SPEED
  0.35; `mock_expert_simulator.py` `step()` has a 0.10 deadband, a 5 % slip on top of a field speed that already
  includes slip, and lets |w| steer while moving. The real robot (dashboard/jetbot_bridge.py `set_motor_speeds`,
  TK-INT-09) has ONE forward speed (`actual_speed_mps`), ONE pivot rate ((pi/2)/`turn_time_90_deg_sec`) and no arc
  steering -> a sim success rate says nothing about the robot.
- Contract (the test enforces it): `config.py` reads the dynamics file from env var `ROBOT_DYNAMICS_FILE` if set, else
  `<repo>/setup/robot_dynamics.json`; `ACTION_STOP/FORWARD/PIVOT_LEFT/PIVOT_RIGHT` = 0/1/2/3;
  `MAX_LINEAR_SPEED` = `actual_speed_mps`; `MAX_ANGULAR_SPEED` = (pi/2)/`turn_time_90_deg_sec`;
  `MIN_DRIVE_SPEED` = `min_drive_speed` (names stay importable — model.py, il_policy_runner.py etc. import them);
  `SimulationEnvironment.step(cmd_v, cmd_w, dt)` mirrors the bridge: |v| < 0.01 and |w| < 0.02 -> stop;
  |v| < 0.01 -> pivot at +-MAX_ANGULAR_SPEED by the sign of w, no translation; |v| >= 0.01 -> forward at
  MAX_LINEAR_SPEED, w ignored; new `step_action(action, dt)` maps the 4 actions onto `step`.
  Do not add slip on top of `actual_speed_mps`. Python 3.6 compatible syntax (this config may reach the Jetson).
- Allowed files: `imitation_learning/config.py`, `imitation_learning/mock_expert_simulator.py`
- Forbidden: `tests/`, `setup/robot_dynamics.json`, `imitation_learning/test_il_pipeline.py`, the policy / model / runner
  files, everything else. No dynamics number literal (0.294, 0.3, 5.236, 0.19, ...) in the two files.
- Acceptance command: `.venv/bin/python tests/acceptance/test_il_sim_dynamics.py -v`
- Threshold: 7/7 tests x 2 dynamics files (real, synthetic): forward speed = `actual_speed_mps` +-3 % (also for
  v = 0.02 / 0.05 / 0.10 / 0.30, +-3 %); heading change while driving <= 2 deg/s with w = +-1; pivot rate =
  +-(pi/2)/`turn_time_90_deg_sec` +-3 % for |w| = 0.5 and 3.0 and for the two pivot actions; pivot translation <= 5 cm
  over 2 s; stop creep <= 1 cm and <= 1 deg.
- Regression: `tests/acceptance/test_il_sim_success.py::test_sim_physics_matches_field_dynamics` passes (Supervisor
  removed its `MAX_LINEAR_SPEED <= max_safe_speed_mps` line, which contradicted the field speed);
  `.venv/bin/python imitation_learning/test_il_pipeline.py`: tests 1-4 and 6 stay green.
  **Accepted consequence (decided by the Supervisor, 2026-11-16):** `test_05_dagger_session_execution` goes RED
  (DAgger final success 0.0 %, measured on the prototype; expert 16.7 % vs ~50 % before) because the old continuous
  expert cannot drive stop-turn-go. That is the IL-03 guard doing its job. Do NOT edit that test or its assert to
  make it green; TK-IL-05 (new expert) and TK-IL-06 (retrain) turn it green. State it in the REPORT.
  `test_il_sim_success.py` (TK-IL-01) is PAUSED, its other numbers are not a regression gate until TK-IL-06.
- Decided by the human (2026-11-16): the sim follows `actual_speed_mps` (what the robot physically does, measured at
  cruise_speed 0.2) and IGNORES `max_safe_speed_mps` (0.14). The human re-measured at the chassis instead of the
  camera nose: 0.294 -> 0.286 m/s; pivot at turn_speed 0.14: 90 deg in 0.53 s, 180 deg 1.06 s (human to apply to the file; the test reads
  the file, so no code or test change is needed when it is).
- Depends on: —
