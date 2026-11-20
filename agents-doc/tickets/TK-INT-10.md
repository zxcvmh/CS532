### TK-INT-10: Bridge forward drive must be ONE PWM (cruise_speed), not a v-dependent window
- Role: integration
- Status: DONE 2026-11-20 — auditor PASS reported by the human (re-run by the Supervisor 2026-11-20: acceptance green on HEAD e15a8e1). Earlier note: READY (2026-11-16) — locked test `tests/acceptance/test_bridge_forward_single_pwm.py` fails on HEAD for the
  ticket's reasons (steady PWM 0.112 / 0.122 / 0.140 for v = 0.02 / 0.05 / 0.10 instead of cruise 0.2; w steers the
  wheels, w = 3 even reverses the left one; after a pivot the forward PWM sticks at 0.14) and a Supervisor reference
  (in memory, not in the repo) passes 6/6, and keeps `tests/test_bridge_spin_step.py` 4/4.
- Problem: `set_motor_speeds` section "3. Linear drive" maps wheel speeds through a window around
  `max_safe_speed_mps` onto PWM [`min_drive_speed`, `cruise_speed`]. With the field file of 2026-11-16
  (min_drive 0.08, max_safe 0.1, cruise 0.2) the robot drives at PWM 0.14-0.175 for most commands, so
  `actual_speed_mps` (0.284, measured at cruise 0.2), the IL sim (TK-IL-04) and every SLAM assumption describe a
  robot that does not exist. Same fix pattern as TK-INT-09 for the pivot.
- Required: stop-turn-go. |v| >= 0.01 and no pivot -> both logical wheel PWMs = sign(v) x `cruise_speed` after the soft
  start, for any |v| and any w (w ignored); soft start = a PWM ramp 0 -> cruise over `soft_start_ramp_sec`
  (non-decreasing, starts below cruise, never above it), which mirrors `setup/calibrate_motors.py`
  `soft_start_drive`; backward symmetric; a pivot resets the forward ramp; stop = 0. Trim stays in
  `set_raw_motor_speeds` (do not apply it here). `min_drive_speed` / `max_safe_speed_mps` must not influence the
  forward PWM. Hint (Supervisor reference, ~6 lines): one module-level magnitude `_fwd_pwm_mag` raised by
  `CRUISE_SPEED / ramp * dt` per call and capped at `CRUISE_SPEED`, zeroed in the full-stop and spin branches.
- Allowed files: `dashboard/jetbot_bridge.py` (only `set_motor_speeds` and the new state variable)
- Forbidden: `tests/`, `setup/robot_dynamics.json`, `dashboard/backend/`, everything else. Python 3.6 compatible.
- Acceptance command: `.venv/bin/python tests/acceptance/test_bridge_forward_single_pwm.py -v`
- Threshold: 6/6 over four dynamics sets (field file, old file, two synthetic): steady PWM = cruise_speed +-1e-6.
- Regression: `.venv/bin/python tests/test_bridge_spin_step.py` (4 OK);
  `.venv/bin/python dashboard/test_path_following_and_modular_architecture.py`; py3.6 `ast.parse(..., feature_version=(3,6))`.
- NOT in this ticket: deployment. The Supervisor diffs against `~/03-MinhHieu/jetbot_bridge.py` and deploys the bridge
  TOGETHER with the new dynamics file once the auditor passes. The human must not copy the new dynamics file to the
  Jetson before that, or its speed becomes command-dependent (0.122-0.2 PWM).
- Depends on: —
