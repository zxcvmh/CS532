### TK-INT-07: Bridge ramps spin PWM 0 -> turn_speed over 0.24 s; calibration used a step
- Role: supervisor (hardware)
- Status: IN PROGRESS — code + test done, not deployed / not field-checked
- Evidence: `setup/calibrate_motors.py:333` measured `turn_time_90_deg_sec` with `drive_raw` (PWM step);
  `dashboard/jetbot_bridge.py` spin branch slewed PWM at 0.8/s (0.19 / 0.8 = 0.24 s). turn_speed ==
  min_drive_speed, so the ramp is below the measured drive threshold; whether the motor moves during it
  is UNVERIFIED. Removing the ramp makes the bridge equal to the calibration by construction.
- Fix: spin PWM applied as a step. Test `tests/test_bridge_spin_step.py` (RED 0.04 PWM -> GREEN 0.19).
- Field check (human): command a 90° turn from the dashboard, confirm ≈ 90°.
