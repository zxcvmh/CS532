### TK-INT-05: Robot veers right — trim applied to the wrong wheel (Supervisor hotfix)
- Role: supervisor (hotfix during first field run, 2026-11-15)
- Status: DONE (needs field confirmation)
- Root cause: `setup/calibrate_motors.py:217-218` measures trim on the RIGHT wheel
  (`r_spd = base_spd * trim`), but `dashboard/jetbot_bridge.py:902` and
  `motion/planning/control/motor_dynamics.py:91` applied it to the LEFT wheel → ~9 % left-over-right
  speed difference with trim 1.045 → constant right drift.
- Fix: trim now scales the right wheel in both files; PWM tests updated (7 ok, 1 skip).
  Deployed to `~/03-MinhHieu/jetbot_bridge.py` (sha256 fe334e3db1d5).
- Also fixed during deploy: Supervisor had copied `motion/planning/disparity_extender.py` to the Jetson;
  the bridge needs `dashboard/disparity_extender.py` (`evaluate(..., cmd_v)`), which caused
  `[ERROR in hardware_reflex_task] evaluate() got an unexpected keyword argument 'cmd_v'` —
  the reflex stop was inactive during that run. Restored (sha256 45c501e6bcc2).
