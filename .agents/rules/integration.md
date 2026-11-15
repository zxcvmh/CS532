---
trigger: glob
globs: "dashboard/backend/**, dashboard/jetbot_bridge.py, dashboard/*.sh, motion/planning/control/**, motion/planning/controller.py, motion/planning/cbf_safety_filter.py"
description: "Runtime wiring, bridge, motor command and safety-filter rules."
---

# Integration rules

- Data path: LiDAR/camera on Jetson (`dashboard/jetbot_bridge.py`) → WebSocket → backend
  (`dashboard/backend/app/`) → `navigation/coordinator.py` picks IL policy or classical planner
  (`USE_IL_NAVIGATION`) → velocity command → bridge → PWM.
- The bridge has its own reflex stop and DDE evasion. Any change to stopping distance must keep the
  bridge reflex as the last line of defence; never disable it to make a test pass.
- PWM conversion lives in two places (`dashboard/jetbot_bridge.py` and
  `motion/planning/control/motor_dynamics.py`). Both read `setup/robot_dynamics.json`. If you change
  one formula, check the other and say so in the report.
- Prove the backend still starts: `cd dashboard/backend && ../../.venv/bin/python -c "import app.main"`.
- Jetson deploy steps are written as a script or command list for the Supervisor; you never run them.
