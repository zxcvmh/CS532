### TK-INT-11: Emergency-stop paths are unproven (demo safety)
- Role: integration (Supervisor measures first)
- Status: DRAFT — needs a measurement before any code: (1) dashboard / API `emergency_stop` (laptop -> WiFi -> bridge)
  and (2) `teleop_gamepad.py` button A, which calls `robot.stop()` in its OWN process, while the bridge is running.
  Nothing shows that either pre-empts an active automatic command or how long motors take to reach 0.
- Plan: Supervisor writes a measurement protocol (trigger -> motor PWM 0, 20 trials, p95) and, if the gamepad path
  does not pre-empt the bridge, a locked test for a gamepad -> bridge stop that wins over every other source.
- Threshold (proposal, to confirm after the Jetson measurement): trigger -> PWM 0 p95 <= 300 ms for the gamepad path.
- Depends on: Jetson reachable from the home network.
