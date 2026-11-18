### TK-INT-04: Velocity latch in AUTO mode makes the robot icon spin between scans
- Role: integration
- Status: CANCELLED 2026-11-16 (premise was wrong; human approved the cut)
  binary path: `unpack_binary_telemetry` returns `linear_speed`/`angular_speed`, the parser looks for
  `linear_vel`/`angular_vel`. coord velocities come only from the planner command. Re-scope before READY:
  decide whether the backend should use bridge-reported w at all (it is slewed, not physical either).
  Not a blocker for TK-SLAM-04.
- Problem: `dashboard/backend/app/hardware/telemetry_parser.py:406-409` never accepts v=0, so the
  interpolation at `:424-433` keeps rotating with a stale ω, then snaps back (perceived lag/jerk).
