### TK-IL-02: IL action caps hardcoded instead of read from robot_dynamics.json
- Role: il-policy
- Status: SUPERSEDED by TK-IL-04 (2026-11-15)
- Problem: `imitation_learning/config.py` hardcodes `MAX_LINEAR_SPEED=0.16`, `MIN_DRIVE_SPEED=0.10`,
  `MAX_ANGULAR_SPEED=0.35`; field file says `min_drive_speed=0.19`, `max_safe_speed_mps=0.14`,
  `turn_time_90_deg_sec=0.3`. The sim deadband therefore does not match the real robot.
  Changing caps requires recollecting data and retraining.
- Acceptance: `test_sim_physics_matches_field_dynamics` in the locked test (currently FAIL: 0.1 != 0.19),
  and `test_runtime_policy_avoids_crossing_pedestrians` must still pass afterwards.
- Depends on: TK-IL-01
