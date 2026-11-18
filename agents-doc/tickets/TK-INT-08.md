### TK-INT-08: MotionAdapter.insert_low_obstacles raises TypeError on the cartographer path
- Role: integration
- Status: DRAFT (found by the Supervisor while writing TK-SLAM-08's test, 2026-11-15)
- Evidence: `motion_adapter.py:175-180` calls `self.cartographer.insert_low_obstacles(..., ttl_sec=ttl_sec)`;
  `cartographer_adapter.py:275` has no `ttl_sec` parameter → TypeError whenever the camera sends
  `low_obstacles` (`telemetry_parser.py:316-320`). Whether the Jetson currently sends them is unverified.
