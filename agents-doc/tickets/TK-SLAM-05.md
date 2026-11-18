### TK-SLAM-05: Production runs anchor OFF while the test runs it ON; test cannot see heading error
- Role: slam
- Status: DEFERRED 2026-11-16 (human approved the cut)
- Problem: `dashboard/backend/app/hardware/telemetry_parser.py:465-466` disables the anchor;
  `tests/test_hector_anchor_slam.py:31-44` uses a circular room (rotation-invariant scan) for 1 step.
