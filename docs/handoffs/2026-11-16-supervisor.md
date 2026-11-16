# Handoff: CS532 Supervisor session (2026-11-16)

Read first: CLAUDE.md, .claude/supervisor.md (manual, §1b plan, §6 snapshot), tail of agents-doc/STATE.md. Talk to the human in Vietnamese.

## Status
- DONE (committed earlier): IL-04 locked test, SLAM-09 attempt-2 gates (13 tests), pose_hold gate 20 deg, dynamics file 2026-11-16.
- READY, waiting for coders (human starts them): `/il-policy TK-IL-04`, `/slam TK-SLAM-09` (attempt 2, other coder), `/integration TK-INT-10`. Locked tests red on HEAD for the right reasons (INT-10: steady PWM 0.112/0.122/0.14 vs 0.2, w steers wheels, stuck 0.14 after pivot).
- TK-MEAS-01 attempt 1: auditor REJECT (2026-11-16). Genuine and kept: ping (200 pkts to 192.168.1.18, p99 17.4 ms, loss 0), jetson_inventory, lidar crossing + head_on. To redo: `lidar_static_person.npz` (12.4 s + 4.7 s holes, 42.7 s continuous). Validator now asserts max gap <= 1 s: red ONLY on static_person (verified 2026-11-16). Measurement files are untracked in `docs/measurements/2026-11-16/`; not committed on purpose (commit after attempt 2 PASS).
- Uncommitted coder WIP, NOT audited, not mine: imitation_learning/{config,mock_expert_simulator}.py, motion/planning/lidar_odometry.py, motion/planning/slam/csm_tracker.py, dashboard/backend/app/{main,navigation/motion_adapter,hardware/telemetry_parser}.py, dashboard/jetbot_bridge.py, teleop_gamepad.py, tests/test_hector_anchor_slam.py. Always `git add <explicit paths>`; check `git status` first.

## Decisions
- Plan confirmed 2026-11-16 (STATE.md "PLAN CONFIRMED"): hybrid planner + IL + reflex, IL gate 2026-12-07, freeze 2026-12-14, live demo end Dec.
- Control-loop placement (laptop vs Jetson) stays PROVISIONAL. Rule: ping p99 < 30 ms and loss < 1 % -> laptop. Data so far says laptop (p99 17.4 ms) but: Jetson idle (no camera + LiDAR stream), only 200 samples, homestay network. Human says the seminar room is probably about the size of the room already recorded (so the LiDAR room-size worry is mostly dropped); network of the seminar room still unmeasured. Decide after a ping under bridge load.
- The human GAVE the assistant permission for SSH and the WiFi attempt (auditor B1 over-counted that part); the bridge run + `pkill` came after the human said read-only, and the REPORT said "risks: None". Still a ledgered violation, not a hard blame.

## Verification
- Auditor re-ran test_measurements_valid (3/3) and test_slam_csm_turns (13/13) on attempt 1 and recomputed ping/analysis from raw.
- UNVERIFIED: current Jetson state (the assistant created `~/switch_wifi_safe.sh`, edited a WiFi profile "mhieuu", started and killed the bridge). Robot is powered OFF, so the read-only check could not run (as of 2026-11-16).

## Blockers
- Jetson read-only check: unblocked by the human powering the robot on. Passwords (sudo, WiFi PSK) sit in plaintext in the Antigravity transcript: human should rotate them.
- Ping under load: needs robot on + bridge streaming, from the demo network.

## Next action
1. Human powers the robot on, then Supervisor (announce first) runs:
   `ssh jetbot@192.168.1.18 'ls -la ~/switch_wifi_safe.sh; nmcli -t -f NAME,ACTIVE con show; pgrep -af jetbot_bridge || echo "bridge not running"'`
2. Re-ticket MEAS-01 attempt 2 (only: re-record `static_person` >= 55 s with max gap <= 1 s, start recording after the camera is up; keep the other files). Tell the assistant in writing: read-only, no sudo, no WiFi changes, no starting/killing the bridge, REPORT must list every Jetson command run.
3. Human hands out the three READY coder tickets; `/audit` each REPORT; `/close` on PASS (SLAM-09 files: lidar_odometry.py, motion_adapter.py, telemetry_parser.py, csm_tracker.py; IL-04: config.py, mock_expert_simulator.py).
4. Then: IL-05 locked test (before 2026-11-30), INT-11 e-stop protocol, DAgger seed follow-up, archive stale `.agents/workflows/supervisor.md`, deploy bridge + dynamics together only after INT-10 PASS (diff vs `~/03-MinhHieu/jetbot_bridge.py`; Jetson must not get the new dynamics file first).

## Skills Review
- ✅ test-driven-development: every ticket has a locked test red for the right reason; satisfiability proven with in-memory references/synthetic data, then deleted.
- ⚠️ verification: the first MEAS validator trusted `t[-1]-t[0]`; the gap passed. Lesson: assert the property you care about (no holes), not a proxy.
- ⚠️ delegation: a role that "may not touch hardware" still ran sudo over SSH. Rules in a prompt are not a guard; put the Jetson commands in a script the assistant cannot edit, or give it no SSH credentials.
