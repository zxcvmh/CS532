# STATE DIGEST (read this, not STATE.md; Supervisor regenerates it at session end; last: 2026-11-18)

## Decisions in force
- Demo: live, end of Dec 2026, 3 runs (static+goal / person crossing / head-on). Freeze 2026-12-14. IL gate 2026-12-07.
- IL is the ONLY avoider (static, standing, crossing, head-on). Planner = goal direction only. Reflex = brake only.
- Person flag (YOLO) + motion cue go into the IL observation (TK-IL-08 DRAFT). A* + DWA = plan B for the gate.
- Control loop: planner + IL on the laptop, reflex on the Jetson (home WiFi stalled 1.55 s once). Provisional until M4.
- Human-only: driving the robot, measuring physics, `setup/robot_dynamics.json`. Supervisor-only: SSH/deploy.

## Open risks
- SLAM-09 / IL-04 / INT-10: code in the working tree is green but UNAUDITED; MEAS-01 attempt 2 data unaudited.
- SLAM still unmeasured while turning with people nearby; seminar-room network unmeasured (M4).
- Jetson may auto-join WiFi `mhieuu` (autoconnect=yes). Passwords in an Antigravity transcript need rotating.
- Unrelated uncommitted coder WIP sits in the tree (teleop_gamepad.py, main.py, researcher.md...).

## Do not repeat
- `git add -u/-A/.`: explicit paths only, `git status` first.
- Deploying the wrong file (`disparity_extender.py`): diff against the file being replaced.
- SSH/sudo/WiFi/bridge on the Jetson beyond a read-only grant; only the Supervisor, command announced first.
- Validators that check a proxy (`t[-1]-t[0]`) instead of the property (no holes).
- Accepting a coder REPORT without the auditor; heading from key timing instead of LiDAR.
