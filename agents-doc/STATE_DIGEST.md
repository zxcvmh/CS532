# STATE DIGEST (read this, not STATE.md; Supervisor regenerates it at session end; last: 2026-11-22)

## Decisions in force
- Demo: live, end of Dec 2026, 3 runs (static+goal / person crossing / head-on). IL gate 2026-12-07, freeze 2026-12-14.
- IL is the ONLY avoider; planner = goal direction; reflex = brake only. Teacher may use privileged obstacles, student never.
- Fast pivot accepted (human): turn_speed 0.12, 0.64 s / 90 deg = ~14.4 deg per sweep. Tracker clamp is a constant, not read from dynamics.
- Observation v2 (113-D = legacy 41 + person flag + ego-compensated motion) is TK-IL-08, READY; student = 4-class MLP classifier, BC->DAgger (recommended, human not yet confirmed).
- FROZEN by human: WiFi/M4, TK-TEST-01, low-battery pivot start, chmod 444 on dynamics, bridge RAM 1.55 GB (budget 1.5).

## Open risks
- Motion block is noisy while pivoting on real data (5 % beams false, p95 23 %) and untested with real people; real person flag (YOLO) not built.
- Teacher keeps only 0.10 m clearance (demo needs 0.30 m); head-on 4 % collision -> TK-IL-09, needs the human's person-speed protocol.
- DAgger/BC loops call the expert WITHOUT `privileged` (15 % head-on): must be fixed in TK-IL-06.
- Foreign uncommitted files: TK-SLAM-09.md edit, needs-human/TK-TEST-01.md.

## Do not repeat
- `git add -u/-A/.`; explicit paths. No push. Check `git status` first.
- Locked tests must not depend on a file the human edits (dynamics): pin or copy it.
- Count exactly the quantity a rule uses (score runs, not error runs); calibrate on a known case.
- `pgrep -f` matches your own shell: anchor the pattern. Scratchpad is wiped: record numbers in tickets.
- Never accept a coder REPORT without the auditor.
