# STATE DIGEST (read this, not STATE.md; Supervisor regenerates it at session end; last: 2026-11-26)

## Decisions in force
- Demo: live, end of Dec 2026, 3 runs (static+goal / person crossing / head-on). IL gate 2026-12-07, freeze 2026-12-14.
- IL is the ONLY avoider; planner = goal direction; reflex = brake only. Teacher may use privileged obstacles, student never.
- Fast pivot accepted: turn_speed 0.12, 0.64 s / 90 deg. Robot speed stays 0.284 m/s. Observation v2 = 113-D (TK-IL-08 DONE).
- Student = 4-class MLP, input 117-D (obs v2 + one-hot previous action), BC->DAgger. Locked test `tests/acceptance/test_il_student.py` committed, RED.
- 2026-11-26 human allows `git push` of main to origin (zxcvmh/CS532); only explicit paths are committed.
- FROZEN: WiFi/M4, TK-TEST-01, low-battery pivot start, chmod 444 on dynamics, bridge RAM 1.55 GB.

## Open risks
- TK-IL-06 not reachable yet: best prototype (128x3) = static 95, standing 98, head-on 92/8 OK; class A 87 % (12 % timeout), crossing 91 % / 9 % coll (gate 90/2). Size is not the lever; DAgger loss rises (teacher hysteresis hidden from student). Numbers: `docs/measurements/2026-11-26/`. Prototype code lost.
- Unknown: teacher's own crossing/head-on collision on the eval seeds (clearance only 0.10 m; TK-IL-09) -> gate may be teacher-limited.
- Motion block noisy while pivoting on real data; real person flag (YOLO) not built.
- Foreign uncommitted files: TK-SLAM-09.md edit, needs-human/TK-TEST-01.md, robot_dynamics.json, docs/measurements/2026-11-22/.

## Do not repeat
- `git add -u/-A/.`; explicit paths. Check `git status` first.
- Locked tests must not depend on a file the human edits (dynamics).
- `pgrep -f` matches your own shell. Scratchpad (/tmp) is wiped: keep code in the repo (e.g. scripts/ or docs/), not only numbers.
- Never accept a coder REPORT without the auditor; never claim a background job is alive without checking.
