# Handoff 2026-11-22: CS532 JetBot (Supervisor session)

Read first: `CLAUDE.md`, `.claude/supervisor.md` (section 6 refreshed today), `agents-doc/STATE_DIGEST.md` (not STATE.md), `agents-doc/TICKETS.md`.
Talk to the human in Vietnamese; code, tickets, commits, STATE.md in English. Local commits only, never push; explicit `git add <paths>`.
Reconciled against the repo at the end of this session (`git status`, `git log`, `pgrep`): HEAD `711210b`, tree clean except two files that are NOT ours.

## Status
- DONE and verified this session (command re-run by the Supervisor on 2026-11-20..22, tree at the commit shown):
  - TK-SLAM-10 `2bb93a2` (status TRACKING/WEAK/LOST, clamp pinned, pose unchanged): `test_slam_csm_status.py` 4/4, `test_slam_csm_dynamics_independent.py` 1/1, `test_slam_csm_turns.py` 13/13, `test_hector_anchor_slam.py` 4/4.
  - TK-INT-12 `b351eea` (pivot 0.12 / 0.64 s / 1.28 s): `test_pivot_rate_vs_sweep.py` 2/2; field check on a NEW recording `docs/measurements/2026-11-21/lidar_turns_064s_21_11{,_dedup}.npz`: still err max 3.6 deg, moving worst 6.4 deg (was 12-14).
  - TK-IL-04, TK-INT-10: closed on the human's word "auditor PASS" (acceptance re-run green); code was already in `e15a8e1`.
  - TK-IL-05 `71fa939` (teacher stop-turn-go native, optional `privileged`): `test_il_expert.py` 4/4 (A 97/0, B1 99/1, B2 100/0), `test_il_sim_dynamics.py` 7/7; auditor PASS incl. unseen seeds.
- READY, not started: **TK-IL-08** (observation v2: person flag + ego-compensated motion, 113-D). Locked test `tests/acceptance/test_il_observation.py` 11 tests, RED 11/11 on HEAD (missing API), GREEN 11/11 on a Supervisor reference that lived in the scratchpad (scratchpad was wiped; the numbers are in the ticket).
- DRAFT: TK-IL-06 (BC -> DAgger classifier; notes from the IL-05 audit appended), TK-IL-07 (sim-to-real), TK-IL-09 (teacher clearance 0.30 m soft, head-on <= 2 %), TK-INT-11 (e-stop paths), CV tickets, TK-SLAM-07/08 superseded.
- Deployment state (as of 2026-11-21; re-verify: `ssh jetbot@192.168.1.18 'md5sum ~/03-MinhHieu/jetbot_bridge.py ~/setup/robot_dynamics.json'` and compare with the repo): Jetson `jetbot_bridge.py` is byte-identical to `dashboard/jetbot_bridge.py` (INT-10 bridge) and `~/setup/robot_dynamics.json` equals the repo file (0.12 / 0.64 / 1.28). Bridge was running (PID 9831, ~1.55 GB RSS). The Supervisor deployed nothing this session; the human edited the Jetson file.

## Decisions
- IL is the ONLY avoider; reflex = brake; planner = goal direction (unchanged, 2026-11-16).
- Fast pivot accepted by the human (reaction time over map quality): 0.64 s per 90 deg = ~14.4 deg per LiDAR sweep, above the clean 8 deg/sweep. Ceiling in the locked test is 15. Why: robot must react to obstacles.
- Tracker clamp `CSM_MAX_DTH_STEP_RAD` is a constant (0.37047 rad), independent of `robot_dynamics.json`. Why: a human edit of the pivot time silently broke the audited SLAM-09 test and golden.
- LOST = 5 consecutive sweeps with score < `SCORE_LOST` 0.10 (WEAK is < 0.64); warmup (submap < 2 range data) = TRACKING. Why: real turns reach score 0.17, white noise 0.01-0.05.
- Teacher root cause: the old expert emitted arcs, the stop-turn-go sim drops `w` while driving (47 % / 52 % collision). Teacher may use privileged true obstacles; the student never does.
- Observation v2 APPENDS 72 channels to the legacy 41-D (legacy untouched until IL-06 switches).
- RECOMMENDED, not yet decided by the human: student = small MLP classifier over 4 actions (cross-entropy, BC -> DAgger, mirror augmentation); 1-D circular CNN only if MLP < 90 %; no SpiralMLP/RNN/RL for now. Why: `train_bc.py:111` uses MSE on (v, w), which averages left and right detours into "straight".
- FROZEN by the human (2026-11-22, reopen only when asked): WiFi/M4 seminar-room test, `needs-human/TK-TEST-01.md`, Jetson low-battery start from rest at `turn_speed` 0.12, `chmod 444 setup/robot_dynamics.json`, bridge RAM vs the 1.5 GB budget.

## Changes
Commits this session, newest first: `711210b` IL-06 notes; `71fa939` IL-05 code; `3813c49` IL-08 READY; `8bdfdb0` IL-08 test + IL-09; `212ae11` IL-05 READY; `ee705bf` IL-05 test; `b351eea` INT-12 close + recording; `3d07377` dynamics; `2bb93a2` SLAM-10; `7ac091f`/`643b952` SLAM-10 attempt 2; `af244d3` IL-04/INT-10 close. Branch `main`, no remote use.
Uncommitted, NOT ours: `agents-doc/tickets/TK-SLAM-09.md` (status text edit by the harness/coder), `agents-doc/needs-human/TK-TEST-01.md` (a coder's frozen question).

## Verification
- UNVERIFIED: motion block on REAL people (only a static room recording exists); real person flag (YOLO, TK-CV-02 not built); low-battery pivot start; TK-IL-08 reference passes the test but a coder implementation does not exist yet; student accuracy (no new training done); nothing was deployed or driven by the Supervisor.
- Measured but small-sample: status recall 8/10 (exactly at the 0.80 limit), 2 sweeps > 6 deg in the new recording and none flagged.
- Real-data finding: on `lidar_turns_064s_21_11_dedup.npz` the ego-compensated motion block is quiet when the robot is still (0 % beams >= 0.3) but 5.2 % of beams read >= 0.3 while pivoting (p95 23 %), from SLAM heading error and LiDAR smear. TK-IL-07 must inject it.
- Teacher on 4 classes x 50 seeds (coder's expert): static 100/0, standing person 100/0, crossing 100/0, head-on 96/4; min clearance is always ~0.10 m (demo criterion 0.30 m). A hard 0.30 m margin collapses success in the reference (standing 50 %).
- `test_il_pipeline.py::test_05` stays the accepted RED; `test_02` is unseeded (expert 75-100 % between runs).

## Blockers
- TK-IL-09 final thresholds: human must choose the demo protocol for a person who walks AT the robot (max 0.6 m/s, or robot stops early and the person steers). At 1 m/s in a 3 m arena the robot (0.284 m/s, pivot 0.64 s) physically cannot dodge. Unblocked by the human's answer.
- Student model choice: needs the human's yes (or a counter-proposal) before the TK-IL-06 test is written.

## Next action
1. Human, in Antigravity: `/il-policy TK-IL-08`, then give the REPORT to the `quality-auditor`.
2. Supervisor on the audit PASS: `/close TK-IL-08` (explicit paths: `imitation_learning/{config,dataset,mock_expert_simulator}.py` + ticket/index/STATE).
3. Supervisor meanwhile (needs the human's yes on the model): write the locked test `tests/acceptance/test_il_student.py` for TK-IL-06 (4-class classifier on observation v2, labels from the PRIVILEGED teacher, fixed seeds, NN-driven steps >= 90 %, success >= 90 %, collision <= 2 %); it must be red for the right reason. Needed before the IL gate 2026-12-07 (freeze 2026-12-14, demo end of December).
First command of a fresh session: `git status --short && git log --oneline -5 && cat agents-doc/STATE_DIGEST.md`.

## Skills Review
- OK `superpowers:test-driven-development`: every new locked test (SLAM-10 status, INT-12 pivot rate, IL-05 expert, IL-08 observation) was run RED on HEAD and GREEN on a scratchpad reference before READY.
- WARN `verification-before-completion`: the `big` recording failure of SLAM-09 appeared only after the human edited the dynamics file, i.e. my locked tests depended on a mutable human file. Fixed by the dynamics-independence test; check other tests for the same coupling (`test_il_sim_dynamics.py` reads the file by design).
- WARN own error: SLAM-10 ticket claimed "longest bad run 3 sweeps" counting heading error instead of score < 0.64 (really 6-8). Always count the exact quantity the rule uses.
- WARN scratchpad is wiped between days: the IL-05 and IL-08 reference implementations were lost once. Keep a reference only as long as the ticket needs it, record the measured numbers in the ticket (done), and rebuild from the test's docstring if needed.
- WARN shell gotcha hit three times: `pgrep -f "python scripts/record_lidar.py"` matches the calling shell's own command line and `kill` then killed my shell. Use an anchored pattern (`^\.venv/bin/python scripts/record_lidar`) or the PID of the background job.
- Environment: no guardrail checks that a locked test does not read a file a human edits (the dynamics coupling above); `scripts/check_ticket.py` could flag locked tests that read `setup/robot_dynamics.json` without pinning it.
