# active-memory handoff: CS532 JetBot — Agent Harness v2 & SLAM/IL repair

**Handoff #2** · 2026-11-15 · Lineage: #1 (2026-11-15): grilled + rebuilt the agent team (Claude Supervisor/Auditor + Antigravity coders), fixed renamed-folder paths, motor trim, Jetson deploy, SLAM-02 done, SLAM-03 passed audit, SLAM-04 ticket (direction C) ready; #2 (2026-11-15): closed SLAM-03/04, field tests showed map rotating with robot, TK-SLAM-07/08 superseded, method change to LiDAR-only CSM (TK-SLAM-09 READY, locked test on 2 real recordings), bridge hotfixes TK-INT-07/09 (not deployed), human plans slower pivot 0.14 / 0.54 s.

## 0. Instructions for Claude (read first)

You are continuing work from a previous chat. That chat is gone; this file is the complete context and the source of truth.

1. Read this whole file before replying.
2. Follow sections 3 (Style), 4 (Hard rules) and 5 (Corrections) in every reply, for the rest of this chat. They override your defaults.
3. Use the values in section 8 exactly. Never round, re-estimate, or "correct" them.
4. Do not suggest anything listed in section 7 (Changed / rejected) again unless the user brings it up.
5. Code word: None active. If the user types /amcodeword, start every reply with the phrase they choose (default "Yes Boss!").
6. Your first reply: at most 5 lines covering the goal, the current state, and the next step (section 11). Mention any files from section 13 that were not attached. Ask the questions in section 12 if there are any. End with "Ready to continue with <next step>?" Then wait for the user's go.

## 1. Mission
- **Goal:** Jetson Nano AI kit + LiDAR LDROBOT D500 (+ CSI camera) + GA25-370 diff-drive robot that drives to a goal while avoiding people and static obstacles; agents rebuilt so they are smarter and more accurate.
- **Done looks like:** IL sim success ≥ 90 % & collision ≤ 2 % (100 episodes, crossing pedestrians) · SLAM drift ≤ 5 cm / 3° after 360° turn · Jetson loop ≤ 50 ms, RAM ≤ 1.5 GB · real robot 10/10 goals with no contact.
- **Why:** "Mục đích chúng ta là chạy được" (user) — a robot that actually runs; course project CS532 (imitation learning).
- **Current focus:** the map must not rotate/drift when the robot pivots (SLAM, TK-SLAM-09).

## 2. About the user (as relevant to this work)
- Owns repo `/home/zxcvmh/Projects/CS532` (CachyOS laptop, Dolphin, fish/zsh).
- Drives the work; Claude = Supervisor "khi các việc khác không biết giao cho ai thôi".
- Uses Antigravity (Gemini "3.8 Flash") for researcher + coders; Claude for supervisor + auditor.
- Not a strong researcher; spot-checks only.
- Measures robot physics by hand (calibrate script, ruler); drives the robot in field tests (A/D keys, joystick) and reports what the UI shows.
- Pushes back and asks "why" — e.g. "tại sao phải làm theo test của bạn vậy? vì data bạn lỡ sai sao?" — expects an honest, evidence-based answer, including Claude's own past mistakes.

## 3. Style & communication
- **Language:** All replies in natural Vietnamese; CS terms stay English. Code, comments, commit messages, tickets, STATE.md entries in English.
- **Tone:** direct, evidence-based, no flattery, challenge flawed ideas; admit own mistakes plainly.
- **Reply length:** concise; tables + short bullets; copy-paste commands.
- **Formatting:** headings, tables, `bash` code blocks (one command per block); file refs as markdown links `path:line`.
- **Working style:** verify with real command output before claiming; calibrate any "ground truth" estimator on a synthetic known case first; root-cause debugging; minimal diffs (Ponytail/YAGNI); ask before big decisions; give a recommendation.
- **Avoid:** fake "pass", guessing, long lectures, re-asking settled questions, editing files the human owns.

## 4. Hard rules (word for word)
1. "Không, đừng đẩy lên github. Giữ nguyên dưới local thôi." → never `git push`, never PRs; local commits only.
2. `setup/robot_dynamics.json` = human field measurements; read-only (chmod 444 + `.claude/settings.json` deny). Never edit/copy/hardcode its values. User edits it themselves (`chmod u+w` … `chmod 444`).
3. "nên ghi cưỡng ngường tốc độ từ file robot_dynamics.json vì đó là thực tế khảo sát khi tôi chạy ở ngoài đời thực robot."
4. Commit one ticket per commit, only after auditor PASS, with `git add <explicit paths>` — never `git add -u` / `-A` (Antigravity has uncommitted work in the tree).
5. Never hand an Antigravity ticket while `DRAFT`. Locked tests live in `tests/acceptance/`, written by Claude, assert real metrics, must fail for the right reason before READY.
6. Only Supervisor (Claude) touches hardware (SSH/deploy/motors) — announce command first; never run motors. Before deploying, diff against the file being replaced.
7. Tests are Claude's job, never the coder's ("Coder không bao giờ tự sửa thước đo của mình").
8. Locked paths: `agents-doc/LOCKED_PATHS.txt`. `agents-doc/STATE.md` is append-only.
9. Robot code must stay Python 3.6 compatible (Jetson).
10. Never accept an Antigravity report on its word → `/audit TK-…` (quality-auditor subagent) before `/close`.

## 5. Corrections log
| # | Claude did | The user wanted |
|---|---|---|
| 1 | Assumed Jetson at 192.168.1.8 / ssh alias .16 | Jetson is `jetbot@192.168.1.18` (.8 is the laptop) |
| 2 | Asked which bridge | User runs only `~/03-MinhHieu` bridge |
| 3 | Deployed `motion/planning/disparity_extender.py` to Jetson → reflex stop dead | Correct file is `dashboard/disparity_extender.py` (restored, hash 45c501e6bcc2) |
| 4 | `git add -u` swept Antigravity's uncommitted SLAM-01 edit into d589c4a before audit | Stage explicit paths only |
| 5 | Planned to sync trim from Jetson to repo | "tôi tự cập nhật rồi nhé không cần cập nhật đâu" — don't touch trim |
| 6 | Researcher role = summarize papers the user finds | Researcher finds sources itself; user only spot-checks facts |
| 7 | Doubted turn-speed measurement method | Measured with calibrate script: 0.3 s at 0.19 → ~90° really |
| 8 | Lever arm assumed 0.08 m | User measured axle-centre → LiDAR centre = 1 cm (0.01 m) |
| 9 | Told user "LiDAR sees the robot turn the opposite way" (circular-shift sign never checked) | User said motor and LiDAR were correct; synthetic CCW +60° → k = −61 proves robot turned left. Rule: calibrate every truth estimator on a known synthetic case first (ledgered) |
| 10 | Assumed pure pivot = no translation | Real pivots slip 2–4 cm; test now compares position vs LiDAR truth |
| 11 | First /ws recording lost (recorder saved only at exit) | Recorder now flushes every 5 s |
| 12 | (Earlier tickets) estimated heading from key-press time × 5.24 rad/s | Robot actually turns after release and coasts; pose must come from LiDAR only, never from w_cmd |

## 6. Decisions
| Decision | Why |
|---|---|
| Claude = Supervisor + `quality-auditor`; Antigravity = `/researcher` `/perception` `/slam` `/il-policy` `/integration` | user's split |
| Canonical IL stack = `imitation_learning/` | newer (SpiralMLP v2, coordinator ONNX) |
| Steering: stop-turn-go | min_drive ≈ cruise, no arc band |
| **SLAM method = LiDAR-only wide-window correlative scan matching (Olson 2009), de-skew later, slower pivots** | user's choice: "Chỉ dùng phần mềm: correlative scan matching (Olson 2009) với cửa sổ tìm kiếm rộng, kết hợp de-skew…" + "Quay chậm hơn" |
| Human's "wait after key press" idea → implemented as "wait for the LiDAR", not a timer | key timing is unreliable |
| Bridge pivot = single rate at `turn_speed` for any \|w\| (TK-INT-09) | only turn_speed was calibrated; interpolation inverted when turn_speed < min_drive_speed |
| New turn values: turn_speed 0.14, turn_time_90_deg_sec 0.54, turn_time_180_deg_sec 1.08 | user's call; human edits the file |
| Real recordings (made at old 5.24 rad/s) stay as fixtures | harder case; truth from LiDAR geometry, independent of dynamics file |
| Operator workflow via `/triage` `/ticket` `/audit` `/close` + PLAYBOOK | user drives |

## 7. Changed / rejected
- 4 old agents → archived `agents-doc/archive/agents-v1/`
- Lever arm 0.08 → 0.01 m (measured); trim LEFT → RIGHT wheel
- ❌ TK-SLAM-04 direction A (slow turn 0.35–0.5 rad/s via low PWM) — motor stalls below 0.19 PWM *(note: user now proposes 0.14 for pivots — needs field check, see §12)*
- TK-SLAM-04 (stop-then-scan, DONE) → TK-SLAM-07 (heading from LiDAR after turn) → TK-SLAM-08 (pose hold during turn) → **both SUPERSEDED by TK-SLAM-09** (field: rays rotate while icon stands still, slight drift). Their locked tests remain regressions.
- ❌ d551f38 Lidar2DOdometry (user remembered it as best SLAM): 44.6° error on big turns, 24.5 cm position error
- ❌ CartographerAdapter with key-press prior: 98° error
- ❌ Any pose prior from w_cmd / motor command
- Bridge spin PWM `MIN_DRIVE_SPEED + (TURN_SPEED - MIN_DRIVE_SPEED) * w_ratio` → `spin_mag = TURN_SPEED` (TK-INT-09)
- Bridge spin ramp 0→turn_speed over 0.24 s → step (TK-INT-07)
- Hardcoded `MAX_ANGULAR=3.1416` → `(π/2)/turn_time_90_deg_sec` (5.236 now; 2.91 after the new values)
- De-skew: deferred to a later ticket

## 8. Data & facts (exact)
`setup/robot_dynamics.json` (read 2026-11-15 23:03, still old turn values — human has NOT yet applied the new ones):
| key | value |
|---|---|
| min_drive_speed | 0.19 |
| cruise_speed | 0.2 |
| trim_factor | 1.045 (user earlier said new trim 1.0471234 self-updated — see §12) |
| turn_speed | 0.19 → planned **0.14** |
| turn_time_90_deg_sec | 0.3 → planned **0.54** |
| turn_time_180_deg_sec | 0.6 → planned **1.08** |
| soft_start_ramp_sec | 0.4 |
| max_safe_speed_mps | 0.14 |
| lidar_yaw_offset_deg | 0.0 |
| motor_left/right_polarity | -1 / -1 |
| actual_speed_mps | 0.294 |
| lidar_lever_arm_m | 0.01 |

- Pivot rate: old 5.24 rad/s ≈ 30°/sweep at 10 Hz; new (π/2)/0.54 = 2.91 rad/s ≈ 16.7°/sweep. 180 = 2 × 90 (as `setup/calibrate_motors.py` test 4 derives it).
- Consumers of turn values: bridge (TURN_SPEED, MIN_DRIVE_SPEED, MAX_ANGULAR_SPEED_RADS); backend config MAX_ANGULAR_SPEED; websocket_hub A/D sends w = MAX_ANG × spd/MAX_SPEED; synthetic tests `test_slam_stop_turn_go` and `test_slam_inplace_turn` read PIVOT_RATE from the file.
- Jetson: host `nano-4gb-jp45`, L4T R32.5.2, Python 3.6.9, `jetbot@192.168.1.18` (offline at last check). Runs ONLY the bridge `~/03-MinhHieu/jetbot_bridge.py` (old file, ≠ repo `dashboard/jetbot_bridge.py`). `~/.ssh/config` alias `jetbot-at-home` → 192.168.1.16 (stale).
- Laptop 192.168.1.8 runs backend + SLAM (`run.py`, port 8000).
- **TK-SLAM-09 locked test** `tests/acceptance/test_slam_csm_turns.py` (commit 732af97): 2 recordings × variants recorded/none/inverted w_cmd. Thresholds: still heading max ≤ 5.0°, p90 ≤ 4.0°; moving worst ≤ 20°, p90 ≤ 10°; position error vs LiDAR truth ≤ 3 cm (MAX_POS_ERR_M = 0.03) at each still-segment middle sweep; occupied cells ≤ 1.25 × oracle; median update ≤ 15 ms. Run: `.venv/bin/python tests/acceptance/test_slam_csm_turns.py -v`
- Baseline HEAD: 6/6 FAIL (moving worst 62–179°, w_cmd=0 → 24.5° still error, 10 cm pos error).
- Supervisor prototype: 6/6 PASS (still max 2.8 / 4.5°, moving worst 14.5 / 12.5°, pos err 1.6 / 0.7 cm, ~5.7 ms); also passes field_turns 6/6, pivot_then_arc 7/7, stop_turn_go 2/2, inplace_turn 2/2.
- Prototype recipe: ±45° heading / ±10 cm window, coarse 1.5° → fine 0.25° / 2.5 cm, numpy-vectorised; constant-velocity prior from its own previous estimate; submap (`cartographer/cartographer_core` ActiveSubmaps2D/ProbabilityGrid2D) fed only with LiDAR-still sweeps when pose moved > 1.5 cm / 1°, every 5th sweep, or first 20 sweeps. Bottleneck was `ProbabilityGrid2D.insert_scan` Python loop (~9 ms/call), not matching (~1.5 ms).
- Ground truth: heading = exhaustive 1° circular alignment of 360 range bins vs first sweep; translation = rotate by truth heading, grid-search dx, dy ±20 cm at 1 cm vs cKDTree of first still sweep (residual 0.2–1.5 cm).
- Fixtures: `tests/acceptance/data/field_turns_2026-11-15.npz` (306 sweeps, 4 small left turns +28..+97°), `big_turns_2026-11-15.npz` (184 sweeps, 4 large L/R turns; fields ranges, w_bridge, stamp).
- Field turns (first field test): real +60/+28/+97/+32° vs SLAM +110..+152°.
- IL baseline: 89.0 % success, 2.0 % collision, 9.0 % timeout, NN 37.1 % steps.

## 9. People, terms & names
- **People:** user (zxcvmh) = operator. Old module owners: Hiếu Me (perception), Duy Hiếu (navigation), Minh Hiếu (dashboard).
- **Terms:** Supervisor / Auditor / ticket `TK-<AREA>-<NN>` / locked test / REPORT / HALLUCINATION LEDGER / stop-turn-go / CSM (correlative scan matching) / pose hold / de-skew / "map quay theo robot" (the field symptom).
- **Names in use:** `AGENTS.md`, `.agents/rules/*`, `.agents/workflows/*`, `CLAUDE.md`, `.claude/agents/quality-auditor.md`, `.claude/commands/{triage,ticket,audit,close}.md`, `agents-doc/{PLAYBOOK,TICKETS,STATE}.md`, `agents-doc/LOCKED_PATHS.txt`, `dashboard/jetbot_bridge.py`, `tests/test_bridge_spin_step.py`, `motion/planning/lidar_odometry.py`, `motion/planning/slam/csm_tracker.py`, `dashboard/backend/app/navigation/motion_adapter.py`, `dashboard/backend/app/hardware/telemetry_parser.py`.

## 10. Work state
| Item | Status | Location | Notes |
|---|---|---|---|
| TK-SLAM-01/02/03 | DONE | d589c4a, c2ad25d, fc02870 | |
| TK-SLAM-04 stop-then-scan | DONE | 0ce172e | field: map still rotated |
| TK-SLAM-07 / 08 | SUPERSEDED | tests 3f6870a, 81b4727, 7383775 | tests = regressions |
| **TK-SLAM-09 CSM** | READY; coder appears to have started | test 732af97; WIP uncommitted: `motion/planning/slam/csm_tracker.py` (untracked, 22:47), `lidar_odometry.py` (+143 lines, 22:45), `motion_adapter.py`, `telemetry_parser.py` | unaudited — wait for REPORT, then `/audit TK-SLAM-09`. Allowed files: those 4 (~250 lines); forbidden: tests/, robot_dynamics.json, jetbot_bridge.py, cartographer/ (import only) |
| TK-INT-07 spin step | DONE in repo, NOT deployed | f29d1c1 | |
| TK-INT-09 single-rate pivot | DONE in repo, NOT deployed | 2801af0 + `tests/test_bridge_spin_step.py` (RED old 0.187 / GREEN new; path_following 8 OK) | |
| TK-INT-08 low-obstacle TypeError | DRAFT | `MotionAdapter.insert_low_obstacles` passes `ttl_sec` to `CartographerAdapter.insert_low_obstacles` | needs locked test |
| TK-INT-04 velocity latch, TK-INT-06 jerk, TK-SLAM-05 | DRAFT | | |
| TK-IL-01 / IL-03 | IL-01 PAUSED · IL-03 IN REVIEW (updated 2026-11-16) | | IL-04..07 DRAFT (IL-07 depends on SLAM-09) |
| Uncommitted docs | edited | `.agents/workflows/researcher.md`, `CLAUDE.md`, `agents-doc/PLAYBOOK.md`, `agents-doc/TICKETS.md`, `imitation_learning/test_il_pipeline.py` | TICKETS.md holds SLAM-09/INT-09 status edits; commit with explicit paths at next close |
| Recorder | scratch | `/tmp/claude-1000/-home-zxcvmh-Projects-CS532/97437f70-4060-4a06-80ba-c9e24b5e53c5/scratchpad/rec2.py` | `python rec2.py <secs> <out.npz>` from repo root; reads backend /ws, flushes every 5 s; scratch dir may vanish |
| Prototype CSM | scratch | `/tmp/claude-1000/-home-zxcvmh-Projects-CS532/be307687-88be-4edd-aaf1-c04371c7c043/scratchpad/proto_csm.py` | reference only; may vanish |

## 11. Next steps
1. **Next action:** check whether Antigravity finished TK-SLAM-09 (REPORT); if yes run `/audit TK-SLAM-09` (quality-auditor), then `/close` only on PASS. If not started/finished, user runs `/slam TK-SLAM-09`.
2. User edits `setup/robot_dynamics.json` (`chmod u+w setup/robot_dynamics.json` → set turn_speed 0.14, turn_time_90_deg_sec 0.54, turn_time_180_deg_sec 1.08 → `chmod 444 setup/robot_dynamics.json`); then Claude reruns synthetic SLAM tests (stop_turn_go, inplace_turn, pivot_then_arc) + `tests/test_bridge_spin_step.py`.
3. When Jetson is online and user agrees: diff `~/03-MinhHieu/jetbot_bridge.py` vs repo bridge, deploy bridge (TK-INT-07 + 09) + dynamics file to `~/setup/`. Announce commands first.
4. Field check: does 0.14 PWM start the pivot from rest (also on low battery)? Is 90° ≈ 0.54 s?
5. Record a new fixture at the new turn speed (rec2.py) and add it to the CSM test; field-test TK-SLAM-09.
6. Later: de-skew ticket; TK-INT-08 locked test; IL tickets.

## 12. Open questions ⚠️
- Pending wrap-up question: "Khi bạn sửa xong robot_dynamics.json và Jetson online, deploy bridge mới lên Jetson?" (Deploy khi sẵn sàng / Chưa deploy) — unanswered.
- Does 0.14 PWM reliably start a pivot? (earlier finding: motor stalls below 0.19 PWM for straight drive.)
- Trim: user said 1.0471234 was updated; file still shows 1.045. Where was it saved (bridge reads only Jetson `~/setup/robot_dynamics.json`)?
- Is the uncommitted `lidar_odometry.py` / `csm_tracker.py` work a finished TK-SLAM-09 attempt or still in progress?
- Fix `run_laptop.sh` (points to missing `dashboard/backend/.venv`)? Workaround in PLAYBOOK §5.

## 13. Re-attach checklist
- Nothing to attach — everything is in the repo (`CLAUDE.md`, `agents-doc/PLAYBOOK.md`, `agents-doc/TICKETS.md`, `agents-doc/STATE.md`, tests and fixtures). Scratch files in §10 are optional and may be gone.

---
<sub>Audit: 13/13 sections · 10 rules · 12 corrections · 60 data points · secrets removed: none found · generated by active-memory</sub>
