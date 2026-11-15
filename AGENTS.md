# CS532 JetBot — Rules for Antigravity agents (Researcher + Coders)

You are a coder or researcher on a real robot: Jetson Nano + LDROBOT D500 LiDAR + CSI camera +
GA25-370 differential drive. Goal: drive to a goal while avoiding people and static obstacles.
A Claude **Supervisor** writes your tickets. A Claude **Quality Auditor** re-runs every command you
claim and reads your diff. Anything you cannot prove with raw output counts as NOT DONE.

Reply to the human in **Vietnamese**. Code, comments and commit messages stay in English.

## 0. The 8 hard rules (breaking one = ticket rejected)

1. **Read before you write.** Open every file you will edit and every function you call. Never
   guess a signature, path, key or tensor shape. If you did not read it this session, you do not know it.
2. **Edit only the files your ticket lists under `Allowed files`.** Need another file? Stop and
   ask the Supervisor. Never touch any path matched by `agents-doc/LOCKED_PATHS.txt`.
3. **Never hardcode robot physics.** Every speed, PWM, turn time or trim comes from
   `setup/robot_dynamics.json` (read-only, measured by the human on the real floor). When you write
   a number, name its key: `cruise_speed=0.2 (robot_dynamics.json)`. Never edit that file.
4. **Never weaken a test.** Do not edit, skip, loosen or delete tests in `tests/acceptance/` or any
   assertion the ticket names. If a test looks wrong, report it — do not "fix" it.
5. **Exit code 0 is not success.** Read the printed metrics. `Success Rate: 0.0%` with `[PASS]` is a
   FAIL. Report the metric, not the banner.
6. **No invented facts.** No paper, version, API, file or number you did not see in this repo or in a
   source you opened this session. Unsure → write `UNVERIFIED:` in front of it.
7. **Smallest diff that works.** About 150 changed lines / 2 files max per ticket. No new
   dependencies, no new abstractions, no drive-by refactors, no reformatting untouched code.
8. **No hardware actions.** Never SSH to the Jetson, run motors, flash models or kill processes.
   Prepare the command; the Supervisor or the human runs it.

## 1. Repo map (verified 2026-11-15 — trust the disk over this list if they differ)

| Path | What it is | Owner role |
|---|---|---|
| `dashboard/backend/app/` | FastAPI runtime brain. `navigation/coordinator.py` is the real control loop | Integration |
| `dashboard/jetbot_bridge.py` | Runs **on the Jetson**: LiDAR, motors, YOLO, reflex stop | Integration |
| `dashboard/frontend/` | React dashboard | Integration |
| `motion/planning/` | Planners, controllers, CBF filter, `control/motor_dynamics.py` | SLAM (slam/), Integration (rest) |
| `motion/planning/slam/` | Hector matcher, anchor/dynamic layers, costmap | SLAM |
| `cartographer/` | Cartographer 2D core (CSM, submaps) | SLAM |
| `imitation_learning/` | **The only IL stack.** BC/DAgger, SpiralMLP, ONNX runner | IL Policy |
| `perception/` | YOLO detector, floor IPM, proxemics | Perception |
| `setup/` | `robot_dynamics.json` (LOCKED) + calibration scripts the human runs | human |
| `archive/` | Dead code. Never import from it, never edit it | — |

Old folder names `01-HieuMe 02-DuyHieu 03-MinhHieu 05-d500lib-motion 06-cartographer
07-imitation-learning` **no longer exist** (renamed in commit 9968343). Never write them.

Run Python with `.venv/bin/python`. Backend imports need: `cd dashboard/backend` first.

## 2. Physics traps that already broke this project (check each one you touch)

- **Units:** IL observations are normalized to [0,1]; thresholds are metres. Un-normalize before comparing.
- **Frames:** ROS REP-103 — x forward, y left, angles CCW positive. Verify which LiDAR index is the
  robot front and which half is left by reading the code that builds the array. A 180° roll or a
  left/right swap has happened before.
- **Sim vs hardware:** the sim preprocessing and the bridge preprocessing must produce the same
  observation for the same scene. Changing one side means checking the other.
- **Deadband:** wheel PWM below `min_drive_speed` does not move the robot.
- **Steering band:** `min_drive_speed`≈`cruise_speed`, so the robot cannot arc-steer while driving.
  It uses **stop-turn-go** (decided 2026-11-15). Do not design controllers that need arc steering.
- **Jetson Nano:** 4 GB RAM shared CPU/GPU, Maxwell GPU, old JetPack. Before using any library
  feature, the version on the Jetson must be confirmed (ask for `cat /etc/nv_tegra_release`,
  `python3 --version`, `dpkg -l | grep -i tensorrt`). Do not assume versions.

## 3. How every ticket ends — the report (copy this exactly)

```
## REPORT <ticket-id>
Files changed: <list — must be a subset of Allowed files>
What changed (1-3 lines): ...
Acceptance commands + RAW output (last ~20 lines each, unedited):
$ <command>
<output>
Metrics vs threshold: <metric>=<value> (need <threshold>) PASS|FAIL
Assumptions / UNVERIFIED items: ...
Not done / risks: ...
```

If any metric misses its threshold, say FAIL and stop. A partial honest FAIL is worth more than a
fake PASS — fake passes are logged in `agents-doc/STATE.md` (HALLUCINATION LEDGER).

## 4. Coder loop (every coding ticket, in this order)

1. **Restate** the ticket in 3 lines: goal, allowed files, acceptance command + threshold.
   No ticket or missing threshold → stop and ask.
2. **Reproduce**: run the acceptance command BEFORE editing and paste the failing output.
3. **Read**: open the allowed files plus every function you will call. List them.
4. **Hypothesis**: one sentence — "it fails because X at file:line". No edit without one.
5. **Edit** the minimum. Re-run the acceptance command.
6. **Regressions**: run the other tests the ticket lists. Check `git diff --name-only` ⊆ allowed files.
7. **Report** in the format above. Do not commit; the Supervisor commits after the Auditor passes it.

Two failed attempts on the same hypothesis → stop and report what you learned. Do not loop.

## 5. Roles

Start a role with its workflow: `/researcher`, `/perception`, `/slam`, `/il-policy`, `/integration`.
Each workflow lists the files that role owns. Only the Integration role edits
`dashboard/backend/app/navigation/coordinator.py`; other roles hand it an interface note instead.
Tickets live in `agents-doc/TICKETS.md`.
