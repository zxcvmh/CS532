### TK-IL-08: Observation v2 for the student — person flag + ego-compensated motion (IL is the ONLY avoider)
- Role: il-policy (the Supervisor wrote the locked test first)
- Status: DONE 2026-11-22 (auditor PASS). Evidence: `test_il_observation.py` 11/11, `test_il_expert.py` 4/4, `test_il_sim_dynamics.py` 7/7; static scene beams >= 0.3 = 0.000 (limit 0.01); person motion +0.50 / -0.50 incl. while pivoting (+0.53); unseen seeds 1000-1049: static 100/0, standing 100/0, crossing 98/2, head-on 100/0; 0 invalid v2 obs; diff +169 lines (budget ~150). Carried forward: IL-06 must require an explicit `rng` when miss/false-alarm rate > 0 (`mock_expert_simulator.py:372` falls back to global `random`); `get_person_mask360` raycast duplicates `get_lidar_scan` (drift risk); false-alarm window is a fixed 20 rays.
- (history) READY (2026-11-22) — hand off ONLY after TK-IL-05 is audited and committed (same file `mock_expert_simulator.py` is in the working tree with unaudited IL-05 edits) — locked test `tests/acceptance/test_il_observation.py` (11 tests) is RED on HEAD for the right
  reason (11/11: no `extract_observation_vector_v2`, `mirror_sample_v2`, `get_person_mask360`, `OBSERVATION_DIM_V2`) and GREEN
  (11/11) on a Supervisor reference in the scratchpad. Assumptions approved by the human 2026-11-22.
- Human decision 2026-11-16: IL does ALL avoidance (static obstacle, standing person, crossing, head-on); the planner only
  gives the goal direction; reflex = brake only; the person flag comes from the camera (YOLO), plus a motion cue.
- Problem: the student sees ONE LiDAR frame: `OBSERVATION_DIM` = 41 (36 beams, goal, v/w forced to 0). A person walking at the
  robot and a wall/obstacle at the same range give the SAME vector, so no network can learn to yield to one and not to the other.
  Measured 2026-11-21: a teacher that ignores the person's velocity gets 2 % head-on success, 62 % crossing (TK-IL-05 test).
- Required (the test fixes names and semantics, read its docstring): observation v2 = 113-D = the legacy 41-D UNCHANGED
  + 36 per-beam person flags + 36 per-beam ego-compensated motion values; `config.OBSERVATION_DIM_V2` = 113 and
  `config.MOTION_NORM_M` = 0.10 (legacy `OBSERVATION_DIM` stays 41: runtime, ONNX and old tests keep working until TK-IL-06
  switches over); `dataset.extract_observation_vector_v2`, `dataset.mirror_sample_v2` (the new blocks must be mirrored too, or
  augmentation teaches the wrong side); `SimulationEnvironment.get_person_mask360(persons, miss_rate, false_alarm_rate, rng)`.
  Motion = previous scan moved into the current robot frame with the robot's own motion (dx, dy, dth), per-ray difference, median
  over each 10-deg sector, / 0.10 m, clipped to [-1, 1]: +0.5 = 5 cm closer in one 0.1 s step.
- Evidence (Supervisor prototype, scratchpad): static scenery max |motion| 0.01 while the robot pivots 14 deg or steps 2.8 cm;
  person head-on +0.50, receding -0.50 (robot still +0.50 / -0.50, pivoting +0.53 / -0.53). NOT in sim: on the REAL 2026-11-21
  recording (static room, SLAM pose as ego-motion) the motion block is quiet when the robot is still (0 % beams >= 0.3) but while
  pivoting 5.2 % of the beams read >= 0.3 on average (p95 23 % of beams in the worst sweeps) from SLAM heading error and LiDAR
  smear. TK-IL-07 must inject this noise (sim-to-real); a person is flagged AND moving, the flag gates the motion.
- Allowed files: `imitation_learning/config.py`, `imitation_learning/dataset.py`, `imitation_learning/mock_expert_simulator.py`
  (the `SimulationEnvironment` class: add `get_person_mask360`; do not change step physics or the expert here). ~150 changed lines max.
- Forbidden files: everything else, including `tests/`, `setup/robot_dynamics.json`, `model.py`, `train_bc.py`,
  `il_policy_runner.py`, `dashboard/`, `agents-doc/LOCKED_PATHS.txt`. No training, no wiring into the runtime in this ticket.
- Acceptance command: `.venv/bin/python tests/acceptance/test_il_observation.py -v` (~50 s).
- Threshold (numbers): static scenery |motion| >= 0.3 on <= 1 % of beams (mean) and max |motion| p95 <= 0.10 (robot pivoting and
  stepping); approaching person strongest-beam motion >= +0.35 median and >= +0.25 at the 10th percentile, receding mirrored
  (robot still and pivoting); detector rates within +-0.03 of miss 0.2 / false alarm 0.1 over 2000 calls; the 4 scenario classes x 50
  seeds with the teacher: success >= 90 %, collision <= 5 % each and 0 invalid v2 observations. NOT gated here (see TK-IL-09):
  the human target 2 % collision (head-on is 4 % today) and the 0.30 m clearance (printed as a METRIC, the teacher keeps 0.10 m).
- Regression commands: `.venv/bin/python tests/acceptance/test_il_expert.py -v` (4 OK);
  `.venv/bin/python tests/acceptance/test_il_sim_dynamics.py -v` (7 OK).
- Depends on: TK-IL-05 (auditor PASS + close; the scenario classes call the new expert with `privileged`).
- Assumptions: (a) v2 is APPENDED to the legacy 41-D (two code paths for a while, removed when IL-06 switches); (b) in sim the person
  flag is the true silhouette with a configurable miss/false-alarm rate, the real flag comes from YOLO clusters (TK-CV-02, not built);
  (c) on the robot the ego-motion comes from the CSM pose of consecutive sweeps (TK-SLAM-09/10), not from commands; its error in
  pivots makes the motion block noisy (measured above) and is handled in TK-IL-07; (d) one-step (0.1 s) motion is enough: a
  longer history is not needed until a test shows it; (e) 0.10 m per step is the normaliser (a 1 m/s person saturates at +-1).
  Human-approved: 2026-11-22
- Needs-human: none open.
