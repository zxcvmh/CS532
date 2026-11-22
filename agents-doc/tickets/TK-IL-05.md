### TK-IL-05: Teacher must emit stop-turn-go commands and reach >= 95 % on the new sim (root cause: arcs the sim drops)
- Role: il-policy
- Status: READY (2026-11-22) — locked test `tests/acceptance/test_il_expert.py` written, RED on HEAD for the right reasons
  (4/4 fail: no `privileged` parameter; 4221 / 4507 / 3113 arc commands; group A 47 % success / 52 % collision, B1 72/28,
  B2 3/97) and GREEN (4/4) on a Supervisor reference expert in the scratchpad (A 99/0, B1 85/15, B2 93/7). Assumptions approved by the human 2026-11-22.
- Problem (evidence): the sim (TK-IL-04) is stop-turn-go: `SimulationEnvironment.step` executes "forward" OR "pivot" and
  ignores `w` while `v` > 0.01 (`imitation_learning/mock_expert_simulator.py` step()). `GeometricExpertPolicy.get_action`
  still steers with arcs (`w_cmd = -2.8 * lat_err` together with `v_cmd`), so every evasive steer is silently dropped.
  That, not missing pedestrian information, explains the 47 % / 52 %. The first idea of this ticket ("expert may read the
  pedestrian's true pose and velocity") is still needed for people who do NOT stop (head-on: 98 % collision without velocity
  in the prototype), but alone it does not fix the arcs.
- Required: (1) `get_action(..., apply_dart_noise=False, privileged=None)` where `privileged` = list of true obstacles
  `(x, y, radius, vx, vy)` in the world frame (teacher-only: the student never gets it); old callers (DAgger, dataset) keep
  working without it; (2) the command is always `(0, 0)`, `(v > 0, 0)` or `(0, w != 0)`: never an arc; (3) it re-plans every
  step and stops when nothing is safe. Hint (the reference, not mandatory): evaluate "pivot n steps, then forward H steps"
  plans against predicted obstacle positions (constant velocity) and the walls, keep a clearance margin, choose the plan that ends
  closest to the goal, STOP if none is clear. The reference has margin 0.10 m, horizon 24 steps; do NOT copy it blindly,
  the coder owns the final code.
- Allowed files: `imitation_learning/mock_expert_simulator.py` (the expert class only; the sim class stays as is, it is the yardstick).
- Forbidden files: everything else, including `tests/`, `setup/robot_dynamics.json`, `imitation_learning/config.py`,
  `agents-doc/LOCKED_PATHS.txt`. Robot-side code stays Python 3.6 compatible (the expert is sim-side, but keep it simple).
- Acceptance command: `.venv/bin/python tests/acceptance/test_il_expert.py -v` (~12 s).
- Threshold (numbers): group A (the 100 IL-01 episodes) success >= 95 %, collision <= 1 %; B1 crossing person who never stops
  (speed 0.25-0.6 m/s) success >= 80 %, collision <= 18 %; B2 head-on person who never stops success >= 88 %, collision <= 10 %;
  0 arc commands in every group. B is a FLOOR, not a target (the reference sits just above it). Real target 90 % / 2 %
  belongs to TK-IL-08, which must also measure the physically unavoidable fraction.
- Regression commands: `.venv/bin/python tests/acceptance/test_il_sim_dynamics.py -v` (7 OK);
  `.venv/bin/python tests/acceptance/test_il_sim_success.py TestILSimAcceptance.test_expert_baseline_reported` (prints the
  expert rate; must be >= 95 % success now); `.venv/bin/python imitation_learning/test_il_pipeline.py` — NOTE
  `test_05` was already accepted RED after IL-04 (ledger 2026-11-16); report its state, do not "fix" it here.
- Depends on: TK-IL-04 (DONE).
- Assumptions: (a) the teacher may use privileged true obstacle positions and velocities (the student will get a person flag and
  motion from TK-IL-08, not the true state); (b) the B-group floors above are provisional until the unavoidable fraction is
  measured; (c) the pedestrian models of groups A and B are the Supervisor's (cooperative person for A, constant-velocity
  non-stopping person for B), not real human data; (d) a clearance of 0.10 m is accepted in sim for the teacher, the 0.3 m
  demo clearance is enforced in TK-IL-08.
  Human-approved: 2026-11-22
- Needs-human: none open.
