### TK-IL-01: Runtime policy must pass the crossing-pedestrian acceptance test
- Role: il-policy
- Status: PAUSED 2026-11-15 — sim dynamics are wrong (see TK-IL-04); re-run as regression after TK-IL-06.
  Plan: `docs/plans/2026-11-15-il-roadmap.md`
- Locked test: `tests/acceptance/test_il_sim_success.py` (evaluates exactly what
  `il_onnx_bridge.get_il_policy_runner()` loads, 100 seeds, 1 crossing pedestrian + 0-2 statics).
- Baseline 2026-11-15 (Supervisor run):
  - runtime policy: success **89.0 %**, collision **2.0 %**, timeout 9.0 % — failed seeds
    `[17, 34, 38, 44, 48, 49, 54, 62, 69, 83, 86]`
  - expert (`GeometricExpertPolicy`, the teacher): success **48.0 %**, collision **40.0 %**
  - the neural net drives only **40.2 %** of control steps; the other ~60 % are hand-written rules in
    `imitation_learning/il_policy_runner.py` (terminal approach, yield, escape pivot, bypass latch,
    flank follow, pivot-to-goal).
- Decision (human → Supervisor, 2026-11-15): goal is a robot that runs, so the rules+NN hybrid is
  accepted for now. The NN-driven share of control steps must be REPORTED (baseline 40.2 %) and must
  not drop below 30 %; a later ticket fixes the expert so the NN can take over more.
- Allowed files: `imitation_learning/il_policy_runner.py`
- Hint: start from the failed seeds; run one seed and log the step where it times out or collides.
- Acceptance command: `.venv/bin/python tests/acceptance/test_il_sim_success.py -v`
- Threshold: `test_runtime_policy_avoids_crossing_pedestrians` passes (success ≥ 90 %, collision ≤ 2 %, NN-driven steps ≥ 30 %).
- Depends on: —
