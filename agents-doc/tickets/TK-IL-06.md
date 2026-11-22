### TK-IL-06: Retrain BC → DAgger as a discrete classifier, NN drives ≥ 90 %
- Role: il-policy
- Status: DRAFT — Supervisor raises MIN_NN_SHARE_PCT to 90 in `test_il_sim_success.py` first.
- Allowed files: `imitation_learning/{model.py,train_bc.py,dagger_session.py,il_policy_runner.py}`
  (split into two attempts if > 150 changed lines)
- Acceptance command: `.venv/bin/python tests/acceptance/test_il_sim_success.py -v`
- Threshold: success ≥ 90 %, collision ≤ 2 %, NN-driven steps ≥ 90 %; only an emergency-stop rule may override.
- Depends on: TK-IL-05, TK-IL-03
- Notes from the TK-IL-05 audit (2026-11-22, must be in the final ticket and locked test):
  (a) the DAgger/BC data loops (`dagger_session.py:136`, `data_collector.py:71`) call the expert WITHOUT `privileged`; that path
  gets 15 % on head-on people, so labels would be wrong. Both must pass the true obstacle list `(x, y, r, vx, vy)`; add
  `imitation_learning/data_collector.py` to the allowed files. (b) Train a 4-class classifier (cross-entropy) on observation v2
  (TK-IL-08), not MSE on (v, w) (`train_bc.py:111`): MSE averages left and right detours into "straight". (c) DART is nearly a no-op in
  this sim; do not count it as data diversity. (d) `test_il_pipeline.py::test_02` is unseeded; the locked IL-06 test must fix seeds.
