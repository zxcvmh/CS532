### TK-IL-06: Retrain BC → DAgger as a discrete classifier, NN drives ≥ 90 %
- Role: il-policy
- Status: DRAFT — Supervisor raises MIN_NN_SHARE_PCT to 90 in `test_il_sim_success.py` first.
- Allowed files: `imitation_learning/{model.py,train_bc.py,dagger_session.py,il_policy_runner.py}`
  (split into two attempts if > 150 changed lines)
- Acceptance command: `.venv/bin/python tests/acceptance/test_il_sim_success.py -v`
- Threshold: success ≥ 90 %, collision ≤ 2 %, NN-driven steps ≥ 90 %; only an emergency-stop rule may override.
- Depends on: TK-IL-05, TK-IL-03
