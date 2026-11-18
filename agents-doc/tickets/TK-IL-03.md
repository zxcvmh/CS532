### TK-IL-03: `test_il_pipeline.py` fakes PASS and overwrites the runtime model
- Role: il-policy
- Status: DONE 2026-11-16 (human approved closing after audit 1: code PASS, scope REJECT on docs only — the coder
  edited TICKETS.md + created docs/plans/2026-11-15-il-roadmap.md; Supervisor corrected and owns them now).
  Evidence: `Ran 6 tests ... OK`, `test_artifacts/` sha256 unchanged, `git status imitation_learning/` clean of
  artifacts; IL sim regression 89.0 % / 2.0 % unchanged. Follow-up: DAgger success is unseeded
  (16.7 % vs 66.7 % on two runs) → the new `> 0` assert can flake.
- Problem: `imitation_learning/test_il_pipeline.py` exits 0 and prints
  `[PASS] DAgger aggregation verified!` while printing `Final Success Rate: 0.0%`. It writes into
  `imitation_learning/test_artifacts/models/`, which is the FIRST path
  `dashboard/backend/app/navigation/il_onnx_bridge.py:44` loads in production.
- Allowed files: `imitation_learning/test_il_pipeline.py`
- Acceptance: (1) the pipeline test writes only into a temp dir (`tempfile.mkdtemp()`), proven by
  `git status --short imitation_learning/` showing no modified tracked files after a run;
  (2) it no longer prints `[PASS]` for a stage whose printed success rate is 0.
- Regression: `.venv/bin/python tests/acceptance/test_il_sim_success.py -v` numbers unchanged.
- Depends on: —
