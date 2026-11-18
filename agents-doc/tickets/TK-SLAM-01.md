### TK-SLAM-01: Backend cannot start — `arena_bounds` interface mismatch
- Role: slam
- Status: DONE 2026-11-15 — auditor PASS (session "CartographerAdapter arena_bounds parameter").
  Fix landed in `cartographer_adapter.py:41-43,52` but was swept into Supervisor commit d589c4a
  before the audit (process violation, see ledger). Follow-up: drop the catch-all `**kwargs`.
- Problem: `cd dashboard/backend && ../../.venv/bin/python -c "import app.main"` →
  `TypeError: CartographerAdapter.__init__() got an unexpected keyword argument 'arena_bounds'`.
  Caller: `dashboard/backend/app/navigation/motion_adapter.py:22-29`.
  Callee: `dashboard/backend/app/navigation/cartographer_adapter.py:32-42`.
  Introduced by the unfinished Hector/anchor refactor (commits 5719eba, a5879cf).
- Allowed files: `dashboard/backend/app/navigation/cartographer_adapter.py`
- Acceptance command: `cd dashboard/backend && ../../.venv/bin/python -c "import app.main"`
- Threshold: no `arena_bounds` TypeError. Report the NEXT error verbatim if one appears (do not fix it).
- Regression commands: `.venv/bin/python tests/test_hector_anchor_slam.py`;
  `cd dashboard && PYTHONPATH=../motion:backend ../.venv/bin/python test_path_following_and_modular_architecture.py`
  (baseline 2026-11-15: 7 ok, 1 skipped)
- Depends on: —
