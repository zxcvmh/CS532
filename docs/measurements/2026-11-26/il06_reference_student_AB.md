# TK-IL-06 reference student, A/B run (2026-11-26) — numbers only, code NOT preserved

Prototype (torch, GPU `fit()`), written by Supervisor + Antigravity in a scratchpad that is now wiped. Not a deliverable.
Setup: 117-D input (obs v2 113 + one-hot previous executed action, L/R swapped by mirror aug), 4-class classifier,
BC 1500 eps, DAgger 5 iters x 300 eps, bc_ep 60, dag_ep 40, ONE training seed. Eval = `tests/acceptance/test_il_student.py` (100 eps/class, NN drives 100 % of steps).

| class (gate) | A 256x2 (97k params) | B 128x3 (49k params) |
|---|---|---|
| A (>=90 / <=2) | 70 % succ, 0 % coll, 30 % timeout  FAIL | 87 / 1 / 12 % timeout  FAIL |
| static (>=90 / <=2) | 81 / 0 / 19 to  FAIL | 95 / 0 / 5 to  PASS |
| standing (>=90 / <=2) | 77 / 0 / 23 to  FAIL | 98 / 0 / 2 to  PASS |
| crossing (>=90 / <=2) | 93 / 7 coll  FAIL | 91 / 9 coll  FAIL |
| head-on (>=85 / <=8) | 94 / 6  PASS | 92 / 8  PASS |
| tiny train repeat (deterministic) | PASS | PASS |

Train-rollout success per DAgger iteration (beta changes, so only a trend): A 1.00 -> 0.97 -> 0.96 -> 0.94 -> 0.88; B 0.99 -> 0.95 -> 0.95 -> 0.96 -> 0.96.
Loss RISES over DAgger iterations in both (A 0.100 -> 0.144, B 0.116 -> 0.217): newly added on-policy states are harder to fit, labels are inconsistent.

Reading (Supervisor, hypotheses not yet proven):
1. Width/depth is not the lever: the bigger net timed out more (30 % vs 12 % on A). Two failure modes are separate: timeouts (L/R chatter / stuck states) and crossing collisions (7-9 %).
2. Crossing/head-on collisions may be partly TEACHER-limited (teacher keeps ~0.10 m clearance, head-on teacher ~4 % collision, TK-IL-09). The teacher's rate on the SAME eval seeds has not been measured: cheapest next experiment.
3. Imitation gap: teacher has hidden hysteresis (`_last_turn_dir`, `_pivot_origin`) the student cannot see; during student rollouts the teacher's hysteresis is updated from its own labels, not the student's executed action.
