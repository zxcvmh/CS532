# TK-IL-06 Teacher Baseline & Hysteresis Diagnostic (2026-11-27)

- **Evaluated object:** `GeometricExpertPolicy` (`imitation_learning/mock_expert_simulator.py`) with `privileged` obstacles on the exact locked eval generators and seeds from `tests/acceptance/test_il_student.py` (`CLASSES`: 100 episodes/class, 450 max steps, 10 Hz stop-turn-go).
- **Modes tested:**
  1. `stateful`: standard teacher (`reset_state()` once per episode; keeps `_last_turn_dir`, `_pivot_origin`).
  2. `action_synced`: teacher `_last_turn_dir` & `_pivot_origin` overwritten every step strictly from `prev_act` (the 4-D previous-action one-hot already present in the 117-D student input).
  3. `stateless`: `expert.reset_state()` before every single step (no hysteresis).

## 1. Closed-Loop Teacher Performance vs `test_il_student.py` Gates

| Class (gate: min succ / max coll) | `stateful` (succ / coll / tout) | `action_synced` (succ / coll / tout) | `stateless` (succ / coll / tout) | `stateful` p5 clear / >=0.30m |
|---|---|---|---|---|
| A (>=90% / <=2%) | 97% / 0% / 3% (PASS) | 97% / 0% / 3% (PASS) | 97% / 0% / 3% (PASS) | 0.100 m / 6% |
| static (>=90% / <=2%) | 100% / 0% / 0% (PASS) | 100% / 0% / 0% (PASS) | 100% / 0% / 0% (PASS) | 0.100 m / 0% |
| standing (>=90% / <=2%) | 100% / 0% / 0% (PASS) | 100% / 0% / 0% (PASS) | 100% / 0% / 0% (PASS) | 0.100 m / 0% |
| crossing (>=90% / <=2%) | 100% / 0% / 0% (PASS) | 100% / 0% / 0% (PASS) | 100% / 0% / 0% (PASS) | 0.030 m / 11% |
| headon (>=85% / <=8%) | 100% / 0% / 0% (PASS) | 100% / 0% / 0% (PASS) | 100% / 0% / 0% (PASS) | 0.100 m / 0% |

## 2. L↔R Pivot Chatter & State Equivalence

| Class | `stateful` chatter mean / max | `action_synced` chatter mean / max | `stateless` chatter mean / max | Same-state disagreement (`stateful` vs `action_synced`) |
|---|---|---|---|---|
| A | 0.00 / 0 | 0.00 / 0 | 0.18 / 2 | 28/9486 (0.295%) |
| static | 0.00 / 0 | 0.00 / 0 | 0.00 / 0 | 62/7593 (0.817%) |
| standing | 0.00 / 0 | 0.00 / 0 | 0.00 / 0 | 58/7973 (0.727%) |
| crossing | 0.00 / 0 | 0.00 / 0 | 0.06 / 1 | 23/7808 (0.295%) |
| headon | 0.00 / 0 | 0.00 / 0 | 0.00 / 0 | 29/7720 (0.376%) |

## 3. DAgger Label-Inconsistency Probe (25% Random Action Perturbations)

Measures how often an unsynced teacher (whose `_last_turn_dir` / `_pivot_origin` reflects its own past labels rather than the executed student action) disagrees with an `action_synced` teacher on the same state.

| Class | Overall step disagreement | Right-after-deviation step disagreement |
|---|---|---|
| A | 18.29% (14844 steps) | 38.91% (3644 steps) |
| static | 18.09% (13815 steps) | 39.66% (3371 steps) |
| standing | 18.72% (15264 steps) | 39.23% (3752 steps) |
| crossing | 14.32% (10656 steps) | 43.80% (2603 steps) |
| headon | 13.97% (10333 steps) | 41.62% (2525 steps) |

## 4. Failure Details (`stateful` vs `action_synced`)

- `stateful` / `A` (3 fails): `[(0, 'timeout', 450, 0.106, 0), (9, 'timeout', 450, 0.112, 0), (36, 'timeout', 450, 0.143, 0)]`
- `stateful` / `static` (0 fails): `[]`
- `stateful` / `standing` (0 fails): `[]`
- `stateful` / `crossing` (0 fails): `[]`
- `stateful` / `headon` (0 fails): `[]`
- `action_synced` / `A` (3 fails): `[(0, 'timeout', 450, 0.106, 0), (9, 'timeout', 450, 0.112, 0), (36, 'timeout', 450, 0.152, 0)]`
- `action_synced` / `static` (0 fails): `[]`
- `action_synced` / `standing` (0 fails): `[]`
- `action_synced` / `crossing` (0 fails): `[]`
- `action_synced` / `headon` (0 fails): `[]`

Elapsed runtime: 221.1 s.
