### TK-IL-09: Teacher quality — head-on collisions <= 2 % and soft clearance toward 0.30 m
- Role: il-policy (Supervisor writes the locked test first)
- Status: DRAFT (2026-11-22) — no locked test yet; needs the TK-IL-05 audit first.
- Problem (measured 2026-11-22 on the coder's TK-IL-05 expert, 50 seeds per class): success 96-100 % but the clearance to people and
  obstacles is always about 0.10 m (the planner margin), the human demo criterion is no contact and >= 0.30 m. Head-on person
  4 % collision (2/50). The student will copy 0.10 m, so the real robot would pass people at a hand's width.
- Evidence of what does NOT work (Supervisor reference, hard margin): a hard 0.30 m margin drops success (standing person on the path
  50 % with timeouts, static obstacle 82 %) and a moving person still collides (crossing 20 %, head-on 24 %): the plan family
  "pivot n, then forward" cannot route a 0.54 m wide person in a 2.4 m arena with that margin. Needs a COST-based margin (prefer
  clearance, accept less only when no alternative), longer plans or a replan on the person's predicted path.
- Required (to be turned into numbers by the locked test after IL-08): per scenario class (IL-08 classes, 100 seeds): collision
  <= 2 %, success >= 95 %, share of episodes with min clearance >= 0.30 m >= 80 %. Also measure the physically unavoidable fraction
  (e.g. 1 m/s head-on in a 3 m arena cannot be dodged by a robot that pivots 90 deg in 0.64 s and drives 0.284 m/s) and exclude it explicitly.
- Allowed files: `imitation_learning/mock_expert_simulator.py` (expert class only).
- Depends on: TK-IL-05 DONE, TK-IL-08 (classes and metrics).
- Assumptions: pending (to be written with the locked test).
- Needs-human: RESOLVED 2026-11-22 (human chose the Supervisor recommendation after the feasibility research; numbers are COMPUTED
  with ASSUMED r_robot 0.10 m, r_person 0.27 m, detect+latency 0.5-0.8 s, stop distance 0.05 m, none measured on the robot):
  (1) crossing person 1.0-1.1 m/s: robot brakes, no contact, clearance >= 0.30 m if the crossing point is >= ~1.0 m ahead;
  (2) head-on person <= 0.6 m/s: robot dodges, d_start >= 3.5 m, clearance ~0.3-0.5 m;
  (3) head-on person 1.0-1.1 m/s: only "robot stops, no contact" is scored, the person steps aside; 0.30 m is NOT promised.
  Physically unavoidable starts (d_start below the d_min table: 0.6 m/s 2.45 m, 0.8 m/s 3.14 m, 1.0 m/s 3.83 m, 1.1 m/s 4.18 m for
  gap 0.30 at t_dl 0.5 s) are excluded from the metric. Open measurement: stopping distance from 0.284 m/s (5 runs), detect-to-pivot latency.
