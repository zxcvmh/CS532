### TK-IL-05: Privileged expert ≥ 95 % on the new sim
- Role: il-policy
- Status: DRAFT — locked `tests/acceptance/test_il_expert.py` written after TK-IL-04 is DONE.
- Problem: teacher success 48 %, collision 40 % on the locked seeds.
- Idea: expert may read the pedestrian's true pose and velocity (privileged teacher); student sees LiDAR only.
- Allowed files: `imitation_learning/mock_expert_simulator.py` (expert class only)
- Threshold: expert success ≥ 95 %, collision ≤ 1 % on the 100 locked seeds.
- Depends on: TK-IL-04
