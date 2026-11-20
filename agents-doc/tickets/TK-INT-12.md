### TK-INT-12: Limit the pivot rate to <= 15 deg per LiDAR sweep (human setting + field check)
- Role: supervisor (the human measures and edits `setup/robot_dynamics.json`; no coder edits)
- Status: READY (2026-11-20) — acceptance 2/2 GREEN with the human's 0.64 s file (14.4 deg/sweep); field check on a NEW recording still open, so not DONE.
- (earlier) Status: READY (2026-11-20) — human-owned: the locked test `tests/acceptance/test_pivot_rate_vs_sweep.py` is RED on today's file (17.4 deg/sweep at 0.53 s) and goes green once the human applies the new dynamics (0.64 s -> ~14.4 deg/sweep). No coder.
- Human decision 2026-11-20: "turn fast so the robot reacts to obstacles"; pivot `turn_speed` 0.12, 90 deg in 0.64 s, 180 deg in 1.28 s accepted. This is ABOVE the clean 8 deg/sweep limit (the ceiling is therefore 15 deg/sweep, a human trade-off, not a physical limit). Expected cost from the recordings: bin 10-17 deg/sweep -> max error 9.5 deg, ~2.8 % sweeps above 6 deg; TK-SLAM-10 flags those. The tracker's own map-write gate (`step_rot <= 14 deg`) sits right at 14.4 deg/sweep, so the map is mostly NOT written during pivots (acceptable; map is written when still).
- Problem: heading error of the CSM tracker (TK-SLAM-09) during a pivot grows with the rotation inside ONE LiDAR sweep.
  Measured by the Supervisor on the two real recordings `tests/acceptance/data/{field,big}_turns_2026-11-15.npz`
  (tracker heading vs the LiDAR-derived truth of `test_slam_csm_turns.py`, pooled moving sweeps; scratchpad
  analysis, not in the repo):

  | true rotation per sweep | mean err (field / big) | max err | sweeps with err > 6 deg |
  |---|---|---|---|
  | <= 8 deg | 1.0 / 1.4 deg | 4.2 deg | 0.0 % |
  | <= 10 deg | | 6.4 deg | 1.0 % |
  | <= 17 deg | | 9.5 deg | 2.8 % |
  | 30-60 deg | n/a / 8.9 deg | 14.3 deg | |

  Errors last 1-3 sweeps (0.1-0.3 s) and then recover, as the human saw in the field. Current pivot: 90 deg in
  `turn_time_90_deg_sec` = 0.53 s -> ~170 deg/s; LiDAR sweep period measured from the fixtures = 0.102 s (median) ->
  ~17 deg per sweep. The tracker already refuses to write the map above 14 deg/step (`csm_tracker.py`), so today's
  pivot is above its own clean-motion limit.
- Required (original, clean target; the accepted ceiling is 15, see decision above): pivot rate <= 8 deg per sweep = <= ~78 deg/s = `turn_time_90_deg_sec` >= ~1.15 s (about 2.2x slower than the
  0.53 s in the file today). The number is OUR measurement on two recordings (10 sweeps with err > 6 deg in total), not a
  published figure: provisional until the human re-measures at the new speed. No paper was consulted.
- Allowed files: none for coders. Human edits `setup/robot_dynamics.json` (`turn_speed` is the lever; then re-time 90 and
  180 deg, 3 trials each). Supervisor deploys bridge + dynamics TOGETHER after TK-INT-10 PASS (diff against the Jetson file).
- Forbidden files: everything else.
- Acceptance command: `.venv/bin/python tests/acceptance/test_pivot_rate_vs_sweep.py -v` (2 tests: pivot deg/sweep from the dynamics
  file x median sweep period of the recordings <= 15; 90 and 180 deg timings agree within 20 %). Red on today's file (17.4).
- Threshold (numbers): <= 15 deg per sweep (target 8); field check at the new speed: replay `test_slam_csm_turns.py` on a NEW
  recording made at that speed, moving worst error <= 5 deg (target; the current 12-14 deg worst case is the baseline).
- Regression commands: `.venv/bin/python tests/acceptance/test_slam_csm_turns.py -v` (13 OK, unchanged);
  `.venv/bin/python tests/test_bridge_spin_step.py` (4 OK).
- Depends on: TK-INT-10 PASS (deploy together), a free field slot for the human.
- Assumptions: the demo tolerates a pivot ~2.2x slower (90 deg in ~1.15 s instead of 0.53 s); a lower `turn_speed` PWM
  still starts the pivot from rest on a low battery (an open risk in supervisor.md); the sweep period is 0.102 s on
  the demo LiDAR too; the 8 deg limit from two recordings is only a target; the human accepts the 15 deg ceiling for faster reaction.
  Human-approved: 2026-11-20
- Needs-human: none open. Field check: the human records one NEW turn recording at 0.64 s; the Supervisor replays it (moving worst error target <= 10 deg, baseline 12-14).
