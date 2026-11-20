# Plan 2026-11-20: SLAM heading spikes during fast pivots

Tickets: `agents-doc/tickets/TK-INT-12.md` (pivot rate, human) and `agents-doc/tickets/TK-SLAM-10.md` (match score + status, coder).
Nothing here is built. No code or test was written; the evidence is from scratchpad prototypes on the two real turn
recordings and the three person recordings.

## Why (evidence)
- Heading error of the CSM tracker grows with the rotation inside one LiDAR sweep: mean 1.0-1.4 deg at <= 8 deg/sweep,
  max 4.2 deg; 8.9 deg mean and 14.3 deg max at 30-60 deg/sweep. Spikes last 1-3 sweeps, then recover (what the human saw).
- Today's pivot is ~170 deg/s (90 deg in 0.53 s) = ~17 deg per sweep (sweep 0.102 s). Likely cause: motion distortion inside
  one sweep. Not proven; the data only fits.
- Idea taken from `lidar_localization_ros2`: a match-quality score and a status. Not taken: "keep the predicted pose when a
  match is rejected" — tried in a prototype, heading error rose to 62-179 deg because this robot has no odometry or IMU.
- Not studied: a published threshold. The 8 deg/sweep limit is our own measurement on two recordings (provisional).

## Order of work
| # | Step | Owner | Done when |
|---|---|---|---|
| 1 | Approve the Assumptions of both tickets (adds `Human-approved: <date>`); decide how much slower the pivot may be | Human | `check_ticket.py <ID> --ready` has no FAIL |
| 2 | Write the locked tests `test_pivot_rate_vs_sweep.py` and `test_slam_csm_status.py`; each must be red on today's tree for the right reason | Supervisor (needs the human's go) | tickets move to READY |
| 3 | Pivot slower: lower `turn_speed`, re-time 90 deg and 180 deg (3 trials each), check it starts from rest on a low battery, edit `setup/robot_dynamics.json` | Human | `turn_time_90_deg_sec` >= ~1.15 s |
| 4 | `/slam TK-SLAM-10` | Antigravity coder | REPORT with raw output |
| 5 | `/audit TK-SLAM-10` and `/audit TK-INT-12` (`check_ticket.py <ID> --base <SHA at hand-off>`) | quality-auditor | PASS |
| 6 | Deploy bridge + dynamics together (after TK-INT-10 PASS, diff against the Jetson file, command announced first) | Supervisor, human says go | Jetson runs the new pivot |
| 7 | Record one new turn recording at the new speed; replay with the locked SLAM test | Human records, Supervisor replays | moving worst error <= 5 deg (baseline 12-14 deg) |
| 8 | `/close` each ticket with explicit paths | Supervisor | commit + STATE entry |

## Success numbers
- Pivot <= 8 deg per sweep (<= ~78 deg/s).
- Status: >= 80 % of sweeps with error > 6 deg flagged, <= 10 % false alarms, person recordings TRACKING in >= 98 % of sweeps, no LOST.
- Published pose unchanged by TK-SLAM-10; `test_slam_csm_turns.py` stays 13/13.

## Risks / open
- Small sample: 10 sweeps with error > 6 deg in total; constants `SCORE_WEAK` 0.64 and `LOST_AFTER` 5 are provisional.
- A slower pivot may not start from rest on a low battery; may lengthen the demo turns (IL stop-turn-go).
- The status has no consumer yet (dashboard / IL / reflex); a separate ticket if wanted.
- If heading spikes remain after steps 3 and 7, the next candidate is de-skew (needs per-beam time of the D500; not ticketed).
- Map-based localization on a pre-scanned room (the other idea from that repo) is NOT in this plan; revisit only if drift, not spikes, is the problem.
