### TK-INT-06: Motion is jerky (unexplained)
- Role: supervisor (needs field log) → integration
- Status: DEFERRED 2026-11-16 (human approved the cut; reopen only with a captured bridge log)
- Candidates (unverified): reflex-task exception every loop (fixed in TK-INT-05),
  0 → min_drive_speed=0.19 PWM jump at start, backend velocity latch (TK-INT-04).
