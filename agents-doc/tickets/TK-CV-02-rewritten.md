### TK-CV-02 (rewritten): Person-aware LiDAR layer — label clusters, mask person beams out of scan matching
- Role: perception + slam (split into two tickets when it starts: labelling, then CSM masking)
- Status: DRAFT (see the Status line above) — needs a real recording with a person walking (TK-CV-04 session).
- Required behaviour: a LiDAR cluster whose azimuth falls inside a detection box is a PERSON (else static/unknown);
  persons keep the 0.40 m bubble; the beams of persons are removed before `CSMTracker.update` so a crossing person
  does not pull the pose; IL observations are NOT changed (stay 41-D, no retrain).
- Measurement for the report (ablation): pose error with a person walking, masking OFF vs ON, same recording.
- Depends on: TK-SLAM-09, TK-CV-04, a recording with people.
