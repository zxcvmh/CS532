### TK-CV-02 (rewritten): Person-aware LiDAR layer — label clusters, mask person beams out of scan matching
- Role: perception + slam (split into two tickets when it starts: labelling, then CSM masking)
- Status: DRAFT — needs a real synchronised camera + LiDAR recording with a person walking (TK-CV-04 session).
- Required behaviour: a LiDAR cluster whose azimuth falls inside a detection box is a PERSON (else static/unknown);
  persons keep the 0.40 m bubble; the beams of persons are removed before `CSMTracker.update` so a crossing person
  does not pull the pose; provides the 36 per-beam person flags consumed by IL observation v2 (113-D, TK-IL-08 DONE 2026-11-22; legacy 41-D kept only until TK-IL-06 switches over).
- Measurement for the report (ablation): pose error with a person walking, masking OFF vs ON, same recording.
- Depends on: TK-SLAM-09 (DONE), TK-IL-08 (DONE), TK-CV-03, TK-CV-04, a synchronised recording with people (camera + LiDAR).
