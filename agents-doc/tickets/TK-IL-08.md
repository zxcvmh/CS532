### TK-IL-08: IL is the ONLY avoider — person channel + motion (2-frame) observation
- Role: il-policy (Supervisor writes the locked test first)
- Status: DRAFT (human decision 2026-11-16: IL does ALL avoidance — static obstacles, standing person, crossing person,
  head-on person; the global planner only supplies the goal direction; reflex = brake only, no steering)
- Problem: `config.py` `OBSERVATION_DIM = 41` (36 beams + goal + state) is a SINGLE frame, so the student cannot tell a
  crossing person from a wall; and no person label reaches the policy (course CS532 = computer vision in HRI).
- Required (to be turned into numbers by the locked test): (1) observation gains a per-beam person flag (0/1, from the
  YOLO-labelled LiDAR clusters, TK-CV-02 A; in sim = the true pedestrian beams, with a configurable miss/false-alarm rate);
  (2) observation gains motion: the previous frame (or the beam-wise range delta) so a crossing/approaching person is
  observable; (3) the sim scenarios cover: static obstacle on the path, person standing still, person crossing, person
  head-on, each with >= 0.3 m clearance; (4) NN-driven steps >= 90 %, reflex may only stop, never steer.
- Allowed files: `imitation_learning/config.py`, `imitation_learning/mock_expert_simulator.py`,
  `imitation_learning/model.py` (split in two attempts if > 150 lines)
- Threshold: per scenario class (4 classes x 25 seeds): success >= 90 %, collision <= 2 %, contact clearance >= 0.3 m.
- Depends on: TK-IL-04, TK-IL-05; TK-CV-02 A for the real person flag (sim can run before it).
- Open: whether the real person flag comes from the camera only; ArUco (TK-CV-05) is not needed for this.
