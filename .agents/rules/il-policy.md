---
trigger: glob
globs: "imitation_learning/**, dashboard/backend/app/navigation/il_onnx_bridge.py"
description: "Imitation-learning policy, dataset, training and ONNX runner rules."
---

# IL policy rules

- `imitation_learning/config.py` defines the observation (currently 41 dims: 36 LiDAR beams + goal
  distance + sin/cos goal angle + v + w). Change the layout only together with the sim, the runner
  and a retrain — a layout change silently breaks every saved model.
- The runtime loads `imitation_learning/test_artifacts/models/*.onnx`. Running
  `test_il_pipeline.py` **overwrites those files**. Never ship a model produced by a test run;
  save trained models to `imitation_learning/models/` and say which file is deployed.
- `config.py` currently hardcodes `MAX_LINEAR_SPEED`, `MIN_DRIVE_SPEED`, `MAX_ANGULAR_SPEED`
  instead of reading `setup/robot_dynamics.json`. Those caps scale the policy output, so changing
  them requires retraining — do it only in a ticket that says so.
- Evaluation must report, over a fixed seed list: success rate, collision rate, timeout rate,
  mean time-to-goal, and episodes with a pedestrian crossing. Acceptance: success ≥ 90%,
  collision ≤ 2% on 100 episodes.
- The expert (oracle) labels the data. If the expert itself fails a scenario, the policy cannot
  learn it — measure expert success first and report it next to the policy's.
- Inference budget on Jetson: whole control loop ≤ 50 ms. Report CPU latency on the laptop as a
  proxy and mark Jetson latency `UNVERIFIED` until the human measures it.
