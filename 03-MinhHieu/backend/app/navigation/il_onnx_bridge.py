"""
Imitation Learning ONNX Bridge for Backend Navigation Coordinator
=================================================================
CS532 Autonomous JetBot — Module 07-imitation-learning integration
Plugs trained ONNX Policy directly into FastAPI Backend coordinator.
"""

import os
import sys
import logging
from typing import Optional

logger = logging.getLogger("jetbot.il_onnx_bridge")

cur_dir = os.path.dirname(os.path.abspath(__file__))
candidates_il = [
    os.path.abspath(os.path.join(cur_dir, "../../../../07-imitation-learning")),
    os.path.abspath(os.path.join(cur_dir, "../../../07-imitation-learning")),
    os.path.abspath("07-imitation-learning"),
    os.path.abspath("../07-imitation-learning"),
]
il_dir = None
for c in candidates_il:
    if os.path.exists(c):
        il_dir = c
        break

if il_dir and il_dir not in sys.path:
    sys.path.insert(0, il_dir)


_RUNNER_INSTANCE = None


def get_il_policy_runner():
    """Returns singleton instance of standard ONNX Policy Runner."""
    global _RUNNER_INSTANCE
    if _RUNNER_INSTANCE is not None:
        return _RUNNER_INSTANCE

    try:
        from il_policy_runner import ILPolicyRunner

        candidate_models = [
            os.path.join(il_dir, "test_artifacts", "models", "dagger_policy.onnx"),
            os.path.join(il_dir, "models", "dagger_policy.onnx"),
            os.path.join(il_dir, "test_artifacts", "models", "bc_policy.onnx"),
            os.path.join(il_dir, "models", "bc_policy.onnx"),
        ]

        resolved_model = None
        for m_path in candidate_models:
            if os.path.exists(m_path):
                resolved_model = m_path
                break

        if resolved_model:
            logger.info(f"[ONNX IL] Initializing Production ONNX Runner: {resolved_model}")
            _RUNNER_INSTANCE = ILPolicyRunner(model_path=resolved_model)
            print(f"[ONNX IL ENGINE] Initialized successfully with: {os.path.basename(resolved_model)}")
            return _RUNNER_INSTANCE

        else:
            logger.warning(f"No ONNX policy found in candidate paths: {candidate_models}")
            return None

    except Exception as e:
        logger.error(f"Failed to initialize ILPolicyRunner: {e}")
        return None
