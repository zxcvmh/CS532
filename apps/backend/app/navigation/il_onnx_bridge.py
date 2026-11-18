"""
Imitation Learning ONNX Bridge for Backend Navigation Coordinator
=================================================================
CS532 Autonomous JetBot — jetbot_il integration
Plugs trained ONNX Policy directly into FastAPI Backend coordinator.
"""

import os
import sys
import logging

logger = logging.getLogger("jetbot.il_onnx_bridge")

_REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../.."))
_PACKAGES_DIR = os.path.join(_REPO_ROOT, "packages")
if _PACKAGES_DIR not in sys.path and os.path.isdir(_PACKAGES_DIR):
    sys.path.insert(0, _PACKAGES_DIR)

_RUNNER_INSTANCE = None


def get_il_policy_runner():
    """Returns singleton instance of standard ONNX Policy Runner."""
    global _RUNNER_INSTANCE
    if _RUNNER_INSTANCE is not None:
        return _RUNNER_INSTANCE

    try:
        from jetbot_il.runtime.il_policy_runner import ILPolicyRunner
        from jetbot_il.config import MODELS_DIR

        candidate_models = [
            os.path.join(MODELS_DIR, "dagger_policy.onnx"),
            os.path.join(MODELS_DIR, "bc_policy.onnx"),
            os.path.join(_REPO_ROOT, "tests", "imitation_learning", "test_artifacts", "models", "dagger_policy.onnx"),
            os.path.join(_REPO_ROOT, "tests", "imitation_learning", "test_artifacts", "models", "bc_policy.onnx"),
        ]

        resolved_model = next((m for m in candidate_models if os.path.exists(m)), None)

        if resolved_model:
            logger.info(f"[ONNX IL] Initializing Production ONNX Runner: {resolved_model}")
            _RUNNER_INSTANCE = ILPolicyRunner(model_path=resolved_model)
            print(f"[ONNX IL ENGINE] Initialized successfully with: {os.path.basename(resolved_model)}")
            return _RUNNER_INSTANCE

        logger.warning(f"No ONNX policy found in candidate paths: {candidate_models}")
        return None

    except Exception as e:
        logger.error(f"Failed to initialize ILPolicyRunner: {e}")
        return None
