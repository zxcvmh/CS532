"""
Floor IPM proxy module pointing to single source in jetbot_hardware.floor_ipm_edge.
"""
import os
import sys

_PACKAGES_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../../packages"))
if _PACKAGES_DIR not in sys.path and os.path.isdir(_PACKAGES_DIR):
    sys.path.insert(0, _PACKAGES_DIR)

try:
    from jetbot_hardware.floor_ipm_edge import FloorIPMTransformer
except ImportError:
    class FloorIPMTransformer:
        def __init__(self, *args, **kwargs):
            pass

__all__ = ["FloorIPMTransformer"]
