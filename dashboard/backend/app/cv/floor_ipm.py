"""
Floor IPM proxy module pointing to single source in 03-MinhHieu/floor_ipm.py.
"""
import sys
import os

_root_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../"))
if _root_dir not in sys.path:
    sys.path.insert(0, _root_dir)

try:
    from floor_ipm import FloorIPMTransformer
except ImportError:
    class FloorIPMTransformer:
        def __init__(self, *args, **kwargs):
            pass

__all__ = ["FloorIPMTransformer"]
