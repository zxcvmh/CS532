#!/usr/bin/env python3
"""
ScanGate for LiDAR Telemetry Stream.
Deduplicates LiDAR scans by tracking scan_seq and hardware timestamp,
eliminating 2-3x redundant odometry updates and map over-counting.

Phase 5: ONE range filter, sourced from ``Config`` (LIDAR_MIN_RANGE /
MAPPING_MAX_RANGE) instead of the local 0.15/12.0 literals, and a wall-clock
staleness fallback so a frozen/failing ``scan_seq`` can never wedge the
pipeline permanently.
"""
import os
import sys
import time
from typing import Optional, Dict, Any, List

try:
    from app.config import Config
except ImportError:  # standalone execution (self-test) from this directory
    sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))
    from app.config import Config

MIN_RANGE_M = float(Config.LIDAR_MIN_RANGE)
MAX_RANGE_M = float(Config.MAPPING_MAX_RANGE)
# Force-accept a scan if the gate has accepted nothing for this long. Uses wall clock,
# never the packet stamp, so it stays deterministic under test.
STALE_FORCE_ACCEPT_SEC = 0.20

class ScanTick:
    def __init__(self, seq: int, dt: float, ranges: List[float], stamp: float):
        self.seq = seq
        self.dt = dt
        self.ranges = ranges
        self.stamp = stamp

    def __repr__(self):
        return f"<ScanTick seq={self.seq} dt={self.dt:.3f}s points={len(self.ranges)}>"

class ScanGate:
    def __init__(self, fallback_tol: float = 0.001):
        self.last_seq: Optional[int] = None
        self.last_stamp: float = 0.0
        self.last_ranges: Optional[List[float]] = None
        self.fallback_tol = fallback_tol
        self.synthetic_seq = 0
        # Wall-clock stamp of the last ACCEPTED scan (Phase 5). Deliberately not the
        # packet stamp: a stuck hardware stamp must not look "fresh" forever.
        self._last_accept_wall: float = 0.0

    @staticmethod
    def _normalise_dense(raw: List[Any]) -> List[float]:
        """Applies the single Config-sourced range window to a dense 360-bin ring."""
        ranges = [0.0] * 360
        for i, value in enumerate(raw[:360]):
            try:
                d = float(value)
            except (ValueError, TypeError):
                continue
            if d > 0.0 and MIN_RANGE_M <= d <= MAX_RANGE_M:
                ranges[i] = d
        return ranges

    def accept(self, data: Dict[str, Any]) -> Optional[ScanTick]:
        raw_ranges = data.get("lidar_ranges") or data.get("ranges")
        if raw_ranges:
            raw_ranges = self._normalise_dense(list(raw_ranges))
        else:
            raw_lidar = data.get("lidar")
            if raw_lidar and isinstance(raw_lidar, list) and len(raw_lidar) >= 15:
                # Legacy [angle_deg, dist_m] pair list (older bridges only).
                ranges = [0.0] * 360
                for item in raw_lidar:
                    if len(item) >= 2:
                        try:
                            ang = int(round(float(item[0]))) % 360
                            dist = float(item[1])
                            if MIN_RANGE_M <= dist <= MAX_RANGE_M:
                                ranges[ang] = dist
                        except (ValueError, TypeError):
                            continue
                raw_ranges = ranges
            else:
                return None

        # Check explicit sequence and timestamp
        seq = data.get("scan_seq")
        stamp = data.get("scan_stamp") or data.get("timestamp") or time.time()

        now = time.time()
        if stamp is None:
            stamp = now

        # A `scan_seq` of 0 (or absent) means "the hardware has not produced a
        # revolution counter yet" -- i.e. treat it as absent, NOT as a sequence value,
        # otherwise the very first seq=0 would look like a permanent duplicate and wedge
        # the pipeline (BUG-11).
        has_hw_seq = isinstance(seq, int) and seq != 0
        stale = bool(self._last_accept_wall) and (now - self._last_accept_wall) > STALE_FORCE_ACCEPT_SEC

        # Path 1: Hardware scan_seq provided (and stale force-accept bypasses the dedup)
        if has_hw_seq:
            if not stale and self.last_seq is not None and seq == self.last_seq:
                return None  # Drop duplicate scan!
            effective_stamp = now if (stale and self.last_seq is not None and seq == self.last_seq) else stamp
            dt = 0.1
            if self.last_stamp > 0:
                dt = max(0.02, min(0.3, effective_stamp - self.last_stamp))
            self.last_seq = seq
            self.last_stamp = effective_stamp
            self.last_ranges = raw_ranges
            self._last_accept_wall = now
            return ScanTick(seq=seq, dt=dt, ranges=raw_ranges, stamp=effective_stamp)

        # Path 2: Fallback content-based check (no usable hardware seq)
        if not stale and self.last_ranges is not None and len(self.last_ranges) == len(raw_ranges):
            indices = [0, 45, 90, 135, 180, 225, 270, 315]
            diff = sum(abs(raw_ranges[i] - self.last_ranges[i]) for i in indices if i < len(raw_ranges))
            if diff < self.fallback_tol:
                return None  # Identical scan, drop duplicate!

        self.synthetic_seq += 1
        dt = 0.1
        if self.last_stamp > 0:
            dt = max(0.02, min(0.3, stamp - self.last_stamp))
        self.last_stamp = stamp
        self.last_ranges = raw_ranges
        self._last_accept_wall = now
        return ScanTick(seq=self.synthetic_seq, dt=dt, ranges=raw_ranges, stamp=stamp)

# ─────────────────────────────────────────────────────────────
# SELF-TEST SUITE (5/5 TEST CASES)
# ─────────────────────────────────────────────────────────────
if __name__ == "__main__":
    print("[TEST] Running ScanGate 5-Case Self-Test...")
    gate = ScanGate()

    # Case 1: First scan with scan_seq is accepted
    p1 = {"scan_seq": 100, "scan_stamp": 1000.0, "ranges": [1.0] * 360}
    t1 = gate.accept(p1)
    assert t1 is not None and t1.seq == 100, "Case 1 Failed"

    # Case 2: Duplicate scan_seq (same seq=100) is rejected (None)
    p2 = {"scan_seq": 100, "scan_stamp": 1000.045, "ranges": [1.0] * 360}
    t2 = gate.accept(p2)
    assert t2 is None, "Case 2 Failed: Duplicate scan was not dropped"

    # Case 3: New scan_seq (seq=101) with dt computed from timestamps
    p3 = {"scan_seq": 101, "scan_stamp": 1000.100, "ranges": [1.05] * 360}
    t3 = gate.accept(p3)
    assert t3 is not None and t3.seq == 101 and abs(t3.dt - 0.100) < 0.001, f"Case 3 Failed: {t3}"

    # Case 4: Fallback path - identical range contents without seq is dropped
    gate_fb = ScanGate()
    fb1 = {"ranges": [2.0] * 360, "timestamp": 100.0}
    fb2 = {"ranges": [2.0] * 360, "timestamp": 100.045}
    assert gate_fb.accept(fb1) is not None, "Case 4a Failed"
    assert gate_fb.accept(fb2) is None, "Case 4b Failed: Fallback duplicate was not dropped"

    # Case 5: Empty or invalid lidar payload returns None
    assert gate.accept({}) is None, "Case 5a Failed"
    assert gate.accept({"lidar": []}) is None, "Case 5b Failed"

    print("✅ All 5/5 ScanGate self-tests PASSED successfully!")
