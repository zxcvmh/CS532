#!/usr/bin/env python3
"""
Command Multiplexer for NVIDIA JetBot Hardware Bridge.
Single Source of Truth for Motor Dispatch (50 Hz).
Priority Hierarchy:
  1. EMERGENCY_STOP (highest priority: immediate halt, persists until cleared)
  2. Reflex Stop (immediate reflex stop from DDE)
  3. Reflex Override (DDE active evasion heading/speed, TTL=0.3s)
  4. User Drive (MANUAL or AUTONOMOUS, TTL=0.4s watchdog)
"""
import time
import threading
from typing import Callable, Optional, Tuple

SRC_MANUAL = "MANUAL"
SRC_AUTO = "AUTONOMOUS"

class CommandMux:
    def __init__(
        self,
        apply_fn: Callable[[float, float], None],
        stop_fn: Callable[[], None],
        hz: float = 50.0,
        watchdog_s: float = 0.4
    ):
        self.apply_fn = apply_fn
        self.stop_fn = stop_fn
        self.period = 1.0 / max(1.0, float(hz))
        self.watchdog_s = float(watchdog_s)

        self._lock = threading.Lock()
        self._thread: Optional[threading.Thread] = None
        self._running = False

        # State
        self._estop = False
        self._reflex_stop = False
        self._reflex_cmd: Optional[Tuple[float, float, float]] = None  # (v, w, expiry_time)
        self._user_cmd: Optional[Tuple[str, float, float, float]] = None  # (src, v, w, timestamp)
        self._last_cmd_sent: Tuple[float, float] = (0.0, 0.0)
        self._is_stopped = True

    def start(self):
        with self._lock:
            if not self._running:
                self._running = True
                self._thread = threading.Thread(target=self._loop, name="CommandMux", daemon=True)
                self._thread.start()

    def stop(self):
        with self._lock:
            self._running = False
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.0)
        self.stop_fn()

    def estop(self):
        """Trigger emergency stop (highest priority)."""
        with self._lock:
            self._estop = True
            self.stop_fn()
            self._is_stopped = True

    def clear_estop(self):
        with self._lock:
            self._estop = False

    def reflex_stop(self):
        """Reflex stop triggered by critical proximity."""
        with self._lock:
            self._reflex_stop = True
            self.stop_fn()
            self._is_stopped = True

    def clear_reflex_stop(self):
        with self._lock:
            self._reflex_stop = False

    def is_reflex_active(self) -> bool:
        with self._lock:
            return self._reflex_stop

    def reflex_override(self, linear: float, angular: float, ttl: float = 0.3):
        """Submit reactive evasion override with time-to-live."""
        with self._lock:
            self._reflex_cmd = (float(linear), float(angular), time.monotonic() + ttl)

    def submit_user(self, src: str, linear: float, angular: float):
        """Submit user velocity command (MANUAL or AUTONOMOUS)."""
        with self._lock:
            if abs(linear) < 0.005 and abs(angular) < 0.02:
                self._user_cmd = None
                try:
                    self.stop_fn()
                except Exception as e:
                    print(f"[MUX ERROR] stop_fn: {e}")
                self._is_stopped = True
            else:
                self._user_cmd = (src, float(linear), float(angular), time.monotonic())

    def on_disconnect(self):
        """Called immediately upon network disconnect."""
        with self._lock:
            self._user_cmd = None
            self._reflex_cmd = None
            self.stop_fn()
            self._is_stopped = True

    def _loop(self):
        while self._running:
            t0 = time.monotonic()
            v_out, w_out, should_stop = self._evaluate_priority(t0)

            if should_stop:
                if not self._is_stopped:
                    try:
                        self.stop_fn()
                    except Exception as e:
                        print(f"[MUX ERROR] stop_fn: {e}")
                    self._is_stopped = True
                    self._last_cmd_sent = (0.0, 0.0)
            else:
                try:
                    self.apply_fn(v_out, w_out)
                    self._is_stopped = False
                    self._last_cmd_sent = (v_out, w_out)
                except Exception as e:
                    print(f"[MUX ERROR] apply_fn: {e}")

            elapsed = time.monotonic() - t0
            sleep_time = self.period - elapsed
            if sleep_time > 0.001:
                time.sleep(sleep_time)

    def _evaluate_priority(self, now: float) -> Tuple[float, float, bool]:
        with self._lock:
            # 1. HARD EMERGENCY STOP (Highest priority, persists until explicitly cleared)
            if self._estop:
                return 0.0, 0.0, True

            # 2. Dynamic Reflex Stop (Safety Brake for frontal obstruction)
            if self._reflex_stop:
                # Operational Authority Check:
                # If operator commands MANUAL retreat (reverse v < -0.01 or in-place rotation |v| < 0.02, |w| > 0.02),
                # allow the robot to disengage and clear reflex stop.
                if self._user_cmd is not None:
                    src, uv, uw, u_stamp = self._user_cmd
                    if src == SRC_MANUAL and (now - u_stamp <= self.watchdog_s):
                        if uv < -0.01 or (abs(uv) < 0.02 and abs(uw) > 0.02):
                            self._reflex_stop = False
                            return uv, uw, False
                return 0.0, 0.0, True

            # 2. Reflex Override (if active and not expired)
            if self._reflex_cmd is not None:
                rv, rw, r_expiry = self._reflex_cmd
                if now < r_expiry:
                    user_src = self._user_cmd[0] if self._user_cmd else SRC_MANUAL
                    if user_src == SRC_AUTO:
                        return rv, rw, False
                else:
                    self._reflex_cmd = None

            # 3. User Command with Watchdog (0.4s)
            if self._user_cmd is not None:
                src, uv, uw, u_stamp = self._user_cmd
                if now - u_stamp <= self.watchdog_s:
                    if abs(uv) < 0.005 and abs(uw) < 0.02:
                        return 0.0, 0.0, True
                    return uv, uw, False
                else:
                    # Watchdog expired!
                    return 0.0, 0.0, True

            # No active command -> stop
            return 0.0, 0.0, True

# ─────────────────────────────────────────────────────────────
# SELF-TEST SUITE (10/10 TEST CASES)
# ─────────────────────────────────────────────────────────────
if __name__ == "__main__":
    print("[TEST] Running CommandMux 10-Case Self-Test...")
    applied = []
    stopped = []

    def mock_apply(v, w):
        applied.append((round(v, 3), round(w, 3)))

    def mock_stop():
        stopped.append(time.monotonic())

    mux = CommandMux(apply_fn=mock_apply, stop_fn=mock_stop, hz=100, watchdog_s=0.1)
    mux.start()

    # Case 1: Initial state is stopped
    time.sleep(0.02)
    assert len(applied) == 0, "Case 1 Failed"

    # Case 2: Submit user command (MANUAL)
    mux.submit_user(SRC_MANUAL, 0.10, 0.0)
    time.sleep(0.03)
    assert len(applied) > 0 and applied[-1] == (0.10, 0.0), f"Case 2 Failed: {applied}"

    # Case 3: Watchdog expiry after 0.1s
    time.sleep(0.12)
    assert len(stopped) > 0, "Case 3 Failed: Watchdog did not stop"

    # Case 4: User zero velocity command
    applied.clear()
    stopped.clear()
    mux.submit_user(SRC_MANUAL, 0.0, 0.0)
    time.sleep(0.03)
    assert len(stopped) > 0, "Case 4 Failed"

    # Case 5: Reflex override in AUTONOMOUS mode takes priority
    applied.clear()
    mux.submit_user(SRC_AUTO, 0.12, 0.0)
    mux.reflex_override(0.05, 0.30, ttl=0.08)
    time.sleep(0.03)
    assert applied[-1] == (0.05, 0.30), f"Case 5 Failed: {applied[-1]}"

    # Case 6: Reflex override expires and reverts to AUTONOMOUS command
    time.sleep(0.07)
    mux.submit_user(SRC_AUTO, 0.12, 0.0)
    time.sleep(0.03)
    assert applied[-1] == (0.12, 0.0), f"Case 6 Failed: {applied[-1]}"

    # Case 7: Reflex override ignored in MANUAL mode (user retains steering)
    applied.clear()
    mux.submit_user(SRC_MANUAL, 0.14, -0.20)
    mux.reflex_override(0.05, 0.30, ttl=0.10)
    time.sleep(0.03)
    assert applied[-1] == (0.14, -0.20), f"Case 7 Failed: {applied[-1]}"

    # Case 8: Reflex stop halts immediately even in MANUAL mode
    stopped.clear()
    mux.reflex_stop()
    time.sleep(0.02)
    assert len(stopped) > 0, "Case 8 Failed"
    mux.clear_reflex_stop()

    # Case 9: E-STOP blocks everything
    stopped.clear()
    applied.clear()
    mux.estop()
    mux.submit_user(SRC_AUTO, 0.14, 0.0)
    time.sleep(0.03)
    assert len(applied) == 0, "Case 9 Failed: E-STOP was bypassed"
    mux.clear_estop()

    # Case 10: on_disconnect clears state and stops
    stopped.clear()
    mux.submit_user(SRC_MANUAL, 0.10, 0.0)
    time.sleep(0.02)
    mux.on_disconnect()
    time.sleep(0.02)
    assert len(stopped) > 0, "Case 10 Failed"

    mux.stop()
    print("✅ All 10/10 CommandMux self-tests PASSED successfully!")
