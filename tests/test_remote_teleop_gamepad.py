#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Unit tests for RemoteBackendRobot adapter and remote teleop CLI flags.
"""

import unittest
import json
import time
import threading
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler
from urllib.error import URLError

# Target class to be imported
from teleop_gamepad import RemoteBackendRobot


class MockBackendHandler(BaseHTTPRequestHandler):
    received_requests = []

    def do_POST(self):
        content_length = int(self.headers.get("Content-Length", 0))
        body = self.rfile.read(content_length) if content_length > 0 else b""
        payload = {}
        if body:
            try:
                payload = json.loads(body.decode("utf-8"))
            except Exception:
                payload = {"raw": body.decode("utf-8", errors="ignore")}

        MockBackendHandler.received_requests.append({
            "path": self.path,
            "payload": payload,
            "headers": dict(self.headers),
        })

        if self.path.startswith("/slow"):
            time.sleep(0.3)

        try:
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            self.wfile.write(b'{"ok": true}')
        except (BrokenPipeError, ConnectionResetError):
            pass

    def log_message(self, format, *args):
        pass  # Suppress console log spam during tests


class TestRemoteBackendRobot(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        MockBackendHandler.received_requests = []
        cls.server = ThreadingHTTPServer(("127.0.0.1", 0), MockBackendHandler)
        cls.port = cls.server.server_port
        cls.base_url = f"http://127.0.0.1:{cls.port}"
        cls.server_thread = threading.Thread(target=cls.server.serve_forever, daemon=True)
        cls.server_thread.start()

    @classmethod
    def tearDownClass(cls):
        cls.server.shutdown()
        cls.server.server_close()

    def setUp(self):
        MockBackendHandler.received_requests.clear()
        self.robot = RemoteBackendRobot(self.base_url, timeout=0.08)

    def test_01_set_motors_payload(self):
        """Verify set_motors sends correct JSON payload to /drive."""
        ok = self.robot.set_motors(0.3, -0.2)
        self.assertTrue(ok)
        self.assertEqual(len(MockBackendHandler.received_requests), 1)
        req = MockBackendHandler.received_requests[0]
        self.assertEqual(req["path"], "/drive")
        self.assertAlmostEqual(req["payload"].get("left"), 0.3)
        self.assertAlmostEqual(req["payload"].get("right"), -0.2)

    def test_02_stop_payload(self):
        """Verify stop sends request to /stop or /api/robot/stop."""
        ok = self.robot.stop()
        self.assertTrue(ok)
        self.assertEqual(len(MockBackendHandler.received_requests), 1)
        req = MockBackendHandler.received_requests[0]
        self.assertIn(req["path"], ["/stop", "/api/robot/stop"])

    def test_03_timeout_and_unreachable_backend_resilience(self):
        """Verify client handles backend timeout or unreachability without raising uncaught exceptions."""
        # Unreachable port test
        unreachable_robot = RemoteBackendRobot("http://127.0.0.1:59999", timeout=0.05)
        # Should return False or None, but NOT crash or raise exception
        try:
            res = unreachable_robot.set_motors(0.2, 0.2)
            self.assertFalse(res)
        except Exception as e:
            self.fail(f"unreachable robot raised unexpected exception: {e}")

        # Slow endpoint triggering timeout
        slow_robot = RemoteBackendRobot(f"{self.base_url}/slow", timeout=0.05)
        # Note: /slow will receive POST /slow/drive
        try:
            res = slow_robot.set_motors(0.1, 0.1)
            self.assertFalse(res)
        except Exception as e:
            self.fail(f"slow robot raised unexpected exception on timeout: {e}")

    def test_04_motion_convenience_methods(self):
        """Verify forward, backward, left, right helper methods work properly."""
        self.robot.forward(0.25)
        self.assertEqual(MockBackendHandler.received_requests[-1]["payload"], {"left": 0.25, "right": 0.25})

        self.robot.backward(0.25)
        self.assertEqual(MockBackendHandler.received_requests[-1]["payload"], {"left": -0.25, "right": -0.25})

        self.robot.left(0.25)
        self.assertEqual(MockBackendHandler.received_requests[-1]["payload"], {"left": -0.25, "right": 0.25})

        self.robot.right(0.25)
        self.assertEqual(MockBackendHandler.received_requests[-1]["payload"], {"left": 0.25, "right": -0.25})

    def test_05_parse_cli_args_various_formats(self):
        """Verify CLI argument parsing for --remote, -r, and --keyboard."""
        from teleop_gamepad import parse_cli_args, init_robot

        # Explicit URL
        url, kb = parse_cli_args(["--remote", "http://192.168.1.50:8000"])
        self.assertEqual(url, "http://192.168.1.50:8000")
        self.assertFalse(kb)

        # -r flag with URL
        url, kb = parse_cli_args(["-r", "http://jetbot.local:8000"])
        self.assertEqual(url, "http://jetbot.local:8000")
        self.assertFalse(kb)

        # --remote flag without URL (default to 127.0.0.1:8000)
        url, kb = parse_cli_args(["--remote"])
        self.assertEqual(url, "http://127.0.0.1:8000")
        self.assertFalse(kb)

        # -r flag without URL
        url, kb = parse_cli_args(["-r"])
        self.assertEqual(url, "http://127.0.0.1:8000")
        self.assertFalse(kb)

        # --remote=URL format
        url, kb = parse_cli_args(["--remote=http://custom-host:9000"])
        self.assertEqual(url, "http://custom-host:9000")

        # Keyboard + remote combined
        url, kb = parse_cli_args(["--keyboard", "--remote", "http://127.0.0.1:8000"])
        self.assertEqual(url, "http://127.0.0.1:8000")
        self.assertTrue(kb)

        # Default without flags
        url, kb = parse_cli_args([])
        self.assertIsNone(url)
        self.assertFalse(kb)

        # init_robot with remote URL produces RemoteBackendRobot
        r = init_robot("http://127.0.0.1:8000")
        self.assertIsInstance(r, RemoteBackendRobot)

    def test_06_frequency_compression_filter(self):
        """Verify frequency compression avoids flooding duplicate commands within 0.2s heartbeat."""
        robot = RemoteBackendRobot(self.base_url, timeout=0.08)
        MockBackendHandler.received_requests.clear()

        # First command sends immediately
        robot.set_motors(0.3, 0.3)
        self.assertEqual(len(MockBackendHandler.received_requests), 1)

        # Duplicate command immediately after should be suppressed by filter
        robot.set_motors(0.3, 0.3)
        self.assertEqual(len(MockBackendHandler.received_requests), 1)

        # Significant speed change (> 0.01) sends immediately
        robot.set_motors(0.35, 0.3)
        self.assertEqual(len(MockBackendHandler.received_requests), 2)

    def test_07_stale_command_latch_stop(self):
        """Verify letting go of stick (0.0, 0.0) sends stop immediately to prevent stale motion."""
        robot = RemoteBackendRobot(self.base_url, timeout=0.08)
        MockBackendHandler.received_requests.clear()

        # Drive forward
        robot.set_motors(0.3, 0.3)
        self.assertEqual(len(MockBackendHandler.received_requests), 1)
        self.assertEqual(MockBackendHandler.received_requests[-1]["path"], "/drive")

        # Release joystick: (0.0, 0.0) must send /stop immediately!
        robot.set_motors(0.0, 0.0)
        self.assertEqual(len(MockBackendHandler.received_requests), 2)
        self.assertIn(MockBackendHandler.received_requests[-1]["path"], ["/stop", "/api/robot/stop"])

        # Immediate repeated 0.0, 0.0 should not spam duplicate stop packets
        robot.set_motors(0.0, 0.0)
        self.assertEqual(len(MockBackendHandler.received_requests), 2)

    def test_08_estop_priority(self):
        """Verify direct call to stop() is always sent with high priority."""
        robot = RemoteBackendRobot(self.base_url, timeout=0.08)
        MockBackendHandler.received_requests.clear()

        # Drive
        robot.set_motors(0.4, 0.4)
        self.assertEqual(len(MockBackendHandler.received_requests), 1)

        # Emergency Stop
        robot.stop()
        self.assertEqual(len(MockBackendHandler.received_requests), 2)
        self.assertIn(MockBackendHandler.received_requests[-1]["path"], ["/stop", "/api/robot/stop"])

    def test_11_drive_arcade_and_direct_logical_direction(self):
        """Verify drive_arcade and drive_direct send logical forward (positive) commands without inverted polarity."""
        import teleop_gamepad
        robot = RemoteBackendRobot(self.base_url, timeout=0.08)
        teleop_gamepad.robot = robot
        MockBackendHandler.received_requests.clear()

        # 1. Forward Drive (Stick pushed UP: throttle=1.0, steering=0.0)
        l, r = teleop_gamepad.drive_arcade(throttle=1.0, steering=0.0, speed_scale=0.20)
        self.assertAlmostEqual(l, 0.20)
        self.assertAlmostEqual(r, 0.20)
        req = MockBackendHandler.received_requests[-1]
        self.assertEqual(req["path"], "/drive")
        # Must be positive (forward) and balanced (no double trim on client)
        self.assertAlmostEqual(req["payload"]["left"], 0.20)
        self.assertAlmostEqual(req["payload"]["right"], 0.20)

        # 2. Key 'W' Direct Forward
        teleop_gamepad.drive_direct(0.20, 0.20)
        req = MockBackendHandler.received_requests[-1]
        self.assertEqual(req["path"], "/drive")
        self.assertAlmostEqual(req["payload"]["left"], 0.20)
        self.assertAlmostEqual(req["payload"]["right"], 0.20)

        # 3. Backward Drive (throttle=-1.0, steering=0.0)
        l, r = teleop_gamepad.drive_arcade(throttle=-1.0, steering=0.0, speed_scale=0.20)
        self.assertAlmostEqual(l, -0.20)
        self.assertAlmostEqual(r, -0.20)
        req = MockBackendHandler.received_requests[-1]
        self.assertAlmostEqual(req["payload"]["left"], -0.20)
        self.assertAlmostEqual(req["payload"]["right"], -0.20)

        # 4. Turn Left in place (throttle=0.0, steering=-1.0) -> left negative, right positive
        l, r = teleop_gamepad.drive_arcade(throttle=0.0, steering=-1.0, speed_scale=0.20)
        self.assertAlmostEqual(l, -0.20)
        self.assertAlmostEqual(r, 0.20)
        req = MockBackendHandler.received_requests[-1]
        self.assertAlmostEqual(req["payload"]["left"], -0.20)
        self.assertAlmostEqual(req["payload"]["right"], 0.20)


class TestRealFastAPIBackendIntegration(unittest.TestCase):
    """End-to-End integration tests with the real FastAPI backend."""
    @classmethod
    def setUpClass(cls):
        import sys
        import os
        import socket
        import uvicorn

        backend_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "dashboard", "backend"))
        if backend_dir not in sys.path:
            sys.path.insert(0, backend_dir)

        from app.main import app

        def find_free_port():
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.bind(("127.0.0.1", 0))
                return s.getsockname()[1]

        cls.port = find_free_port()
        cls.base_url = f"http://127.0.0.1:{cls.port}"

        class UvicornServer(threading.Thread):
            def __init__(self, target_app, port):
                super().__init__(daemon=True)
                self.config = uvicorn.Config(target_app, host="127.0.0.1", port=port, log_level="error")
                self.server = uvicorn.Server(self.config)

            def run(self):
                self.server.run()

            def stop(self):
                self.server.should_exit = True

        cls.server = UvicornServer(app, cls.port)
        cls.server.start()

        import urllib.request
        started = False
        for _ in range(50):
            try:
                with urllib.request.urlopen(f"{cls.base_url}/status", timeout=0.1) as r:
                    if r.status == 200:
                        started = True
                        break
            except Exception:
                time.sleep(0.05)
        if not started:
            raise RuntimeError(f"FastAPI backend failed to start on port {cls.port}")

    @classmethod
    def tearDownClass(cls):
        if hasattr(cls, "server"):
            cls.server.stop()

    def test_09_real_backend_drive_and_status(self):
        """Verify real FastAPI backend responds to /drive and reflects in /status."""
        import urllib.request
        robot = RemoteBackendRobot(self.base_url, timeout=0.2)

        # Send forward drive command
        ok = robot.set_motors(0.3, 0.3, force=True)
        self.assertTrue(ok)

        # Check real backend /status
        with urllib.request.urlopen(f"{self.base_url}/status", timeout=0.5) as resp:
            data = json.loads(resp.read().decode("utf-8"))
            self.assertTrue(data.get("ok"))
            self.assertTrue(data.get("moving"))

        # Send stop command
        ok = robot.stop()
        self.assertTrue(ok)

        # Check real backend /status after stop
        with urllib.request.urlopen(f"{self.base_url}/status", timeout=0.5) as resp:
            data = json.loads(resp.read().decode("utf-8"))
            self.assertTrue(data.get("ok"))
            self.assertFalse(data.get("moving"))

    def test_10_default_speed_adheres_to_robot_dynamics(self):
        """Verify default teleop speed strictly adheres to cruise_speed=0.2 (robot_dynamics.json)."""
        import teleop_gamepad
        self.assertAlmostEqual(teleop_gamepad.current_speed_limit, 0.20, places=2)
        self.assertAlmostEqual(teleop_gamepad.cruise_speed, 0.20, places=2)
        self.assertAlmostEqual(teleop_gamepad.min_drive_speed, 0.08, places=2)
        self.assertEqual(teleop_gamepad.SPEED_LEVELS[teleop_gamepad.current_gear_idx], 0.20)


if __name__ == "__main__":
    unittest.main()
