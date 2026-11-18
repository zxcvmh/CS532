#!/usr/bin/env python3
"""
Cartographer Map & Pose Bridge to Web Dashboard
================================================
Subscribes to:
- `/map` (nav_msgs/msg/OccupancyGrid) from Cartographer Occupancy Grid Node.
- `/tf` (geometry_msgs/msg/TransformStamped) for global robot pose.
Encodes the Cartographer map into base64 and forwards it to the web dashboard WebSocket
so the operator dashboard visualizes Google Cartographer 2D submaps in real time.
"""
import sys
import json
import base64
import time
import asyncio
import threading
from typing import Optional

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
    from nav_msgs.msg import OccupancyGrid
    from tf2_ros import Buffer, TransformListener
    HAS_ROS2 = True
except ImportError:
    HAS_ROS2 = False
    Node = object

try:
    import websockets
    HAS_WEBSOCKETS = True
except ImportError:
    HAS_WEBSOCKETS = False


class MapBridgeToWeb(Node if HAS_ROS2 else object):
    def __init__(self):
        if not HAS_ROS2:
            raise RuntimeError("ROS 2 (rclpy) is required to run MapBridgeToWeb.")
        super().__init__('cartographer_map_bridge_to_web')

        self.declare_parameter('dashboard_ws_url', 'ws://127.0.0.1:8000/ws')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('publish_throttle_hz', 2.0)

        self.dashboard_ws_url = self.get_parameter('dashboard_ws_url').value
        self.map_topic = self.get_parameter('map_topic').value
        self.throttle_hz = float(self.get_parameter('publish_throttle_hz').value)
        self.min_period = 1.0 / max(0.1, self.throttle_hz)

        # QoS for Map: Reliable, Transient Local (REP-2003)
        map_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self._map_callback,
            map_qos
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.last_sent_time = 0.0
        self.latest_payload = None
        self._lock = threading.Lock()
        self._running = True

        self.ws_thread = threading.Thread(target=self._run_ws_client, daemon=True)
        self.ws_thread.start()

        self.get_logger().info(
            f"[INIT] Map Bridge to Web active (Sub: {self.map_topic} -> Forward: {self.dashboard_ws_url})"
        )

    def _map_callback(self, msg: OccupancyGrid):
        now = time.time()
        if now - self.last_sent_time < self.min_period:
            return

        w = msg.info.width
        h = msg.info.height
        res = msg.info.resolution
        ox = msg.info.origin.position.x
        oy = msg.info.origin.position.y

        # Data is int8 array [-1, 0..100]
        # Encode as bytes where negative numbers stay uint8 representation (255 for -1)
        raw_bytes = bytes([b if b >= 0 else (256 + b) for b in msg.data])
        b64_str = base64.b64encode(raw_bytes).decode('ascii')

        # Get latest pose from TF (map -> base_link)
        robot_pose = {"x": 0.0, "y": 0.0, "theta": 0.0}
        try:
            if self.tf_buffer.can_transform('map', 'base_link', rclpy.time.Time()):
                t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
                robot_pose["x"] = t.transform.translation.x
                robot_pose["y"] = t.transform.translation.y
                # Yaw from quaternion
                qx = t.transform.rotation.x
                qy = t.transform.rotation.y
                qz = t.transform.rotation.z
                qw = t.transform.rotation.w
                import math
                siny_cosp = 2.0 * (qw * qz + qx * qy)
                cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
                robot_pose["theta"] = math.atan2(siny_cosp, cosy_cosp)
        except Exception:
            pass

        payload = {
            "type": "map_update",
            "resolution": round(res, 4),
            "width": w,
            "height": h,
            "origin_x": round(ox, 4),
            "origin_y": round(oy, 4),
            "data_b64": b64_str,
            "pose": robot_pose,
            "timestamp": now,
            "source": "google_cartographer_2d"
        }

        with self._lock:
            self.latest_payload = payload
        self.last_sent_time = now

    def _run_ws_client(self):
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        loop.run_until_complete(self._ws_loop())

    async def _ws_loop(self):
        if not HAS_WEBSOCKETS:
            return
        while self._running:
            try:
                async with websockets.connect(self.dashboard_ws_url) as ws:
                    self.get_logger().info(f"[WS] Connected to dashboard at {self.dashboard_ws_url}")
                    last_delivered = 0.0
                    while self._running:
                        payload_to_send = None
                        with self._lock:
                            if self.latest_payload and self.last_sent_time > last_delivered:
                                payload_to_send = self.latest_payload
                                last_delivered = self.last_sent_time

                        if payload_to_send:
                            await ws.send(json.dumps(payload_to_send))

                        await asyncio.sleep(0.2)
            except Exception as e:
                await asyncio.sleep(2.0)


def main(args=None):
    if not HAS_ROS2:
        print("[ERROR] ROS 2 environment is not sourced.")
        sys.exit(1)
    rclpy.init(args=args)
    node = MapBridgeToWeb()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._running = False
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
