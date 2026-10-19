#!/usr/bin/env python3
"""
JetBot Binary to ROS 2 LaserScan Adapter (Distributed Edge-Compute Architecture)
================================================================================
Reference: Google Cartographer 2D (Hess et al. 2016), ROS 2 REP-103 / REP-105.
Receives 1,130-byte JB-Binary composite frames from NVIDIA JetBot (Jetson Nano)
and publishes standard sensor_msgs/msg/LaserScan to `/scan` along with REP-105 TF tree.
"""
import sys
import os
import math
import time
import struct
import socket
import asyncio
import threading
from typing import Optional, List, Tuple

# ROS 2 Imports with graceful fallback for standalone inspection
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
    from sensor_msgs.msg import LaserScan
    from geometry_msgs.msg import TransformStamped
    from std_msgs.msg import Float32, Header
    import tf2_ros
    HAS_ROS2 = True
except ImportError:
    HAS_ROS2 = False
    Node = object  # type: ignore

try:
    import websockets
    HAS_WEBSOCKETS = True
except ImportError:
    HAS_WEBSOCKETS = False

# JB-Binary Protocol v1.0 Definitions
MAGIC_JB = b"JB"
PROTOCOL_VERSION = 1
MSG_TYPE_COMPOSITE = 4
HEADER_STRUCT = struct.Struct("<2sBBIdH")
HEADER_SIZE = HEADER_STRUCT.size          # 18 bytes
TELEMETRY_STRUCT = struct.Struct("<8f")
TELEMETRY_SIZE = TELEMETRY_STRUCT.size    # 32 bytes
TOTAL_FRAME_SIZE = 1130                   # 18 + 32 + 360*2 + 360 = 1,130 bytes


def unpack_jb_frame(buf: bytes) -> Optional[dict]:
    """Unpack raw 1,130-byte JB-Binary frame into structured Python dictionary."""
    if len(buf) < TOTAL_FRAME_SIZE:
        return None
    magic, ver, mtype, seq, stamp, plen = HEADER_STRUCT.unpack(buf[:HEADER_SIZE])
    if magic != MAGIC_JB or ver != PROTOCOL_VERSION:
        return None

    offset = HEADER_SIZE
    x, y, th, v, w, near_obs, bat, temp = TELEMETRY_STRUCT.unpack(buf[offset:offset + TELEMETRY_SIZE])
    offset += TELEMETRY_SIZE

    # 360 uint16 range measurements in millimeters
    ranges_mm = struct.unpack_from("<360H", buf, offset)
    offset += 720

    # 360 uint8 intensity values
    intensities = struct.unpack_from("<360B", buf, offset)

    return {
        "seq": seq,
        "timestamp": stamp,
        "pose": {"x": x, "y": y, "theta": th},
        "linear_vel": v,
        "angular_vel": w,
        "nearest_obstacle": near_obs,
        "battery": bat,
        "cpu_temp": temp,
        "ranges_mm": ranges_mm,
        "intensities": intensities,
    }


class JetbotRosBridgeNode(Node if HAS_ROS2 else object):
    """
    ROS 2 Bridge Node:
    - Receives JB-Binary frames over WebSocket or UDP.
    - Publishes sensor_msgs/msg/LaserScan on `/scan` (QoS: SENSOR_DATA).
    - Publishes static transform: `base_link` -> `laser_frame` (+0.08m lever arm).
    """

    def __init__(self):
        if not HAS_ROS2:
            raise RuntimeError("ROS 2 (rclpy) is required to run JetbotRosBridgeNode.")
        super().__init__('jetbot_ros_bridge_node')

        # Parameter Declarations
        self.declare_parameter('connection_mode', 'ws_client')  # 'ws_client', 'ws_server', 'udp'
        self.declare_parameter('ws_url', 'ws://127.0.0.1:8000/ws/robot')
        self.declare_parameter('ws_host', '0.0.0.0')
        self.declare_parameter('ws_port', 8765)
        self.declare_parameter('udp_port', 8765)
        self.declare_parameter('laser_frame_id', 'laser_frame')
        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('laser_offset_x', 0.08)
        self.declare_parameter('laser_offset_y', 0.00)
        self.declare_parameter('laser_offset_z', 0.07)
        self.declare_parameter('range_min', 0.05)
        self.declare_parameter('range_max', 6.0)

        self.conn_mode = self.get_parameter('connection_mode').value
        self.ws_url = self.get_parameter('ws_url').value
        self.ws_host = self.get_parameter('ws_host').value
        self.ws_port = int(self.get_parameter('ws_port').value)
        self.udp_port = int(self.get_parameter('udp_port').value)
        self.laser_frame_id = self.get_parameter('laser_frame_id').value
        self.base_frame_id = self.get_parameter('base_frame_id').value
        self.offset_x = float(self.get_parameter('laser_offset_x').value)
        self.offset_y = float(self.get_parameter('laser_offset_y').value)
        self.offset_z = float(self.get_parameter('laser_offset_z').value)
        self.range_min = float(self.get_parameter('range_min').value)
        self.range_max = float(self.get_parameter('range_max').value)

        # QoS Profiles (REP-2003: Best effort, keep last 5 for high-rate sensor streams)
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
            durability=QoSDurabilityPolicy.VOLATILE
        )

        self.scan_pub = self.create_publisher(LaserScan, '/scan', sensor_qos)
        self.battery_pub = self.create_publisher(Float32, '/jetbot/battery', 10)

        # Static TF Broadcaster: base_link -> laser_frame
        self.static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self.broadcast_static_tf()

        self.total_scans_received = 0
        self.last_scan_time = 0.0
        self._running = True

        # Launch background receiver thread
        self.receiver_thread = threading.Thread(target=self._run_receiver, daemon=True)
        self.receiver_thread.start()

        self.get_logger().info(
            f"[INIT] JetBot ROS 2 Bridge active (Mode: {self.conn_mode}, Target: {self.ws_url if self.conn_mode=='ws_client' else self.ws_port})"
        )

    def broadcast_static_tf(self):
        """Broadcast REP-105 static transform: base_link -> laser_frame."""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.base_frame_id
        t.child_frame_id = self.laser_frame_id
        t.transform.translation.x = self.offset_x
        t.transform.translation.y = self.offset_y
        t.transform.translation.z = self.offset_z
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        self.static_tf_broadcaster.sendTransform(t)
        self.get_logger().info(
            f"[TF] Static Transform published: {self.base_frame_id} -> {self.laser_frame_id} (offset x={self.offset_x}m)"
        )

    def publish_laserscan(self, frame_data: dict):
        """Convert unpacked JB-Binary frame into standard sensor_msgs/LaserScan."""
        now_msg = self.get_clock().now().to_msg()
        msg = LaserScan()
        msg.header.stamp = now_msg
        msg.header.frame_id = self.laser_frame_id

        # LDROBOT D500: Full 360 degrees rotation counter-clockwise
        msg.angle_min = 0.0
        msg.angle_max = 2.0 * math.pi
        msg.angle_increment = (2.0 * math.pi) / 360.0
        msg.time_increment = 0.1 / 360.0  # 10 Hz sweep period
        msg.scan_time = 0.1
        msg.range_min = self.range_min
        msg.range_max = self.range_max

        ranges_mm = frame_data["ranges_mm"]
        min_mm = int(self.range_min * 1000.0)
        max_mm = int(self.range_max * 1000.0)

        # Convert uint16 mm to meters; invalid returns mapped to +inf per ROS standard
        msg.ranges = [
            float(r) / 1000.0 if (min_mm < r <= max_mm) else float('inf')
            for r in ranges_mm
        ]
        msg.intensities = [float(i) for i in frame_data["intensities"]]

        self.scan_pub.publish(msg)
        self.total_scans_received += 1
        self.last_scan_time = time.time()

        if frame_data.get("battery") is not None:
            bat_msg = Float32()
            bat_msg.data = float(frame_data["battery"])
            self.battery_pub.publish(bat_msg)

    def _run_receiver(self):
        """Background thread executing the asynchronous network reception loop."""
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        if self.conn_mode == 'ws_client':
            loop.run_until_complete(self._ws_client_loop())
        elif self.conn_mode == 'ws_server':
            loop.run_until_complete(self._ws_server_loop())
        elif self.conn_mode == 'udp':
            self._udp_loop()
        else:
            self.get_logger().error(f"Unknown connection_mode: {self.conn_mode}")

    async def _ws_client_loop(self):
        """Connect as a WebSocket client to the hardware bridge or dashboard."""
        if not HAS_WEBSOCKETS:
            self.get_logger().error("websockets package not installed. Run: pip install websockets")
            return

        backoff = 1.0
        while self._running:
            try:
                self.get_logger().info(f"[WS] Connecting to {self.ws_url}...")
                async with websockets.connect(
                    self.ws_url,
                    ping_interval=3.0,
                    ping_timeout=3.0,
                    close_timeout=2.0,
                    max_size=10_000_000
                ) as ws:
                    self.get_logger().info(f"[WS] Connected successfully to {self.ws_url}!")
                    backoff = 1.0
                    while self._running:
                        msg = await ws.recv()
                        # Real-time lock: If incoming frames accumulated in the client deque,
                        # drain to the newest binary frame so ROS LaserScan publishes at 0 lag.
                        msgs_deque = getattr(ws, "messages", None)
                        if msgs_deque and len(msgs_deque) > 0:
                            while msgs_deque:
                                item = msgs_deque.popleft()
                                if isinstance(item, bytes):
                                    msg = item

                        if isinstance(msg, bytes):
                            frame = unpack_jb_frame(msg)
                            if frame:
                                self.publish_laserscan(frame)
                        elif isinstance(msg, str):
                            # Non-binary telemetry (e.g. heartbeat/meta)
                            pass
            except asyncio.CancelledError:
                break
            except Exception as e:
                self.get_logger().warning(f"[WS] Connection error: {e}. Reconnecting in {backoff:.1f}s...")
                await asyncio.sleep(backoff)
                backoff = min(backoff * 1.5, 10.0)

    async def _ws_server_loop(self):
        """Host a dedicated WebSocket server for JetBot hardware bridge."""
        if not HAS_WEBSOCKETS:
            self.get_logger().error("websockets package not installed.")
            return

        async def handler(websocket):
            self.get_logger().info(f"[WS-SERVER] JetBot client connected from {websocket.remote_address}")
            try:
                async for message in websocket:
                    if isinstance(message, bytes):
                        frame = unpack_jb_frame(message)
                        if frame:
                            self.publish_laserscan(frame)
            except Exception as ex:
                self.get_logger().warning(f"[WS-SERVER] Client disconnected: {ex}")

        server = await websockets.serve(handler, self.ws_host, self.ws_port)
        self.get_logger().info(f"[WS-SERVER] Listening on ws://{self.ws_host}:{self.ws_port}")
        await server.wait_closed()

    def _udp_loop(self):
        """Listen on UDP socket for zero-overhead datagram streaming."""
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(('0.0.0.0', self.udp_port))
        self.get_logger().info(f"[UDP] Listening on 0.0.0.0:{self.udp_port}")
        while self._running:
            try:
                data, _ = sock.recvfrom(2048)
                if len(data) >= TOTAL_FRAME_SIZE:
                    frame = unpack_jb_frame(data)
                    if frame:
                        self.publish_laserscan(frame)
            except Exception as e:
                if self._running:
                    self.get_logger().warning(f"[UDP] Error: {e}")


def main(args=None):
    if not HAS_ROS2:
        print("[ERROR] ROS 2 environment is not sourced. Run: source /opt/ros/<distro>/setup.bash")
        sys.exit(1)
    rclpy.init(args=args)
    node = JetbotRosBridgeNode()
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
