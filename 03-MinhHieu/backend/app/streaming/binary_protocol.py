"""
JB-Binary Protocol v1.0
High-performance, zero-copy binary serialization for 360-degree LiDAR and JetBot Telemetry.
Payload size: Exactly 1,130 Bytes (Fits inside a single Wi-Fi MTU frame of 1,500 Bytes).
"""
import struct
import time
from typing import Tuple, List, Optional, Union
import numpy as np

# Header Layout: 18 Bytes
# Magic(2s) 'JB', Version(B)=1, MsgType(B)=4 (Composite), Seq(I)=uint32, Stamp(d)=float64, PayloadLen(H)=uint16
HEADER_STRUCT = struct.Struct("<2sBBIdH")
HEADER_SIZE = HEADER_STRUCT.size  # 18

# Telemetry Layout: 32 Bytes
# x(f), y(f), theta(f), lin_vel(f), ang_vel(f), nearest_obs(f), battery(f), cpu_temp(f) -> 8 x float32
TELEMETRY_STRUCT = struct.Struct("<8f")
TELEMETRY_SIZE = TELEMETRY_STRUCT.size  # 32

MAGIC_JB = b"JB"
PROTOCOL_VERSION = 1
MSG_TYPE_COMPOSITE = 4
TOTAL_FRAME_SIZE = 1130  # 18 + 32 + 360*2 + 360 = 1130


class BinaryTelemetryPacker:
    """Zero-copy binary packer for JetBot LiDAR point cloud & telemetry."""

    def __init__(self):
        self.seq = 0

    def pack(
        self,
        pose_x: float,
        pose_y: float,
        pose_theta: float,
        linear_vel: float,
        angular_vel: float,
        nearest_obs: Optional[float],
        battery: float,
        cpu_temp: float,
        ranges_m: Union[List[float], np.ndarray],
        intensities: Optional[Union[List[int], np.ndarray]] = None,
        timestamp: Optional[float] = None,
        seq: Optional[int] = None
    ) -> bytes:
        if seq is not None:
            self.seq = int(seq) & 0xFFFFFFFF
        else:
            self.seq = (self.seq + 1) & 0xFFFFFFFF
        ts = timestamp if timestamp is not None else time.time()

        # 1. Telemetry body (32B)
        telem_bytes = TELEMETRY_STRUCT.pack(
            float(pose_x),
            float(pose_y),
            float(pose_theta),
            float(linear_vel),
            float(angular_vel),
            float(nearest_obs if nearest_obs is not None else 0.0),
            float(battery if battery is not None else 100.0),
            float(cpu_temp if cpu_temp is not None else 43.5)
        )

        # 2. Ranges array: 360 values of uint16 millimeters (720B)
        if isinstance(ranges_m, np.ndarray):
            r_arr = ranges_m
        else:
            r_arr = np.array(ranges_m if ranges_m is not None else [0.0]*360, dtype=np.float32)
        if len(r_arr) != 360:
            padded = np.zeros(360, dtype=np.float32)
            n = min(len(r_arr), 360)
            padded[:n] = r_arr[:n]
            r_arr = padded

        ranges_mm = np.clip(r_arr * 1000.0, 0, 65535).astype(np.uint16)
        ranges_bytes = ranges_mm.tobytes()

        # 3. Intensities array: 360 values of uint8 (360B)
        if intensities is None:
            int_bytes = bytes([128] * 360)
        elif isinstance(intensities, np.ndarray):
            int_bytes = np.clip(intensities, 0, 255).astype(np.uint8).tobytes()
        else:
            i_arr = np.array(intensities, dtype=np.int32)
            if len(i_arr) != 360:
                padded_i = np.zeros(360, dtype=np.uint8)
                n = min(len(i_arr), 360)
                padded_i[:n] = np.clip(i_arr[:n], 0, 255)
                int_bytes = padded_i.tobytes()
            else:
                int_bytes = np.clip(i_arr, 0, 255).astype(np.uint8).tobytes()

        body = telem_bytes + ranges_bytes + int_bytes
        header = HEADER_STRUCT.pack(MAGIC_JB, PROTOCOL_VERSION, MSG_TYPE_COMPOSITE, self.seq, ts, len(body))
        return header + body


def unpack_binary_telemetry(buf: bytes) -> Optional[dict]:
    """Unpack 1,130-byte JB-Binary frame into structured Python dictionary."""
    if len(buf) < TOTAL_FRAME_SIZE:
        return None
    magic, ver, mtype, seq, stamp, plen = HEADER_STRUCT.unpack(buf[:HEADER_SIZE])
    if magic != MAGIC_JB or ver != PROTOCOL_VERSION:
        return None

    x, y, th, v, w, near_obs, bat, temp = TELEMETRY_STRUCT.unpack(buf[HEADER_SIZE:HEADER_SIZE + TELEMETRY_SIZE])
    ranges_mm = np.frombuffer(buf[50:770], dtype=np.uint16)
    ranges_m = ranges_mm.astype(np.float32) / 1000.0
    intensities = np.frombuffer(buf[770:1130], dtype=np.uint8)

    return {
        "seq": seq,
        "timestamp": stamp,
        "pose": {"x": x, "y": y, "theta": th},
        "linear_speed": v,
        "angular_speed": w,
        "nearest_obstacle": near_obs if near_obs > 0.001 else None,
        "battery": bat,
        "cpu_temp": temp,
        "ranges": ranges_m,
        "intensities": intensities,
    }


def patch_binary_pose(
    buf: bytes,
    pose_x: float,
    pose_y: float,
    pose_theta: float,
    linear_vel: Optional[float] = None,
    angular_vel: Optional[float] = None,
) -> bytes:
    """In-place update of SLAM world pose and velocities inside binary frame before forwarding to frontend."""
    ba = bytearray(buf)
    if len(ba) >= HEADER_SIZE + 20:
        if linear_vel is not None and angular_vel is not None:
            struct.pack_into("<5f", ba, HEADER_SIZE, float(pose_x), float(pose_y), float(pose_theta), float(linear_vel), float(angular_vel))
        else:
            struct.pack_into("<3f", ba, HEADER_SIZE, float(pose_x), float(pose_y), float(pose_theta))
    return bytes(ba)
