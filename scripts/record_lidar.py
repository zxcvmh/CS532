#!/usr/bin/env python3
"""
scripts/record_lidar.py — Record D500 LiDAR sweeps into .npz files for TK-MEAS-01.

Records 360-degree LiDAR sweeps over Wi-Fi directly from jetbot_bridge.py or backend.
Saves .npz with keys matching field_turns_2026-11-15.npz:
  - t: (n,) float64 timestamps
  - ranges: (n, 360) float32 ranges in meters
  - seq: (n,) int32 scan sequence numbers
  - stamp: (n,) float64 hardware timestamps
  - w_bridge: (n,) float32 angular speed
  - v_bridge: (n,) float32 linear speed

Flushes to disk every 5 seconds (atomic save) so data is never lost on interruption.
"""

import argparse
import asyncio
import datetime
import json
import logging
import os
import signal
import struct
import sys
import time
from typing import Optional

import numpy as np

try:
    import websockets
except ImportError:
    print("Error: websockets package required. Install with pip install websockets", file=sys.stderr)
    sys.exit(1)

logging.basicConfig(level=logging.INFO, format="[%(asctime)s] %(levelname)s: %(message)s")
logger = logging.getLogger("record_lidar")

HEADER_STRUCT = struct.Struct("<2sBBIdH")
HEADER_SIZE = HEADER_STRUCT.size  # 18
TELEMETRY_STRUCT = struct.Struct("<8f")
TELEMETRY_SIZE = TELEMETRY_STRUCT.size  # 32
MAGIC_JB = b"JB"
PROTOCOL_VERSION = 1
TOTAL_FRAME_SIZE = 1130


class LidarRecorder:
    def __init__(self, output_path: str, duration_s: float = 60.0, flush_interval_s: float = 5.0):
        self.output_path = output_path
        self.duration_s = duration_s
        self.flush_interval_s = flush_interval_s

        self.t_list = []
        self.ranges_list = []
        self.seq_list = []
        self.stamp_list = []
        self.w_list = []
        self.v_list = []

        self.start_time: Optional[float] = None
        self.last_flush_time = 0.0
        self.is_recording = False
        self.stop_event = asyncio.Event()

    def process_binary_frame(self, buf: bytes) -> bool:
        if len(buf) < TOTAL_FRAME_SIZE:
            return False
        magic, ver, mtype, seq, stamp, plen = HEADER_STRUCT.unpack(buf[:HEADER_SIZE])
        if magic != MAGIC_JB or ver != PROTOCOL_VERSION:
            return False

        x, y, th, v, w, near_obs, bat, temp = TELEMETRY_STRUCT.unpack(
            buf[HEADER_SIZE : HEADER_SIZE + TELEMETRY_SIZE]
        )
        ranges_mm = np.frombuffer(buf[50:770], dtype=np.uint16)
        ranges_m = ranges_mm.astype(np.float32) / 1000.0

        now = time.time()
        if self.start_time is None:
            self.start_time = now
            logger.info("First scan received! Recording started for %.1f s...", self.duration_s)

        self.t_list.append(now)
        self.ranges_list.append(ranges_m)
        self.seq_list.append(int(seq))
        self.stamp_list.append(float(stamp))
        self.w_list.append(float(w))
        self.v_list.append(float(v))

        self.check_periodic_flush(now)
        if (now - self.start_time) >= self.duration_s:
            logger.info("Reached target duration %.1f s (%d scans).", self.duration_s, len(self.t_list))
            self.stop_event.set()
        return True

    def process_json_frame(self, text: str) -> bool:
        try:
            data = json.loads(text)
        except Exception:
            return False

        ranges = data.get("lidar_ranges") or data.get("ranges")
        if not ranges or len(ranges) < 180:
            return False

        arr = np.zeros(360, dtype=np.float32)
        n = min(len(ranges), 360)
        arr[:n] = [float(x) for x in ranges[:n]]

        now = time.time()
        if self.start_time is None:
            self.start_time = now
            logger.info("First scan received! Recording started for %.1f s...", self.duration_s)

        self.t_list.append(now)
        self.ranges_list.append(arr)
        self.seq_list.append(int(data.get("seq", len(self.t_list))))
        self.stamp_list.append(float(data.get("timestamp", now)))
        self.w_list.append(float(data.get("angular_speed", 0.0)))
        self.v_list.append(float(data.get("linear_speed", 0.0)))

        self.check_periodic_flush(now)
        if (now - self.start_time) >= self.duration_s:
            logger.info("Reached target duration %.1f s (%d scans).", self.duration_s, len(self.t_list))
            self.stop_event.set()
        return True

    def check_periodic_flush(self, now: float):
        if now - self.last_flush_time >= self.flush_interval_s:
            self.flush()
            self.last_flush_time = now

    def flush(self):
        if not self.t_list:
            return
        os.makedirs(os.path.dirname(os.path.abspath(self.output_path)), exist_ok=True)
        tmp_path = self.output_path + ".tmp.npz"
        t_arr = np.array(self.t_list, dtype=np.float64)
        r_arr = np.array(self.ranges_list, dtype=np.float32)
        seq_arr = np.array(self.seq_list, dtype=np.int32)
        stamp_arr = np.array(self.stamp_list, dtype=np.float64)
        w_arr = np.array(self.w_list, dtype=np.float32)
        v_arr = np.array(self.v_list, dtype=np.float32)

        np.savez_compressed(
            tmp_path,
            t=t_arr,
            ranges=r_arr,
            seq=seq_arr,
            stamp=stamp_arr,
            w_bridge=w_arr,
            v_bridge=v_arr,
        )
        os.replace(tmp_path, self.output_path)
        logger.info(
            "Flushed %d scans (%.1f s) to %s",
            len(self.t_list),
            float(t_arr[-1] - t_arr[0]) if len(t_arr) > 1 else 0.0,
            self.output_path,
        )


async def run_server(recorder: LidarRecorder, host: str, port: int):
    logger.info("Starting WebSocket server on ws://%s:%d/ws/robot", host, port)

    async def handle_connection(websocket):
        logger.info("Bridge connected from %s", websocket.remote_address)
        try:
            async for message in websocket:
                if isinstance(message, bytes):
                    recorder.process_binary_frame(message)
                elif isinstance(message, str):
                    recorder.process_json_frame(message)
                if recorder.stop_event.is_set():
                    break
        except Exception as e:
            logger.warning("Connection ended: %s", e)

    server = await websockets.serve(handle_connection, host, port)
    await recorder.stop_event.wait()
    server.close()
    await server.wait_closed()


async def run_client(recorder: LidarRecorder, url: str):
    logger.info("Connecting as client to %s", url)
    async with websockets.connect(url) as ws:
        # Request binary telemetry if connected to backend /ws
        try:
            await ws.send(json.dumps({"type": "enable_binary_telemetry", "enabled": True}))
        except Exception:
            pass

        async for message in ws:
            if isinstance(message, bytes):
                recorder.process_binary_frame(message)
            elif isinstance(message, str):
                recorder.process_json_frame(message)
            if recorder.stop_event.is_set():
                break


def main():
    parser = argparse.ArgumentParser(description="Record JetBot D500 LiDAR data into .npz")
    parser.add_argument("--scenario", choices=["static_person", "crossing", "head_on"], default="static_person",
                        help="Recording scenario name")
    parser.add_argument("--duration", type=float, default=60.0, help="Duration in seconds (default 60)")
    parser.add_argument("--output", type=str, default=None, help="Output .npz file path")
    parser.add_argument("--mode", choices=["server", "client"], default="server",
                        help="Server mode (bridge connects here) or Client mode (connect to dashboard)")
    parser.add_argument("--host", type=str, default="0.0.0.0", help="Listen host for server mode")
    parser.add_argument("--port", type=int, default=8000, help="Port for server mode (default 8000)")
    parser.add_argument("--url", type=str, default="ws://localhost:8000/ws", help="URL for client mode")
    parser.add_argument("--flush-interval", type=float, default=5.0, help="Flush interval in seconds")

    args = parser.parse_args()

    date_str = datetime.date.today().isoformat()
    if args.output:
        out_path = args.output
    else:
        out_path = os.path.join("docs", "measurements", date_str, f"lidar_{args.scenario}.npz")

    recorder = LidarRecorder(
        output_path=out_path,
        duration_s=args.duration,
        flush_interval_s=args.flush_interval,
    )

    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)

    def handle_signal():
        logger.info("Stop signal received. Finishing recording...")
        recorder.stop_event.set()

    for sig in (signal.SIGINT, signal.SIGTERM):
        try:
            loop.add_signal_handler(sig, handle_signal)
        except NotImplementedError:
            pass

    try:
        if args.mode == "server":
            loop.run_until_complete(run_server(recorder, args.host, args.port))
        else:
            loop.run_until_complete(run_client(recorder, args.url))
    finally:
        recorder.flush()
        if recorder.t_list:
            dur = recorder.t_list[-1] - recorder.t_list[0]
            hz = len(recorder.t_list) / max(0.1, dur)
            print(f"\n[DONE] Recorded {len(recorder.t_list)} scans in {dur:.2f} s ({hz:.1f} Hz)")
            print(f"File saved: {recorder.output_path} ({os.path.getsize(recorder.output_path)} bytes)\n")
        else:
            print(f"\n[WARN] No scans recorded. Empty file was not saved.\n")


if __name__ == "__main__":
    main()
