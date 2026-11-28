#!/usr/bin/env python3
"""
scripts/record_cv_session.py — Synchronized field recorder for CV + LiDAR sessions (TK-CV-04 & TK-CV-02).

Records in one session:
  - <out_dir>/lidar.npz          : 360-ray LiDAR sweeps (t, ranges, seq, stamp, w_bridge, v_bridge)
  - <out_dir>/detections.jsonl   : robot_meta / YOLO detection packets with scan_seq
  - <out_dir>/frames/            : JPEG snapshots polled from JetBot /snapshot
  - <out_dir>/frames_index.jsonl : Frame index linking each JPEG to nearest scan_seq & detections

Protocol matches dashboard/jetbot_bridge.py (1130-byte JB binary frames, robot_meta JSON, HTTP /snapshot).
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
import threading
import time
import urllib.request
from pathlib import Path
from typing import Any, Dict, List, Optional

import numpy as np

try:
    import websockets
except ImportError:
    websockets = None  # Allows offline unit/smoke testing of packet handlers

logging.basicConfig(level=logging.INFO, format="[%(asctime)s] %(levelname)s: %(message)s")
logger = logging.getLogger("record_cv_session")

HEADER_STRUCT = struct.Struct("<2sBBIdH")
HEADER_SIZE = HEADER_STRUCT.size  # 18
TELEMETRY_STRUCT = struct.Struct("<8f")
TELEMETRY_SIZE = TELEMETRY_STRUCT.size  # 32
MAGIC_JB = b"JB"
PROTOCOL_VERSION = 1
TOTAL_FRAME_SIZE = 1130


class CVSessionRecorder:
    """Thread-safe synchronized recorder for LiDAR sweeps, YOLO metadata, and camera JPEGs."""

    def __init__(
        self,
        out_dir: str,
        duration_s: float = 60.0,
        flush_interval_s: float = 5.0,
    ):
        self.out_dir = Path(out_dir)
        self.frames_dir = self.out_dir / "frames"
        self.out_dir.mkdir(parents=True, exist_ok=True)
        self.frames_dir.mkdir(parents=True, exist_ok=True)

        self.lidar_path = self.out_dir / "lidar.npz"
        self.detections_path = self.out_dir / "detections.jsonl"
        self.frames_index_path = self.out_dir / "frames_index.jsonl"

        self.duration_s = float(duration_s)
        self.flush_interval_s = float(flush_interval_s)

        self._lock = threading.Lock()
        self.t_list: List[float] = []
        self.ranges_list: List[np.ndarray] = []
        self.seq_list: List[int] = []
        self.stamp_list: List[float] = []
        self.w_list: List[float] = []
        self.v_list: List[float] = []

        self.latest_scan_seq: int = -1
        self.latest_scan_stamp: float = 0.0
        self.latest_detections: List[Dict[str, Any]] = []
        self.latest_yolo_active: bool = False

        self.detection_packet_count: int = 0
        self.detection_first_t: Optional[float] = None
        self.detection_last_t: Optional[float] = None
        self.frame_count: int = 0

        self.start_time: Optional[float] = None
        self.last_flush_time: float = 0.0
        self.stop_event: Optional[asyncio.Event] = None
        self.stop_flag = threading.Event()

    def _mark_started(self, now: float) -> None:
        if self.start_time is None:
            self.start_time = now
            self.last_flush_time = now
            logger.info("Recording started (target %.1f s) -> %s", self.duration_s, self.out_dir)

    def _check_duration(self, now: float) -> None:
        if self.start_time is not None and (now - self.start_time) >= self.duration_s:
            self.stop_flag.set()
            if self.stop_event is not None:
                self.stop_event.set()

    def process_binary_frame(self, buf: bytes, host_t: Optional[float] = None) -> bool:
        """Parse 1130-byte JB binary telemetry + 360-ray LiDAR frame."""
        if len(buf) < TOTAL_FRAME_SIZE:
            return False
        magic, ver, _mtype, seq, stamp, _plen = HEADER_STRUCT.unpack(buf[:HEADER_SIZE])
        if magic != MAGIC_JB or ver != PROTOCOL_VERSION:
            return False

        _x, _y, _th, v, w, _near_obs, _bat, _temp = TELEMETRY_STRUCT.unpack(
            buf[HEADER_SIZE : HEADER_SIZE + TELEMETRY_SIZE]
        )
        ranges_mm = np.frombuffer(buf[50:770], dtype=np.uint16)
        ranges_m = ranges_mm.astype(np.float32) / 1000.0

        now = time.time() if host_t is None else float(host_t)
        need_flush = False
        with self._lock:
            self._mark_started(now)
            self.t_list.append(now)
            self.ranges_list.append(ranges_m)
            self.seq_list.append(int(seq))
            self.stamp_list.append(float(stamp))
            self.w_list.append(float(w))
            self.v_list.append(float(v))
            self.latest_scan_seq = int(seq)
            self.latest_scan_stamp = float(stamp)
            if now - self.last_flush_time >= self.flush_interval_s:
                need_flush = True
                self.last_flush_time = now

        if need_flush:
            self.flush_lidar()
        self._check_duration(now)
        return True

    def process_json_frame(self, text: str, host_t: Optional[float] = None) -> bool:
        """Parse robot_meta JSON detections (and fallback JSON lidar if present)."""
        try:
            data = json.loads(text)
        except Exception:
            return False
        if not isinstance(data, dict):
            return False

        now = time.time() if host_t is None else float(host_t)
        handled = False

        # Optional JSON lidar fallback
        ranges = data.get("lidar_ranges") or data.get("ranges")
        if isinstance(ranges, list) and len(ranges) >= 180:
            arr = np.zeros(360, dtype=np.float32)
            n = min(len(ranges), 360)
            arr[:n] = [float(x) for x in ranges[:n]]
            seq = int(data.get("scan_seq", data.get("seq", len(self.t_list))))
            stamp = float(data.get("scan_stamp", data.get("timestamp", now)))
            with self._lock:
                self._mark_started(now)
                self.t_list.append(now)
                self.ranges_list.append(arr)
                self.seq_list.append(seq)
                self.stamp_list.append(stamp)
                self.w_list.append(float(data.get("angular_speed", 0.0)))
                self.v_list.append(float(data.get("linear_speed", 0.0)))
                self.latest_scan_seq = seq
                self.latest_scan_stamp = stamp
            handled = True

        is_meta = (
            data.get("type") == "robot_meta"
            or "detections" in data
            or "yolo_active" in data
        )
        if is_meta:
            dets = data.get("detections")
            if not isinstance(dets, list):
                dets = []
            yolo_active = bool(data.get("yolo_active", False))
            scan_seq = int(data.get("scan_seq", self.latest_scan_seq))
            scan_stamp = float(data.get("scan_stamp", self.latest_scan_stamp))
            record = {
                "host_t": now,
                "bridge_timestamp": data.get("timestamp"),
                "scan_seq": scan_seq,
                "scan_stamp": scan_stamp,
                "yolo_active": yolo_active,
                "detections": dets,
            }
            with self._lock:
                self._mark_started(now)
                self.latest_detections = dets
                self.latest_yolo_active = yolo_active
                if scan_seq >= 0:
                    self.latest_scan_seq = scan_seq
                    self.latest_scan_stamp = scan_stamp
                self.detection_packet_count += 1
                if self.detection_first_t is None:
                    self.detection_first_t = now
                self.detection_last_t = now
                with self.detections_path.open("a", encoding="utf-8") as f:
                    f.write(json.dumps(record) + "\n")
            handled = True

        self._check_duration(now)
        return handled

    def save_snapshot_bytes(self, jpeg_bytes: bytes, host_t: Optional[float] = None) -> Path:
        """Persist a raw JPEG frame and link it to the latest scan_seq and detections."""
        if not jpeg_bytes:
            raise ValueError("Empty JPEG snapshot bytes.")
        now = time.time() if host_t is None else float(host_t)
        with self._lock:
            self._mark_started(now)
            idx = self.frame_count
            self.frame_count += 1
            seq = self.latest_scan_seq
            stamp = self.latest_scan_stamp
            dets = list(self.latest_detections)
            yolo_active = self.latest_yolo_active

            fname = f"frame_{idx:05d}_seq_{seq}.jpg"
            fpath = self.frames_dir / fname
            fpath.write_bytes(jpeg_bytes)

            entry = {
                "frame_idx": idx,
                "filename": f"frames/{fname}",
                "host_t": now,
                "scan_seq": seq,
                "scan_stamp": stamp,
                "yolo_active": yolo_active,
                "detections": dets,
            }
            with self.frames_index_path.open("a", encoding="utf-8") as f:
                f.write(json.dumps(entry) + "\n")
        return fpath

    def flush_lidar(self) -> None:
        """Atomically save accumulated LiDAR scans to lidar.npz."""
        with self._lock:
            if not self.t_list:
                return
            t_arr = np.array(self.t_list, dtype=np.float64)
            r_arr = np.array(self.ranges_list, dtype=np.float32)
            seq_arr = np.array(self.seq_list, dtype=np.int32)
            stamp_arr = np.array(self.stamp_list, dtype=np.float64)
            w_arr = np.array(self.w_list, dtype=np.float32)
            v_arr = np.array(self.v_list, dtype=np.float32)

        tmp_path = str(self.lidar_path) + ".tmp.npz"
        np.savez_compressed(
            tmp_path,
            t=t_arr,
            ranges=r_arr,
            seq=seq_arr,
            stamp=stamp_arr,
            w_bridge=w_arr,
            v_bridge=v_arr,
        )
        os.replace(tmp_path, self.lidar_path)

    def summary(self) -> Dict[str, Any]:
        """Compute recording statistics for end-of-session verification."""
        with self._lock:
            n_scans = len(self.t_list)
            dur = float(self.t_list[-1] - self.t_list[0]) if n_scans > 1 else 0.0
            hz = (n_scans - 1) / dur if dur > 0.0 else 0.0
            max_gap = float(np.max(np.diff(self.t_list))) if n_scans > 1 else 0.0
            det_count = self.detection_packet_count
            det_dur = (
                float(self.detection_last_t - self.detection_first_t)
                if self.detection_first_t is not None and self.detection_last_t is not None
                else 0.0
            )
            det_hz = (det_count - 1) / det_dur if det_dur > 0.0 else 0.0
            n_frames = self.frame_count
        return {
            "lidar_scans": n_scans,
            "lidar_duration_s": round(dur, 2),
            "lidar_hz": round(hz, 2),
            "lidar_max_gap_s": round(max_gap, 3),
            "detection_packets": det_count,
            "detection_hz": round(det_hz, 2),
            "camera_frames": n_frames,
            "out_dir": str(self.out_dir),
        }


def _snapshot_worker(
    recorder: CVSessionRecorder,
    snapshot_url: str,
    fps: float,
    timeout: float = 2.0,
) -> None:
    period = 1.0 / max(0.2, float(fps))
    while not recorder.stop_flag.is_set():
        t0 = time.monotonic()
        try:
            req = urllib.request.Request(snapshot_url, headers={"User-Agent": "cs532-cv-rec/1.0"})
            with urllib.request.urlopen(req, timeout=timeout) as resp:
                data = resp.read()
            if data:
                recorder.save_snapshot_bytes(data)
        except Exception as exc:
            logger.debug("Snapshot poll skipped: %s", exc)
        elapsed = time.monotonic() - t0
        sleep_s = max(0.02, period - elapsed)
        if recorder.stop_flag.wait(sleep_s):
            break


async def _run_server(recorder: CVSessionRecorder, host: str, port: int) -> None:
    logger.info("Listening WebSocket server on ws://%s:%d/ws/robot", host, port)

    async def handle_conn(ws):
        logger.info("Bridge connected from %s", ws.remote_address)
        try:
            async for msg in ws:
                if isinstance(msg, bytes):
                    recorder.process_binary_frame(msg)
                elif isinstance(msg, str):
                    recorder.process_json_frame(msg)
                if recorder.stop_event is not None and recorder.stop_event.is_set():
                    break
        except Exception as exc:
            logger.warning("WebSocket ended: %s", exc)

    server = await websockets.serve(handle_conn, host, port)
    if recorder.stop_event is not None:
        await recorder.stop_event.wait()
    server.close()
    await server.wait_closed()


async def _run_client(recorder: CVSessionRecorder, url: str) -> None:
    logger.info("Connecting WebSocket client to %s", url)
    async with websockets.connect(url) as ws:
        try:
            await ws.send(json.dumps({"type": "enable_binary_telemetry", "enabled": True}))
        except Exception:
            pass
        async for msg in ws:
            if isinstance(msg, bytes):
                recorder.process_binary_frame(msg)
            elif isinstance(msg, str):
                recorder.process_json_frame(msg)
            if recorder.stop_event is not None and recorder.stop_event.is_set():
                break


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        description="Record synchronized LiDAR sweeps, YOLO detections, and camera frames."
    )
    parser.add_argument("--scenario", default="cv_person", help="Session label for default output path")
    parser.add_argument("--duration", type=float, default=60.0, help="Recording duration in seconds")
    parser.add_argument("--out-dir", type=str, default=None, help="Output directory")
    parser.add_argument("--mode", choices=["server", "client"], default="server")
    parser.add_argument("--host", default="0.0.0.0", help="Server bind host")
    parser.add_argument("--port", type=int, default=8000, help="Server bind port")
    parser.add_argument("--url", default="ws://localhost:8000/ws", help="WebSocket URL for client mode")
    parser.add_argument(
        "--snapshot-url",
        default="http://192.168.1.18:8080/snapshot",
        help="JetBot HTTP /snapshot URL",
    )
    parser.add_argument("--camera-fps", type=float, default=5.0, help="Camera snapshot polling FPS")
    parser.add_argument("--no-camera", action="store_true", help="Disable HTTP snapshot polling")
    parser.add_argument("--flush-interval", type=float, default=5.0, help="LiDAR .npz flush period (s)")
    args = parser.parse_args(argv)

    if websockets is None:
        print("Error: websockets package required to run live recording.", file=sys.stderr)
        return 1

    if args.out_dir:
        out_dir = args.out_dir
    else:
        date_str = datetime.date.today().isoformat()
        out_dir = os.path.join("docs", "measurements", date_str, f"cv_session_{args.scenario}")

    recorder = CVSessionRecorder(
        out_dir=out_dir,
        duration_s=args.duration,
        flush_interval_s=args.flush_interval,
    )

    cam_thread: Optional[threading.Thread] = None
    if not args.no_camera and args.snapshot_url:
        cam_thread = threading.Thread(
            target=_snapshot_worker,
            args=(recorder, args.snapshot_url, args.camera_fps),
            daemon=True,
        )
        cam_thread.start()

    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    recorder.stop_event = asyncio.Event()

    def _stop_handler(*_args):
        recorder.stop_flag.set()
        if recorder.stop_event is not None:
            loop.call_soon_threadsafe(recorder.stop_event.set)

    for sig in (signal.SIGINT, signal.SIGTERM):
        try:
            signal.signal(sig, _stop_handler)
        except Exception:
            pass

    try:
        if args.mode == "server":
            loop.run_until_complete(_run_server(recorder, args.host, args.port))
        else:
            loop.run_until_complete(_run_client(recorder, args.url))
    finally:
        recorder.stop_flag.set()
        if cam_thread is not None:
            cam_thread.join(timeout=2.0)
        recorder.flush_lidar()
        loop.close()

    stats = recorder.summary()
    print(
        "\n[DONE] LiDAR: {lidar_scans} scans ({lidar_duration_s}s, {lidar_hz} Hz, max_gap={lidar_max_gap_s}s) | "
        "Detections: {detection_packets} pkts ({detection_hz} Hz) | "
        "Frames: {camera_frames} -> {out_dir}".format(**stats)
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
