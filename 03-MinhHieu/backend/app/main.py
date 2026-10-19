"""
Main FastAPI Application Entrypoint for JetBot Dashboard.
Clean Architecture orchestrator connecting hardware bridge, streaming hub, and navigation coordinator.
"""
import os
import sys
import asyncio
import logging
import json
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from starlette.websockets import WebSocketDisconnected
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import StreamingResponse, Response
from fastapi.staticfiles import StaticFiles

# Phase 8: rotating file logging BEFORE anything else can log.
from app.logging_setup import configure_logging
from app.api import routes
from app.hardware.bridge_client import BridgeClient
from app.streaming.mjpeg_streamer import MjpegStreamer
from app.streaming.websocket_hub import WebSocketHub
from app.navigation.coordinator import NavigationCoordinator

configure_logging()

# Instantiate decoupled clean architecture modules
bridge_client = BridgeClient()
mjpeg_streamer = MjpegStreamer()
coordinator = NavigationCoordinator(bridge=bridge_client, streamer=mjpeg_streamer)
ws_manager = WebSocketHub(coordinator)

app = FastAPI(title="JetBot Dashboard API")
bridge_process = None


@app.on_event("startup")
async def startup_event():
    global bridge_process
    asyncio.ensure_future(coordinator.update())
    asyncio.ensure_future(ws_manager.broadcaster_task())

    # Auto-start hardware bridge ONLY when explicitly requested via AUTO_START_BRIDGE="1"
    # Default is "0" to prevent duplicate process collision when running distributed or via start_all.sh
    auto_bridge = os.environ.get("AUTO_START_BRIDGE", "0")
    should_start = (auto_bridge == "1")

    if should_start:
        candidates = [
            os.path.abspath(os.path.join(os.path.dirname(__file__), "../../jetbot_bridge.py")),
            os.path.abspath(os.path.join(os.path.dirname(__file__), "../jetbot_bridge.py")),
            "/home/jetbot/03-MinhHieu/jetbot_bridge.py",
            "/home/jetbot/mhieu/jetbot_bridge.py",
        ]
        for b_path in candidates:
            if os.path.exists(b_path):
                try:
                    import subprocess
                    print(f"\n[HARDWARE] Launching JetBot bridge: {b_path}\n", flush=True)
                    env = dict(os.environ)
                    tegra_ld = "/usr/lib/aarch64-linux-gnu/tegra-egl:/usr/lib/aarch64-linux-gnu/tegra"
                    current_ld = env.get("LD_LIBRARY_PATH", "")
                    env["LD_LIBRARY_PATH"] = f"{tegra_ld}:{current_ld}" if current_ld else tegra_ld
                    bridge_process = subprocess.Popen([sys.executable, b_path, "127.0.0.1"], env=env)
                    break
                except Exception as e:
                    print(f"[WARN] Could not auto-start bridge: {e}", flush=True)


@app.on_event("shutdown")
async def shutdown_event():
    global bridge_process
    coordinator.running = False
    await coordinator.stop()
    if bridge_process:
        try:
            bridge_process.terminate()
        except Exception:
            pass


app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

routes.robot = coordinator
app.include_router(routes.router)


# ─── UIT TUT #3 COMPATIBILITY REST ENDPOINTS ───
@app.get("/health")
async def health_check():
    return {"ok": True}


@app.get("/version")
async def version_info():
    return {"version": "1.0.0", "agent": "cs532_jetbot", "ok": True}


# ─── NATIVE MJPEG CAMERA STREAM ───
@app.api_route("/video_feed", methods=["GET", "HEAD"])
@app.api_route("/stream", methods=["GET", "HEAD"])
@app.api_route("/api/stream/mjpeg", methods=["GET", "HEAD"])
async def video_feed_endpoint():
    """Native MJPEG HTTP stream, viewable directly in <img src='/video_feed'>"""
    return StreamingResponse(
        mjpeg_streamer.frame_generator(),
        media_type="multipart/x-mixed-replace; boundary=frame"
    )


@app.get("/snapshot")
async def snapshot_endpoint():
    """Returns single JPEG frame for snapshot or fallback HTTP polling."""
    frame, mime = mjpeg_streamer.get_snapshot()
    if frame:
        return Response(content=frame, media_type=mime)
    return Response(content=b"", status_code=503)


# ─── WEBSOCKET ENDPOINTS ───
@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await ws_manager.connect(websocket)
    try:
        while True:
            data = await websocket.receive_json()
            await ws_manager.handle_message(websocket, data)
    except (WebSocketDisconnect, Exception):
        ws_manager.disconnect(websocket)


logger = logging.getLogger("jetbot.main")

@app.websocket("/ws/robot")
async def robot_hardware_endpoint(websocket: WebSocket):
    """Endpoint for physical JetBot (jetbot_bridge.py)."""
    await websocket.accept()
    await bridge_client.register_connection(websocket)
    coordinator.add_event("SUCCESS", "Physical JetBot connected via Hardware Bridge!")
    logger.info("Physical JetBot connected via /ws/robot")
    try:
        while True:
            msg = await websocket.receive()
            if msg.get("type") == "websocket.disconnect":
                logger.info("Physical JetBot disconnected cleanly (websocket.disconnect)")
                break

            # Proactive Receive-Queue Draining:
            # If a network latency spike or thread pause accumulated multiple messages in
            # Uvicorn's WebSocket deque, drain intermediate stale binary telemetry frames
            # immediately so the coordinator ALWAYS processes the freshest real-time frame!
            proto = getattr(websocket._receive, "__self__", None)
            msgs_deque = getattr(proto, "messages", None)
            drained_binary_count = 0
            pending_controls = []

            if msgs_deque and len(msgs_deque) > 0:
                while msgs_deque:
                    item = msgs_deque.popleft()
                    if isinstance(item, bytes) and len(item) >= 18 and item[:2] == b"JB":
                        # Newer binary frame supersedes older queued frames
                        msg = {"type": "websocket.receive", "bytes": item}
                        drained_binary_count += 1
                    else:
                        pending_controls.append(item)

                if getattr(proto, "_put_message_waiter", None) is not None:
                    proto._put_message_waiter.set_result(None)
                    proto._put_message_waiter = None

                if drained_binary_count > 0:
                    logger.info(
                        f"⚡ [Real-Time Lock] Drained {drained_binary_count} backlogged frames from WebSocket buffer."
                    )

            # Process any interleaved control messages
            for ctrl in pending_controls:
                try:
                    data = json.loads(ctrl) if isinstance(ctrl, str) else json.loads(ctrl.decode("utf-8"))
                    mtype = data.get("type")
                    if mtype in ("camera_frame", "robot_meta"):
                        await coordinator.handle_camera_frame(data)
                    elif mtype == "hardware_emergency_brake":
                        coordinator.add_event("ERROR", f"🚨 Phanh khẩn cấp phần cứng: {data.get('reason', 'Vật cản sát mui xe')}")
                    elif mtype == "edge_reflex_evasion":
                        coordinator.add_event("WARNING", f"⚡ Phản xạ né tránh tại biên: hướng {data.get('heading_deg')}°, khoảng hở {data.get('clearance_m')}m")
                except Exception as ex_ctrl:
                    logger.debug(f"Error handling control message: {ex_ctrl}")

            if "bytes" in msg and msg["bytes"]:
                raw = msg["bytes"]
                if len(raw) >= 18 and raw[:2] == b"JB":
                    try:
                        await coordinator.handle_binary_telemetry(raw)
                    except Exception as ex:
                        logger.exception(f"Exception handling binary telemetry: {ex}")
            elif "text" in msg and msg["text"]:
                try:
                    data = json.loads(msg["text"])
                    mtype = data.get("type")
                    if mtype == "robot_telemetry":
                        await coordinator.handle_real_telemetry(data)
                    elif mtype in ("camera_frame", "robot_meta"):
                        await coordinator.handle_camera_frame(data)
                    elif mtype == "hardware_emergency_brake":
                        coordinator.add_event("ERROR", f"🚨 Phanh khẩn cấp phần cứng: {data.get('reason', 'Vật cản sát mui xe')}")
                    elif mtype == "edge_reflex_evasion":
                        coordinator.add_event("WARNING", f"⚡ Phản xạ né tránh tại biên: hướng {data.get('heading_deg')}°, khoảng hở {data.get('clearance_m')}m")
                except Exception as ex:
                    logger.exception(f"Exception handling text message: {ex}")
    except (WebSocketDisconnect, WebSocketDisconnected, RuntimeError):
        logger.info("Physical JetBot connection closed")
    except Exception as e:
        logger.exception(f"Physical JetBot connection error: {e}")
    finally:
        if bridge_client.unregister_connection(websocket):
            coordinator.on_hardware_disconnected()


# ─── STATIC FRONTEND MOUNT ───
dist_candidates = [
    os.path.abspath(os.path.join(os.path.dirname(__file__), "../../frontend/dist")),
    os.path.abspath(os.path.join(os.path.dirname(__file__), "../../dist")),
    os.path.abspath(os.path.join(os.path.dirname(__file__), "../dist")),
    os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../frontend/dist")),
]
for dist_path in dist_candidates:
    if os.path.exists(dist_path) and os.path.isdir(dist_path):
        app.mount("/", StaticFiles(directory=dist_path, html=True), name="static")
        break
