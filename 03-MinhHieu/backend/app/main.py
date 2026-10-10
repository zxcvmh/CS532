import os
import sys
import asyncio
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import StreamingResponse, Response
from fastapi.staticfiles import StaticFiles
from app.api import routes
from app.robot.mock_robot import MockRobot
from app.websocket import WebSocketManager

robot = MockRobot()
ws_manager = WebSocketManager(robot)

app = FastAPI(title="JetBot Dashboard API")

bridge_process = None

# Compatible with Python 3.6.9 (Ubuntu 18.04 JetPack) and modern Python
@app.on_event("startup")
async def startup_event():
    global bridge_process
    # ponytail: asyncio.ensure_future works on Python 3.6+ without requiring create_task (3.7+)
    asyncio.ensure_future(robot.update())
    asyncio.ensure_future(ws_manager.broadcaster_task())

    # Auto-start hardware bridge if running on actual JetBot hardware and not disabled
    is_tegra = os.path.exists("/etc/nv_tegra_release") or (hasattr(os, "uname") and os.uname().machine == "aarch64")
    auto_bridge = os.environ.get("AUTO_START_BRIDGE")
    should_start_bridge = (auto_bridge == "1") or (auto_bridge is None and is_tegra)

    if should_start_bridge:
        bridge_candidates = [
            os.path.abspath(os.path.join(os.path.dirname(__file__), "../../jetbot_bridge.py")),
            os.path.abspath(os.path.join(os.path.dirname(__file__), "../jetbot_bridge.py")),
            "/home/jetbot/03-MinhHieu/jetbot_bridge.py",
            "/home/jetbot/mhieu/jetbot_bridge.py",
            "/home/jetbot/cs532/jetbot_bridge.py",
        ]
        for b_path in bridge_candidates:
            if os.path.exists(b_path):
                try:
                    import subprocess
                    print(f"\n[HARDWARE] Found JetBot hardware bridge at: {b_path}")
                    print(f"[HARDWARE] Auto-launching CSI Camera, LiDAR, and Motors in background...\n")
                    env = dict(os.environ)
                    tegra_ld = "/usr/lib/aarch64-linux-gnu/tegra-egl:/usr/lib/aarch64-linux-gnu/tegra"
                    current_ld = env.get("LD_LIBRARY_PATH", "")
                    env["LD_LIBRARY_PATH"] = f"{tegra_ld}:{current_ld}" if current_ld else tegra_ld
                    bridge_process = subprocess.Popen(
                        [sys.executable, b_path, "127.0.0.1"],
                        env=env
                    )
                    break
                except Exception as e:
                    print(f"[WARN] Could not auto-start bridge: {e}")

@app.on_event("shutdown")
async def shutdown_event():
    global bridge_process
    robot.running = False
    await robot.stop()
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

routes.robot = robot
app.include_router(routes.router)


# ─────────────────────────────────────────────────────────────
# UIT TUT #3 COMPATIBILITY REST ENDPOINTS
# ─────────────────────────────────────────────────────────────
@app.get("/health")
async def health_check():
    """UIT Tut 3: Health check endpoint"""
    return {"ok": True}

@app.get("/version")
async def version_info():
    """UIT Tut 3: Agent version information"""
    return {"version": "1.0.0", "agent": "cs532_jetbot", "ok": True}


# ─────────────────────────────────────────────────────────────
# NATIVE MJPEG CAMERA STREAM (UIT Tut 3 & Ponytail standard)
# ─────────────────────────────────────────────────────────────
async def mjpeg_frame_generator():
    while True:
        try:
            frame = await robot.get_camera_frame()
            if frame:
                mime = "image/bmp" if frame.startswith(b"BM") else "image/jpeg"
                header = f"--frame\r\nContent-Type: {mime}\r\n\r\n".encode("ascii")
                yield (header + frame + b"\r\n")
            await asyncio.sleep(0.066)  # ~15 FPS
        except asyncio.CancelledError:
            break
        except Exception:
            await asyncio.sleep(0.1)

@app.api_route("/video_feed", methods=["GET", "HEAD"])
@app.api_route("/stream", methods=["GET", "HEAD"])
async def video_feed_endpoint():
    """Native MJPEG HTTP stream, viewable directly in <img src='/video_feed'>"""
    return StreamingResponse(
        mjpeg_frame_generator(),
        media_type="multipart/x-mixed-replace; boundary=frame"
    )

@app.get("/snapshot")
async def snapshot_endpoint():
    """Returns a single JPEG frame for snapshot or fallback HTTP polling"""
    frame = await robot.get_camera_frame()
    if frame:
        mime = "image/bmp" if frame.startswith(b"BM") else "image/jpeg"
        return Response(content=frame, media_type=mime)
    return Response(content=b"", status_code=503)


# ─────────────────────────────────────────────────────────────
# WEBSOCKET ENDPOINTS
# ─────────────────────────────────────────────────────────────
@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await ws_manager.connect(websocket)
    try:
        while True:
            data = await websocket.receive_json()
            await ws_manager.handle_message(websocket, data)
    except (WebSocketDisconnect, Exception):
        ws_manager.disconnect(websocket)


@app.websocket("/ws/robot")
async def robot_hardware_endpoint(websocket: WebSocket):
    """Endpoint for physical JetBot (jetbot_bridge.py)"""
    await websocket.accept()
    robot.real_ws = websocket
    robot.is_real_connected = True
    robot._add_event("SUCCESS", "Physical JetBot connected via Hardware Bridge!")
    try:
        while True:
            data = await websocket.receive_json()
            if data.get("type") == "robot_telemetry":
                await robot.handle_real_telemetry(data)
    except (WebSocketDisconnect, Exception):
        pass
    finally:
        robot.is_real_connected = False
        robot.real_ws = None
        robot._camera_connected = False
        robot._lidar_connected = False
        robot._yolo_connected = False
        robot._battery_connected = False
        robot._add_event("WARNING", "Physical JetBot disconnected. Falling back to simulation.")


# ─────────────────────────────────────────────────────────────
# STATIC FRONTEND MOUNT (Ponytail: Serve React build directly)
# ─────────────────────────────────────────────────────────────
# Mount static build so no Node.js/npm is needed on Jetson Nano
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


