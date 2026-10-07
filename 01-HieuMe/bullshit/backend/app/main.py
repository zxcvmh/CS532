import asyncio
from contextlib import asynccontextmanager
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from app.api import routes
from app.robot.mock_robot import MockRobot
from app.websocket import WebSocketManager
from app.robot.ros2_robot import ROS2Robot

robot = ROS2Robot()
ws_manager = WebSocketManager(robot)


@asynccontextmanager
async def lifespan(app: FastAPI):
    # Startup
    asyncio.create_task(robot.update())
    asyncio.create_task(ws_manager.broadcaster_task())
    yield
    # Shutdown
    robot.running = False
    await robot.stop()


app = FastAPI(title="JetBot Dashboard API", lifespan=lifespan)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

routes.robot = robot
app.include_router(routes.router)


@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await ws_manager.connect(websocket)
    try:
        while True:
            data = await websocket.receive_json()
            await ws_manager.handle_message(websocket, data)
    except (WebSocketDisconnect, Exception) as e:
        ws_manager.disconnect(websocket)
