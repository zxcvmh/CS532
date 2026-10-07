from fastapi import APIRouter
from pydantic import BaseModel
from app.robot.mock_robot import MockRobot

router = APIRouter()
robot: MockRobot = None

class GoalRequest(BaseModel):
    x: float
    y: float

class ModeRequest(BaseModel):
    mode: str

class DriveRequest(BaseModel):
    left: float = 0.0
    right: float = 0.0
    linear: float = 0.0
    angular: float = 0.0

# ─── UIT TUT #3 COMPATIBILITY REST ENDPOINTS ───
@router.post("/drive")
async def drive_endpoint(req: DriveRequest):
    """UIT Tut 3: POST /drive {"left": 0.5, "right": 0.5}"""
    track_width = 0.115
    if req.left != 0.0 or req.right != 0.0:
        linear = (req.left + req.right) / 2.0 * 0.5
        angular = (req.right - req.left) / track_width * 1.0 * 0.5
    else:
        linear = req.linear
        angular = req.angular
    await robot.set_velocity(linear, angular)
    return {"ok": True, "linear": linear, "angular": angular}

@router.post("/stop")
async def stop_endpoint():
    """UIT Tut 3: POST /stop"""
    await robot.emergency_stop()
    return {"ok": True, "status": "stopped"}

@router.get("/status")
async def uit_status():
    """UIT Tut 3: GET /status"""
    pose = await robot.get_pose()
    bat = await robot.get_battery()
    return {
        "ok": True,
        "moving": abs(robot.linear_vel) > 0.01 or abs(robot.angular_vel) > 0.01,
        "left": round(robot.linear_vel - robot.angular_vel * 0.115 / 2.0, 2),
        "right": round(robot.linear_vel + robot.angular_vel * 0.115 / 2.0, 2),
        "pose": {"x": pose.x, "y": pose.y, "theta": pose.theta},
        "battery": bat,
        "mode": robot.mode.value
    }

# ─── DASHBOARD API ENDPOINTS ───
@router.get("/api/robot/status")
async def get_status():
    return await robot.get_status()

@router.get("/api/robot/pose")
async def get_pose():
    return await robot.get_pose()

@router.get("/api/map")
async def get_map():
    return await robot.get_map()

@router.post("/api/map/reset")
async def reset_map():
    await robot.reset_map()
    return {"status": "success"}

@router.post("/api/navigation/goal")
async def set_goal(req: GoalRequest):
    await robot.set_goal(req.x, req.y)
    return {"status": "success"}

@router.post("/api/navigation/cancel")
async def cancel_goal():
    await robot.cancel_goal()
    return {"status": "success"}

@router.post("/api/robot/stop")
async def stop_robot():
    await robot.emergency_stop()
    return {"status": "success"}

@router.post("/api/robot/mode")
async def set_mode(req: ModeRequest):
    await robot.set_mode(req.mode)
    return {"status": "success"}
