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

@router.get("/api/robot/status")
async def get_status():
    return await robot.get_status()

@router.get("/api/robot/pose")
async def get_pose():
    return await robot.get_pose()

@router.get("/api/map")
async def get_map():
    return await robot.get_map()

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
