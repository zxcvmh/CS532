from pydantic import BaseModel
from enum import Enum
from typing import List, Dict, Optional, Any

class NavigationStatusEnum(str, Enum):
    IDLE = "IDLE"
    PLANNING = "PLANNING"
    NAVIGATING = "NAVIGATING"
    REACHED = "REACHED"
    BLOCKED = "BLOCKED"
    CANCELLED = "CANCELLED"
    ERROR = "ERROR"

class ModeEnum(str, Enum):
    MANUAL = "MANUAL"
    AUTONOMOUS = "AUTONOMOUS"

class RobotPose(BaseModel):
    x: float
    y: float
    theta: float

class LidarPoint(BaseModel):
    x: float
    y: float

class LidarScan(BaseModel):
    points: List[LidarPoint]

class Detection(BaseModel):
    class_name: str
    confidence: float
    bbox: List[int]
    distance: float

class DetectionList(BaseModel):
    objects: List[Detection]

class MapData(BaseModel):
    resolution: float
    width: int
    height: int
    origin_x: float
    origin_y: float
    data: List[int]

class NavigationGoal(BaseModel):
    x: float
    y: float

class NavigationStatus(BaseModel):
    status: NavigationStatusEnum
    goal_x: Optional[float] = None
    goal_y: Optional[float] = None
    distance_remaining: Optional[float] = None
    progress: Optional[float] = None
    velocity: Optional[float] = None

class SensorHealth(BaseModel):
    name: str
    status: str
    last_update: float

class RobotStatus(BaseModel):
    connected: bool
    battery: float
    mode: ModeEnum
    pose: RobotPose
    linear_vel: float
    angular_vel: float
    sensors: List[SensorHealth]

class CommandMessage(BaseModel):
    type: str
    data: Dict[str, Any]

class EventLog(BaseModel):
    timestamp: str
    level: str
    message: str

class PlannedPath(BaseModel):
    points: List[LidarPoint]
