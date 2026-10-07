from abc import ABC, abstractmethod
from typing import List
from app.models.schemas import (
    RobotPose, RobotStatus, LidarScan, LidarPoint, 
    SensorHealth, MapData, NavigationStatus, PlannedPath, DetectionList
)

class RobotController(ABC):
    @abstractmethod
    async def get_pose(self) -> RobotPose: pass
    @abstractmethod
    async def get_battery(self) -> float: pass
    @abstractmethod
    async def set_velocity(self, linear: float, angular: float): pass
    @abstractmethod
    async def stop(self): pass
    @abstractmethod
    async def emergency_stop(self): pass
    @abstractmethod
    async def set_mode(self, mode: str): pass
    @abstractmethod
    async def get_mode(self) -> str: pass
    @abstractmethod
    async def get_status(self) -> RobotStatus: pass

class SensorProvider(ABC):
    @abstractmethod
    async def get_lidar_scan(self) -> LidarScan: pass
    @abstractmethod
    async def get_camera_frame(self) -> bytes: pass
    @abstractmethod
    async def get_sensor_health(self) -> List[SensorHealth]: pass

class MapProvider(ABC):
    @abstractmethod
    async def get_map(self) -> MapData: pass
    @abstractmethod
    async def get_trajectory(self) -> List[LidarPoint]: pass

class NavigationManager(ABC):
    @abstractmethod
    async def set_goal(self, x: float, y: float): pass
    @abstractmethod
    async def cancel_goal(self): pass
    @abstractmethod
    async def get_status(self) -> NavigationStatus: pass
    @abstractmethod
    async def get_planned_path(self) -> PlannedPath: pass

class DetectionProvider(ABC):
    @abstractmethod
    async def get_detections(self) -> DetectionList: pass
