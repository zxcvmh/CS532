"""
MockRobot Compatibility Facade.
Inherits from NavigationCoordinator, providing seamless backward-compatibility.
"""
from app.models.schemas import NavigationStatusEnum, ModeEnum, RobotPose, RobotStatus
from app.navigation.coordinator import NavigationCoordinator


class MockRobot(NavigationCoordinator):
    """Facade class aliasing NavigationCoordinator for backward compatibility."""

    def __init__(self, bridge=None, streamer=None):
        super().__init__(bridge=bridge, streamer=streamer)


__all__ = ["MockRobot", "NavigationStatusEnum", "ModeEnum", "RobotPose", "RobotStatus"]
