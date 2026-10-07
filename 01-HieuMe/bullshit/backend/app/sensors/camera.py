"""
Camera sensor module.

This module provides camera-related functionality.
For the mock implementation, camera frames are generated in MockRobot.
For real JetBot integration, implement a ROS2CameraProvider here:

Example:
    class ROS2Camera(SensorProvider):
        def __init__(self, node):
            self.node = node
            self.latest_frame = None
            # Subscribe to camera topic
            self.sub = node.create_subscription(
                Image, '/camera/image_raw', self._callback, 10
            )
        
        async def get_camera_frame(self) -> bytes:
            # Convert ROS2 Image to JPEG bytes
            return cv2.imencode('.jpg', self.latest_frame)[1].tobytes()
"""
