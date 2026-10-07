"""
Navigation manager module.

For real JetBot integration, implement a ROS2NavigationManager here:

Example:
    class ROS2Navigation(NavigationManager):
        def __init__(self, node):
            self.node = node
            self.nav_client = ActionClient(
                node, NavigateToPose, 'navigate_to_pose'
            )
            self.status = NavigationStatus(status=NavigationStatusEnum.IDLE)
        
        async def set_goal(self, x: float, y: float):
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose.pose.position.x = x
            goal_msg.pose.pose.position.y = y
            goal_msg.pose.header.frame_id = 'map'
            self.nav_client.send_goal_async(goal_msg)
            self.status.status = NavigationStatusEnum.NAVIGATING
        
        async def cancel_goal(self):
            self.nav_client.cancel_goal_async()
            self.status.status = NavigationStatusEnum.CANCELLED
"""
