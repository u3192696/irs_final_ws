#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient, ActionServer, GoalResponse, CancelResponse
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

from hs_robot_system_interfaces.action import NavigateTask


class NavController(Node):
    def __init__(self):
        super().__init__('nav_controller')
        
        # Action client for Nav2
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.get_logger().info('🤖 Waiting for NavigateToPose action server...')
        self.nav_client.wait_for_server()
        self.get_logger().info('🤖 NavController connected to Nav2 server.')

        # NavigateTask action server for workflow_manager
        self.nav_server = ActionServer(
            self,
            NavigateTask,
            'navigate_task',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

        # Available locations
        self.locations = {
            'A': self.make_pose(6.0, -1.5, math.pi),
            'B': self.make_pose(6.0, -0.5, math.pi),
            'C': self.make_pose(6.0, 0.5, math.pi),
            'PLACE': self.make_pose(31.0, -4.5, (3 * math.pi / 2))
        }

        self.get_logger().info('🤖 NavController ready for workflow tasks.')

    # Helper function to generate a pose
    def make_pose(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose
    
    # Action goal received
    def goal_callback(self, goal_request):
        self.get_logger().info(f'🤖 Received goal request: {goal_request.target}')
        target = goal_request.target.upper()
        if target not in self.locations:
            self.get_logger().warn(f'🤖 Rejected goal: invalid target "{target}".')
            return GoalResponse.REJECT
        self.get_logger().info(f'🤖 Accepted navigation goal: {target}')
        return GoalResponse.ACCEPT

    # Cancel request
    def cancel_callback(self, goal_handle):
        self.get_logger().info('🤖 Navigation cancelled by client.')
        return CancelResponse.ACCEPT

    # Execute navigation sequence
    async def execute_callback(self, goal_handle):
        target = goal_handle.request.target
        feedback = NavigateTask.Feedback()
        result = NavigateTask.Result()
        self.get_logger().info(f'🤖 Executing navigation task to {target}')

        # Wait for Nav2 server
        #self.nav_client.wait_for_server()

        # Set up NavigateToPose goal
        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = self.locations[target]

        # Send goal to Nav2
        goal_future = self.nav_client.send_goal_async(nav_goal)
        nav_goal_handle = await goal_future
        if not nav_goal_handle.accepted:
            self.get_logger().error('🤖 Nav2 NavigateToPose goal rejected.')
            goal_handle.abort()
            result.success = False
            result.message = 'Nav2 goal rejected'
            return result

        # Provide initial feedback
        feedback.progress = 0.2
        feedback.status = 'Goal accepted by Nav2'
        goal_handle.publish_feedback(feedback)

        # Wait for Nav2 result asynchronously
        result_future = nav_goal_handle.get_result_async()

        # Check for cancellation during motion
#        while not result_future.done():
#            if goal_handle.is_cancel_requested:
#                self.get_logger().warn('🤖 Navigation cancelled mid-execution.')
#                nav_goal_handle.cancel_goal_async()
#                goal_handle.canceled()
#                result.success = False
#                result.message = 'Cancelled'
#                return result
#
#            feedback.progress += 0.1 if feedback.progress < 1.0 else 0.99
#            feedback.status = 'Navigating...'
#            goal_handle.publish_feedback(feedback)
#            await rclpy.sleep(0.5)

        # Nav2 finished
        nav_result = await result_future
        nav_status = nav_result.status
        if nav_status != 4:  # 4 = SUCCEEDED
            self.get_logger().error(f'🤖 Nav2 failed with status: {nav_status}')
            goal_handle.abort()
            result.success = False
            result.message = f'Nav2 error code {nav_status}'
            return result

        self.get_logger().info(f'🤖 Successfully reached {target}.')
        feedback.progress = 1.0
        feedback.status = 'Arrived'
        goal_handle.publish_feedback(feedback)

        goal_handle.succeed()
        result.success = True
        result.message = 'Arrived at destination'
        return result

def main():
    rclpy.init()
    node = NavController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🤖 NavController Interrupted")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
