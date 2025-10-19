#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

#from hs_robot_system_interfaces.srv import PLCLocation, RobotState

# define a simple srv with "string target" and "bool success"
from hs_robot_system_interfaces.srv import MoveTo

class NavController(Node):
    def __init__(self):
        super().__init__('nav_controller')
        
        # Action client for Nav2
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.get_logger().info('🤖 Waiting for NavigateToPose action server...')
        self.nav_client.wait_for_server()
        self.get_logger().info('🤖 NavController connected to Nav2 server.')

        # Available locations
        self.locations = {
            'A': self.make_pose(6.0, -1.5, math.pi),
            'B': self.make_pose(6.0, -0.5, math.pi),
            'C': self.make_pose(6.0, 0.5, math.pi),
            'place': self.make_pose(31.0, -4.5, (3 * math.pi / 2))
        }

        # Expose services for workflow_manager
        self.pick_service = self.create_service(MoveTo, 'go_to_pick', self.handle_go_to_pick)
        self.place_service = self.create_service(MoveTo, 'go_to_place', self.handle_go_to_place)

        self.get_logger().info('🤖 NavController ready for workflow service calls.')
    
    # Helper function to generate a pose
    def make_pose(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    # Workflow-triggered service handlers
    def handle_go_to_pick(self, request, response):
        target = request.target.strip().upper()
        if target not in self.locations:
            self.get_logger().error(f'🤖 Invalid pick target: {target}')
            response.success = False
            return response

        self.get_logger().info(f'🤖 Navigating to pick location: {target}')
        response.success = self.navigate_to(target)
        return response

    def handle_go_to_place(self, request, response):
        self.get_logger().info('🤖 Navigating to place location...')
        response.success = self.navigate_to('place')
        return response

    # Synchronous navigation to target key
    def navigate_to(self, location_key):
        pose = self.locations[location_key]
        pose.header.stamp = self.get_clock().now().to_msg()

        goal = NavigateToPose.Goal()
        goal.pose = pose

        send_future = self.nav_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('🤖 Navigation goal rejected.')
            return False

        self.get_logger().info('🤖 Navigation goal accepted; executing...')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()
        if result.status == 4:
            self.get_logger().info('🤖 Navigation goal completed successfully.')
            return True
        else:
            self.get_logger().warn(f'🤖 Navigation failed (result code {result.status}).')
            return False

def main(args=None):
    rclpy.init(args=args)
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
