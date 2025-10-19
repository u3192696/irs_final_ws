#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

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
            'PLACE': self.make_pose(31.0, -4.5, (3 * math.pi / 2))
        }

        # Expose services for workflow_manager
        #self.pick_service = self.create_service(MoveTo, 'go_to_pick', self.handle_go_to_pick)
        #self.place_service = self.create_service(MoveTo, 'go_to_place', self.handle_go_to_place)

        self.go_to_goal_srv = self.create_service(MoveTo, 'go_to_goal', self.handle_go_to_goal)

        # To keep track of outstanding service responses ???
        self.pending_responses = {}

        self.get_logger().info('🤖 NavController ready for workflow service calls.')
    
    def handle_go_to_goal(self, request, response):
        goal_key = request.target.strip().upper()
        if goal_key not in self.locations:
            self.get_logger().error(f'🤖 Unknown target: {goal_key}')
            response.success = False
            return response
        
        pose = self.locations[goal_key]
        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = pose
        
        send_future = self.nav_client.send_goal_async(nav_goal)
        send_future.add_done_callback(lambda f: self.goal_sent_cb(f, goal_key))
        response.success = True
        return response
        
    def goal_sent_cb(self, send_future, goal_key):
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('🤖 Navigation goal was rejected.')
            return
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(lambda f: self.goal_result_cb(f, goal_key))

    def goal_result_cb(self, result_future, goal_key):
        result = result_future.result()

        is_success = (getattr(result, 'status', None) == 4)
        response.success = is_success
        if is_success:
            self.get_logger().info(f'🤖 Arrived at {goal_key}! Navigation successful.')
        else:
            self.get_logger().warn(f'🤖 Navigation to {goal_key} failed, status code={getattr(result, "status", None)}')
        return response

    # Helper function to generate a pose
    def make_pose(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

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
