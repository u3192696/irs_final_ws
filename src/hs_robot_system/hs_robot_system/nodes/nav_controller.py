#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from hs_robot_system_interfaces.srv import PLCLocation, RobotState

class NavController(Node):
    def __init__(self):
        super().__init__('nav_controller')
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.state_client = self.create_client(RobotState, 'robot_state_service')
        self.plc_client = self.create_client(PLCLocation, 'get_plc_location')

        while not self.state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('🤖 Waiting for robot_state_service...')
        while not self.plc_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('🤖 Waiting for get_plc_location...')

        # Define locations
        self.locations = {
            'A': self.make_pose(6.0, -1.5, math.pi),
            'B': self.make_pose(6.0, -0.5, math.pi),
            'C': self.make_pose(6.0, 0.5, math.pi),
            'place': self.make_pose(31.0, -4.5, (3 * math.pi/2)) 
        }
        self.last_requested_location = None

        self.create_timer(2.0, self.periodic_state_update)

    def make_pose(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = x
        pose.pose.position.y = y
        qz = math.sin(yaw * 0.5)
        qw = math.cos(yaw * 0.5)
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose

    def periodic_state_update(self):
        # Poll robot state and take actions
        req = RobotState.Request()
        req.command = "get"
        req.new_state = ""
        fut = self.state_client.call_async(req)
        fut.add_done_callback(self.on_state_response)

    def on_state_response(self, fut):
        resp = fut.result()
        if not resp or not resp.success:
            self.get_logger().warn("🤖 No response from robot_state_service")
            return
        state = resp.current_state.strip().lower()
        if state == 'idle':
            self.request_plc_location()
        elif state == 'move_to_pick':
            if self.last_requested_location:
                self.navigate_to(self.last_requested_location, next_state='ready_to_pick')
        elif state == 'move_to_place':
            self.navigate_to('place', next_state='ready_to_place')

    def request_plc_location(self):
        req = PLCLocation.Request()
        fut = self.plc_client.call_async(req)
        fut.add_done_callback(self.on_plc_location_received)

    def on_plc_location_received(self, fut):
        try:
            resp = fut.result()
            location = resp.location.strip().upper()
            self.get_logger().info(f"🤖 PLC location: {location}")
            if location in self.locations:
                self.last_requested_location = location
                self.set_robot_state("move_to_pick")
            else:
                self.get_logger().warn(f"🤖 Unknown location {location}")
        except Exception as e:
            self.get_logger().error(str(e))

    def navigate_to(self, location_key, next_state):
        pose = self.locations[location_key]
        pose.header.stamp = self.get_clock().now().to_msg()
        if not self.client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn("🤖 Nav2 server unavailable")
            return
        goal = NavigateToPose.Goal()
        goal.pose = pose
        send_goal_future = self.client.send_goal_async(goal)
        send_goal_future.add_done_callback(lambda fut: self.on_nav_goal_response(fut, next_state))

    def on_nav_goal_response(self, fut, next_state):
        goal_handle = fut.result()
        if not goal_handle.accepted:
            self.get_logger().warn('🤖 Nav2 goal was rejected')
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda fut: self.on_nav_result(fut, next_state))

    def on_nav_result(self, fut, next_state):
        result = fut.result()
        if result.status == 4:  # SUCCEEDED
            self.get_logger().info('🤖 Nav goal reached successfully')
            self.set_robot_state(next_state)
        else:
            self.get_logger().warn(f'🤖 Navigation failed with status {result.status}')

    def set_robot_state(self, new_state):
        req = RobotState.Request()
        req.command = "set"
        req.new_state = new_state
        fut = self.state_client.call_async(req)
        rclpy.spin_until_future_complete(self, fut)

def main(args=None):
    rclpy.init(args=args)
    node = NavController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🤖 Interrupted")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
