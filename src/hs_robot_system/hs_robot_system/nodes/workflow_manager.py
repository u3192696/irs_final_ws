#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_srvs.srv import Trigger
from hs_robot_system_interfaces.srv import MoveTo, PLCLocation

class WorkflowManager(Node):
    def __init__(self):
        super().__init__('workflow_manager')
        
        # Clients to interact with subsystems
        self.plc_client = self.create_client(PLCLocation, 'get_plc_location')
        self.nav_pick = self.create_client(MoveTo, 'go_to_pick')
        self.nav_place = self.create_client(MoveTo, 'go_to_place')
        self.arm_pick = self.create_client(Trigger, 'perform_pick')
        self.arm_place = self.create_client(Trigger, 'perform_place')

        self.timer = self.create_timer(3.0, self.loop)
        self.phase = 'idle'
        self.target = None

    def loop(self):
        if self.phase != 'idle':
            return  # Skip if already executing a function

        self.get_logger().info('🧠 Starting workflow cycle...')
        self.phase = 'running'
        self.run_sequence()

    def run_sequence(self):
        # 1. Get location from PLC
        loc_req = PLCLocation.Request()
        loc_future = self.plc_client.call_async(loc_req)
        rclpy.spin_until_future_complete(self, loc_future)
        self.target = loc_future.result().location.upper()

        # 2. Navigate to pick
        self.get_logger().info(f'🧠 Going to pick location {self.target}')
        self.call_service(self.nav_pick, MoveTo.Request(target=self.target))

        # 3. Pick
        self.get_logger().info('🧠 Performing pick sequence...')
        self.call_service(self.arm_pick, Trigger.Request())

        # 4. Navigate to place
        self.get_logger().info('🧠 Moving to place area...')
        self.call_service(self.nav_place, MoveTo.Request(target='place'))

        # 5. Place
        self.get_logger().info('🧠 Performing place sequence...')
        self.call_service(self.arm_place, Trigger.Request())

        self.get_logger().info('🧠✅ Workflow complete, returning to idle phase')
        self.phase = 'idle'

    def call_service(self, client, request):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        result = future.result()
        return result.success if hasattr(result, 'success') else False

def main():
    rclpy.init()
    node = WorkflowManager()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
