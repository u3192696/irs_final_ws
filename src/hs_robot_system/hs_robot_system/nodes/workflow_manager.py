#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
#from rclpy.executors import SingleThreadedExecutor

from std_srvs.srv import Trigger
from hs_robot_system_interfaces.srv import MoveTo, PLCLocation

class WorkflowManager(Node):
    def __init__(self):
        super().__init__('workflow_manager')
        
        # Clients to interact with subsystems
        self.plc_client = self.create_client(PLCLocation, 'get_plc_location')
        self.nav_to_goal = self.create_client(MoveTo, 'go_to_goal')
        #self.nav_place = self.create_client(MoveTo, 'go_to_place')
        self.arm_pick = self.create_client(Trigger, 'perform_pick')
        self.arm_place = self.create_client(Trigger, 'perform_place')

        self.phase = 'idle'
        self.target = None

        self.timer = self.create_timer(3.0, self.loop)

    def loop(self):
        if self.phase != 'idle':
            return  # Skip if already executing a move->pick->move->place workflow

        self.get_logger().info('🧠 Starting workflow cycle...')
        self.phase = 'running'
        self.start_workflow()

    # Step 1: handle PLC location
    def start_workflow(self):
        self.get_logger().info('🧠 Requesting PLC location...')
        loc_req = PLCLocation.Request()
        future = self.plc_client.call_async(loc_req)
        future.add_done_callback(self.plc_response_callback)

    def plc_response_callback(self, future):
        try:
            response = future.result()
            self.target = response.location.upper()
            self.get_logger().info(f'🧠 PLC responded with location {self.target}')
            self.start_navigation_to_pick()
        except Exception as e:
            self.get_logger().error(f'🧠❌ PLC response processing failed: {e}')
            self.phase = 'error'

    # Step 2: navigation to pick
    def start_navigation_to_pick(self):
        self.get_logger().info(f'🧠 Navigating to pick location {self.target}...')
        move_req = MoveTo.Request()
        move_req.target = self.target
        future = self.nav_to_goal.call_async(move_req)
        future.add_done_callback(self.navigation_pick_callback)

    def navigation_pick_callback(self, future):
        try:
            result = future.result()
            if hasattr(result, 'success') and not result.success:
                self.get_logger().warn('🧠⚠️ Navigation to pick location failed.')
                self.phase = 'error'
                return
            self.get_logger().info('🧠 Starting pick sequence...')
            pick_future = self.arm_pick.call_async(Trigger.Request())
            pick_future.add_done_callback(self.pick_action_callback)
        except Exception as e:
            self.get_logger().error(f'🧠❌ Pick sequence failed: {e}')
            self.phase = 'error'

    # Step 3: pick
    def pick_action_callback(self, future):
        self.get_logger().info('🧠 Pick complete → moving to place area...')
        move_req = MoveTo.Request()
        move_req.target = 'PLACE'
        future = self.nav_to_goal.call_async(move_req)
        future.add_done_callback(self.navigation_place_callback)

    # Step 4: navigate to place
    def navigation_place_callback(self, future):
        self.get_logger().info('🧠 Performing place sequence...')
        place_future = self.arm_place.call_async(Trigger.Request())
        place_future.add_done_callback(self.place_action_callback)

    # Step 5: place object
    def place_action_callback(self, future):
        self.get_logger().info('🧠✅ Workflow completed successfully.')
        self.phase = 'idle'

def main():
    rclpy.init()
    node = WorkflowManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🧠 WorkflowManager Interrupted")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
