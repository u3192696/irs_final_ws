#!/usr/bin/env python3
import asyncio
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from hs_robot_system_interfaces.srv import PLCLocation
from hs_robot_system_interfaces.action import ArmTask, NavigateTask

# ----- CONSTANTS -----
WORKFLOW_STRINGS = {
    0: "Badness in Progress",
    1: "Idle",
    2: "Moving to Pick location",
    3: "Picking",
    4: "Moving to Place location",
    5: "Placing"
}

class WorkflowManager(Node):    
    def __init__(self):
        super().__init__('workflow_manager')
        
        self.phase = 0
        self.box_location = 'NONE'
        self.busy = True

        # Setup and connect to service and action servers
        conn_errors = 0
        # Service client for PLC
        self.plc_client = self.create_client(PLCLocation, 'get_plc_location')
        while not self.plc_client.wait_for_service(timeout_sec=3.0) and conn_errors < 30:
            self.get_logger().warn('🧠 Waiting for PLCLocation service....')
            conn_errors += 1
        self.get_logger().info('🧠 PLCLocation ready')

        # Action clients for navigation and arm control
        self.nav_action = ActionClient(self, NavigateTask, 'navigate_task')
        while not self.nav_action.wait_for_server(timeout_sec=3.0) and conn_errors < 30:
            self.get_logger().info('🧠 Waiting for NavigateTask server....')
            conn_errors += 1
        self.get_logger().info('🧠 NavigateTask ready')


        self.arm_action = ActionClient(self, ArmTask, 'arm_task')
        while not self.arm_action.wait_for_server(timeout_sec=3.0) and conn_errors < 30:
            self.get_logger().info('🧠 Waiting for ArmTask server....')
            conn_errors += 1
        self.get_logger().info('🧠 ArmTask ready')

        if conn_errors < 30:
            time.sleep(10) # sleep for 10 seconds to let everything spin up correctly
            self.phase = 1
            self.busy = False
            self.get_logger().info('🧠 WorkflowManager ready')
        
    def check_plc(self):
        self.busy = True
        req = PLCLocation.Request()
        future = self.plc_client.call_async(req)
        future.add_done_callback(self.check_plc_response)
    
    def check_plc_response(self, future):
        try:
            result = future.result()
            if not result or not result.location:
                self.get_logger().debug('🧠 Idle — No box detected.')
                self.box_location = 'NONE'
                #wait 5 seconds before asking again
                time.sleep(5)
                return
            
            location = result.location.upper()
            self.get_logger().info(f'🧠 Box detected at {location}.')
            self.phase = 2
            self.box_location = location
            self.busy = False

        except Exception as e:
            self.get_logger().error(f'🧠 PLC check failed: {e}')
            self.phase = 0

    def move_to_pick(self):
        if not self.busy:
            self.busy = True
            self.send_nav_action(self.box_location)
    
    def send_nav_action(self, target):
        self.get_logger().info('🧠 Sending Navigation Action.....')
        
        # Create goal
        goal = NavigateTask.Goal()
        goal.target = target

        # Send goal
        self.get_logger().info('🧠 Sending NavigateTask goal')
        nav_goal_future = self.nav_action.send_goal_async(goal)
        nav_goal_future.add_done_callback(self.nav_goal_response_cb)

    def nav_goal_response_cb(self, goal_future):
        nav_goal_handle = goal_future.result()
        if nav_goal_handle.accepted:
            self.get_logger().info('🧠 NavigateTask goal accepted')
            nav_goal_result_future = nav_goal_handle.get_result_async()
            nav_goal_result_future.add_done_callback(self.nav_goal_result_cb)

    def nav_goal_result_cb(self, result_future):
        result = result_future.result().result
        #result_msg = future.result().message
        self.get_logger().info(f'🧠 NavigateTask goal result success: {result.success}')
        self.get_logger().info(f'🧠 NavigateTask goal result message: {result.message}')
        
        if self.phase == 2:
            self.phase = 3
        elif self.phase == 4:
            self.phase = 0
        else:
            self.phase = 0
        
        self.busy = False

    def send_arm_action(self, target):
        self.get_logger().info('🧠 Sending Arm Action.....')

        # Create goal
        goal = ArmTask.Goal()
        goal.mode = target

        # Send goal
        self.get_logger().info('🧠 Sending ArmTask goal')
        arm_goal_future = self.arm_action.send_goal_async(goal)
        arm_goal_future.add_done_callback(self.arm_goal_response_cb)

    def arm_goal_response_cb(self, goal_future):
        arm_goal_handle = goal_future.result()
        if arm_goal_handle.accepted:
            self.get_logger().info('🧠 ArmTask goal accepted')
            arm_goal_result_future = arm_goal_handle.get_result_async()
            arm_goal_result_future.add_done_callback(self.arm_goal_result_cb)

    def arm_goal_result_cb(self, result_future):
        result = result_future.result().result
        if result.success:
            self.get_logger().info(f'🧠 ArmTask goal result success: {result.success}')
            self.get_logger().info(f'🧠 ArmTask goal result message: {result.message}')
            self.phase = 4
            self.busy = False
        else:
            self.get_logger().info(f'🧠 ArmTask goal result success: {result.success}')
            self.get_logger().info(f'🧠 ArmTask goal result message: {result.message}')
            self.phase = 0
            self.busy = True

    def pick_box(self):
        if not self.busy:
            self.busy = True
            self.send_arm_action('pick')

    def move_to_place(self):
        if not self.busy:
            self.busy = True
            self.send_nav_action('PLACE')


    def place_box(self):
        pass

    def workflow_action_default(self):
        self.phase = 0

    def loop(self):
        if self.busy:
            return

        WORKFLOW_ACTIONS = {
            1: self.check_plc,
            2: self.move_to_pick,
            3: self.pick_box,
            4: self.move_to_place,
            5: self.place_box
        }
        action_handle = WORKFLOW_ACTIONS.get(self.phase, self.workflow_action_default)
        action_handle()


def main():
    rclpy.init()
    node = WorkflowManager()
    try:
        while node.phase:
            node.loop()
            rclpy.spin_once(node, timeout_sec=0.5)
        node.get_logger().info("🧠 WorkflowManager Badness")
    except KeyboardInterrupt:
        node.get_logger().info("🧠 WorkflowManager Interrupted")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
