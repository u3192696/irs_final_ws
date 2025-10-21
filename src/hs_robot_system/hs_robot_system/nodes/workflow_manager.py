#!/usr/bin/env python3
import asyncio
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from hs_robot_system_interfaces.srv import PLCLocation
from hs_robot_system_interfaces.action import ArmTask, NavigateTask

class WorkflowManager(Node):
    def __init__(self):
        super().__init__('workflow_manager')
        
        # Service client for PLC communication
        self.plc_client = self.create_client(PLCLocation, 'get_plc_location')
        
        # Action clients for navigation and arm control
        self.nav_action = ActionClient(self, NavigateTask, 'navigate_task')
        self.arm_action = ActionClient(self, ArmTask, 'arm_task')

        # Poll PLC every 5 s when idle
        self.create_timer(5.0, self.loop)

        self.workflow_running = False
        self.phase = 'idle'

    def loop(self):
        # Poll PLC only when not already executing a workflow
        if self.workflow_running:
            return

        if not self.plc_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('🧠 PLC service not available.')
            return

        req = PLCLocation.Request()
        future = self.plc_client.call_async(req)
        future.add_done_callback(self._check_plc_response)

    def _check_plc_response(self, future):
        try:
            result = future.result()
            if not result or not result.location:
                self.get_logger().debug('🧠 Idle — No box detected.')
                return
            location = result.location.upper()
            self.get_logger().info(f'🧠 Box detected at {location}. Starting workflow...')
            self.workflow_running = True
            #asyncio.ensure_future(self.run_workflow(location))
            #self.create_task(self.run_workflow(location))

            # schedule coroutine manually for Humble
            #loop = asyncio.get_event_loop()
            #loop.create_task(self.run_workflow(location))

            threading.Thread(target=lambda: asyncio.run(self.run_workflow(location)), daemon=True).start()

        except Exception as e:
            self.get_logger().error(f'🧠 PLC check failed: {e}')
            self.workflow_running = False

    async def run_workflow(self, location):
        self.get_logger().info('🧠 run_workflow coroutine started...')
        try:
            # Step 1 → Navigate to pick
            self.send_nav_action(location, 'navigate to pick location')

            # Step 2 → Pick
            #await self.send_arm_action('pick', 'perform pick action')

            # Step 3 → Navigate to place
            #await self.send_nav_action('PLACE', 'navigate to place location')

            # Step 4 → Place
            #await self.send_arm_action('place', 'perform place action')

            #self.get_logger().info('🧠✅ Box placed successfully — returning to idle.')
            #self.phase = 'idle'

        except Exception as e:
            self.get_logger().error(f'🧠❌ Workflow error: {e}')
            self.phase = 'error'

        #finally:
            #self.workflow_running = False

#    async def send_nav_action(self, target, description):
#        self.get_logger().info('🧠 Sending Navigation Action 2.....')
#        await self._send_action(client=self.nav_action, goal_cls=NavigateTask.Goal, goal_kwargs={'target': target}, feedback_name='status', description=description)
    def send_nav_action(self, target, description):
        self.get_logger().info('🧠 Sending Navigation Action.....')
        self.get_logger().info('🧠 Waiting for NavigateTask server')
        self.nav_action.wait_for_server()
        self.get_logger().info('🧠 NavigateTask ready')

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


    async def send_arm_action(self, mode, description):
        await self._send_action(client=self.arm_action, goal_cls=ArmTask.Goal, goal_kwargs={'mode': mode}, feedback_name='step', description=description)

    async def _send_action(self, client, goal_cls, goal_kwargs, feedback_name, description):
        goal_msg = goal_cls(**goal_kwargs)
        self.get_logger().info(f'🧠 Waiting for server: {description}...')
        await client.wait_for_server()
        self.get_logger().info(f'🧠 Sending action: {description}...')

        send_goal_future = client.send_goal_async(goal_msg, feedback_callback=lambda fb: (self._on_feedback(fb, feedback_name), None)[1])
        #send_goal_future = client.send_goal_async(goal_msg, feedback_callback=self._on_feedback_wrapper(feedback_name))
        goal_handle = await send_goal_future

        if not goal_handle.accepted:
            raise RuntimeError(f'{description} goal was rejected.')

        result_future = goal_handle.get_result_async()
        result = await result_future

        if not result.result.success:
            raise RuntimeError(f'{description} failed: {result.result.message}')

        self.get_logger().info(f'🧠 {description} completed successfully.')

    #def _on_feedback_wrapper(self, feedback_name):
    #    # return a function with no return value
    #    def callback(feedback_msg):
    #        self._on_feedback(feedback_msg, feedback_name)
    #    return callback

    def _on_feedback(self, feedback_msg, key):
        value = getattr(feedback_msg.feedback, key)
        progress = getattr(feedback_msg.feedback, 'progress', 0.0)
        self.get_logger().info(f'🧠 Feedback: {value} ({progress*100:.0f}%)')

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
