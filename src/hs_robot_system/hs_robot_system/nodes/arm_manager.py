#!/usr/bin/env python3

import time
import math
from typing import List

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint

# robot_state_service
from hs_robot_system_interfaces.srv import RobotState

class ArmManager(Node):

    JOINT_NAMES: List[str] = [
        'joint_1',
        'joint_2',
        'joint_3',
        'joint_4',
        'joint_5',
        'joint_6'
    ]
    PLANNING_GROUP: str = 'tmr_arm'
    ACTION_NAME: str = '/move_action'

    def __init__(self):
        super().__init__('arm_manager')
        self.client = ActionClient(self, MoveGroup, self.ACTION_NAME)
        self.get_logger().info('🦾 Waiting for MoveGroup action server...')
        self.client.wait_for_server()
        self.get_logger().info('🦾 MoveGroup action server ready.')

        # Add state client for robot_state_service
        self.state_client = self.create_client(RobotState, 'robot_state_service')
        while not self.state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('🦾 Waiting for robot_state_service...')
        self.get_logger().info('🦾 Connected to robot_state_service.')

        # Timers/pollers
        self.create_timer(2.0, self.periodic_state_check)

        # Precompute joint targets in radians
        def degrees_to_radians(self, degrees_list: List[float]) -> List[float]:
            return [math.radians(deg) for deg in degrees_list]
        
        self.pick_rad = self.degrees_to_radians([0.0, 45.0, 45.0, 0.0, 90.0, 0.0])
        self.carry_rad = self.degrees_to_radians([180.0, 0.0, 90.0, 0.0, 90.0, 0.0])
        self.place_rad = self.degrees_to_radians([0.0, 40.0, 45.0, 5.0, 90.0, 0.0])

    # --- Helper functions ---
    # Update (set) robot state
    def set_robot_state(self, new_state):
        request = RobotState.Request()
        request.command = 'set'
        request.new_state = new_state
        future = self.state_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        if not response or not response.success:
            self.get_logger().warn(f"🦾 State update failed: {response.message if response else 'No response'}")
        else:
            self.get_logger().info(f"🦾 State updated → {new_state}")
    
    # Plan and move arm to goal 
    def _goal_from_joints(self, joints: List[float]) -> MoveGroup.Goal:
        goal = MoveGroup.Goal()
        req = MotionPlanRequest()
        req.group_name = self.PLANNING_GROUP
        req.num_planning_attempts = 10
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = 0.5
        req.max_acceleration_scaling_factor = 0.5
        
        cs = Constraints()
        for name, val in zip(self.JOINT_NAMES, joints):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = val
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            cs.joint_constraints.append(jc)
        
        req.goal_constraints.append(cs)
        goal.request = req
        goal.planning_options.plan_only = False # plan + execute
        return goal

    def _send_and_wait(self, joints: List[float]) -> bool:
        goal = self._goal_from_joints(joints)
        send_fut = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_fut)
        handle = send_fut.result()
        if not handle or not handle.accepted:
            self.get_logger().error('🦾 Goal rejected.')
            return False
        res_fut = handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_fut)
        res = res_fut.result()
        return bool(res and res.status == 4) # 4 = STATUS_SUCCEEDED

    # --- Arm action sequences ---
    def pick_sequence(self):
        self.get_logger().info('🦾 Picking up box...')
        if not self._send_and_wait([0.0, 0.0, 1.57, 0.0, 1.57, 0.0]):
            return
        time.sleep(1.0)
        if not self._send_and_wait(self.pick_rad):
            return
        time.sleep(1.0)
        if not self._send_and_wait(self.carry_rad):
            return
        self.get_logger().info('🦾 Pick complete.')
        # Setting state moved to periodic_state_check()

    def place_sequence(self):
        self.get_logger().info('🦾 Placing box...')
        if not self._send_and_wait(self.place_rad):
            return
        time.sleep(1.0)
        if not self._send_and_wait(self.carry_rad):
            return
        self.get_logger().info('🦾 Place complete.')
        # Setting state moved to periodic_state_check()

    # --- Periodic State Check ---
    def periodic_state_check(self):
        request = RobotState.Request()
        request.command = 'get'
        request.new_state = ''
        future = self.state_client.call_async(request)

        def on_state_response(fut):
            response = fut.result()
            if not response or not response.success:
                self.get_logger().warn('🦾 No state from robot_state_service')
                return
            state = response.current_state.strip().lower()
            if state == 'ready_to_pick':
                self.pick_sequence()
                self.set_robot_state('move_to_place')
            elif state == 'ready_to_place':
                self.place_sequence()
                self.set_robot_state('idle')

        future.add_done_callback(on_state_response)

def main(args=None):
    rclpy.init(args=args)
    node = ArmManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('🦾 Interrupted by user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

