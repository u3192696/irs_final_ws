#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint

from hs_robot_system_interfaces.srv import RobotState

JOINT_NAMES = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
PLANNING_GROUP = 'tmr_arm'
ACTION_NAME = '/move_action'

# Example sets, substitute with your actual pick/place joint configs
PICK_JOINTS = [0.0, 0.0, 1.57, 0.0, 1.57, 0.0]
PLACE_JOINTS = [0.5, 0.2, 1.2, 0.0, 1.0, 0.2]
HOME_JOINTS = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

class ArmManager(Node):
    def __init__(self):
        super().__init__('arm_manager')
        self.state = 'idle'
        self.busy = False

        # State service
        self.state_client = self.create_client(RobotState, 'robot_state_service')
        self.get_logger().info('🦾 Waiting for robot_state_service...')
        self.state_client.wait_for_service()

        # Arm MoveGroup action
        self.movegroup_client = ActionClient(self, MoveGroup, ACTION_NAME)
        self.get_logger().info('🦾 Waiting for MoveGroup action server...')
        self.movegroup_client.wait_for_server()
        self.get_logger().info('🦾 Ready for pick/place operations.')

        # Start main robot loop
        self.create_timer(2.0, self.main_loop)

    def main_loop(self):
        if self.busy:
            return  # Don't run if already executing a sequence

        self.get_logger().info('🦾 Checking robot state')
        req = RobotState.Request()
        req.command = 'get'

        future = self.state_client.call_async(req)
        future.add_done_callback(self.on_state_response)

    def on_state_response(self, future):
        resp = future.result()
        if resp is None:
            self.get_logger().error('🦾 No state service response.')
            return

        current_state = resp.current_state  # use the field in your response!
        self.get_logger().info(f'🦾 State: {current_state}')

        if current_state == 'ready_to_pick':
            self.get_logger().info('🦾 State: ready_to_pick; starting pick sequence.')
            self.busy = True
            self.pick_sequence()
        elif current_state == 'ready_to_place':
            self.get_logger().info('🦾 State: ready_to_place; starting place sequence.')
            self.busy = True
            self.place_sequence()
        # else: idle

    def send_arm_goal(self, joints):
        goal = MoveGroup.Goal()
        goal.request = self._make_joint_goal(joints)
        goal.planning_options.plan_only = False

        self.get_logger().info(
            f"🦾 Sending MoveGroup goal: {[f'{n}: {p}' for n, p in zip(JOINT_NAMES, joints)]}"
        )

        result_future = self.movegroup_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, result_future)
        goal_handle = result_future.result()

        self.get_logger().info(f"🦾 Goal accepted: {getattr(goal_handle, 'accepted', 'None')}")

        if not goal_handle.accepted:
            self.get_logger().error('🦾 Arm motion goal was rejected by MoveGroup!')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result

        self.get_logger().info(f"🦾 MoveGroup error_code: {getattr(result, 'error_code', 'None')}")

        return result.error_code.val == result.error_code.SUCCESS

    def _make_joint_goal(self, joints):
        req = MotionPlanRequest()
        req.group_name = PLANNING_GROUP
        
        c = Constraints()
        c.joint_constraints = [
            JointConstraint(joint_name=n, position=p, weight=1.0)
            for n, p in zip(JOINT_NAMES, joints)
        ]

        req.goal_constraints = [c]
        return req

    def pick_sequence(self):
        ok = self.send_arm_goal(PICK_JOINTS)
        if ok:
            self.get_logger().info('🦾 Pick sequence done. Requesting state: object_picked')
            self._set_robot_state('object_picked')
        else:
            self.get_logger().error('🦾 Pick failed.')
        self.busy = False

    def place_sequence(self):
        ok = self.send_arm_goal(PLACE_JOINTS)
        if ok:
            self.get_logger().info('🦾 Place sequence done. Requesting state: object_placed')
            self._set_robot_state('object_placed')
        else:
            self.get_logger().error('🦾 Place failed.')
        self.busy = False

    def _set_robot_state(self, new_state):
        req = RobotState.Request()
        req.command = 'set'
        req.new_state = new_state
        future = self.state_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        resp = future.result()
        if resp is not None and resp.success:
            self.get_logger().info(f"🦾 State updated to '{new_state}'.")
        else:
            self.get_logger().error(f"🦾 Failed to set state '{new_state}'.")

def main(args=None):
    rclpy.init(args=args)
    node = ArmManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
