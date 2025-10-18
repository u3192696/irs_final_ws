#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from moveit_msgs.action import MoveGroup

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
        self.create_timer(1.0, self.main_loop)

    def main_loop(self):
        if self.busy:
            return  # Don't run if already executing a sequence

        # Example: check external state
        req = RobotState.Request()
        req.command = 'get'

        future = self.state_client.call_async(req)
        rclpy.task.spin_until_future_complete(self, future)
        resp = future.result()

        if resp is None:
            self.get_logger().error('🦾 No state service response.')
            return

        current_state = resp.state
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

        result_future = self.movegroup_client.send_goal_async(goal)
        rclpy.task.spin_until_future_complete(self, result_future)
        goal_handle = result_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('🦾 Arm motion goal was rejected by MoveGroup!')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.task.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        return result.error_code.val == result.error_code.SUCCESS

    def _make_joint_goal(self, joints):
        req = MotionPlanRequest()
        req.group_name = PLANNING_GROUP
        req.goal_constraints.joint_constraints = [
            JointConstraint(joint_name=n, position=p, weight=1.0)
            for n, p in zip(JOINT_NAMES, joints)
        ]
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
        rclpy.task.spin_until_future_complete(self, future)
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
