#!/usr/bin/env python3
import math
import time
import json
import asyncio

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, CancelResponse, GoalResponse
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint
from std_msgs.msg import String

from hs_robot_system_interfaces.action import ArmTask

# ----- CONSTANTS -----
JOINT_NAMES = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
PLANNING_GROUP = 'tmr_arm'
MOVEGROUP_ACTION = '/move_action'

# ----- ARM POSITIONS -----
PICK_POS = [0.0, math.radians(45), math.radians(45), 0.0, math.radians(90), 0.0]
PLACE_POS = [0.0, math.radians(40), math.radians(45), math.radians(5), math.radians(90), 0.0]
CARRY_POS = [math.radians(180), 0.0, math.radians(90), 0.0, math.radians(90), 0.0]
HOME_POS = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

class ArmManager(Node):
    def __init__(self):
        super().__init__('arm_manager')
        
        # Subscribe to /hmi/unified_status for successful pick checks
        self.unified_status_data = None
        self.create_subscription(String, '/hmi/unified_status', self.unified_status_callback, 10)

        # Setup MoveGroup action client
        self.move_client = ActionClient(self, MoveGroup, MOVEGROUP_ACTION)
        self.get_logger().info('🦾 Waiting for MoveGroup action server...')
        self.move_client.wait_for_server()
        self.get_logger().info('🦾 Connected to MoveGroup server.')

        # Setup ArmTask action server for workflow_manager
        self.server = ActionServer(
            self,
            ArmTask,
            'arm_task',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )
        self.get_logger().info('🦾 ArmManager ready for workflow tasks.')

    def unified_status_callback(self, msg):
        try:
            data = json.loads(msg.data)
            self.unified_status_data = data
        except Exception as e:
            self.get_logger().warn(f"🦾 Failed to parse /hmi/unified_status: {e}")
    
    # Action goal received
    def goal_callback(self, goal_request):
        self.get_logger().info(f'🦾 New ArmTask goal: {goal_request.mode}')
        return GoalResponse.ACCEPT

    # Cancel request
    def cancel_callback(self, goal_handle):
        self.get_logger().info('🦾 Cancel request received for ArmTask.')
        return CancelResponse.ACCEPT

    # Create MoveGroup planning request
    def make_request(self, joint_targets):
        req = MotionPlanRequest()
        req.group_name = PLANNING_GROUP

        goal_constraints = []
        for i, position in enumerate(joint_targets):
            j = JointConstraint()
            j.joint_name = JOINT_NAMES[i]
            j.position = position
            j.tolerance_above = 0.001
            j.tolerance_below = 0.001
            j.weight = 1.0
            goal_constraints.append(j)

        req.goal_constraints = [Constraints(name='target_pose', joint_constraints=goal_constraints)]
        return req

    async def execute_callback(self, goal_handle):
        mode = goal_handle.request.mode.lower()
        feedback = ArmTask.Feedback()
        result = ArmTask.Result()

        self.get_logger().info(f"🦾 Executing ArmTask: {mode}")

        if mode == "pick":
            sequences = [
                [PICK_POS],   # Move to pick
                [CARRY_POS]   # Move to carry only once weight detected
            ]
        elif mode == "place":
            sequences = [
                [PLACE_POS],  # Move to place
                [HOME_POS]    # Return home after weight cleared
            ]
        else:
            msg = f"Unknown mode: {mode}"
            self.get_logger().error(msg)
            goal_handle.abort()
            result.success = False
            result.message = msg
            return result

        for step_index, step_sequence in enumerate(sequences):
            posture = step_sequence[0]
            feedback.step = f"Step {step_index + 1} of {len(sequences)}"
            feedback.progress = float(step_index + 1) / len(sequences)
            goal_handle.publish_feedback(feedback)

            move_goal = MoveGroup.Goal()
            move_goal.request = self.make_request(posture)

            move_goal_future = self.move_client.send_goal_async(move_goal)
            move_goal_handle = await move_goal_future

            if not move_goal_handle.accepted:
                self.get_logger().warn("🦾 MoveGroup goal rejected.")
                goal_handle.abort()
                result.success = False
                result.message = "Planning failed."
                return result

            self.get_logger().info(f"🦾 Executing motion for step {step_index + 1}")
            result_future = move_goal_handle.get_result_async()
            move_result = await result_future
            code = move_result.result.error_code.val

            if code != 1:
                self.get_logger().error(f"🦾 MoveGroup execution failed (code: {code})")
                goal_handle.abort()
                result.success = False
                result.message = f"MoveGroup error: {code}"
                return result

            # On "pick" wait for box to be detected by checking weight_raw
            if mode == "pick" and step_index == 0:
                self.get_logger().info("🦾 Waiting for box detection (weight_raw == ': 0 kg')...")
                time.sleep(0.75)  # brief blocking pause
                #while True:
                #    time.sleep(0.75) # brief blocking pause
                #    if self.unified_status_data:
                #        box_weight = self.unified_status_data.get("box", {}).get("weight_raw", "")
                #        if box_weight.strip() == ": 0 kg":
                #            self.get_logger().info("🦾 Box attached")
                #            break
                    
            if mode == "place" and step_index == 0:
                self.get_logger().info("🦾 Pausing for box to drop")
                time.sleep(0.75)  # brief blocking pause

        goal_handle.succeed()
        self.get_logger().info("🦾 ArmTask motion completed successfully.")
        result.success = True
        result.message = "Motion complete."
        return result        
    
    # Execute ArmTask goal
#    async def execute_callback(self, goal_handle):
#        mode = goal_handle.request.mode.lower()
#        feedback = ArmTask.Feedback()
#        result = ArmTask.Result()
#        self.get_logger().info(f'🦾 Executing ArmTask: {mode}')
#
#        # Define motion sequences by mode
#        if mode == 'pick':
#            sequence = [HOME_POS, PICK_POS, CARRY_POS]
#        elif mode == 'place':
#            sequence = [CARRY_POS, PLACE_POS, HOME_POS]
#        else:
#            msg = f'Unknown mode: {mode}'
#            self.get_logger().error(msg)
#            goal_handle.abort()
#            result.success = False
#            result.message = msg
#            return result
#
#        # Execute each waypoint with MoveGroup
#        #self.move_client.wait_for_server()
#
#        for i, posture in enumerate(sequence):
#            feedback.step = f'Step {i+1}/{len(sequence)}: moving joints'
#            feedback.progress = float(i+1) / len(sequence)
#            goal_handle.publish_feedback(feedback)
#
#            move_goal = MoveGroup.Goal()
#            move_goal.request = self._make_request(posture)
#
#            goal_future = self.move_client.send_goal_async(move_goal)
#            move_goal_handle = await goal_future
#
#            if not move_goal_handle.accepted:
#                self.get_logger().warn('🦾 MoveGroup goal rejected.')
#                goal_handle.abort()
#                result.success = False
#                result.message = 'Planning failed'
#                return result
#
#            # Wait on MoveGroup result
#            result_future = move_goal_handle.get_result_async()
#            move_result = await result_future
#            code = move_result.result.error_code.val
#
#            # Check MoveGroup result code
#            if code != 1:  # SUCCESS == 1
#                self.get_logger().error(f'🦾 MoveGroup execution failed with code: {code}')
#                goal_handle.abort()
#                result.success = False
#                result.message = f'MoveGroup error {code}'
#                return result
#
#            if goal_handle.is_cancel_requested:
#                goal_handle.canceled()
#                self.get_logger().warn('🦾 ArmTask motion canceled by client.')
#                result.success = False
#                result.message = 'Cancelled'
#                return result

        goal_handle.succeed()
        self.get_logger().info('🦾 ArmTask motion completed successfully.')
        result.success = True
        result.message = 'Motion complete'
        return result
    
def main():
    rclpy.init()
    node = ArmManager()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('🦾 ArmManager interrupted.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
