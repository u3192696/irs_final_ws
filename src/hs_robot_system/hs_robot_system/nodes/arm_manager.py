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
from moveit_msgs.srv import GetStateValidity
from sensor_msgs.msg import JointState
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
        
        # Subscribe to /joint_states to ensure joint state synchronisation
        self.latest_joint_states = None
        self.create_subscription(JointState, '/joint_states', self._joint_state_callback, 10)
        
        self.validity_client = self.create_client(GetStateValidity, '/check_state_validity')

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
    
    def _joint_state_callback(self, msg):
        self.latest_joint_states = msg
    
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
            j.tolerance_above = 0.1
            j.tolerance_below = 0.1
            j.weight = 1.0
            goal_constraints.append(j)

        req.goal_constraints = [Constraints(name='target_pose', joint_constraints=goal_constraints)]
        return req

    async def execute_callback(self, goal_handle):
        mode = goal_handle.request.mode.lower()
        feedback = ArmTask.Feedback()
        result = ArmTask.Result()

        self.get_logger().info(f"🦾 Executing ArmTask: {mode}")

        # Define motion sequences depending on mode
        if mode == "pick":
            sequences = [
                [PICK_POS],   # Move to pick
                [CARRY_POS]   # Move to carry only once weight detected
            ]
        elif mode == "place":
            sequences = [
                [PLACE_POS],  # Move to place
                [CARRY_POS]    # Return home after box cleared
            ]
        else:
            self.get_logger().info(f"🦾 Error - Unknown mode: {mode}")
            goal_handle.abort()
            result.success = False
            result.message = msg
            return result

        # Execute each step in the sequence
        for step_index, step_sequence in enumerate(sequences):
            posture = step_sequence[0]
            feedback.step = f"Step {step_index + 1} of {len(sequences)}"
            feedback.progress = float(step_index + 1) / len(sequences)
            goal_handle.publish_feedback(feedback)


            # --- Build MotionPlan request ---
            move_goal = MoveGroup.Goal()
            move_goal.request = self.make_request(posture)
            #move_goal.request.start_state = current_state if current_state else move_goal.request.start_state

            # Refresh current joint state before planning
            current_state = self._make_current_state()
            if current_state:
                move_goal.request.start_state = current_state
            else:
                self.get_logger().warn("🦾 No current joint state available. Using default.")

            # Attempt up to 10 retries if execution fails
            max_attempts = 10
            attempt = 1
            success = False

            while attempt <= max_attempts and not success:
                self.get_logger().info(f"🦾 Executing MoveGroup attempt {attempt}/{max_attempts}")
                move_goal_future = self.move_client.send_goal_async(move_goal)
                move_goal_handle = await move_goal_future

                if not move_goal_handle.accepted:
                    self.get_logger().warn(f"🦾❌ MoveGroup goal rejected on attempt {attempt}.")
                    attempt += 1
                    time.sleep(1.0)
                    continue

                self.get_logger().info(f"🦾 MoveGroup goal accepted on attempt {attempt}.")
                result_future = move_goal_handle.get_result_async()
                move_result = await result_future
                code = move_result.result.error_code.val

                if code == 1:
                    self.get_logger().info(f"🦾✅ MoveGroup step succeeded on attempt {attempt}.")
                    success = True
                else:
                    self.get_logger().warn(f"🦾⚠️ MoveGroup failed (code: {code}) on attempt {attempt}.")

                    # Allow time for Unity to update /joint_states
                    time.sleep(2.0)

                    # Refresh robot joint state before retry
                    refreshed_state = self._make_current_state()
                    if refreshed_state:
                        move_goal.request.start_state = refreshed_state

                    attempt += 1
                    if attempt <= max_attempts:
                        self.get_logger().info("🦾 Retrying motion...")
                    else:
                        self.get_logger().error(f"🦾❌ All {max_attempts} attempts failed for this step.")

            if not success:
                goal_handle.abort()
                result.success = False
                result.message = f"MoveGroup failed after {max_attempts} attempts."
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

        # --- Success ---
        goal_handle.succeed()
        self.get_logger().info("🦾 ArmTask motion completed successfully.")
        result.success = True
        result.message = "Motion complete."
        return result        
    
    # --- Helper to capture and format current joint state for MoveIt ---
    def _make_current_state(self):
        try:
            from moveit_msgs.msg import RobotState
            from sensor_msgs.msg import JointState
            state = RobotState()
            joint_state = JointState()
            joint_state.name = JOINT_NAMES

            if hasattr(self, "latest_joint_states"):
                joint_state.position = list(self.latest_joint_states.position)
            else:
                self.get_logger().warn("🦾 No joint state cache found, setting zeros.")
                joint_state.position = [0.0] * len(JOINT_NAMES)

            state.joint_state = joint_state
            return state
        except Exception as e:
            self.get_logger().error(f"🦾 Failed to read current state: {e}")
            return None


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
