#!/usr/bin/env python3
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint

from std_srvs.srv import Trigger

JOINT_NAMES = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
PLANNING_GROUP = 'tmr_arm'
ACTION_NAME = '/move_action'

# ----- ARM POSITIONS -----
PICK_POS = [0.0, ((45*math.pi)/180), ((45*math.pi)/180), 0.0, ((90*math.pi)/180), 0.0]
PLACE_POS = [0.0, ((40*math.pi)/180), ((45*math.pi)/180), ((5*math.pi)/180), ((90*math.pi)/180), 0.0]
CARRY_POS = [((180*math.pi)/180), 0.0, ((90*math.pi)/180), 0.0, ((90*math.pi)/180), 0.0]
HOME_POS = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

class ArmManager(Node):
    def __init__(self):
        super().__init__('arm_manager')
        

        # Setup MoveGroup action client
        self.client = ActionClient(self, MoveGroup, ACTION_NAME)
        self.get_logger().info('🦾 Waiting for MoveGroup action server...')
        self.client.wait_for_server()
        self.get_logger().info('🦾 Connected to MoveGroup server.')

        # Services exposed to workflow_manager
        self.pick_service = self.create_service(Trigger, 'perform_pick', self.perform_pick)
        self.place_service = self.create_service(Trigger, 'perform_place', self.perform_place)
        self.get_logger().info('🦾 ArmManager ready for workflow service calls.')

    # ----- WORKFLOW ENTRY POINTS -----
    def perform_pick(self, request, response):
        self.get_logger().info('🦾 Starting pick sequence...')
        success = self.pick_sequence()
        response.success = success
        return response

    def perform_place(self, request, response):
        self.get_logger().info('🦾 Starting place sequence...')
        success = self.place_sequence()
        response.success = success
        return response

    # ----- ARM SEQUENCES -----
    def pick_sequence(self, service_response_callback):
        def to_pick_done(success):
            if not success:
                service_response_callback(False)
                return
            # Replace time.sleep() with Timer
            self.get_logger().info('🦾 Completed pick... Moving to carry')
            self.create_timer(3.0, lambda: self.moveto(CARRY_POS, to_carry_done))
    
        def to_carry_done(success):
            self.get_logger().info('🦾 Completed carry')
            service_response_callback(success)
    
        self.moveto(PICK_POS, to_pick_done)

    def perform_pick(self, request, response):
        def complete_pick(success):
            response.success = success
            self.get_logger().info('🦾 Ready to navigate to PLACE location')
            return response
        self.pick_sequence(complete_pick)
    
    def place_sequence(self, service_response_callback):
        def to_place_done(success):
            if not success:
                service_response_callback(False)
                return
            # Replace time.sleep() with Timer
            self.get_logger().info('🦾 Completed place... Moving to home')
            self.create_timer(1.0, lambda: self.moveto(HOME_POS, to_home_done))
    
        def to_home_done(success):
            self.get_logger().info('🦾 Completed home')
            service_response_callback(success)
    
        self.moveto(PLACE_POS, to_place_done)

    def perform_place(self, request, response):
        def complete_place(success):
            response.success = success
            self.get_logger().info('🦾 Completed place... Returning to idle')
            return response
        self.place_sequence(complete_place)

    def _move_to(self, joints, final_callback):
        goal = self._goal_from_joints(joints)
        send_future = self.client.send_goal_async(goal)
        
        def goal_sent_callback(send_future):
            goal_handle = send_future.result()
            if not goal_handle.accepted:
                self.get_logger().error("🦾 MoveGroup goal rejected.")
                final_callback(False)
                return
            result_future = goal_handle.get_result_async()
            
            def result_callback(result_future):
                result = result_future.result()
                error_code = result.result.error_code.val
                if error_code == 1:
                    self.get_logger().info("🦾 MoveGroup motion succeeded.")
                    final_callback(True)
                else:
                    self.get_logger().error(f"🦾 MoveGroup motion failed code {error_code}.")
                    final_callback(False)
            
            result_future.add_done_callback(result_callback)
        send_future.add_done_callback(goal_sent_callback)

    def _goal_from_joints(self, joints):
        goal = MoveGroup.Goal()
        req = MotionPlanRequest()
        req.group_name = PLANNING_GROUP
        req.num_planning_attempts = 10
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = 0.5
        req.max_acceleration_scaling_factor = 0.5

        cs = Constraints()
        for jname, jval in zip(JOINT_NAMES, joints):
            jc = JointConstraint()
            jc.joint_name = jname
            jc.position = jval
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            cs.joint_constraints.append(jc)
        req.goal_constraints.append(cs)
        goal.request = req
        goal.planning_options.plan_only = False
        return goal
   
def main(args=None):
    rclpy.init(args=args)
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
