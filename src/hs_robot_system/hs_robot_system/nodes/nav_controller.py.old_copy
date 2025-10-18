#!/usr/bin/env python3
import math
import time
import json
import threading

import rclpy
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String

# ----------------------------------------------------------
# Robot State and PLC Box Location Services
from hs_robot_system_interfaces.srv import RobotState
from hs_robot_system_interfaces.srv import PLCLocation

# ----------------------------------------------------------
# Log throttle limiter
last_feedback_time = 0

# --- Helper function to build a PoseStamped ---
def make_pose(x: float, y: float, yaw: float) -> PoseStamped:
    """
    Create a PoseStamped (position + orientation) in the 'map' frame.
    """
    ps = PoseStamped()
    ps.header.frame_id = 'map'  # Nav2 goals are usually in 'map' frame
    ps.pose.position.x = x
    ps.pose.position.y = y

    # Convert yaw (in radians) into quaternion (Z-only)
    half = yaw * 0.5
    ps.pose.orientation.z = math.sin(half)
    ps.pose.orientation.w = math.cos(half)
    return ps

# --- Function to drive robot manually for a duration ---
def drive(linear_speed, angular_speed, duration_sec):
    twist = Twist()
    twist.linear.x = linear_speed
    twist.angular.z = angular_speed
    start_time = node.get_clock().now()
    node.get_logger().info(f"🤖 Executing drive command for {duration_sec} seconds.")
    while (node.get_clock().now() - start_time).nanoseconds < duration_sec * 1e9:
        cmd_pub.publish(twist)
        time.sleep(0.1)
    # Stop after motion
    twist.linear.x = 0.0
    twist.angular.z = 0.0
    cmd_pub.publish(twist)
    node.get_logger().info("🤖 Drive command finished.")

def main():
    # --- Define goal locations ---
    locations = {
        'A': make_pose(6.0, -1.5, math.pi),
        'B': make_pose(6.0, -0.5, math.pi),
        'C': make_pose(6.0, 0.5, math.pi),
        'place': make_pose(31.0, -4.5, (3 * math.pi/2)) 
    }
    
    # Initialise ROS 2 and create a node
    rclpy.init()
    node = rclpy.create_node('hs_nav_controller')

    # Create an ActionClient for the NavigateToPose action
    client = ActionClient(node, NavigateToPose, 'navigate_to_pose')

    # --- Services ---
    # PLCLocation Service client setup
    plc_client = node.create_client(PLCLocation, 'get_plc_location')

    # Make sure the service is available before use
    node.get_logger().info('🤖 Waiting for PLC location service...')
    plc_client.wait_for_service()
    node.get_logger().info('🤖 PLC location service available.')

    # Robot State Service client setup
    state_client = node.create_client(RobotState, 'robot_state_service')
    
    # Make sure the service is available before use
    node.get_logger().info('🤖 Connecting to robot_state_service...')
    state_client.wait_for_service()
    node.get_logger().info('🤖 Connected to robot_state_service.')

    # --- Helper functions for Robot State service ---
    # Query the robot_state_service for the current state.
    def get_robot_state() -> str:
        request = RobotState.Request()
        request.command = 'get'
        request.new_state = ''
        future = state_client.call_async(request)
        rclpy.spin_until_future_complete(node, future)
        response = future.result()
        if response.success:
            return response.current_state.strip().lower()
        else:
            node.get_logger().warn('🤖 Failed to get robot state.')
            return 'unknown'

    # Set the robot state via the service.
    def set_robot_state(new_state: str):  
        request = RobotState.Request()
        request.command = 'set'
        request.new_state = new_state
        future = state_client.call_async(request)
        rclpy.spin_until_future_complete(node, future)
        response = future.result()
        if response.success:
            node.get_logger().info(f"🤖 State updated → {new_state}")
        else:
            node.get_logger().warn(f"🤖 State update failed: {response.message}")

    # --- Publishers ---
    # Publisher: tell others when robot arrives; publishes to robot/arrived that robot is at location
    arrival_pub = node.create_publisher(String, 'robot/arrived', 10)
    
    # Temp node creation. This will eventually be in the arm node
    arm_status_pub = node.create_publisher(String, 'robot/arm_status', 10)
    
    # Used for backup and spin.
    cmd_pub = node.create_publisher(Twist, 'cmd_vel', 10)

    # --- Function to send a goal and wait for result ---
    def send_and_wait(pose: PoseStamped) -> bool:
        node.get_logger().info('🤖 Waiting for Nav2 action server...')
        client.wait_for_server()

        pose.header.stamp = node.get_clock().now().to_msg()
        goal = NavigateToPose.Goal()
        goal.pose = pose

        done_event = threading.Event()  # synchronization object
        result_holder = {'success': False}

        def feedback_cb(fb):
            global last_feedback_time
            if hasattr(fb.feedback, 'distance_remaining'):
                dist = fb.feedback.distance_remaining
                # Throttle the log output to reduce clutter
                now_sec = time.time()
                if now_sec - last_feedback_time > 2.0:
                    node.get_logger().info(f"🤖 Distance remaining: {dist:.2f} m")
                    last_feedback_time = now_sec

        def goal_response_cb(future):
            goal_handle = future.result()
            if not goal_handle.accepted:
                node.get_logger().error('🤖 Goal was rejected by Nav2!')
                done_event.set()
                return

            node.get_logger().info('🤖 Goal accepted, waiting for result...')
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(result_cb)

        def result_cb(future):
            result = future.result()
            status = result.status
            node.get_logger().info(f'🤖 Nav2 returned status: {status}')
            if status == 4:
                node.get_logger().info('🤖 Goal reached successfully!')
                msg = String()
                msg.data = "arrived"
                arrival_pub.publish(msg)
                node.get_logger().info("🤖 Published 'arrived' message for arm node.")
                result_holder['success'] = True
                if get_robot_state() == 'move_to_pick':
                    set_robot_state('picking')
                elif get_robot_state() == 'move_to_place':
                    set_robot_state('placing')
                else:
                    set_robot_state('unknown')
            else:
                node.get_logger().warn(f'🤖 Navigation failed with status {status}.')
            done_event.set()

        send_goal_future = client.send_goal_async(goal, feedback_cb)
        send_goal_future.add_done_callback(goal_response_cb)

        # Keep spinning until result callback sets the event
        while rclpy.ok() and not done_event.is_set():
            rclpy.spin_once(node, timeout_sec=0.1)
        
        return result_holder['success']

    # Check every 5 seconds for new box location when idle 
    def periodic_plc_request():
        node.get_logger().info('🤖 Starting Periodic PLC Request')
        # Async get current robot state
        state_req = RobotState.Request()
        state_req.command = 'get'
        state_req.new_state = ''
        state_future = state_client.call_async(state_req)

        def state_response_cb(future):
            response = future.result()
            if response and response.success and response.current_state.strip().lower() == 'idle':
                node.get_logger().info('🤖 Requesting new PLC box location...')
                request_plc_location()
        state_future.add_done_callback(state_response_cb)
    node.create_timer(5.0, periodic_plc_request)

    def request_plc_location():
        """
        Query the PLC location service for the next box pickup point.
        Returns True if navigation has been started, False otherwise.
        """
        request = PLCLocation.Request()
        # If your service needs no request fields, skip filling anything
        future = plc_client.call_async(request)
        
        node.get_logger().info('🤖 PLC Request Location')
        
        def response_cb(fut):
            try:
                response = fut.result()
                location = response.location.strip().upper()
                node.get_logger().info(f"🤖 Received PLC location from service: {location}")
                if location in locations:
                    set_robot_state('move_to_pick')
                    threading.Thread(target=send_and_wait, args=(locations[location],)).start()
                else:
                    node.get_logger().warn(f"🤖 Unknown location '{location}' — ignored.")
            except Exception as e:
                node.get_logger().error(f"🤖 PLC location service call failed: {e}")

        future.add_done_callback(response_cb)
        return True

    # --- arm status (robot/arm_status) callback ---
    def arm_status_callback(msg: String):
        state = msg.data.strip().lower()
        
        if state == 'ready':
            current_state = get_robot_state()

            if current_state == 'picking':
                # Pick complete -> Move to place
                node.get_logger().info("🤖 Pick complete moving to place point.")
                set_robot_state('move_to_place')

                # Step 1: Reverse for 1 second (negative linear speed)
                node.get_logger().info("🤖 Reversing 0.5 meters before turning.")
                drive(linear_speed=-0.2, angular_speed=0.0, duration_sec=1.0)

                # Step 2: Rotate 180 degrees (pi radians)
                node.get_logger().info("🤖 Rotating 180 degrees.")
                drive(linear_speed=0.0, angular_speed=0.3, duration_sec=10.0)

                # Step 3: Proceed to place waypoint
                node.get_logger().info("🤖 Heading to place waypoint.")
                send_and_wait(locations['place']) #going to dropoff point

            elif current_state == 'placing':
                # Place complete → Return to idle
                node.get_logger().info("🤖 Place complete — returning to idle.")
                set_robot_state('idle')
            
    # --- Subscribe to the arm topic ---
    node.create_subscription(String, 'robot/arm_status', arm_status_callback, 10) #subscribing to get that info, idk what the topic is

    set_robot_state('idle')

    node.get_logger().info("🤖 Listening to PLC data and ready to move...")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
