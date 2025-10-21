#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node

# If you later define a real srv file, change this import accordingly.
# For now we'll use a generic std_srvs/Trigger approach or a placeholder.
from hs_robot_system_interfaces.srv import PLCLocation


class PLC_Location_Server(Node):
    """
    ROS 2 Service node that provides the next box location from 'PLC'.
    """

    def __init__(self):
        super().__init__('plc_location_service')
        self.get_logger().info('📦 PLC Location Service starting...')
        # Create service (name must match the client in nav_controller)
        self._srv = self.create_service(
            PLCLocation,
            'get_plc_location',
            self.handle_plc_request
        )

        # Internal list of simulated warehouse slots
        self._available_locations = ['A', 'B', 'C']

    def handle_plc_request(self, request, response):
        """
        Handles service call; returns a random location.
        """
        # Fetch current box location from PLC
        location = 'B'
        time.sleep(2)
        self.get_logger().info(f"📦 PLC request received → responding with location {location}")
        response.location = location
        return response


def main(args=None):
    rclpy.init(args=args)
    node = PLC_Location_Server()
    node.get_logger().info('📦✅ PLC Location Service ready and waiting for requests.')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('📦 Shutting down PLC Location Service...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
