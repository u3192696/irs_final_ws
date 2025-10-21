#!/usr/bin/env python3
import time
import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from hs_robot_system_interfaces.srv import PLCLocation


class PLC_Location_Server(Node):
    def __init__(self):
        super().__init__('plc_location_service')
        self.get_logger().info('📦 PLC Location Service starting...')
        
        # Create service
        self._srv = self.create_service(
            PLCLocation,
            'get_plc_location',
            self.handle_plc_request
        )

        # Internal storage for last known /hmi/unified_status data
        self.unified_status_data = None

        # Subscribe to /hmi/unified_status for box locations
        self.create_subscription(String, '/hmi/unified_status', self.unified_status_callback, 10)

        # List of valid box locations
        self._available_locations = ['A', 'B', 'C']

        # Stability tracking variables
        self._last_location = None
        self._last_change_time = time.time()
        self._stable_location = None
        self._stability_threshold = 1.5  # seconds at same location before considered "stopped"

        self.get_logger().info('📦 PLC Location Service ready (listening for /hmi/unified_status).')


    def unified_status_callback(self, msg):
        try:
            data = json.loads(msg.data)
            box_data = data.get("box", {})
            raw_loc = box_data.get("box_location", "").strip()

            if not raw_loc:
                return

            # Extract only the final character ('A', 'B', or 'C')
            current_location = raw_loc[-1]

            # Check if location changed recently (box is moving through location)
            if current_location != self._last_location:
                self._last_change_time = time.time()
                self.get_logger().debug(f"📦 Location changed to {current_location} (timer reset)")
                self._last_location = current_location
            else:
                # Location hasn't changed, check if it's stable long enough
                elapsed = time.time() - self._last_change_time
                if elapsed >= self._stability_threshold:
                    if self._stable_location != self._last_location:
                        self._stable_location = self._last_location
                        self.get_logger().info(f"📦✅ Box location stabilized at {self._stable_location} for {elapsed:.1f}s")
        except Exception as e:
            self.get_logger().warn(f"📦 Failed to parse unified_status: {e}")

    def handle_plc_request(self, request, response):
        if self._stable_location and self._stable_location in self._available_locations:
            response.location = self._stable_location
            self.get_logger().info(f"📦 PLC request → returning STABLE location: {response.location}")
        else:
            self.get_logger().warn("📦 PLC request received but location not stable yet.")
            response.location = "Unknown"
        return response

def main(args=None):
    rclpy.init(args=args)
    node = PLC_Location_Server()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('📦 Shutting down PLC Location Service...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
