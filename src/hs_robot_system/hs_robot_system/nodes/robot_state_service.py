#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from hs_robot_system_interfaces.srv import RobotState


class RobotStateService(Node):
    def __init__(self):
        super().__init__('robot_state_service')
        self._current_state = 'idle'

        self.get_logger().info(f'🤖 Robot State Service starting...')
        self._srv = self.create_service(RobotState, 'robot_state_service', self.handle_request)
        self.get_logger().info(f'✅ Service ready. Initial state: {self._current_state}')

    def handle_request(self, request, response):
        try:
            if request.command.lower() == 'get':
                response.current_state = self._current_state
                response.success = True
                response.message = f'Current state is {self._current_state}.'
                self.get_logger().info(response.message)

            elif request.command.lower() == 'set':
                new_state = request.new_state.strip().upper()
                self.get_logger().info(f'Request to set state → {new_state}')
                self._current_state = new_state
                response.current_state = self._current_state
                response.success = True
                response.message = f'State updated to {self._current_state}.'

            else:
                response.success = False
                response.message = "Unknown command; use 'get' or 'set'."

        except Exception as e:
            self.get_logger().error(f'Exception in handle_request: {e}')
            response.success = False
            response.message = f'Error: {e}'
            response.current_state = self._current_state

        return response


def main(args=None):
    rclpy.init(args=args)
    node = RobotStateService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Robot State Service shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
