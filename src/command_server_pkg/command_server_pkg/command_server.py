#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from command_server_interfaces.srv import ComputeCommand


class CommandServer(Node):
    def __init__(self):
        super().__init__('command_server')
        self.srv = self.create_service(
            ComputeCommand,
            '/compute_command',
            self.compute_command_callback
        )
        self.get_logger().info('CommandServer node started, service: /compute_command')

    def compute_command_callback(self, request, response):
        value = float(request.input)
        if value > 10.0:
            response.output = 'HIGH'
        else:
            response.output = 'LOW'
        self.get_logger().info(f'Received {value}, responding with {response.output}')
        return response


def main(args=None):
    rclpy.init(args=args)
    node = CommandServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


