#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class DataProcessor(Node):
    def __init__(self):
        super().__init__('data_processor')
        self.subscription = self.create_subscription(
            Float32,
            '/sensor_value',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.publisher_ = self.create_publisher(Float32, '/processed_value', 10)
        self.get_logger().info('DataProcessor node started (processing: x2)')

    def listener_callback(self, msg: Float32) -> None:
        original = msg.data
        processed_value = float(original * 2.0)

        out_msg = Float32()
        out_msg.data = processed_value
        self.publisher_.publish(out_msg)
        self.get_logger().debug(f'Received: {original}, processed: {processed_value}')


def main(args=None):
    rclpy.init(args=args)
    node = DataProcessor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


