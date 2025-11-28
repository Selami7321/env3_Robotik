#!/usr/bin/env python3

import math
import random

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class SensorPublisher(Node):
    def __init__(self):
        super().__init__('sensor_publisher')
        self.publisher_ = self.create_publisher(Float32, '/sensor_value', 10)
        self.timer_period = 0.1  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # Üç farklı veri üretim modu için örnek alanlar:
        self._t = 0.0
        self._counter = 0
        self._mode = 'random'  # 'random', 'sin', 'counter'

        self.get_logger().info('SensorPublisher node started (mode: %s)' % self._mode)

    def timer_callback(self):
        if self._mode == 'random':
            value = random.uniform(0.0, 20.0)
        elif self._mode == 'sin':
            value = float(math.sin(self._t))
            self._t += self.timer_period
        else:  # counter
            self._counter += 1
            value = float(self._counter)

        msg = Float32()
        msg.data = value
        self.publisher_.publish(msg)
        self.get_logger().debug(f'Publishing sensor value: {value}')


def main(args=None):
    rclpy.init(args=args)
    node = SensorPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


