#!/usr/bin/env python3
# Temperature Sensor Node

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import random


class TemperatureSensor(Node):
    def __init__(self):
        super().__init__('temperature_sensor')
        self.publisher_ = self.create_publisher(Float32, 'temperature', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.count = 0
        self.get_logger().info('Temperature Sensor Started!')

    def timer_callback(self):
        msg = Float32()
        msg.data = 20.0 + random.random() * 10.0
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data:.2f} degrees C')
        self.count += 1


def main(args=None):
    rclpy.init(args=args)
    node = TemperatureSensor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
