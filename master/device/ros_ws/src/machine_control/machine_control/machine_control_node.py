#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

class McNode(Node):
    def __init__(self):
        super().__init__("machine_control")
        self.get_logger().info("hello ros2!")
        self.counter_ = 0
        self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        self.counter_ += 1
        self.get_logger().info("hello!" + str(self.counter_))

def main(args=None):
    rclpy.init(args=args)
    node = McNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
