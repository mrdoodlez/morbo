#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class McNode(Node):
    def __init__(self):
        super().__init__("machine_control")
        self.subscriber = self.create_subscription(Twist, "turtle1/cmd_vel",
                self.twist_callback, 10)
        self.get_logger().info("machine control node created")

    def twist_callback(self, msg):
        self.get_logger().info("lx: " + str(msg.linear.x)
                + " az: " + str(msg.angular.z))

def main(args=None):
    rclpy.init(args=args)
    node = McNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
