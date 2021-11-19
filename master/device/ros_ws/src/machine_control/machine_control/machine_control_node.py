#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import struct
from smbus2 import SMBus

TRANSFER_CMD_LEN = 16
TRANSFER_PL_LEN = TRANSFER_CMD_LEN - 1

TRANSFER_ADDRESS = 0x3f

CODE_ACK    = 0
CODE_NACK   = 1

CODE_PING   = 2

CODE_STOP   = 3
CODE_SPEEDS = 4

class McNode(Node):
    def __init__(self):
        super().__init__("machine_control")
        self.subscriber = self.create_subscription(Twist, "turtle1/cmd_vel",
                self.twist_callback, 10)
        self.bus = SMBus(1)
        self.get_logger().info("machine control node created")

    def twist_callback(self, msg):
        l = 0
        r = 0
        if msg.linear.x > 0:
            l = r = 0x7f
        elif msg.linear.x < 0:
            l = r = 0xff
        elif msg.angular.z > 0:
            l = 0x7f
            r = 0xff
        elif msg.angular.z < 0:
            l = 0xff
            r = 0x7f
        self.get_logger().info("l: " + str(l) + " r: " + str(r))

        self.send_command(CODE_SPEEDS, l, r)

    def send_command(self, cmd, arg1 = 0, arg2 = 0):
        cmd_buff = [0xFF] * TRANSFER_PL_LEN

        if cmd == CODE_SPEEDS:
            cmd_buff[0] = arg1
            cmd_buff[1] = arg2

        self.bus.write_i2c_block_data(TRANSFER_ADDRESS, cmd, cmd_buff)

        print(cmd_buff)

        #return(cmd_buff)


def main(args=None):
    rclpy.init(args=args)
    node = McNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
