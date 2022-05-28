#!/usr/bin/env python3

import os
import select
import sys
import rclpy

from geometry_msgs.msg import Twist
from example_interfaces.msg import Float32MultiArray
from rclpy.qos import QoSProfile

import termios
import tty

LIN_STEP = 0.01
AN_STEP = 0.1

VER_STEP = 10
HOR_STEP = 10

def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def check_limits(x):
    if x > 180.0:
        x = 180.0
    if x < 0.0:
        x = 0.0
    return x

def main():
    settings = None
    settings = termios.tcgetattr(sys.stdin)

    rclpy.init()

    qos = QoSProfile(depth=10)
    node = rclpy.create_node('morbo_remote')
    vel_publisher = node.create_publisher(Twist, 'cmd_vel', qos)
    turret_publisher = node.create_publisher(Float32MultiArray, "turret_pos", qos)

    set_linear = 0.0
    set_angular = 0.0

    curr_linear = -1
    curr_angular = -1

    set_ver = 90.0
    set_hor = 90.0

    curr_ver = 0.0
    curr_hor = 0.0

    curr_laser = 0.0

    try:
        while(1):
            set_laser = 0.0
            key = get_key(settings)
            if key == 'w':
                set_linear += LIN_STEP
            elif key == 'x':
                set_linear -= LIN_STEP
            elif key == 'a':
                set_angular += AN_STEP
            elif key == 'd':
                set_angular -= AN_STEP
            elif key == ' ' or key == 's':
                set_linear = 0.0
                set_angular = 0.0
            elif key == 'h':
                set_hor += HOR_STEP
            elif key == 'k':
                set_hor -= HOR_STEP
            elif key == 'u':
                set_ver += VER_STEP
            elif key == 'n':
                set_ver -= VER_STEP
            elif key == 'j':
                set_laser = 1.0
            else:
                if (key == '\x03'):
                    break

            if (set_linear != curr_linear) or (set_angular != curr_angular):
                print("linear: {0} angular: {1}".format(set_linear, set_angular))

                twist = Twist()

                twist.linear.x = set_linear
                twist.linear.y = 0.0
                twist.linear.z = 0.0

                twist.angular.x = 0.0
                twist.angular.y = 0.0
                twist.angular.z = set_angular

                vel_publisher.publish(twist)

                curr_linear = set_linear
                curr_angular = set_angular

            set_ver = check_limits(set_ver)
            set_hor = check_limits(set_hor)

            if (set_ver != curr_ver) or (set_hor != curr_hor) or (set_laser != curr_laser):
                print("ver: {0} hor: {1} laser: {2}".format(set_ver, set_hor, set_laser))

                msg = Float32MultiArray()
                msg.data = [set_hor, set_ver, set_laser]

                turret_publisher.publish(msg)

                curr_ver = set_ver
                curr_hor = set_hor
                curr_laser = set_laser

    except Exception as e:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0

        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0

        vel_publisher.publish(twist)

        msg = Float32MultiArray()
        msg.data = [90.0, 90.0, 0.0]

        turret_publisher.publish(msg)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

if __name__ == '__main__':
    main()

'''
    def publish_turret(self):
'''

