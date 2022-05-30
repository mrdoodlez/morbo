#!/usr/bin/env python3

import os
import select
import sys
import socket
import rclpy

from geometry_msgs.msg import Twist
from example_interfaces.msg import Float32MultiArray
from rclpy.qos import QoSProfile

LIN_STEP = 0.01
AN_STEP = 0.1

VER_STEP = 10
HOR_STEP = 10

def get_cmd(connection):
    data = connection.recv(3)
    print(data)

    if data[0:2] == b'mb':
        return data[2]

    return '\x03'

def check_limits(x):
    if x > 180.0:
        x = 180.0
    if x < 0.0:
        x = 0.0
    return x

def main():
    rclpy.init()

    qos = QoSProfile(depth=10)
    node = rclpy.create_node('morbo_remote_tcp')
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
    set_laser = 0.0

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_address = ('localhost', 10000)
    sock.bind(server_address)

    print("Waiting for TCP connection .....")
    sock.listen(1)

    connection, client_address = sock.accept()
    print("accect connection from: ", client_address)

    try:
        while True:
            cmd = get_cmd(connection)
            if cmd == 119:
                set_linear += LIN_STEP
            elif cmd == 120:
                set_linear -= LIN_STEP
            elif cmd == 97:
                set_angular += AN_STEP
            elif cmd == 100:
                set_angular -= AN_STEP
            elif cmd == 32 or cmd == 115:
                set_linear = 0.0
                set_angular = 0.0
            elif cmd == 104:
                set_hor += HOR_STEP
            elif cmd == 107:
                set_hor -= HOR_STEP
            elif cmd == 117:
                set_ver += VER_STEP
            elif cmd == 110:
                set_ver -= VER_STEP
            elif cmd == 106:
                set_laser = 1.0 - set_laser
            else:
                if (cmd == '\x03'):
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

        connection.close()
        sock.close()

if __name__ == '__main__':
    main()

