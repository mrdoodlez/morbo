#!/usr/bin/env python3

import os
import select
import sys
import termios
import tty
import socket

def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main():
    settings = None
    settings = termios.tcgetattr(sys.stdin)

    # Create a TCP/IP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    # Connect the socket to the port where the server is listening
    server_address = ('localhost', 10000)
    sock.connect(server_address)

    try:
        while True:
            key = get_key(settings)
            message = ''
            if key == 'w':
                message = 'mbw'
            elif key == 'x':
                message = 'mbx'
            elif key == 'a':
                message = 'mba'
            elif key == 'd':
                message = 'mbd'
            elif key == ' ' or key == 's':
                message = 'mbs'
            elif key == 'h':
                message = 'mbh'
            elif key == 'k':
                message = 'mbk'
            elif key == 'u':
                message = 'mbu'
            elif key == 'n':
                message = 'mbn'
            elif key == 'j':
                message = 'mbj'
            else:
                if (key == '\x03'):
                    break

            sock.sendall(message.encode())

    except Exception as e:
        print(e)

    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        sock.close()

if __name__ == '__main__':
    main()

