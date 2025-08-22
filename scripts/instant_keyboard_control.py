#!/usr/bin/env python3
import sys, tty, termios
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

def get_char():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return ch

class KeyboardControl(Node):
    def __init__(self):
        super().__init__('keyboard_control')
        self.pub = self.create_publisher(Float64MultiArray,
                                         '/position_controller/commands', 10)
        self.angles = [0.0, 0.0, 0.0]

    def run(self):
        self.get_logger().info('q/a w/s e/d adjust joints; r reset; x exit')
        while True:
            c = get_char()
            if c == 'x':
                break
            elif c == 'q': self.angles[0] += 0.1
            elif c == 'a': self.angles -= 0.1
            elif c == 'w': self.angles[1] += 0.1
            elif c == 's': self.angles[1] -= 0.1
            elif c == 'e': self.angles += 0.1
            elif c == 'd': self.angles -= 0.1
            elif c == 'r': self.angles = [0.0,0.0,0.0]
            self.pub.publish(Float64MultiArray(data=self.angles))

def main():
    rclpy.init()
    KeyboardControl().run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
