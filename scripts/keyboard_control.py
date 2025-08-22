#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class EnterControl(Node):
    def __init__(self):
        super().__init__('enter_keyboard_control')
        self.pub = self.create_publisher(Float64MultiArray,
                                         '/position_controller/commands', 10)

    def run(self):
        self.get_logger().info('Type three angles separated by spaces; q to quit')
        while True:
            line = input('> ')
            if line.strip().lower() in ['q', 'quit', 'exit']:
                break
            try:
                angles = [float(x) for x in line.split()]
                if len(angles) != 3:
                    raise ValueError
                self.pub.publish(Float64MultiArray(data=angles))
            except ValueError:
                self.get_logger().info('Invalid input')

def main():
    rclpy.init()
    EnterControl().run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
