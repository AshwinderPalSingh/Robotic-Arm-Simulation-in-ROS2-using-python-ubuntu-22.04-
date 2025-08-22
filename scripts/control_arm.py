#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class ArmDemo(Node):
    def __init__(self):
        super().__init__('arm_demo')
        self.pub = self.create_publisher(Float64MultiArray,
                                         '/position_controller/commands', 10)
        self.poses = [
            [0.0,  0.0,  0.0],
            [1.2,  0.8, -0.5],
            [-1.0, 0.2,  1.0],
            [0.0, -0.8,  0.0],
        ]
        self.idx = 0
        self.create_timer(2.0, self.step)

    def step(self):
        msg = Float64MultiArray(data=self.poses[self.idx])
        self.pub.publish(msg)
        self.get_logger().info(f'Move to {msg.data}')
        self.idx = (self.idx + 1) % len(self.poses)

def main():
    rclpy.init()
    ArmDemo()
    rclpy.spin()

if __name__ == '__main__':
    main()

