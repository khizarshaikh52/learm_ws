#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

class WelderControl(Node):
    def __init__(self):
        super().__init__('welder_control')
        self.create_subscription(Bool, '/welder_on', self.cb, 10)

    def cb(self, msg: Bool):
        if msg.data:
            self.get_logger().info('WELDER: ON (drive GPIO HIGH)')
        else:
            self.get_logger().info('WELDER: OFF (drive GPIO LOW)')

def main():
    rclpy.init()
    rclpy.spin(WelderControl())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
