#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

import board, busio
from adafruit_pca9685 import PCA9685

class CommandArm(Node):
    def __init__(self):
        super().__init__('command_arm')
        # Subscribe to the �s-pulse widths on /servo_commands
        self.sub = self.create_subscription(
            JointState, '/servo_commands', self.cb_servo_commands, 10
        )
        # Initialize the PCA9685 over I2C
        i2c = busio.I2C(board.SCL, board.SDA)
        self.pca = PCA9685(i2c)
        self.pca.frequency = 50  # 50 Hz for hobby servos
        self.get_logger().info("PCA9685 initialized at 50 Hz")

    def cb_servo_commands(self, msg: JointState):
        """
        msg.position is a list of pulse_widths in �s.
        We convert each to a 16-bit duty cycle for the PCA9685.
        """
        for i, pulse_us in enumerate(msg.position):
            # Compute the 16-bit duty cycle value:
            #   duty / 65535 = pulse_us / (1e6/freq)
            duty = int(pulse_us * self.pca.frequency * 65536 / 1e6)
            # Clamp to valid range
            if duty < 0:
                duty = 0
            elif duty > 0xFFFF:
                duty = 0xFFFF
            # Send to channel
            self.pca.channels[i].duty_cycle = duty

        self.get_logger().debug(f"Servos pulses (�s): {list(msg.position)}")

def main(args=None):
    rclpy.init(args=args)
    node = CommandArm()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # zero all channels on shutdown
        for ch in node.pca.channels:
            ch.duty_cycle = 0
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()