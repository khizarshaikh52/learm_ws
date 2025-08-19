# fk_echo.py
import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PointStamped

def dh_transform(a, alpha, d, theta):
    ca, sa = math.cos(alpha), math.sin(alpha)
    ct, st = math.cos(theta), math.sin(theta)
    return np.array([
        [ct, -st*ca,  st*sa, a*ct],
        [st,  ct*ca, -ct*sa, a*st],
        [0.,     sa,     ca,    d],
        [0.,    0.,    0.,    1.0]
    ], dtype=float)

class FKEcho(Node):
    def __init__(self):
        super().__init__('fk_echo')

        # EDIT THESE: DH for your first 6 joints (ignore gripper).
        # Units: a,d in meters; alpha,theta_offset in radians.
        # Example placeholders � replace with your calibrated values.
        self.a =            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.alpha =        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.d =            [0.1, 0.0, 0.0, 0.10, 0.0, 0.05]
        self.theta_offset = [0.0, 0.0, 0.0, 0.0,  0.0, 0.0]  # your calibrated zeros

        # Joint names must match order coming from /joint_states
        self.joint_names = ['shoulder_pan','shoulder_lift','elbow1','elbow2','wrist_flex','wrist_roll','grip_left']

        self.sub = self.create_subscription(JointState, '/joint_states', self.cb, 10)
        self.pub = self.create_publisher(PointStamped, '/ee_position', 10)

    def cb(self, msg: JointState):
        # Map incoming positions to our order
        name_to_pos = {n: p for n, p in zip(msg.name, msg.position)}
        # Take first 6 joints for FK; skip gripper
        q = []
        for n in self.joint_names[:6]:
            if n not in name_to_pos:
                self.get_logger().warn(f'Missing joint in JointState: {n}')
                return
            q.append(name_to_pos[n])

        # Build transform chain
        T = np.eye(4)
        for i in range(6):
            theta = float(q[i]) + float(self.theta_offset[i])  # radians
            A = dh_transform(self.a[i], self.alpha[i], self.d[i], theta)
            T = T @ A

        # Extract XYZ
        p = PointStamped()
        p.header.stamp = self.get_clock().now().to_msg()
        p.header.frame_id = 'base_link'  # change if different
        p.point.x, p.point.y, p.point.z = float(T[0,3]), float(T[1,3]), float(T[2,3])
        self.pub.publish(p)
        # Also print for a quick glance
        self.get_logger().info(f'EE XYZ: [{p.point.x:.3f}, {p.point.y:.3f}, {p.point.z:.3f}]')

def main():
    rclpy.init()
    node = FKEcho()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
