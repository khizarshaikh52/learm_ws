import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
import math
import numpy as np

DEFAULT_JOINTS = ['shoulder_pan','shoulder_lift','elbow1','elbow2','wrist_flex','wrist_roll','grip_left']

def dh_transform(a, alpha, d, theta):
    ca, sa = math.cos(alpha), math.sin(alpha)
    ct, st = math.cos(theta), math.sin(theta)
    return np.array([
        [ct, -st*ca,  st*sa, a*ct],
        [st,  ct*ca, -ct*sa, a*st],
        [0.0,    sa,     ca,    d],
        [0.0,   0.0,    0.0,  1.0]
    ], dtype=float)

class FKNode(Node):
    def __init__(self):
        super().__init__('fk_node')
        self.declare_parameter('joint_names', DEFAULT_JOINTS)
        self.declare_parameter('dh.a', [0.035,0.160,0.120,0.100,0.080,0.060])
        self.declare_parameter('dh.alpha', [1.5708,0.0,0.0,1.5708,-1.5708,0.0])
        self.declare_parameter('dh.d', [0.080,0.0,0.0,0.0,0.0,0.05])
        self.declare_parameter('dh.theta_off', [0.0]*6)

        self.joint_names = list(self.get_parameter('joint_names').value)[:6]  # FK uses first 6 joints
        self.a = np.array(self.get_parameter('dh.a').value, dtype=float)
        self.alpha = np.array(self.get_parameter('dh.alpha').value, dtype=float)
        self.d = np.array(self.get_parameter('dh.d').value, dtype=float)
        self.theta_off = np.array(self.get_parameter('dh.theta_off').value, dtype=float)

        self.sub = self.create_subscription(JointState, '/desired_joint_states', self.cb, 10)
        self.pub_pose = self.create_publisher(PoseStamped, '/endpoint_pose', 10)

    def cb(self, js: JointState):
        # Map incoming joint positions -> vector q in our joint order
        name_to_pos = {n:p for n,p in zip(js.name, js.position)}
        q = np.array([name_to_pos.get(n, 0.0) for n in self.joint_names], dtype=float)
        T = np.eye(4)
        for i in range(6):
            T = T @ dh_transform(self.a[i], self.alpha[i], self.d[i], q[i] + self.theta_off[i])

        # Fill PoseStamped (position + quaternion from rotation)
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'base_link'
        pose.pose.position.x = float(T[0,3])
        pose.pose.position.y = float(T[1,3])
        pose.pose.position.z = float(T[2,3])
        # rotation -> quaternion
        qw = math.sqrt(max(0.0, 1.0 + T[0,0]+T[1,1]+T[2,2]))/2.0
        qx = (T[2,1]-T[1,2])/(4*qw+1e-9)
        qy = (T[0,2]-T[2,0])/(4*qw+1e-9)
        qz = (T[1,0]-T[0,1])/(4*qw+1e-9)
        pose.pose.orientation.w = float(qw)
        pose.pose.orientation.x = float(qx)
        pose.pose.orientation.y = float(qy)
        pose.pose.orientation.z = float(qz)
        self.pub_pose.publish(pose)

def main():
    rclpy.init()
    node = FKNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
