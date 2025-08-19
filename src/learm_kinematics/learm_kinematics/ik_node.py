import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
import numpy as np
import math

DEFAULT_JOINTS = ['shoulder_pan','shoulder_lift','elbow1','elbow2','wrist_flex','wrist_roll']

def dh_transform(a, alpha, d, theta):
    ca, sa = math.cos(alpha), math.sin(alpha)
    ct, st = math.cos(theta), math.sin(theta)
    return np.array([
        [ct, -st*ca,  st*sa, a*ct],
        [st,  ct*ca, -ct*sa, a*st],
        [0.0,    sa,     ca,    d],
        [0.0,   0.0,    0.0,  1.0]
    ], dtype=float)

def fk(q, a, alpha, d, theta_off):
    T = np.eye(4)
    for i in range(6):
        T = T @ dh_transform(a[i], alpha[i], d[i], q[i] + theta_off[i])
    return T

def pose_error(T_cur, T_goal):
    # position error
    dp = T_goal[:3,3] - T_cur[:3,3]
    # rotation error via small-angle approximation using R_err = R_cur^T R_goal
    Rc = T_cur[:3,:3]; Rg = T_goal[:3,:3]
    Rerr = Rc.T @ Rg
    # vee(Rerr - Rerr^T)/2
    w = 0.5*np.array([Rerr[2,1]-Rerr[1,2], Rerr[0,2]-Rerr[2,0], Rerr[1,0]-Rerr[0,1]])
    return np.concatenate([dp, w])

def quat_to_rot(q):
    w,x,y,z = q
    R = np.array([
        [1-2*(y*y+z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
        [2*(x*y + z*w), 1-2*(x*x+z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w), 2*(y*z + x*w), 1-2*(x*x+y*y)]
    ], dtype=float)
    return R

class IKNode(Node):
    def __init__(self):
        super().__init__('ik_node')
        self.declare_parameter('joint_names', DEFAULT_JOINTS)
        self.declare_parameter('dh.a', [0.035,0.160,0.120,0.100,0.080,0.060])
        self.declare_parameter('dh.alpha', [1.5708,0.0,0.0,1.5708,-1.5708,0.0])
        self.declare_parameter('dh.d', [0.080,0.0,0.0,0.0,0.0,0.05])
        self.declare_parameter('dh.theta_off', [0.0]*6)
        self.declare_parameter('ik.max_iters', 400)
        self.declare_parameter('ik.step', 0.08)
        self.declare_parameter('ik.tol_pos', 0.005)
        self.declare_parameter('ik.tol_rot', 0.05)

        self.joint_names = list(self.get_parameter('joint_names').value)[:6]
        self.a = np.array(self.get_parameter('dh.a').value, dtype=float)
        self.alpha = np.array(self.get_parameter('dh.alpha').value, dtype=float)
        self.d = np.array(self.get_parameter('dh.d').value, dtype=float)
        self.theta_off = np.array(self.get_parameter('dh.theta_off').value, dtype=float)
        self.max_iters = int(self.get_parameter('ik.max_iters').value)
        self.step = float(self.get_parameter('ik.step').value)
        self.tol_pos = float(self.get_parameter('ik.tol_pos').value)
        self.tol_rot = float(self.get_parameter('ik.tol_rot').value)

        self.goal_T = None
        self.q = np.zeros(6)  # seed

        self.sub_goal = self.create_subscription(PoseStamped, '/endpoint_pose_goal', self.cb_goal, 10)
        self.pub_desired = self.create_publisher(JointState, '/desired_joint_states', 10)
        self.timer = self.create_timer(0.05, self.iterate)

    def cb_goal(self, msg: PoseStamped):
        # Compose goal transform
        p = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z], dtype=float)
        q = np.array([msg.pose.orientation.w, msg.pose.orientation.x,
                      msg.pose.orientation.y, msg.pose.orientation.z], dtype=float)
        R = quat_to_rot(q)
        T = np.eye(4)
        T[:3,:3] = R
        T[:3,3] = p
        self.goal_T = T

    def numeric_jacobian(self, q, eps=1e-4):
        J = np.zeros((6,6))
        T0 = fk(q, self.a, self.alpha, self.d, self.theta_off)
        for i in range(6):
            dq = np.zeros(6); dq[i] = eps
            T1 = fk(q + dq, self.a, self.alpha, self.d, self.theta_off)
            e = pose_error(T0, T1) / eps  # derivative wrt q_i
            J[:,i] = e
        return J

    def iterate(self):
        if self.goal_T is None:
            return
        Tcur = fk(self.q, self.a, self.alpha, self.d, self.theta_off)
        e = pose_error(Tcur, self.goal_T)
        pos_err = np.linalg.norm(e[:3])
        rot_err = np.linalg.norm(e[3:])
        if pos_err < self.tol_pos and rot_err < self.tol_rot:
            # publish and hold
            self.publish(self.q)
            return
        J = self.numeric_jacobian(self.q)
        # damped least squares
        lam = 1e-3
        JT = J.T
        dq = self.step * (JT @ np.linalg.solve(J @ JT + lam*np.eye(6), e))
        self.q += dq
        self.publish(self.q)

    def publish(self, q):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = q.tolist()
        self.pub_desired.publish(msg)

def main():
    rclpy.init()
    node = IKNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
