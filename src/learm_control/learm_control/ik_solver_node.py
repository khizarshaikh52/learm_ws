#!/usr/bin/env python3
# IK for a 7-DOF arm (LeArm-style) using Damped Least Squares
# rclpy node: subscribes to /ee_target (PoseStamped) and publishes /learm/joint_commands (JointState)

import math
import numpy as np
from dataclasses import dataclass
from typing import List, Tuple

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration

# Optional TF transform (world -> robot base)
from tf2_ros import Buffer, TransformListener
import tf_transformations as tft
from geometry_msgs.msg import TransformStamped


# ---------------------------
# 1) CONFIG: EDIT THESE
# ---------------------------

# TODO: put your actual joint names (exclude gripper finger mimic joints)
JOINT_NAMES = [
    "shoulder_pan", "shoulder_lift", "elbow1",
    "elbow2", "wrist_flex", "wrist_roll", "wrist_yaw"
]

# TODO: DH parameters for your robot (modified or classic�this uses classic DH):
# For each joint i: a_i, alpha_i (rad), d_i, theta_offset_i (rad)
# Angles in radians, lengths in meters.
# Put your real numbers here!
DH_TABLE = [
    #  a,      alpha,              d,   theta_offset
    [ 0.000,   math.pi/2,         0.10,  0.0],   # J1
    [ 0.250,   0.0,               0.00,  0.0],   # J2
    [ 0.250,   0.0,               0.00,  0.0],   # J3
    [ 0.050,   math.pi/2,         0.00,  0.0],   # J4
    [ 0.000,  -math.pi/2,         0.15,  0.0],   # J5
    [ 0.000,   math.pi/2,         0.00,  0.0],   # J6
    [ 0.000,   0.0,               0.10,  0.0],   # J7 (tool link offset)
]

# TODO: joint hard limits (rad)
JOINT_LIMITS = [
    (-math.pi,  math.pi),   # J1
    (-math.pi/2, math.pi/2),# J2 (example)
    (-math.pi,  math.pi),   # J3
    (-math.pi,  math.pi),   # J4
    (-math.pi,  math.pi),   # J5
    (-math.pi,  math.pi),   # J6
    (-math.pi,  math.pi),   # J7
]

# Solver gains
MAX_ITERS           = 300
POS_TOL             = 1e-3      # meters
ORI_TOL             = 2.0*math.pi/180.0  # rad (~2 deg)
DAMPING_LAMBDA      = 0.02
STEP_CLIP           = 5.0*math.pi/180.0  # rad per iter (safety)
WEIGHT_ORI          = 0.5               # weight orientation vs position
USE_ORIENTATION     = True              # set False if you want position-only IK

# Frames
TARGET_FRAME        = "world"           # incoming PoseStamped frame_id
BASE_FRAME          = "base_link"       # your robot base frame

# Initial home (reasonable seed)
HOME_Q = np.array([0.0, -0.4, 0.8, -0.6, 0.0, 0.0, 0.0], dtype=float)


# ---------------------------
# 2) MATH HELPERS
# ---------------------------

def rot_x(alpha):
    ca, sa = math.cos(alpha), math.sin(alpha)
    return np.array([[1,0,0,0],[0,ca,-sa,0],[0,sa,ca,0],[0,0,0,1]], dtype=float)

def rot_z(theta):
    ct, st = math.cos(theta), math.sin(theta)
    return np.array([[ct,-st,0,0],[st,ct,0,0],[0,0,1,0],[0,0,0,1]], dtype=float)

def trans_x(a):
    return np.array([[1,0,0,a],[0,1,0,0],[0,0,1,0],[0,0,0,1]], dtype=float)

def trans_z(d):
    return np.array([[1,0,0,0],[0,1,0,0],[0,0,1,d],[0,0,0,1]], dtype=float)

def dh_T(a, alpha, d, theta):
    # Classic DH: T = RotZ(theta) * TransZ(d) * TransX(a) * RotX(alpha)
    return rot_z(theta) @ trans_z(d) @ trans_x(a) @ rot_x(alpha)

def clamp(q, limits):
    out = []
    for qi, (lo, hi) in zip(q, limits):
        out.append(min(max(qi, lo), hi))
    return np.array(out, dtype=float)

def skew(v):
    x,y,z = v
    return np.array([[0,-z,y],[z,0,-x],[-y,x,0]], dtype=float)

def rot_to_axis_angle(R):
    # Robust axis-angle from rotation matrix
    angle = math.acos(max(-1.0, min(1.0, (np.trace(R) - 1.0)/2.0)))
    if abs(angle) < 1e-9:
        return np.array([0.0,0.0,0.0]), 0.0
    rx = (R[2,1] - R[1,2])/(2*math.sin(angle))
    ry = (R[0,2] - R[2,0])/(2*math.sin(angle))
    rz = (R[1,0] - R[0,1])/(2*math.sin(angle))
    axis = np.array([rx, ry, rz], dtype=float)
    axis = axis / (np.linalg.norm(axis) + 1e-12)
    return axis, angle

# ---------------------------
# 3) KINEMATICS
# ---------------------------

@dataclass
class ChainFK:
    a: List[float]
    alpha: List[float]
    d: List[float]
    theta_off: List[float]

    def fk_all(self, q: np.ndarray) -> List[np.ndarray]:
        """Return list of transforms from base to each joint (and final tool). length = dof+1"""
        T = np.eye(4, dtype=float)
        T_list = [T.copy()]
        for (ai, al, di, to), qi in zip(zip(self.a, self.alpha, self.d, self.theta_off), q):
            T = T @ dh_T(ai, al, di, qi + to)
            T_list.append(T.copy())
        return T_list

    def jacobian(self, q: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Geometric Jacobian (6xn): top 3 rows linear, bottom 3 rows angular.
           Also returns p_ee, R_ee"""
        Ts = self.fk_all(q)
        T_ee = Ts[-1]
        p_ee = T_ee[:3, 3]
        R_ee = T_ee[:3, :3]

        n = len(q)
        J = np.zeros((6, n), dtype=float)

        for i in range(n):
            z_i = Ts[i][:3, 2]        # z-axis of frame i
            p_i = Ts[i][:3, 3]        # origin of frame i
            J[:3, i] = np.cross(z_i, (p_ee - p_i))
            J[3:, i] = z_i            # revolute joint
        return J, p_ee, R_ee

    def fk_pose(self, q: np.ndarray):
        T = self.fk_all(q)[-1]
        return T[:3, 3], T[:3, :3]


fk = ChainFK(
    a=[row[0] for row in DH_TABLE],
    alpha=[row[1] for row in DH_TABLE],
    d=[row[2] for row in DH_TABLE],
    theta_off=[row[3] for row in DH_TABLE],
)


# ---------------------------
# 4) IK SOLVER (DLS)
# ---------------------------

def solve_ik(q_seed: np.ndarray,
             p_target: np.ndarray,
             R_target: np.ndarray,
             use_orientation: bool = True) -> Tuple[bool, np.ndarray, float, float]:
    """Return (ok, q_sol, pos_err, ori_err)"""
    q = clamp(q_seed.copy(), JOINT_LIMITS)

    for _ in range(MAX_ITERS):
        J, p, R = fk.jacobian(q)

        # Position error
        e_pos = p_target - p
        pos_err = np.linalg.norm(e_pos)

        if use_orientation:
            R_err = R.T @ R_target                 # rotation taking current -> target
            axis, ang = rot_to_axis_angle(R_err)
            e_ori = axis * ang                     # small-angle approx vector
            ori_err = abs(ang)
        else:
            e_ori = np.zeros(3, dtype=float)
            ori_err = 0.0

        if (pos_err < POS_TOL) and ((not use_orientation) or (ori_err < ORI_TOL)):
            return True, clamp(q, JOINT_LIMITS), pos_err, ori_err

        # Stack error (6x1)
        e6 = np.hstack([e_pos, WEIGHT_ORI * e_ori])

        # Weight orientation rows
        W = np.eye(6)
        W[3:, 3:] *= WEIGHT_ORI

        JW = W @ J
        JJt = JW @ JW.T
        lam2I = (DAMPING_LAMBDA ** 2) * np.eye(6)
        dq = JW.T @ np.linalg.solve(JJt + lam2I, W @ e6)

        # Step limiting
        dq = np.clip(dq, -STEP_CLIP, STEP_CLIP)
        q = clamp(q + dq, JOINT_LIMITS)

    # failed
    J, p, R = fk.jacobian(q)
    pos_err = np.linalg.norm(p_target - p)
    if use_orientation:
        R_err = R.T @ R_target
        _, ang = rot_to_axis_angle(R_err)
        ori_err = abs(ang)
    else:
        ori_err = 0.0
    return False, q, pos_err, ori_err


# ---------------------------
# 5) ROS 2 NODE
# ---------------------------

class IKSolverNode(Node):
    def __init__(self):
        super().__init__("ik_solver_node")

        # Publishers / Subscribers
        self.pub_cmd = self.create_publisher(JointState, "/learm/joint_commands", 10)
        self.sub_target = self.create_subscription(PoseStamped, "/ee_target", self.on_target, 10)

        # TF buffer to convert target to base frame if needed
        self.tf_buffer = Buffer(cache_time=Duration(sec=5))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # last solution seed
        self.q_seed = HOME_Q.copy()

        self.get_logger().info("IK solver ready. Publish PoseStamped to /ee_target.")

    def transform_to_base(self, pose_st: PoseStamped) -> PoseStamped:
        if pose_st.header.frame_id == BASE_FRAME:
            return pose_st

        try:
            tf: TransformStamped = self.tf_buffer.lookup_transform(
                BASE_FRAME, pose_st.header.frame_id,
                rclpy.time.Time(),  # latest
                timeout=rclpy.duration.Duration(seconds=0.2).to_msg()
            )
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed ({pose_st.header.frame_id} -> {BASE_FRAME}): {e}. Assuming already in {BASE_FRAME}.")
            return pose_st  # fall back

        # Apply transform to pose
        trans = tf.transform.translation
        rot = tf.transform.rotation
        T = tft.compose_matrix(translate=[trans.x, trans.y, trans.z],
                               angles=tft.euler_from_quaternion([rot.x, rot.y, rot.z, rot.w]))
        p = np.array([pose_st.pose.position.x, pose_st.pose.position.y, pose_st.pose.position.z, 1.0])
        Rq = [pose_st.pose.orientation.x, pose_st.pose.orientation.y, pose_st.pose.orientation.z, pose_st.pose.orientation.w]
        R_pose = tft.quaternion_matrix(Rq)

        T_out = T @ R_pose
        out = PoseStamped()
        out.header.frame_id = BASE_FRAME
        out.header.stamp = self.get_clock().now().to_msg()
        out.pose.position.x, out.pose.position.y, out.pose.position.z = T_out[0,3], T_out[1,3], T_out[2,3]
        q = tft.quaternion_from_matrix(T_out)
        out.pose.orientation.x, out.pose.orientation.y, out.pose.orientation.z, out.pose.orientation.w = q[0], q[1], q[2], q[3]
        return out

    def on_target(self, msg: PoseStamped):
        # transform to base frame if needed
        tgt = self.transform_to_base(msg)

        # build target position/orientation
        p_tgt = np.array([tgt.pose.position.x, tgt.pose.position.y, tgt.pose.position.z], dtype=float)
        q_tgt = [tgt.pose.orientation.x, tgt.pose.orientation.y, tgt.pose.orientation.z, tgt.pose.orientation.w]
        R_tgt = tft.quaternion_matrix(q_tgt)[:3, :3]

        # Solve IK
        ok, q_sol, pos_err, ori_err = solve_ik(self.q_seed, p_tgt, R_tgt, use_orientation=USE_ORIENTATION)
        if not ok:
            self.get_logger().warn(f"IK: no exact solution. sending best effort (pos_err={pos_err:.4f} m, ori_err={ori_err*180/math.pi:.2f} deg)")

        # Remember for next call
        self.q_seed = q_sol.copy()

        # Publish JointState (one-shot)
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = JOINT_NAMES
        js.position = q_sol.tolist()
        self.pub_cmd.publish(js)
        self.get_logger().info(f"Sent joint command. pos_err={pos_err:.4f} m, ori_err={ori_err*180/math.pi:.2f} deg")


def main():
    rclpy.init()
    node = IKSolverNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

