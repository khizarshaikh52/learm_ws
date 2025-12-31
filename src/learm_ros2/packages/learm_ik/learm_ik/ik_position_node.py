#!/usr/bin/env python3
import math
import re
import numpy as np

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

from rcl_interfaces.srv import GetParameters
from rcl_interfaces.msg import ParameterType
from urdf_parser_py.urdf import URDF


def rpy_to_R(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    Rz = np.array([[cy, -sy, 0],
                   [sy,  cy, 0],
                   [0,   0,  1]])
    Ry = np.array([[cp, 0, sp],
                   [0,  1, 0],
                   [-sp, 0, cp]])
    Rx = np.array([[1, 0,  0],
                   [0, cr, -sr],
                   [0, sr, cr]])
    return Rz @ Ry @ Rx


def T_from_xyz_rpy(xyz, rpy):
    T = np.eye(4)
    T[:3, :3] = rpy_to_R(rpy[0], rpy[1], rpy[2])
    T[:3, 3] = np.array(xyz, dtype=float)
    return T


def axis_angle_R(axis, theta):
    axis = np.array(axis, dtype=float)
    n = np.linalg.norm(axis)
    if n < 1e-12:
        return np.eye(3)
    axis = axis / n
    x, y, z = axis
    c, s = math.cos(theta), math.sin(theta)
    C = 1.0 - c
    return np.array([
        [c + x*x*C,     x*y*C - z*s, x*z*C + y*s],
        [y*x*C + z*s,   c + y*y*C,   y*z*C - x*s],
        [z*x*C - y*s,   z*y*C + x*s, c + z*z*C]
    ])


def T_revolute(axis, theta):
    T = np.eye(4)
    T[:3, :3] = axis_angle_R(axis, theta)
    return T


class LeArmIKPosition(Node):
    """
    Position-only IK:
      input: target (x,y,z) in base_link frame
      output: joint angles for your 6 joints -> publish JointTrajectory
    """
    def __init__(self):
        super().__init__("learm_ik_position_node")

        # ---- Parameters ----
        self.declare_parameter("base_link", "base_link")
        self.declare_parameter("ee_link", "wrist_link")
        self.declare_parameter("joint_order", [
            "shoulder_pan", "shoulder_lift", "elbow1", "elbow2", "wrist_flex", "wrist_roll"
        ])

        self.declare_parameter("max_iters", 80)
        self.declare_parameter("pos_tol", 1e-3)          # meters
        self.declare_parameter("alpha", 0.7)             # step size
        self.declare_parameter("damping", 0.05)          # DLS lambda
        self.declare_parameter("fd_eps", 1e-4)           # finite diff rad
        self.declare_parameter("cmd_duration", 2.0)      # seconds for controller
        self.declare_parameter("clamp_to_limits", True)

        self.base_link = self.get_parameter("base_link").value
        self.ee_link = self.get_parameter("ee_link").value
        self.joint_order = list(self.get_parameter("joint_order").value)

        self.max_iters = int(self.get_parameter("max_iters").value)
        self.pos_tol = float(self.get_parameter("pos_tol").value)
        self.alpha = float(self.get_parameter("alpha").value)
        self.lmbda = float(self.get_parameter("damping").value)
        self.fd_eps = float(self.get_parameter("fd_eps").value)
        self.cmd_duration = float(self.get_parameter("cmd_duration").value)
        self.clamp_to_limits = bool(self.get_parameter("clamp_to_limits").value)

        # ---- Robot model from robot_state_publisher ----
        robot_desc = self._get_robot_description()
        # strip ros2_control block to avoid parser warning
        robot_desc = re.sub(r"<ros2_control.*?</ros2_control>", "", robot_desc, flags=re.DOTALL)
        self.robot = URDF.from_xml_string(robot_desc)

        # Precompute joint limits (if available)
        self.joint_limits = {}
        for j in self.robot.joints:
            if j.name in self.joint_order and j.limit is not None:
                self.joint_limits[j.name] = (j.limit.lower, j.limit.upper)

        # Current joint state
        self.q_current = {name: 0.0 for name in self.joint_order}

        # Publishers/subscribers
        self.create_subscription(JointState, "/joint_states", self._on_joint_states, 20)
        self.create_subscription(PointStamped, "/ik/target_point", self._on_target_point, 10)

        self.cmd_pub = self.create_publisher(JointTrajectory, "/arm_controller/joint_trajectory", 10)

        self.get_logger().info(
            f"IK Position node ready. base_link={self.base_link}, ee_link={self.ee_link}\n"
            f"Send target on /ik/target_point (geometry_msgs/PointStamped) in base frame."
        )

    def _get_robot_description(self) -> str:
        client = self.create_client(GetParameters, "/robot_state_publisher/get_parameters")
        if not client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError("Service /robot_state_publisher/get_parameters not available")

        req = GetParameters.Request()
        req.names = ["robot_description"]
        fut = client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)

        resp = fut.result()
        if resp is None or not resp.values:
            raise RuntimeError("Failed to get robot_description")

        val = resp.values[0]
        if val.type != ParameterType.PARAMETER_STRING:
            raise RuntimeError("robot_description parameter is not a string")

        return val.string_value

    def _on_joint_states(self, msg: JointState):
        name_to_pos = dict(zip(msg.name, msg.position))
        for jn in self.joint_order:
            if jn in name_to_pos:
                self.q_current[jn] = float(name_to_pos[jn])

    def _on_target_point(self, msg: PointStamped):
        # Expect target in base frame
        if msg.header.frame_id and msg.header.frame_id != self.base_link:
            self.get_logger().warn(
                f"Target frame_id='{msg.header.frame_id}' != base_link='{self.base_link}'. "
                f"Send target in {self.base_link} for now."
            )
            return

        target = np.array([msg.point.x, msg.point.y, msg.point.z], dtype=float)
        q0 = np.array([self.q_current[j] for j in self.joint_order], dtype=float)

        q_sol, ok, err = self.solve_ik_position(q0, target)

        if not ok:
            self.get_logger().warn(f"IK did not converge. final pos error={err:.6f} m")
        else:
            self.get_logger().info(f"IK converged. final pos error={err:.6f} m")

        self.publish_trajectory(q_sol)

    def publish_trajectory(self, q: np.ndarray):
        jt = JointTrajectory()
        jt.joint_names = list(self.joint_order)

        pt = JointTrajectoryPoint()
        pt.positions = [float(x) for x in q]
        pt.time_from_start = Duration(sec=int(self.cmd_duration), nanosec=int((self.cmd_duration % 1.0) * 1e9))

        jt.points = [pt]
        self.cmd_pub.publish(jt)

    def solve_ik_position(self, q0: np.ndarray, p_des: np.ndarray):
        q = q0.copy()
        for _ in range(self.max_iters):
            T = self.fk(q)
            p = T[:3, 3]
            e = p_des - p  # 3x1

            err = float(np.linalg.norm(e))
            if err < self.pos_tol:
                return q, True, err

            J = self.numeric_jacobian_position(q)  # 3 x N

            # Damped Least Squares: dq = J^T (J J^T + λ^2 I)^-1 e
            JJt = J @ J.T
            A = JJt + (self.lmbda ** 2) * np.eye(3)
            dq = J.T @ np.linalg.solve(A, e)

            q = q + self.alpha * dq

            if self.clamp_to_limits:
                q = self.clamp(q)

        # final
        T = self.fk(q)
        p = T[:3, 3]
        err = float(np.linalg.norm(p_des - p))
        return q, False, err

    def clamp(self, q: np.ndarray) -> np.ndarray:
        qc = q.copy()
        for i, name in enumerate(self.joint_order):
            if name in self.joint_limits:
                lo, hi = self.joint_limits[name]
                qc[i] = float(np.clip(qc[i], lo, hi))
        return qc

    def numeric_jacobian_position(self, q: np.ndarray) -> np.ndarray:
        # Finite difference on FK position
        p0 = self.fk(q)[:3, 3]
        J = np.zeros((3, len(q)))
        for i in range(len(q)):
            q2 = q.copy()
            q2[i] += self.fd_eps
            p1 = self.fk(q2)[:3, 3]
            J[:, i] = (p1 - p0) / self.fd_eps
        return J

    def fk(self, q: np.ndarray) -> np.ndarray:
        # Build parent->joints map
        children = {}
        for j in self.robot.joints:
            children.setdefault(j.parent, []).append(j)

        # Find chain base->ee (DFS)
        path = []

        def dfs(curr):
            if curr == self.ee_link:
                return True
            for joint in children.get(curr, []):
                if dfs(joint.child):
                    path.append(joint)
                    return True
            return False

        if not dfs(self.base_link):
            raise RuntimeError(f"No kinematic path from {self.base_link} to {self.ee_link}")

        path.reverse()

        # joint name -> commanded angle
        q_map = {name: float(val) for name, val in zip(self.joint_order, q)}

        T = np.eye(4)
        for j in path:
            xyz = j.origin.xyz if j.origin and j.origin.xyz is not None else [0, 0, 0]
            rpy = j.origin.rpy if j.origin and j.origin.rpy is not None else [0, 0, 0]
            T_fixed = T_from_xyz_rpy(xyz, rpy)

            if j.type == "fixed":
                T_joint = np.eye(4)
            elif j.type in ("revolute", "continuous"):
                theta = q_map.get(j.name, 0.0)
                axis = j.axis if j.axis is not None else [0, 0, 1]
                T_joint = T_revolute(axis, theta)
            elif j.type == "prismatic":
                d = q_map.get(j.name, 0.0)
                axis = np.array(j.axis if j.axis is not None else [0, 0, 1], dtype=float)
                axis = axis / (np.linalg.norm(axis) + 1e-12)
                T_joint = np.eye(4)
                T_joint[:3, 3] = axis * d
            else:
                raise RuntimeError(f"Unsupported joint type: {j.type}")

            T = T @ T_fixed @ T_joint

        return T


def main():
    rclpy.init()
    node = LeArmIKPosition()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
