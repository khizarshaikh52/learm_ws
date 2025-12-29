#!/usr/bin/env python3
import math
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

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


class FKUrdfNode(Node):
    def __init__(self):
        super().__init__("fk_urdf_node")

        self.declare_parameter("base_link", "base_link")
        self.declare_parameter("ee_link", "wrist_link")

        self.base_link = self.get_parameter("base_link").get_parameter_value().string_value
        self.ee_link = self.get_parameter("ee_link").get_parameter_value().string_value

        robot_desc = self._get_robot_description()
        self.robot = URDF.from_xml_string(robot_desc)

        self.joint_pos = {}
        self.create_subscription(JointState, "/joint_states", self.cb_joint_states, 10)

        self.get_logger().info(f"FK node ready. base_link={self.base_link}, ee_link={self.ee_link}")

    def _get_robot_description(self) -> str:
        client = self.create_client(GetParameters, "/robot_state_publisher/get_parameters")
        if not client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError("Service /robot_state_publisher/get_parameters not available. Is robot_state_publisher running?")

        req = GetParameters.Request()
        req.names = ["robot_description"]

        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        resp = future.result()
        if resp is None or not resp.values:
            raise RuntimeError("Failed to get robot_description from robot_state_publisher")

        val = resp.values[0]
        if val.type != ParameterType.PARAMETER_STRING:
            raise RuntimeError("robot_description parameter is not a string")

        return val.string_value

    def cb_joint_states(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            self.joint_pos[name] = pos

        try:
            T = self.compute_fk(self.base_link, self.ee_link)
            p = T[:3, 3]
            self.get_logger().info(
                f"FK {self.base_link}->{self.ee_link}: x={p[0]:.4f}, y={p[1]:.4f}, z={p[2]:.4f}"
            )
        except Exception as e:
            self.get_logger().warn(f"FK failed: {e}")

    def compute_fk(self, base_link: str, ee_link: str) -> np.ndarray:
        children = {}
        for j in self.robot.joints:
            children.setdefault(j.parent, []).append(j)

        path = []

        def dfs(curr_link: str) -> bool:
            if curr_link == ee_link:
                return True
            for joint in children.get(curr_link, []):
                if dfs(joint.child):
                    path.append(joint)
                    return True
            return False

        if not dfs(base_link):
            raise RuntimeError(f"No kinematic path from {base_link} to {ee_link}")

        path.reverse()

        T = np.eye(4)
        for j in path:
            xyz = j.origin.xyz if j.origin and j.origin.xyz is not None else [0, 0, 0]
            rpy = j.origin.rpy if j.origin and j.origin.rpy is not None else [0, 0, 0]
            T_fixed = T_from_xyz_rpy(xyz, rpy)

            if j.type == "fixed":
                T_joint = np.eye(4)
            elif j.type in ("revolute", "continuous"):
                theta = float(self.joint_pos.get(j.name, 0.0))
                axis = j.axis if j.axis is not None else [0, 0, 1]
                T_joint = T_revolute(axis, theta)
            elif j.type == "prismatic":
                d = float(self.joint_pos.get(j.name, 0.0))
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
    node = FKUrdfNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
