#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import numpy as np
from rcl_interfaces.msg import ParameterDescriptor
import yaml
from .path_tools import douglas_peucker, resample_by_step

class CameraToBase(Node):
    def __init__(self):
        super().__init__('camera_to_base')
        self.declare_parameter('config', '', descriptor=ParameterDescriptor(
            description='YAML with homography H, Z_work_mm, tool_offset_mm, path params'))
        self.declare_parameter('in_topic', '/crack_path/camera')
        self.declare_parameter('out_topic', '/crack_path/base')
        self.declare_parameter('in_frame', 'camera_color_optical_frame')
        self.declare_parameter('out_frame', 'base_link')

        self.params = {}
        cfg = self.get_parameter('config').get_parameter_value().string_value
        if cfg:
            with open(cfg, 'r') as f:
                self.params = yaml.safe_load(f)

        self.H = np.array(self.params.get('homography', {}).get('H',
                      [1,0,0, 0,1,0, 0,0,1]), dtype=float).reshape(3, 3)
        self.Z = float(self.params.get('Z_work_mm', 0.0))
        path_cfg = self.params.get('path', {})
        self.step = float(path_cfg.get('step_mm', 2.0))
        self.tol  = float(path_cfg.get('corner_tol_mm', 0.8))

        self.in_frame = self.get_parameter('in_frame').value
        self.out_frame = self.get_parameter('out_frame').value
        self.out_topic = self.get_parameter('out_topic').value
        self.in_topic  = self.get_parameter('in_topic').value

        self.pub = self.create_publisher(Path, self.out_topic, 10)
        self.create_subscription(Path, self.in_topic, self.cb, 10)

    def cb(self, msg: Path):
        if not msg.poses:
            return

        uv = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        uv = douglas_peucker(uv, self.tol)
        uv = resample_by_step(uv, self.step)

        xy = []
        for (u, v) in uv:
            p = np.array([u, v, 1.0])
            q = self.H @ p
            q = q / (q[2] if q[2] != 0 else 1.0)
            xy.append((float(q[0]), float(q[1])))

        out = Path()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.out_frame
        for (x, y) in xy:
            ps = PoseStamped()
            ps.header = out.header
            ps.pose.position.x = x / 1000.0  # mm -> m
            ps.pose.position.y = y / 1000.0
            ps.pose.position.z = self.Z / 1000.0
            ps.pose.orientation.w = 1.0
            out.poses.append(ps)

        self.pub.publish(out)
        self.get_logger().info(f'Published base path with {len(out.poses)} poses at Z={self.Z}mm')

def main():
    rclpy.init()
    rclpy.spin(CameraToBase())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
