#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from std_msgs.msg import Float64MultiArray, Bool
import time
from .simple_ik import ik_planar_xyz

class PathToJoints(Node):
    def __init__(self):
        super().__init__('path_to_joints')
        self.declare_parameter('joints_topic', '/learm/command_deg')
        self.declare_parameter('welder_topic', '/welder_on')
        self.declare_parameter('speed_deg_s', 20.0)
        self.declare_parameter('dwell_start_s', 0.3)
        self.declare_parameter('dwell_end_s', 0.3)

        self.create_subscription(Path, '/crack_path/base', self.cb, 10)
        self.pub_deg = self.create_publisher(Float64MultiArray, self.get_parameter('joints_topic').value, 10)
        self.pub_weld = self.create_publisher(Bool, self.get_parameter('welder_topic').value, 10)

    def cb(self, msg: Path):
        pts = msg.poses
        if not pts:
            return

        spd = float(self.get_parameter('speed_deg_s').value)
        dwell_s = float(self.get_parameter('dwell_start_s').value)
        dwell_e = float(self.get_parameter('dwell_end_s').value)

        first = pts[0].pose.position
        q0 = ik_planar_xyz((first.x*1000.0, first.y*1000.0, first.z*1000.0))
        self.stream_deg(q0, spd)
        time.sleep(dwell_s)

        self.pub_weld.publish(Bool(data=True))

        for p in pts:
            xyz_mm = (p.pose.position.x*1000.0, p.pose.position.y*1000.0, p.pose.position.z*1000.0)
            q = ik_planar_xyz(xyz_mm)
            self.stream_deg(q, spd)

        time.sleep(dwell_e)
        self.pub_weld.publish(Bool(data=False))
        self.get_logger().info(f'Executed {len(pts)} weld points')

    def stream_deg(self, q_deg, speed_deg_s):
        msg = Float64MultiArray()
        msg.data = q_deg
        self.pub_deg.publish(msg)
        time.sleep(0.03 + max(abs(a) for a in q_deg)/speed_deg_s*0.01)

def main():
    rclpy.init()
    rclpy.spin(PathToJoints())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
