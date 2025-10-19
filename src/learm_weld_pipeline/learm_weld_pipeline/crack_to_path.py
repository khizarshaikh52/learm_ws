#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import json, csv, os

class CrackToPath(Node):
    def __init__(self):
        super().__init__('crack_to_path')
        self.declare_parameter('input_file', '')
        self.declare_parameter('frame_id', 'camera_color_optical_frame')
        self.pub = self.create_publisher(Path, '/crack_path/camera', 10)
        self.timer = self.create_timer(0.5, self.tick)
        self.sent = False

    def load_points(self, path):
        ext = os.path.splitext(path)[1].lower()
        pts = []
        if ext == '.json':
            with open(path, 'r') as f:
                data = json.load(f)
            arr = data.get('points', data)
            pts = [(float(u), float(v)) for u, v in arr]
        else:
            with open(path, 'r') as f:
                reader = csv.DictReader(f)
                if reader.fieldnames and 'u' in reader.fieldnames and 'v' in reader.fieldnames:
                    pts = [(float(r['u']), float(r['v'])) for r in reader]
                else:
                    f.seek(0)
                    reader2 = csv.reader(f)
                    for r in reader2:
                        if len(r) >= 2:
                            pts.append((float(r[0]), float(r[1])))
        return pts

    def tick(self):
        if self.sent:
            return
        infile = self.get_parameter('input_file').get_parameter_value().string_value
        frame = self.get_parameter('frame_id').get_parameter_value().string_value
        if not infile or not os.path.exists(infile):
            self.get_logger().warn('Waiting for input_file..')
            return
        pts = self.load_points(infile)
        path = Path()
        path.header.frame_id = frame
        t = self.get_clock().now().to_msg()
        path.header.stamp = t
        for (u, v) in pts:
            ps = PoseStamped()
            ps.header.frame_id = frame
            ps.header.stamp = t
            ps.pose.position.x = float(u)
            ps.pose.position.y = float(v)
            ps.pose.position.z = 0.0
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self.pub.publish(path)
        self.get_logger().info(f'Published {len(path.poses)} points from {infile} -> /crack_path/camera')
        self.sent = True

def main():
    rclpy.init()
    rclpy.spin(CrackToPath())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
