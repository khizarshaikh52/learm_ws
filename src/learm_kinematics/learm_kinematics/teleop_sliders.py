import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from rclpy.parameter import Parameter
import threading
import time
import math

# Tkinter GUI for sliders (optional headless fallback)
try:
    import tkinter as tk
    TK_OK = True
except Exception:
    TK_OK = False

DEFAULT_JOINTS = ['shoulder_pan','shoulder_lift','elbow1','elbow2','wrist_flex','wrist_roll','grip_left']

class TeleopSliders(Node):
    def __init__(self):
        super().__init__('teleop_sliders')
        self.declare_parameter('joint_names', DEFAULT_JOINTS)
        self.declare_parameter('limits_deg.min', [-180,-90,-120,-120,-180,-180,0])
        self.declare_parameter('limits_deg.max', [ 180, 90, 120, 120, 180, 180,90])
        self.joint_names = list(self.get_parameter('joint_names').value)
        self.lim_min = list(self.get_parameter('limits_deg.min').value)
        self.lim_max = list(self.get_parameter('limits_deg.max').value)

        self.pub = self.create_publisher(JointState, '/desired_joint_states', 10)

        self.deg_positions = [0.0]*len(self.joint_names)
        self.headless = not TK_OK
        if self.headless:
            self.get_logger().warn('No display for Tkinter; running headless. Publishing zeros. Ctrl+C to stop.')
            self.timer = self.create_timer(0.05, self.publish_state)
        else:
            self.start_gui_thread()

    def start_gui_thread(self):
        def run_gui():
            root = tk.Tk()
            root.title("LeArm Teleop Sliders")
            scales = []
            for i, name in enumerate(self.joint_names):
                frame = tk.Frame(root)
                frame.pack(fill='x', padx=8, pady=4)
                tk.Label(frame, text=name, width=16, anchor='w').pack(side='left')
                s = tk.Scale(frame, from_=self.lim_min[i], to=self.lim_max[i],
                             orient='horizontal', length=400, resolution=1)
                s.set(0)
                s.pack(side='left', expand=True, fill='x')
                scales.append(s)

            def tick():
                for i, s in enumerate(scales):
                    self.deg_positions[i] = float(s.get())
                self.publish_state()
                root.after(50, tick)

            root.after(50, tick)
            root.mainloop()

        th = threading.Thread(target=run_gui, daemon=True)
        th.start()
        # Spin a ROS2 timer to keep node alive
        self.timer = self.create_timer(0.5, lambda: None)

    def publish_state(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        # convert deg to rad
        msg.position = [math.radians(d) for d in self.deg_positions]
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = TeleopSliders()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
