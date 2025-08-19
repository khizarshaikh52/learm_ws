
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import tkinter as tk
import traceback
from sensor_msgs.msg import JointState

class ManualServoHome(Node):
    def __init__(self):
        super().__init__('manual_servo_home_calibration')
        self.pub = self.create_publisher(JointState, '/servo_commands', 1)

        self.cmd = JointState()
        self.cmd.name = [f'cmd{str(i).zfill(2)}' for i in range(7)]
        self.cmd.position = [0.0]*7

        self.sliders = [0]*7
        self._build_gui()

        # keep publishing to keep servos alive
        self.timer = self.create_timer(0.1, self._publish)

    def _build_gui(self):
        self.root = tk.Tk()
        self.root.title("7-DOF Servo Home Calibration (ROS 2)")

        MIN_US, MAX_US = 500, 2500
        for i in range(7):
            slider = tk.Scale(
                self.root,
                from_=MIN_US,
                to=MAX_US,
                orient=tk.HORIZONTAL,
                length=600,
                label=f"Servo {i} �s",
                command=lambda v, i=i: self._on_slide(i, int(float(v)))
            )
            slider.set((MIN_US+MAX_US)//2)
            slider.pack(pady=4)

        btn = tk.Button(
            self.root,
            text="Record Home Position",
            command=self._record
        )
        btn.pack(pady=8)

        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    def _on_slide(self, idx, val):
        self.sliders[idx] = val
        self.cmd.position = list(self.sliders)
        # header stamp left fresh in timer callback

    def _publish(self):
        self.cmd.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.cmd)

    def _record(self):
        print("Home pulses (�s):", self.sliders)

    def _on_close(self):
        # zero servos then exit
        self.cmd.position = [0]*7
        self._publish()
        self.root.destroy()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = ManualServoHome()
    try:
        node.root.mainloop()
    except Exception:
        traceback.print_exc()
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
