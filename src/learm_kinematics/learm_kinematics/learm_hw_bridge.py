import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math
import time

# Adafruit PCA9685 high-level ServoKit (simplifies pulse math)
try:
    from adafruit_servokit import ServoKit
except Exception as e:
    ServoKit = None

DEFAULT_JOINTS = [
    'shoulder_pan','shoulder_lift','elbow1','elbow2','wrist_flex','wrist_roll','grip_left'
]

class HardwareBridge(Node):
    def __init__(self):
        super().__init__('learm_hw_bridge')

        # ---- Parameters (override via YAML later) ----
        self.declare_parameter('joint_names', DEFAULT_JOINTS)
        self.declare_parameter('channels',   [0,1,2,3,4,5,6])   # PCA9685 channels for each joint
        self.declare_parameter('angle_min_deg', [-180,-90,-120,-120,-180,-180,0])
        self.declare_parameter('angle_max_deg', [ 180, 90, 120, 120, 180, 180,90])
        self.declare_parameter('zero_offset_deg', [0,0,0,0,0,0,0])   # trim per joint
        self.declare_parameter('invert', [False, False, False, False, False, False, False])
        self.declare_parameter('servo_min_deg', [0]*7)      # physical servo range for Servokit
        self.declare_parameter('servo_max_deg', [180]*7)
        self.declare_parameter('i2c_address', 0x40)
        self.declare_parameter('pwm_freq', 50)              # 50 Hz for servos
        self.declare_parameter('rate_hz', 50.0)             # output rate
        self.declare_parameter('slew_deg_per_s', 180.0)     # speed limit (smoothing)
        self.declare_parameter('deadband_deg', 0.5)         # avoid tiny jitter
        # ------------------------------------------------

        self.joint_names = list(self.get_parameter('joint_names').value)
        self.channels = list(self.get_parameter('channels').value)
        self.amin = [float(x) for x in self.get_parameter('angle_min_deg').value]
        self.amax = [float(x) for x in self.get_parameter('angle_max_deg').value]
        self.zero = [float(x) for x in self.get_parameter('zero_offset_deg').value]
        self.inv  = [bool(x)  for x in self.get_parameter('invert').value]
        self.smin = [int(x)   for x in self.get_parameter('servo_min_deg').value]
        self.smax = [int(x)   for x in self.get_parameter('servo_max_deg').value]
        self.addr = int(self.get_parameter('i2c_address').value)
        self.freq = int(self.get_parameter('pwm_freq').value)
        self.rate_hz = float(self.get_parameter('rate_hz').value)
        self.slew = float(self.get_parameter('slew_deg_per_s').value)
        self.deadband = float(self.get_parameter('deadband_deg').value)

        if ServoKit is None:
            self.get_logger().error("adafruit-circuitpython-servokit not found. Install with: pip3 install adafruit-circuitpython-servokit")
            raise RuntimeError("Missing ServoKit")

        # One 16-channel board on address 0x40 (default)
        try:
            self.kit = ServoKit(channels=16, address=self.addr)
            # Apply per-servo angle limits to protect mechanics
            for i, ch in enumerate(self.channels):
                self.kit.servo[ch].actuation_range = max(1, self.smax[i] - self.smin[i])  # set a range; we�ll map into it
                self.kit.servo[ch].set_pulse_width_range(500, 2500)  # typical; adjust if needed
            # Set frequency (ServoKit sets internally, but ok)
        except Exception as e:
            self.get_logger().error(f"Failed to init PCA9685/ServoKit: {e}")
            raise

        # target angles (deg) from ROS; current command we are outputting (for slew)
        self.target_deg = [0.0]*len(self.channels)
        self.current_deg = [0.0]*len(self.channels)

        self.sub = self.create_subscription(JointState, '/desired_joint_states', self.cb_joint, 10)

        self.dt = 1.0 / self.rate_hz
        self.timer = self.create_timer(self.dt, self.tick)

        self.get_logger().info("Hardware bridge ready: listening on /desired_joint_states")

    def cb_joint(self, js: JointState):
        name_to_pos = {n:p for n,p in zip(js.name, js.position)}
        # map to our order, convert to degrees
        for i, name in enumerate(self.joint_names):
            rad = float(name_to_pos.get(name, 0.0))
            deg = math.degrees(rad)

            # apply zero trim and inversion
            if self.inv[i]:
                deg = -deg
            deg += self.zero[i]

            # clamp to GUI/logic limits
            deg = max(self.amin[i], min(self.amax[i], deg))
            self.target_deg[i] = deg

    def tick(self):
        # Slew-limit toward target and write to servos
        max_step = self.slew * self.dt  # deg per tick
        for i, ch in enumerate(self.channels):
            tgt = self.target_deg[i]
            cur = self.current_deg[i]
            diff = tgt - cur
            if abs(diff) < self.deadband:
                continue
            step = max(-max_step, min(max_step, diff))
            cur += step
            self.current_deg[i] = cur

            # Map logical angle window [amin, amax] -> servo physical window [smin, smax]
            # First normalize to 0..1 within logical range
            if self.amax[i] == self.amin[i]:
                u = 0.5
            else:
                u = (cur - self.amin[i]) / (self.amax[i] - self.amin[i])
            u = max(0.0, min(1.0, u))
            servo_deg = self.smin[i] + u * (self.smax[i] - self.smin[i])

            try:
                self.kit.servo[ch].angle = servo_deg
            except Exception as e:
                self.get_logger().warn(f"Servo write failed ch{ch}: {e}")

def main():
    rclpy.init()
    node = HardwareBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
