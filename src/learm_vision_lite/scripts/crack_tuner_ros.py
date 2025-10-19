import rclpy, cv2, json
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Header, String
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

# local import (since crack_tuner.py is in the same scripts/ dir)
from crack_tuner import detect_cracks

class CrackTunerNode(Node):
    def __init__(self):
        super().__init__("crack_tuner")
        self.bridge = CvBridge()

        # tunables (can be set via --ros-args -p name:=value)
        self.declare_parameter("block_size", 31)
        self.declare_parameter("C", 7)
        self.declare_parameter("min_area", 120)

        qos = QoSProfile(depth=10)
        qos.reliability = QoSReliabilityPolicy.BEST_EFFORT
        qos.history = QoSHistoryPolicy.KEEP_LAST

        # subscribe to relative name; launch or CLI remaps to the camera topic
        self.sub = self.create_subscription(Image, "image_raw", self.cb, qos)
        self.pub_dbg  = self.create_publisher(Image, "/crack_tuner/debug_image", 10)
        self.pub_mask = self.create_publisher(Image, "/crack_tuner/mask", 10)
        self.pub_json = self.create_publisher(String, "/crack_tuner/metrics", 10)
        self.get_logger().info("CrackTuner listening on image_raw")

    def cb(self, msg: Image):
        bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        mask, pretty, metrics = detect_cracks(
            bgr,
            block_size=int(self.get_parameter("block_size").value),
            C=int(self.get_parameter("C").value),
            min_area=int(self.get_parameter("min_area").value),
        )

        hdr = Header(stamp=msg.header.stamp, frame_id=msg.header.frame_id or "camera_link")
        img_overlay = self.bridge.cv2_to_imgmsg(pretty, encoding="bgr8"); img_overlay.header = hdr
        img_mask    = self.bridge.cv2_to_imgmsg(mask,   encoding="mono8"); img_mask.header    = hdr

        self.pub_dbg.publish(img_overlay)
        self.pub_mask.publish(img_mask)
        self.pub_json.publish(String(data=json.dumps(metrics)))

def main():
    rclpy.init()
    node = CrackTunerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()