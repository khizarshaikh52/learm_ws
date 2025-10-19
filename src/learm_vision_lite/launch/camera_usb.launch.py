from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="v4l2_camera",
            executable="v4l2_camera_node",
            name="usb_cam",
            output="screen",
            parameters=[{
                "video_device": "/dev/video0",
                "image_size": [640, 480],
                "frame_rate": 30,
                "output_encoding": "rgb8",
                "camera_frame_id": "camera_link",
                # "camera_info_url": "file:///ABS/PATH/uvc_camera.yaml",  # add after calibration
            }],
        ),
        Node(
            package="image_transport",
            executable="republish",
            name="republish_compressed",
            arguments=["raw", "compressed"],
            remappings=[("in", "/usb_cam/image_raw"), ("out", "/usb_cam/image_raw/compressed")],
            output="screen",
        ),
        Node(
            package="learm_vision_lite",
            executable="crack_tuner_ros.py",   # installed as a script
            name="crack_tuner",
            output="screen",
        ),
    ])