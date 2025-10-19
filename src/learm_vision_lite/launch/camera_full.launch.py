from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # USB camera (publishes /image_raw)
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
            }],
        ),

        # Your crack tuner node
        # NOTE: your script likely subscribes to "/usb_cam/image_raw".
        # We remap that exact name to the camera's /image_raw here.
        Node(
            package="learm_vision_lite",
            executable="crack_tuner_ros.py",
            name="crack_tuner",
            output="screen",
            remappings=[
                ("/usb_cam/image_raw", "/image_raw"),  # works even if it's absolute in your code
            ],
        ),

        # Viewer for processed output
        Node(
            package="image_view",
            executable="image_view",
            name="viewer",
            output="screen",
            remappings=[("image", "/crack_tuner/debug_image")],
        ),
    ])