from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = get_package_share_directory('learm_kinematics')
    params = os.path.join(pkg, 'params', 'hw.yaml')
    return LaunchDescription([
        Node(package='learm_kinematics',
             executable='learm_hw_bridge',
             name='learm_hw_bridge',
             parameters=[params]),
    ])
