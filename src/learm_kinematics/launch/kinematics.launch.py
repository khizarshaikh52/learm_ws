from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = get_package_share_directory('learm_kinematics')
    params = os.path.join(pkg, 'params', 'dh.yaml')

    return LaunchDescription([
        Node(package='learm_kinematics', executable='ik_node',
             name='ik_node', parameters=[params]),
        Node(package='learm_kinematics', executable='fk_node',
             name='fk_node', parameters=[params]),
    ])
