from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    urdf_file_path = os.path.join(
        get_package_share_directory('learm_ros2'),
        'urdf',
        'learm-gazebo.urdf'  # ? update this if your file has a different name
    )

    return LaunchDescription([
        # Launch Gazebo
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),

        # Start robot_state_publisher to publish /tf from URDF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            arguments=[urdf_file_path],
            output='screen'
        ),

        # Spawn robot in Gazebo from URDF file
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'learm_robot',
                '-file', urdf_file_path
            ],
            output='screen'
        )
    ])
