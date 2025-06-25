import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    urdf_file_path = os.path.join(
        get_package_share_directory('learm_ros2'),
        'urdf',
        'learm-gazebo.urdf'
    )

    with open(urdf_file_path, 'r') as infp:
        robot_description = infp.read()

    return LaunchDescription([
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'learm_bot',
                '-topic', 'robot_description',
                '-x', '0', '-y', '0', '-z', '0.2'
            ],
            output='screen'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen'
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            output='screen'
        )
    ])
