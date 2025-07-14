#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package share directory of your robot description
    pkg_share = get_package_share_directory('learm_ros2_description')
    xacro_file = os.path.join(pkg_share, 'urdf', 'gazebo.urdf.xacro')

    # Include the standard Gazebo launch file
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            )
        ),
        launch_arguments={'verbose': 'true'}.items()
    )

    # Robot state publisher to publish TFs using the URDF from xacro
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': Command(['xacro ', xacro_file])
        }]
    )

    # Spawn the robot entity in Gazebo using the generated robot_description
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'LeArm'
        ],
        output='screen'
    )

    # Build the launch description
    ld = LaunchDescription()
    ld.add_action(gazebo_launch)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(spawn_entity_node)
    return ld
