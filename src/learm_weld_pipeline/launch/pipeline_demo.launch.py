#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Resolve package share at launch time
    default_config = PathJoinSubstitution([
        FindPackageShare('learm_weld_pipeline'),
        'config',
        'homography.yaml'
    ])

    cfg_arg = DeclareLaunchArgument(
        'config',
        default_value=default_config,
        description='Path to homography.yaml'
    )
    input_arg = DeclareLaunchArgument(
        'input_file',
        default_value='crack_pixels.csv',
        description='Path to crack pixel CSV/JSON'
    )

    return LaunchDescription([
        cfg_arg,
        input_arg,
        Node(
            package='learm_weld_pipeline',
            executable='crack_to_path',
            name='crack_to_path',
            parameters=[{'input_file': LaunchConfiguration('input_file')}]
        ),
        Node(
            package='learm_weld_pipeline',
            executable='camera_to_base',
            name='camera_to_base',
            parameters=[{'config': LaunchConfiguration('config')}]
        ),
        Node(
            package='learm_weld_pipeline',
            executable='path_to_joints',
            name='path_to_joints'
        ),
        Node(
            package='learm_weld_pipeline',
            executable='welder_control',
            name='welder_control'
        ),
    ])
