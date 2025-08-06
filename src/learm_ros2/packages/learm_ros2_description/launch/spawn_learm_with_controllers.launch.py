#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare('learm_ros2_description').find('learm_ros2_description')

    # 1) Robot description from your gazebo.urdf.xacro
    xacro_file = os.path.join(pkg_share, 'urdf', 'gazebo.urdf.xacro')
    robot_description = {'robot_description': Command(['xacro ', xacro_file])}

    # 2) ros2_control config
    controllers_yaml = os.path.join(pkg_share, 'config', 'learm_controllers.yaml')

    # 3) Launch Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch', 'gazebo.launch.py'
            ])
        )
    )

    # 4) Robot State Publisher
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # 5) ros2_control node
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[robot_description, controllers_yaml]
    )

    # 6) Spawn the robot into Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'learm_robot'],
        output='screen'
    )

    # 7) Spawn each controller
    controller_names = [
        'joint_state_broadcaster',
        'shoulder_pan_controller',
        'shoulder_lift_controller',
        'elbow1_controller',
        'elbow2_controller',
        'wrist_flex_controller',
        'wrist_roll_controller',
        'grip_left_controller'
    ]
    spawners = [
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=[name, '--controller-manager', '/controller_manager'],
            output='screen'
        )
        for name in controller_names
    ]

    # 8) Once-only publish 90° (1.5708 rad) to each position controller
    targets = {
        'shoulder_pan_controller':  1.5708,
        'shoulder_lift_controller': 1.5708,
        'elbow1_controller':        1.5708,
        'elbow2_controller':        1.5708,
        'wrist_flex_controller':    1.5708,
        'wrist_roll_controller':    1.5708,
        'grip_left_controller':     1.5708,
    }
    hold_cmds = []
    for name, rad in targets.items():
        hold_cmds.append(
            ExecuteProcess(
                cmd=[
                    'ros2', 'topic', 'pub', '--once',
                    f'/{name}/command',
                    'std_msgs/msg/Float64',
                    f'{{data: {rad}}}'
                ],
                output='screen'
            )
        )

    return LaunchDescription([
        gazebo,
        rsp_node,
        control_node,
        spawn_entity,
        *spawners,
        *hold_cmds,
    ])
