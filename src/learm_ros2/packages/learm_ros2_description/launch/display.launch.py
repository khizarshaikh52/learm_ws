#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # 1. Locate and process your XACRO
    xacro_file = os.path.join(
        os.getenv('HOME'),
        'learm_ws/src/learm_ros2/packages/learm_ros2_description/urdf',
        'learm.urdf.xacro'
    )
    doc = xacro.process_file(xacro_file)
    urdf_xml = doc.toxml()
    robot_description = {'robot_description': urdf_xml}

    return LaunchDescription([
        # 2. Joint State Publisher (GUI sliders)
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),

        # 3. Robot State Publisher (broadcast URDF→TF)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[robot_description],
            output='screen'
        ),

        # 4. Static transform: world → base_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            output='screen',
            arguments=[
                '0','0','0',    # x y z
                '0','0','0',    # yaw pitch roll
                'world',        # parent frame
                'base_link'     # child frame (adjust if your root link is named differently)
            ]
        ),

        # 5. RViz2 with a preset configuration
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=[
                '-d', '/opt/ros/humble/share/urdf_tutorial/rviz/urdf.rviz'
            ]
        ),
    ])

if __name__ == '__main__':
    generate_launch_description()
