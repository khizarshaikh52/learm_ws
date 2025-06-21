from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # Path to your learm.urdf.xacro file
    xacro_file = '/home/khizar/ros2_ws/ros2_ws/src/learm_ros2/packages/learm_ros2_description/urdf/learm.urdf.xacro'

    # Process the xacro file
    doc = xacro.process_file(xacro_file)
    urdf_xml = doc.toxml()
    robot_description = {'robot_description': urdf_xml}

    return LaunchDescription([
        # Joint State Publisher with sliders
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[robot_description],
            output='screen'
        ),
        # RViz using the same config as urdf_tutorial
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', '/opt/ros/humble/share/urdf_tutorial/rviz/urdf.rviz']
        )
    ])

