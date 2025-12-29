from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command


def generate_launch_description():
    pkg_share = FindPackageShare('learm_ros2_description')

    gui = LaunchConfiguration('gui')
    use_sim_time = LaunchConfiguration('use_sim_time')

    xacro_file = PathJoinSubstitution([pkg_share, 'urdf', 'gazebo.urdf.xacro'])
    controllers_file = PathJoinSubstitution([pkg_share, 'config', 'ros2_controllers.yaml'])
    mesh_dir = pkg_share  # points to .../share/learm_ros2_description

    robot_description = ParameterValue(
        Command([
            'xacro ', xacro_file,
            ' mesh_dir:=', mesh_dir,
            ' ros2_control_params:=', controllers_file
        ]),
        value_type=str
    )

    gazebo_server = ExecuteProcess(
        cmd=['gzserver', '/opt/ros/humble/share/gazebo_ros/worlds/empty.world',
             '-slibgazebo_ros_init.so', '-slibgazebo_ros_factory.so', '-slibgazebo_ros_force_system.so'],
        output='screen'
    )

    gazebo_client = ExecuteProcess(
        cmd=['gzclient'],
        output='screen',
        condition=IfCondition(gui)
    )

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_description}]
    )

    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=['-topic', 'robot_description', '-entity', 'LeArm', '-x', '0', '-y', '0', '-z', '0.2']
    )

    # Delay controller spawners until after Gazebo + plugin are fully up
    spawn_controllers = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
                output='screen'
            ),
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['arm_controller', '--controller-manager', '/controller_manager'],
                output='screen'
            ),
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        gazebo_server,
        gazebo_client,
        rsp,
        spawn,
        spawn_controllers,
    ])
