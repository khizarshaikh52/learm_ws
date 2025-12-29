import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory("learm_ros2_description")

    xacro_file = os.path.join(pkg_share, "urdf", "gazebo.urdf.xacro")
    controllers_file = os.path.join(pkg_share, "config", "ros2_controllers.yaml")

    world_file = os.path.join(
        get_package_share_directory("gazebo_ros"),
        "worlds",
        "empty.world",
    )

    gui = LaunchConfiguration("gui")

    # robot_description for robot_state_publisher
    robot_description = ParameterValue(
        Command([
            "xacro", " ", xacro_file,
            " ", "mesh_dir:=", pkg_share,
            " ", "ros2_control_params:=", controllers_file,
        ]),
        value_type=str,
    )

    gzserver = ExecuteProcess(
        cmd=[
            "gzserver",
            "--verbose",
            world_file,
            "-s", "libgazebo_ros_init.so",
            "-s", "libgazebo_ros_factory.so",
        ],
        output="screen",
    )

    gzclient = ExecuteProcess(
        cmd=["gzclient"],
        output="screen",
        condition=IfCondition(gui),
    )

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
    )

    # --- IMPORTANT ---
    # spawn_entity.py DOES NOT support -param.
    # It REQUIRES one of: -file / -topic / -database / -stdin.
    # We generate a URDF file and spawn with -file.
    urdf_out = "/tmp/learm_gazebo.urdf"

    generate_urdf = ExecuteProcess(
        cmd=[
            "/bin/bash", "-c",
            f"xacro '{xacro_file}' mesh_dir:='{pkg_share}' ros2_control_params:='{controllers_file}' > '{urdf_out}'"
        ],
        output="screen",
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        output="screen",
        arguments=[
            "-entity", "LeArm",
            "-file", urdf_out,
            "-x", "0", "-y", "0", "-z", "0.2",
        ],
    )

    spawn_jsb = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "120",
            "--activate",
        ],
    )

    spawn_arm = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "arm_controller",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "120",
            "--activate",
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true"),

        gzserver,
        gzclient,
        rsp,

        TimerAction(period=1.0, actions=[generate_urdf]),
        TimerAction(period=4.0, actions=[spawn_entity]),
        TimerAction(period=8.0, actions=[spawn_jsb]),
        TimerAction(period=10.0, actions=[spawn_arm]),
    ])
