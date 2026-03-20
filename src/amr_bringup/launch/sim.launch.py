from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import subprocess


def generate_launch_description():
    desc_pkg = get_package_share_directory("amr_description")
    ctrl_pkg = get_package_share_directory("amr_control")
    bringup_pkg = get_package_share_directory("amr_bringup")

    xacro_path = os.path.join(desc_pkg, "urdf", "amr.urdf.xacro")
    controllers_yaml = os.path.join(ctrl_pkg, "config", "controllers.yaml")
    maze_model_path = os.path.join(bringup_pkg, "my_world", "model.sdf")

    robot_desc = subprocess.check_output([
        "xacro",
        xacro_path,
        f"controllers_file:={controllers_yaml}",
    ]).decode()

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("gazebo_ros"),
                "launch",
                "gazebo.launch.py",
            )
        )
    )

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": True, "robot_description": robot_desc}],
    )

    spawn_maze = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-file", maze_model_path,
            "-entity", "my_world",
            "-x", "0.0",
            "-y", "0.0",
            "-z", "0.0",
        ],
        output="screen",
    )

    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic", "robot_description",
            "-entity", "amr",
            "-x", "0.0",
            "-y", "0.0",
            "-z", "0.11",
        ],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
        ],
        output="screen",
    )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "diff_cont",
            "--controller-manager", "/controller_manager",
        ],
        output="screen",
    )

    start_robot_after_maze = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_maze,
            on_exit=[spawn_robot],
        )
    )

    start_jsb_after_robot = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    start_diff_after_jsb = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[diff_drive_spawner],
        )
    )

    return LaunchDescription([
        gazebo,
        rsp,
        spawn_maze,
        start_robot_after_maze,
        start_jsb_after_robot,
        start_diff_after_jsb,
    ])