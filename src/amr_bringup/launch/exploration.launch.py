from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetRemap
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    bringup_pkg = get_package_share_directory("amr_bringup")
    nav2_bringup_pkg = get_package_share_directory("nav2_bringup")

    nav2_params = os.path.join(bringup_pkg, "config", "nav2_params.yaml")

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_pkg, "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "use_sim_time": "True",
            "params_file": nav2_params,
        }.items()
    )

    nav2_group = GroupAction([
        SetRemap(src='/cmd_vel', dst='/diff_cont/cmd_vel_unstamped'),
        nav2_launch,
    ])

    frontier_explorer = Node(
        package='amr_exploration',
        executable='frontier_explorer',
        name='frontier_explorer',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        nav2_group,
        frontier_explorer,
    ])