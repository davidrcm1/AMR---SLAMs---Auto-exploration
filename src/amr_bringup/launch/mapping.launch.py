from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    bringup_pkg = get_package_share_directory("amr_bringup")

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, "launch", "sim.launch.py")
        )
    )

    slam_params = os.path.join(bringup_pkg, "config", "slam_params.yaml")

    slam_toolbox_node = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[slam_params, {"use_sim_time": True}],
    )

    frontier_detector = Node(
        package='amr_exploration',
        executable='frontier_detector',
        name='frontier_detector',
        output='screen',
        parameters=[{'use_sim_time': True}],
    ) 

    return LaunchDescription([
        sim_launch,
        slam_toolbox_node,
        frontier_detector,
    ])