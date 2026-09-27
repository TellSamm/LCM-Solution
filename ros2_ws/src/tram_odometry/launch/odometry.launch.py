import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    share = get_package_share_directory("tram_odometry")
    return LaunchDescription([
        DeclareLaunchArgument("tram_id", default_value="30618", description="30618 (default, jury) | 30639 | '' (averaged calibration)"),
        DeclareLaunchArgument("default_start", default_value="stop_S"),
        DeclareLaunchArgument("params_file", default_value=os.path.join(share, "config", "params.yaml")),
        Node(package="tram_odometry", executable="odometry_node", name="tram_odometry", output="screen",
             parameters=[LaunchConfiguration("params_file"), {"tram_id": LaunchConfiguration("tram_id"), "default_start": LaunchConfiguration("default_start")}]),
    ])
