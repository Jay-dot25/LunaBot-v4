"""Optional read-only age/provenance adapter for dashboard views 1-5."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = str(Path(get_package_share_directory("lunabot_bringup"))
                 / "config/dashboard/adapter.json")
    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time", default_value="true",
            description="Use ROS simulation time for source-stamp age checks",
        ),
        Node(
            package="lunabot_bringup",
            executable="dashboard_telemetry_adapter.py",
            name="lunabot_dashboard_telemetry_adapter",
            output="screen",
            parameters=[{
                "config_path": config,
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }],
        ),
    ])
