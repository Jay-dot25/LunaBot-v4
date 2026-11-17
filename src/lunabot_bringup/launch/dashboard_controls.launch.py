"""Optional guarded dashboard controls relay for view 6."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = str(Path(get_package_share_directory("lunabot_bringup"))
                 / "config/dashboard/controls.json")
    return LaunchDescription([
        Node(
            package="lunabot_bringup",
            executable="dashboard_control_gateway.py",
            name="lunabot_dashboard_control_gateway",
            output="screen",
            parameters=[{"config_path": config}],
        ),
    ])
