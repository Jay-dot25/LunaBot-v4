"""Reusable lunar world component; existing A-L commands remain full stacks."""
import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    assets = Path(get_package_share_directory("lunabot_gazebo"))
    world = str(assets / "worlds" / "lunar_world.sdf")
    paths = [str(assets / "worlds"), str(assets / "models")]
    resources = os.pathsep.join(paths + [os.environ.get("IGN_GAZEBO_RESOURCE_PATH", "")])
    return LaunchDescription([
        DeclareLaunchArgument("headless", default_value="true",
                              description="Run server only; GUI is optional"),
        SetEnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", resources),
        SetEnvironmentVariable("IGN_GAZEBO_SYSTEM_PLUGIN_PATH",
            os.pathsep.join([str(Path(get_package_prefix("lunabot_gazebo")) / "lib"),
                            os.environ.get("IGN_GAZEBO_SYSTEM_PLUGIN_PATH", "")])),
        ExecuteProcess(cmd=["ign", "gazebo", "-r", "-s", world],
                       condition=IfCondition(LaunchConfiguration("headless")),
                       output="screen"),
        ExecuteProcess(cmd=["ign", "gazebo", "-r", world],
                       condition=UnlessCondition(LaunchConfiguration("headless")),
                       output="screen"),
    ])
