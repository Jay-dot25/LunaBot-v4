# Incremental packaging (P1-01)

lunabot_gazebo installs existing world/model/mesh assets without modifying them.
lunabot_bringup installs simulation.launch.py, a shared lunar-world launch component.
It defaults to headless and has no RViz dependency. Existing launch-a through launch-l
remain the full rover/sensor/navigation stacks. This component starts only the world;
rover spawning, bridge, TF, safety and later nodes migrate in subsequent scoped steps.

Build from WSL after sourcing /opt/ros/humble/setup.bash:
colcon build
colcon test
colcon test-result --verbose
source install/setup.bash
ros2 launch lunabot_bringup simulation.launch.py headless:=true

`lunabot_bringup` registers package/install and A–L launcher syntax checks with
ament. Run the full repository Python suite with
`python3 -m unittest discover -s tests -v` after sourcing the workspace.

Use headless:=false for the optional Gazebo GUI. RViz is independently optional.
Dependency check: rosdep check --from-paths src --ignore-src --rosdistro humble.

Metadata uses the latest repository commit author address and LicenseRef-Unspecified because repository
licensing/maintainer identity is not established. Resolve before external distribution;
these packages do not grant a new license to existing assets.

P2-03 adds the native watchdog target to lunabot_gazebo. The package declares the
installed Fortress SDK/transport/message dependencies and installs its library.
The reusable launch supplies the installed plugin path. Legacy A-L launchers
source prepare_safety_plugin.sh for the ignored local .build/native_safety build.
No system dependency installation or ROS/Gazebo distribution migration occurred.
