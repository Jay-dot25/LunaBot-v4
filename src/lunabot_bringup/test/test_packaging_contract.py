"""Workspace-level tests for installed package assets and A–L launch entrypoints."""
from pathlib import Path
import subprocess
import unittest


ROOT = Path(__file__).resolve().parents[3]


class PackagingContractTests(unittest.TestCase):
    def test_reusable_simulation_launch_defaults_to_server_only(self):
        launch = (ROOT / "src/lunabot_bringup/launch/simulation.launch.py").read_text()
        self.assertIn('DeclareLaunchArgument("headless", default_value="true",', launch)
        self.assertIn('cmd=["ign", "gazebo", "-r", "-s", world]', launch)
        self.assertIn("UnlessCondition(LaunchConfiguration(\"headless\"))", launch)
        self.assertNotIn("rviz2", launch)

    def test_gazebo_world_and_model_assets_are_installed(self):
        share = ROOT / "install/lunabot_gazebo/share/lunabot_gazebo"
        self.assertTrue((share / "worlds/lunar_world.sdf").is_file())
        self.assertTrue((share / "models/lunabot_v4/model.sdf").is_file())
        for name in ("lunar_terrain.obj", "lunar_terrain_collision.obj",
                     "lunar_terrain_mission_collision.obj"):
            self.assertTrue((share / "worlds/meshes" / name).is_file(), name)
        world = (share / "worlds/lunar_world.sdf").read_text()
        self.assertIn("meshes/lunar_terrain_mission_collision.obj", world)

    def test_bringup_launches_dashboard_nodes_and_installs_configs(self):
        prefix = ROOT / "install/lunabot_bringup"
        for name in ("simulation.launch.py", "dashboard_telemetry.launch.py",
                     "dashboard_controls.launch.py"):
            self.assertTrue((prefix / "share/lunabot_bringup/launch" / name).is_file())
        for name in ("dashboard_telemetry_adapter.py", "dashboard_control_gateway.py"):
            self.assertTrue((prefix / "lib/lunabot_bringup" / name).is_file())
        for name in ("adapter.json", "controls.json"):
            self.assertTrue((prefix / "share/lunabot_bringup/config/dashboard" / name).is_file())

    def test_phase_a_to_l_entrypoints_and_shell_syntax_are_present(self):
        for phase in "abcdefghijkl":
            wrapper = ROOT / f"launch-{phase}"
            implementation = ROOT / f"scripts/launch-{phase}.sh"
            self.assertTrue(wrapper.is_file(), str(wrapper))
            self.assertTrue(implementation.is_file(), str(implementation))
            self.assertTrue(implementation.stat().st_mode & 0o111, str(implementation))
            launch_text = implementation.read_text()
            for mesh in ("lunar_terrain.obj", "lunar_terrain_collision.obj",
                         "lunar_terrain_mission_collision.obj"):
                self.assertIn(mesh, launch_text, f"{phase}: {mesh}")
            result = subprocess.run(["bash", "-n", str(wrapper)], check=False)
            self.assertEqual(0, result.returncode, str(wrapper))
            result = subprocess.run(["bash", "-n", str(implementation)], check=False)
            self.assertEqual(0, result.returncode, str(implementation))

    def test_phase_l_uses_software_gl_for_headless_wsl_sensors(self):
        launch = (ROOT / "scripts/launch-l.sh").read_text()
        self.assertIn("grep -qiE '(microsoft|wsl)' /proc/sys/kernel/osrelease", launch)
        self.assertIn('"${HEADLESS:-0}" = "1"', launch)
        self.assertIn('LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"', launch)
        self.assertIn('WSL_SOFTWARE_PROFILE="${WSL_SOFTWARE_PROFILE:-1}"', launch)
        self.assertIn('prepare_phase_d_model.py', launch)
        self.assertIn('--watchdog-timeout-sec 1.0', launch)
        self.assertIn('AUTO_GOAL_DISTANCE="${AUTO_GOAL_DISTANCE:-1.5}"', launch)
        self.assertIn('AUTO_GOAL_DISTANCE="${DEMO_AUTO_GOAL_DISTANCE:-$AUTO_GOAL_DISTANCE}"', launch)
        self.assertIn('ASTAR_UNKNOWN_IS_OBSTACLE="false"', launch)
        self.assertIn('REQUIRE_MANUAL_GOAL=false', launch)
        self.assertIn('SPAWN_X="${DEMO_SPAWN_X:-5.5}"', launch)
        self.assertIn('TERRAIN_GRID_WIDTH="240"', launch)
        demo_setup = launch.index('if [ "$DEMO" = "1" ] && [ "$P8_ROUNDTRIP" != "1" ]; then')
        mission_start = launch.index('setsid python3 "$MISSION_PATH" --ros-args')
        self.assertLess(demo_setup, mission_start)
        self.assertIn('wait_for_status_token "/lunabot/terrain/cost_map/status" "COST_MAP_PASS"', launch)
        arm = launch.index('WSL simulation startup arming failed')
        follower = launch.index('start_terrain_follower\nfi')
        runtime_checks = launch.index('[22/23] Runtime validation')
        self.assertLess(follower, arm)
        self.assertLess(arm, runtime_checks)


if __name__ == "__main__":
    unittest.main()
