import json
import math
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
sys.path.insert(0, str(ROOT / "tools"))
from p8_roundtrip import (delivery_id, parse_goal_id, scenario_registry,
                          planner_goal_cell_cost, transform_matches,
                          waypoint_poses)
from make_p8_safety_profile import make_profile


class P8RoundtripTests(unittest.TestCase):
    @staticmethod
    def occupancy_grid(data, *, width=5, height=5, resolution=1.0,
                       frame="map", origin=(0.0, 0.0)):
        pose = SimpleNamespace(
            position=SimpleNamespace(x=origin[0], y=origin[1]),
            orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0))
        return SimpleNamespace(
            header=SimpleNamespace(frame_id=frame),
            info=SimpleNamespace(width=width, height=height,
                                 resolution=resolution, origin=pose),
            data=data)

    def test_goal_preflight_includes_planner_obstacle_inflation(self):
        terrain = self.occupancy_grid([50] * 25)
        obstacles = [0] * 25
        obstacles[2 * 5 + 2] = 100
        obstacle_grid = self.occupancy_grid(obstacles)

        self.assertEqual(100, planner_goal_cell_cost(
            terrain, obstacle_grid, {"x": 3.5, "y": 3.5}))

    def test_goal_preflight_rejects_missing_obstacle_map(self):
        terrain = self.occupancy_grid([0] * 25)
        with self.assertRaisesRegex(RuntimeError, "both required"):
            planner_goal_cell_cost(terrain, None, {"x": 1.5, "y": 1.5})

    def test_goal_id_parser_requires_a_complete_status_token(self):
        self.assertEqual("goal-a91", parse_goal_id(
            "MISSION_DEMO_PASS lifecycle=COMPLETED goal_id=goal-a91 plan=1"))
        self.assertIsNone(parse_goal_id("MISSION_WAITING_FOR_GOAL lifecycle=READY"))

    def test_delivery_ids_are_distinct_by_trial_and_safe_for_command(self):
        first = delivery_id("20261113-p803-trial-01", 1)
        second = delivery_id("20261113-p803-trial-02", 1)
        self.assertRegex(first, r"^[A-Za-z0-9._:-]{1,128}$")
        self.assertNotEqual(first, second)

    def test_world_sites_are_dispatched_in_stable_map_frame(self):
        scenario = {
            "scenario_id": "test-roundtrip",
            "start_pose_world": {"x": 10.0, "y": -2.0, "yaw": 0.0},
            "waypoints": [
                {"id": "habitat", "pose_world":
                 {"x": 10.0, "y": -2.0, "yaw": 0.0}},
                {"id": "science", "pose_world":
                 {"x": 12.0, "y": -2.0, "yaw": 0.0}},
            ],
        }
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "scenario.json"
            path.write_text(json.dumps(scenario), encoding="utf-8")
            registry = scenario_registry(path, (1.0, 3.0, 0.5))
        poses = dict(waypoint_poses(registry))
        self.assertEqual("map", registry["navigation_frame"])
        self.assertAlmostEqual(1.0, poses["habitat"]["x"])
        self.assertAlmostEqual(3.0, poses["habitat"]["y"])
        self.assertAlmostEqual(1.0 + 2.0 * math.cos(0.5),
                               poses["science"]["x"])
        self.assertAlmostEqual(3.0 + 2.0 * math.sin(0.5),
                               poses["science"]["y"])
        self.assertEqual(("habitat", "science", "habitat"),
                         tuple(name for name, _ in waypoint_poses(registry)))

    def test_qualified_map_transform_must_match_new_trial_alignment(self):
        reference = {"x": 0.02, "y": -0.01, "yaw": 0.03}
        self.assertTrue(transform_matches((0.1, 0.04, 0.08), reference))
        self.assertFalse(transform_matches((0.5, 0.0, 0.0), reference))

    def test_slow_sim_profile_keeps_sim_time_and_clock_gates(self):
        source = ROOT / "config/safety/control.json"
        with tempfile.TemporaryDirectory() as temp:
            destination = Path(temp) / "p8-safety.json"
            settings = make_profile(source, destination)
            reloaded = json.loads(destination.read_text(encoding="utf-8"))
        self.assertEqual(3.0, settings["health"]["input_timeout_wall_sec"])
        self.assertEqual(18.0, settings["health"]["path_timeout_sec"])
        self.assertEqual(3.0, reloaded["health"]["max_stamp_age_sim_sec"])
        self.assertEqual(2.0, reloaded["health"]["clock_timeout_wall_sec"])
        self.assertEqual(0.0, reloaded["health"]["map_tf_stamp_offset_sec"])
        self.assertEqual(0.25, reloaded["health"]["future_tolerance_sim_sec"])
        self.assertEqual(0.25, reloaded["health"]["map_tf_future_tolerance_sim_sec"])
        self.assertEqual(30.0, reloaded["health"]["map_tf_motion_grace_wall_sec"])
        self.assertEqual(0.5, reloaded["health"]["health_report_timeout_wall_sec"])

    def test_health_report_timeout_can_be_derived_for_wsl_without_changing_default(self):
        source = ROOT / "config/safety/control.json"
        with tempfile.TemporaryDirectory() as temp:
            destination = Path(temp) / "wsl-safety.json"
            settings = make_profile(
                source, destination, health_report_timeout_wall_sec=2.0)
        self.assertEqual(2.0, settings["health"]["health_report_timeout_wall_sec"])
        baseline = json.loads(source.read_text(encoding="utf-8"))
        self.assertEqual(0.5, baseline["health"]["health_report_timeout_wall_sec"])

    def test_clock_timeout_can_be_tuned_for_wsl_without_changing_default(self):
        source = ROOT / "config/safety/control.json"
        with tempfile.TemporaryDirectory() as temp:
            destination = Path(temp) / "wsl-safety.json"
            settings = make_profile(
                source, destination, clock_timeout_wall_sec=6.0)
        self.assertEqual(6.0, settings["health"]["clock_timeout_wall_sec"])
        baseline = json.loads(source.read_text(encoding="utf-8"))
        self.assertEqual(0.5, baseline["health"]["clock_timeout_wall_sec"])


if __name__ == "__main__":
    unittest.main()
