import json
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
sys.path.insert(0, str(ROOT / "tools"))
from dstar_lite import DStarLite
from run_replan_benchmark import _grid
from terrain_plan import weighted_astar


class DStarLiteTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.base = json.loads((ROOT / "config/navigation_baselines.json").read_text())
        cls.benchmark = json.loads((ROOT / "config/p7_replan_benchmark.json").read_text())

    def test_incremental_updates_match_weighted_astar_for_all_routes(self):
        for scenario in self.base["scenarios"]:
            with self.subTest(route=scenario["route_id"]):
                width, height = scenario["width"], scenario["height"]
                start, goal = tuple(scenario["start"]), tuple(scenario["goal"])
                initial = _grid(self.base, scenario, scenario["obstacles"])
                added = self.benchmark["updates"][scenario["route_id"]]
                updated = _grid(self.base, scenario, scenario["obstacles"] + added)
                dstar = DStarLite(
                    initial, width, height, start, goal,
                    cost_weight=self.base.get("cost_weight", 2.0),
                    blocked_cost=self.base["occupied_threshold"],
                    unknown_cost=min(80, self.base["occupied_threshold"] - 1),
                    unknown_is_blocked=self.base["unknown_is_obstacle"])
                initial_path, initial_cost = dstar.extract_path()
                astar_initial, astar_initial_cost = weighted_astar(
                    initial, width, height, start, goal,
                    cost_weight=self.base.get("cost_weight", 2.0),
                    blocked_cost=self.base["occupied_threshold"],
                    unknown_cost=min(80, self.base["occupied_threshold"] - 1),
                    unknown_is_blocked=self.base["unknown_is_obstacle"])
                self.assertTrue(initial_path)
                self.assertTrue(astar_initial)
                self.assertAlmostEqual(astar_initial_cost, initial_cost, places=7)
                changed = [(x, y) for y in range(height) for x in range(width)
                           if initial[y * width + x] != updated[y * width + x]]
                dstar.update_costs(updated, changed)
                repaired, repaired_cost = dstar.extract_path()
                astar_replanned, astar_replanned_cost = weighted_astar(
                    updated, width, height, start, goal,
                    cost_weight=self.base.get("cost_weight", 2.0),
                    blocked_cost=self.base["occupied_threshold"],
                    unknown_cost=min(80, self.base["occupied_threshold"] - 1),
                    unknown_is_blocked=self.base["unknown_is_obstacle"])
                self.assertTrue(repaired)
                self.assertTrue(astar_replanned)
                self.assertAlmostEqual(astar_replanned_cost, repaired_cost, places=7)

    def test_benchmark_retains_three_matched_trials_per_route_and_mode(self):
        from run_replan_benchmark import run_benchmark

        result = run_benchmark(self.base, self.benchmark)
        self.assertEqual(3, result["trials_per_route_per_mode"])
        self.assertEqual(9, result["paired_trial_count"])
        self.assertEqual(18, result["attempt_count"])
        self.assertTrue(result["all_attempts_retained"])
        self.assertEqual("PASS", result["result"])
        for route in result["route_comparisons"]:
            self.assertEqual(3, route["matched_trials"])


if __name__ == "__main__":
    unittest.main()
