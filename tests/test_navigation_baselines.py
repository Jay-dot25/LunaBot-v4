import json
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
from run_navigation_baselines import evaluate, load_scenarios


class NavigationBaselineTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config = load_scenarios(ROOT / "config/navigation_baselines.json")

    def test_three_trials_per_route_are_retained(self):
        result = evaluate(self.config, 3)
        self.assertEqual(3, result["route_count"])
        self.assertEqual(9, result["attempt_count"])
        self.assertTrue(result["all_attempts_retained"])
        self.assertTrue(all(route["attempts"] == 3 for route in result["routes"]))
        self.assertTrue(all(len(route["trial_indices_retained"]) == 3
                            for route in result["routes"]))

    def test_route_metrics_are_present_for_successes(self):
        result = evaluate(self.config, 3)
        self.assertEqual("PASS", result["result"])
        for attempt in result["attempts"]:
            self.assertEqual("PASS", attempt["status"])
            self.assertGreater(attempt["path_cells"], 0)
            self.assertGreaterEqual(attempt["path_length_m"], 0.0)
            self.assertGreaterEqual(attempt["planning_time_ms"], 0.0)
            self.assertIsNotNone(attempt["min_clearance_m"])

    def test_failed_attempt_is_retained(self):
        scenario = {
            "route_id": "blocked_goal",
            "width": 10,
            "height": 10,
            "start": [1, 1],
            "goal": [8, 8],
            "expected_success": True,
            "obstacles": [[0, 0, 9, 9]],
        }
        result = evaluate({**self.config, "scenarios": [scenario]}, 3)
        self.assertEqual(3, result["attempt_count"])
        self.assertTrue(result["all_attempts_retained"])
        self.assertEqual("FAIL", result["result"])
        self.assertTrue(all(attempt["status"] == "FAIL" and not attempt["success"]
                            for attempt in result["attempts"]))


if __name__ == "__main__":
    unittest.main()
