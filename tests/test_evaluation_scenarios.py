import copy
import json
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
from run_scenarios import evaluate, run_attempt


class EvaluationScenarioTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config = json.loads(
            (ROOT / "config/evaluation/scenarios.json").read_text(encoding="utf-8")
        )

    def test_five_scenarios_each_retain_three_passing_trials(self):
        report = evaluate(self.config)
        self.assertEqual(report["result"], "PASS")
        self.assertEqual(report["scenario_count"], 5)
        self.assertEqual(report["attempt_count"], 15)
        self.assertTrue(report["all_attempts_retained"])
        self.assertTrue(all(row["passes"] == 3 for row in report["scenarios"]))

    def test_seeded_trial_is_reproducible_except_timing(self):
        scenario = self.config["scenarios"][1]
        first = run_attempt(self.config, scenario, 2)
        second = run_attempt(self.config, scenario, 2)
        for result in (first, second):
            for side in ("occupancy", "terrain"):
                for leg in result[side]["legs"]:
                    leg.pop("planning_time_ms")
        self.assertEqual(first, second)

    def test_unexpected_obstacle_requires_a_changed_safe_plan(self):
        scenario = next(item for item in self.config["scenarios"]
                        if item["type"] == "dynamic_obstacle")
        result = run_attempt(self.config, scenario, 1)
        self.assertEqual(result["status"], "PASS")
        self.assertEqual(result["replanning_count"], 1)
        self.assertTrue(result["dynamic_replan"]["updated_path_traversable"])

    def test_requires_three_trials_and_exact_five_scenarios(self):
        too_few = copy.deepcopy(self.config)
        too_few["trials_per_scenario"] = 2
        with self.assertRaises(ValueError):
            evaluate(too_few)
        missing = copy.deepcopy(self.config)
        missing["scenarios"].pop()
        with self.assertRaises(ValueError):
            evaluate(missing)

    def test_runtime_only_metrics_are_marked_unavailable(self):
        report = evaluate(self.config)
        unavailable = report["runtime_metrics_unavailable"]["metrics"]
        self.assertIn("collisions", unavailable)
        self.assertIn("localization error", unavailable)


if __name__ == "__main__":
    unittest.main()
