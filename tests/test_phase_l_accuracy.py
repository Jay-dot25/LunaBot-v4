"""Check that operational completion requires independent physical accuracy."""
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
from phase_l_accuracy import evaluate


class PhaseLAccuracyTests(unittest.TestCase):
    def trial(self, truth_distance=1.0, truth_heading=0.0):
        # World heading is 90 degrees from the map heading at spawn.
        events = [
            dict(kind="odom", sim_time=1.0, pose=(0, 0, 0)),
            dict(kind="slam", sim_time=1.0, pose=(0, 0, 0)),
            dict(kind="goal", sim_time=1.1, frame="map", position=[1, 0]),
            dict(kind="slam", sim_time=1.4, pose=(0.5, 0, 0)),
            dict(kind="slam", sim_time=1.8, pose=(1, 0, 0)),
            dict(kind="odom", sim_time=1.8, pose=(1, 0, 0)),
            dict(kind="reached", sim_time=1.8),
        ]
        heading = 1.5707963267948966
        truth = {
            1.0: (10, 20, heading),
            1.4: (10, 20 + truth_distance / 2, heading),
            1.8: (10, 20 + truth_distance, heading + truth_heading),
        }
        return evaluate(events, truth, expected_goals=1)

    def test_passes_with_rotated_world_origin(self):
        result = self.trial()
        self.assertTrue(result["acceptance_pass"], result)
        self.assertAlmostEqual(result["goals"][0]["physical_goal_error_m"], 0)

    def test_rejects_operational_success_when_rover_overshoots(self):
        result = self.trial(truth_distance=2)
        self.assertFalse(result["acceptance_pass"])
        self.assertAlmostEqual(result["goals"][0]["physical_goal_error_m"], 1)
        self.assertAlmostEqual(result["slam"]["peak_position_error_m"], 1)

    def test_rejects_heading_drift_even_when_position_matches(self):
        result = self.trial(truth_heading=0.5)
        self.assertFalse(result["acceptance_pass"])
        self.assertAlmostEqual(result["slam"]["peak_heading_error_rad"], 0.5)
