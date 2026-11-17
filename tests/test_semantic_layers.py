import unittest
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from semantic_layers import (OBSTACLE, TERRAIN, SemanticGrid, decision_strength,
                             three_class_to_map_labels)


class SemanticLayerTests(unittest.TestCase):
    def test_three_class_model_ids_map_to_traversability_and_unknown(self):
        source = np.array([[0, 1, 2, 255, 99]], dtype=np.uint8)
        mapped = three_class_to_map_labels(source)
        np.testing.assert_array_equal(mapped, [[TERRAIN, OBSTACLE, 0, 0, 0]])

    def test_low_confidence_observation_is_rejected_and_map_layer_is_unknown(self):
        grid = SemanticGrid(4, 3, 0.1, 0.0, 0.0, min_confidence=0.5)
        result = grid.update({2: (TERRAIN, 0.49)}, 1_000_000_000)
        self.assertEqual(result.rejected_low_confidence, 1)
        self.assertEqual(grid.semantic_data(1_000_000_000)[2], -1)

    def test_confidence_combines_and_decays_before_ttl(self):
        grid = SemanticGrid(3, 2, 0.1, 0.0, 0.0, half_life_s=10, ttl_s=40,
                            min_confidence=0.2)
        grid.update({1: (TERRAIN, 0.6)}, 1_000_000_000)
        grid.update({1: (TERRAIN, 0.5)}, 2_000_000_000)
        self.assertAlmostEqual(float(grid.confidence[0, 1]), 0.8)
        fresh = grid.confidence_data(2_000_000_000)[1]
        aged = grid.confidence_data(12_000_000_000)[1]
        self.assertEqual(fresh, 80)
        self.assertEqual(aged, 40)
        self.assertEqual(grid.age_data(12_000_000_000)[1], 10)

    def test_ttl_expiry_clears_label_confidence_and_age(self):
        grid = SemanticGrid(3, 2, 0.1, 0.0, 0.0, ttl_s=5, half_life_s=20,
                            min_confidence=0.1)
        grid.update({0: (OBSTACLE, 0.9)}, 1_000_000_000)
        self.assertEqual(grid.decay(6_000_000_000), 1)
        self.assertEqual(grid.semantic_data(6_000_000_000)[0], -1)
        self.assertEqual(grid.confidence_data(6_000_000_000)[0], -1)
        self.assertEqual(grid.age_data(6_000_000_000)[0], -1)

    def test_simulation_time_zero_is_a_valid_observation_stamp(self):
        grid = SemanticGrid(2, 2, 0.1, 0.0, 0.0, ttl_s=2, half_life_s=5,
                            min_confidence=0.1)
        grid.update({0: (TERRAIN, 0.9)}, 0)
        self.assertEqual(grid.age_data(1_000_000_000)[0], 1)
        self.assertEqual(grid.decay(2_000_000_000), 1)

    def test_recenter_preserves_overlap_and_discards_only_outside_window(self):
        grid = SemanticGrid(4, 3, 1.0, 0.0, 0.0)
        grid.update({5: (TERRAIN, 0.8), 6: (OBSTACLE, 0.9)}, 1_000_000_000)
        removed = grid.recenter(2.0, 1.5)
        self.assertEqual(grid.origin_x, 0.0)
        self.assertEqual(grid.origin_y, 0.0)
        self.assertEqual(removed, 0)
        self.assertEqual(grid.labels[1, 1], TERRAIN)
        self.assertEqual(grid.labels[1, 2], OBSTACLE)
        removed = grid.recenter(10.0, 1.0)
        self.assertGreater(removed, 0)
        self.assertEqual(int(grid.labels.sum()), 0)

    def test_window_is_bounded_for_rover_scale_grid(self):
        grid = SemanticGrid(160, 160, 0.1, -8.0, -8.0)
        self.assertEqual(grid.memory_bytes, 160 * 160 * (1 + 4 + 8 + 4 + 8))
        self.assertLess(grid.memory_bytes, 700_000)

    def test_heuristic_decision_strength_is_bounded_and_finite(self):
        self.assertEqual(decision_strength(roi_margin=0, appearance_margin=0), 0.55)
        self.assertEqual(decision_strength(roi_margin=2, appearance_margin=0), 1.0)
        self.assertEqual(decision_strength(roi_margin=float("nan"), appearance_margin=0), 0.0)


if __name__ == "__main__":
    unittest.main()
