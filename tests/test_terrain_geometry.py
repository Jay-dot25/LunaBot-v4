import math
import unittest
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from terrain_geometry import derive_terrain_layers
from terrain_costs import GridGeometry, fuse_terrain_costs


class TerrainGeometryTests(unittest.TestCase):
    def _layers(self, elevation):
        h, w = elevation.shape
        seen_z = np.full((h, w), 1_000_000_000, dtype=np.int64)
        labels = np.full((h, w), 1, dtype=np.uint8)
        confidence = np.full((h, w), 0.8, dtype=np.float32)
        seen = np.full((h, w), 1_000_000_000, dtype=np.int64)
        return derive_terrain_layers(
            elevation, seen_z, labels, confidence, seen, now_ns=1_000_000_000,
            resolution=0.1)

    def test_flat_surface_is_low_slope_roughness_and_crater_risk(self):
        layers = self._layers(np.zeros((9, 9), dtype=np.float32))
        self.assertEqual(int(layers["slope"][4, 4]), 0)
        self.assertEqual(int(layers["roughness"][4, 4]), 0)
        self.assertEqual(int(layers["crater_risk"][4, 4]), 0)

    def test_plane_slope_uses_fitted_local_height_gradient(self):
        rows, cols = np.mgrid[:9, :9]
        grade = math.tan(math.radians(25.0))
        elevation = (cols * 0.1 * grade).astype(np.float32)
        layers = self._layers(elevation)
        self.assertGreaterEqual(int(layers["slope"][4, 4]), 99)
        self.assertEqual(int(layers["roughness"][4, 4]), 0)

    def test_bowl_depression_gets_crater_risk(self):
        elevation = np.zeros((11, 11), dtype=np.float32)
        elevation[5, 5] = -0.20
        layers = self._layers(elevation)
        self.assertGreaterEqual(int(layers["crater_risk"][5, 5]), 70)

    def test_insufficient_geometry_is_unknown_and_increases_uncertainty(self):
        elevation = np.full((5, 5), np.nan, dtype=np.float32)
        elevation[2, 2] = 0.0
        layers = self._layers(elevation)
        self.assertEqual(int(layers["slope"][2, 2]), -1)
        self.assertEqual(int(layers["roughness"][2, 2]), -1)
        self.assertEqual(int(layers["crater_risk"][2, 2]), -1)
        self.assertEqual(int(layers["uncertainty"][2, 2]), 100)

    def test_shapes_must_be_aligned(self):
        with self.assertRaises(ValueError):
            derive_terrain_layers(np.zeros((2, 2)), np.zeros((2, 1)),
                                  np.zeros((2, 2)), np.zeros((2, 2)),
                                  np.zeros((2, 2)), now_ns=1, resolution=0.1)


class TerrainCostFusionTests(unittest.TestCase):
    def test_terrain_risks_raise_cost_and_obstacles_stay_hard(self):
        costs = fuse_terrain_costs(
            [25, 25, 25, 100], slope=[0, 100, 0, 0],
            roughness=[0, 0, 100, 0], crater_risk=[0, 0, 0, 0],
            uncertainty=[0, 0, 0, 0])
        self.assertEqual(costs[0], 20)
        self.assertGreater(costs[1], costs[0])
        self.assertGreater(costs[2], costs[0])
        self.assertEqual(costs[3], 100)

    def test_unknown_and_shadow_are_caution_not_obstacles(self):
        costs = fuse_terrain_costs(
            [-1, 0, 25, 100], slope=[-1] * 4, roughness=[-1] * 4,
            crater_risk=[-1] * 4, uncertainty=[-1] * 4)
        self.assertEqual(costs[:2], [80, 80])
        self.assertNotEqual(costs[1], 100)
        self.assertEqual(costs[3], 100)

    def test_invalid_layer_length_rejected(self):
        with self.assertRaises(ValueError):
            fuse_terrain_costs([25], slope=[], roughness=[0], crater_risk=[0], uncertainty=[0])

    def test_full_grid_geometry_includes_frame_and_orientation(self):
        base = GridGeometry("map", 10, 10, 0.1, -1.0, -1.0)
        same_rotation = GridGeometry("map", 10, 10, 0.1, -1.0, -1.0,
                                     orientation_xyzw=(0, 0, 0, -1))
        shifted = GridGeometry("map", 10, 10, 0.1, -0.9, -1.0)
        wrong_frame = GridGeometry("odom", 10, 10, 0.1, -1.0, -1.0)
        self.assertTrue(base.compatible(same_rotation))
        self.assertFalse(base.compatible(shifted))
        self.assertFalse(base.compatible(wrong_frame))


if __name__ == "__main__":
    unittest.main()
