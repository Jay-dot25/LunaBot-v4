import sys
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
from materialize_perception import image_key, label_key, parse_yolo_polygons, rasterize_mask


class MaterializePerceptionTests(unittest.TestCase):
    def test_label_filename_parses_to_expected_identity(self):
        self.assertEqual(("B", "off", "180", "L", "0016"), label_key("labels/01_B_off_180_L_0016.txt"))
        self.assertIsNone(label_key("wrong-name.txt"))

    def test_nasa_image_path_matches_label_identity(self):
        path = "Terrain01/PosB_4000_Loff/Sun_180/CamL_249696053_0016.png"
        self.assertEqual(("B", "off", "180", "L", "0016"), image_key(path))
        self.assertIsNone(image_key("Terrain01/PosB_1500_Loff/Sun_180/CamL_249696053_0016.png"))

    def test_polygons_remap_ground_rock_shadow_and_ignore_background(self):
        text = "0 0 0 1 0 1 1 0 1\n1 0 0 1 0 1 1 0 1\n2 0.25 0.25 0.75 0.25 0.75 0.75 0.25 0.75\n3 0.4 0.4 0.6 0.4 0.6 0.6 0.4 0.6"
        polygons = parse_yolo_polygons(text, 20, 20)
        mask = rasterize_mask(polygons, 20, 20)
        self.assertEqual(0, mask[1, 1])
        self.assertEqual(2, mask[6, 6])
        self.assertEqual(4, mask[9, 9])
        self.assertEqual({0, 2, 4}, set(np.unique(mask)))

    def test_background_only_region_is_ignore_and_bad_ids_rejected(self):
        mask = rasterize_mask(parse_yolo_polygons("0 0 0 1 0 1 1 0 1", 8, 8), 8, 8)
        self.assertTrue(np.all(mask == 255))
        with self.assertRaisesRegex(ValueError, "unsupported POLAR-Sim class"):
            parse_yolo_polygons("4 0 0 1 0 1 1", 8, 8)


if __name__ == "__main__":
    unittest.main()
