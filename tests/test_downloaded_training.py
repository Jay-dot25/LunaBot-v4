import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from materialize_downloaded_training import polygon_mask, remap


class DownloadedMappingTests(unittest.TestCase):
    def test_nav_is_not_canonical_id_order(self):
        actual = remap(np.array([[0, 1, 2, 3, 255]], np.uint8),
                       {0: 0, 1: 1, 2: 0, 3: 2, 255: 255})
        self.assertEqual(actual.tolist(), [[0, 1, 0, 2, 255]])

    def test_unknown_label_rejected(self):
        with self.assertRaises(ValueError):
            remap(np.array([[4]], np.uint8), {0: 0})

    def test_polygon_background_is_ignore_and_crater_precedence(self):
        rock = [0, 0, 3, 0, 3, 3, 0, 3]
        crater = [1, 1, 2, 1, 2, 2, 1, 2]
        actual = polygon_mask((5, 5), [(3, [crater]), (2, [rock])])
        self.assertEqual(int(actual[1, 1]), 3)
        self.assertEqual(int(actual[0, 0]), 2)
        self.assertEqual(int(actual[4, 4]), 255)


if __name__ == '__main__':
    unittest.main()
