import copy
import json
import sys
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
from materialize_bedrock_evaluation import remap_baseprod
from validate_perception_manifest import validate


class BedrockEvaluationTests(unittest.TestCase):
    def test_mapping_preserves_unknown_and_source(self):
        source = np.arange(8, dtype=np.uint8).reshape(2, 4)
        original = source.copy()
        mapped = remap_baseprod(source)
        np.testing.assert_array_equal(mapped, [[255,255,255,1],[0,255,2,255]])
        np.testing.assert_array_equal(source, original)

    def test_new_class_is_not_silently_ignored(self):
        with self.assertRaisesRegex(ValueError, 'unsupported'):
            remap_baseprod(np.array([[8]], dtype=np.uint8))

    def test_evaluation_scene_cannot_enter_training(self):
        classes = json.loads((ROOT / 'config/perception/classes.json').read_text())
        sources = json.loads((ROOT / 'config/perception/sources.json').read_text())
        splits = json.loads((ROOT / 'config/perception/splits.json').read_text())
        self.assertEqual([], validate(classes, sources, splits))
        bad = copy.deepcopy(splits)
        row = next(r for r in bad['scene_splits'] if r['source_id'] == 'baseprod-bedrock-evaluation')
        row['train'] = row['test']
        self.assertTrue(any('evaluation-only' in e for e in validate(classes, sources, bad)))


if __name__ == '__main__':
    unittest.main()
