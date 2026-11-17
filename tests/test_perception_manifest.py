import copy
import json
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
from validate_perception_manifest import validate


class PerceptionManifestTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.classes = json.loads((ROOT / "config/perception/classes.json").read_text())
        cls.sources = json.loads((ROOT / "config/perception/sources.json").read_text())
        cls.splits = json.loads((ROOT / "config/perception/splits.json").read_text())

    def test_approved_five_class_scene_split_plan(self):
        self.assertEqual([], validate(self.classes, self.sources, self.splits))

    def test_rejects_scene_leakage_across_splits(self):
        bad = copy.deepcopy(self.splits)
        bad["scene_splits"][0]["test"].append("terrain01")
        self.assertTrue(any("scene leakage" in error
                            for error in validate(self.classes, self.sources, bad)))

    def test_rejects_same_terrain_leak_across_source_aliases(self):
        bad = copy.deepcopy(self.splits)
        alias = copy.deepcopy(bad["scene_splits"][0])
        alias["source_id"] = "nasa-polar-original"
        alias["train"] = []
        alias["validation"] = []
        alias["test"] = ["terrain01"]
        bad["scene_splits"].append(alias)
        self.assertTrue(any("scene leakage" in error
                            for error in validate(self.classes, self.sources, bad)))

    def test_rejects_unknown_or_unlicensed_source(self):
        bad = copy.deepcopy(self.sources)
        bad["selected_sources"][0]["license"] = "unknown"
        self.assertTrue(any("unsupported or missing license" in error
                            for error in validate(self.classes, bad, self.splits)))

    def test_requires_exact_five_class_taxonomy(self):
        bad = copy.deepcopy(self.classes)
        bad["classes"].pop()
        self.assertTrue(any("class names/order" in error
                            for error in validate(bad, self.sources, self.splits)))


if __name__ == "__main__":
    unittest.main()
