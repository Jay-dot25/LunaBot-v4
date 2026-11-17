import csv
import json
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
from perception_metrics import confusion_matrix, summarize_confusion
from train_perception import audit_class_support, load_records, load_score_gates


class PerceptionMetricsTests(unittest.TestCase):
    def test_confusion_ignores_255_and_uses_rows_as_truth(self):
        truth = np.array([[0, 1, 255], [2, 2, 4]], dtype=np.uint8)
        prediction = np.array([[0, 2, 1], [2, 3, 4]], dtype=np.uint8)
        actual = confusion_matrix(truth, prediction)
        self.assertEqual(1, actual[0, 0])
        self.assertEqual(1, actual[1, 2])
        self.assertEqual(1, actual[2, 2])
        self.assertEqual(1, actual[2, 3])
        self.assertEqual(1, actual[4, 4])
        self.assertEqual(5, int(actual.sum()))

    def test_miou_excludes_unsupported_classes_and_reports_null(self):
        matrix = np.zeros((5, 5), dtype=np.int64)
        matrix[0, 0] = 3
        summary = summarize_confusion(matrix, ["regolith", "bedrock", "rock", "crater", "shadow"])
        self.assertEqual(1.0, summary["per_class"]["regolith"]["iou"])
        self.assertIsNone(summary["per_class"]["bedrock"]["iou"])
        self.assertEqual(1, summary["miou_supported_class_count"])
        self.assertAlmostEqual(1.0, summary["miou"])

    def test_confusion_rejects_invalid_prediction_class(self):
        with self.assertRaisesRegex(ValueError, "prediction"):
            confusion_matrix(np.array([0]), np.array([5]))


class PerceptionGateTests(unittest.TestCase):
    def test_requires_complete_frozen_five_class_gates(self):
        with tempfile.TemporaryDirectory() as td:
            path = Path(td) / "gates.json"
            doc = {"schema_version": 1, "gate_id": "smoke-fixture", "minimum_miou": 0.5,
                   "minimum_per_class_iou": {name: 0.25 for name in ("regolith", "bedrock", "rock", "crater", "shadow")}}
            path.write_text(json.dumps(doc), encoding="utf-8")
            parsed = load_score_gates(path, ["regolith", "bedrock", "rock", "crater", "shadow"])
            self.assertEqual("smoke-fixture", parsed["gate_id"])
            self.assertEqual(0.5, parsed["minimum_miou"])

    def test_rejects_incomplete_or_out_of_range_gates(self):
        with tempfile.TemporaryDirectory() as td:
            path = Path(td) / "gates.json"
            doc = {"schema_version": 1, "gate_id": "invalid", "minimum_miou": 0.5,
                   "minimum_per_class_iou": {"regolith": 1.2}}
            path.write_text(json.dumps(doc), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "exactly the classes in the supplied class manifest"):
                load_score_gates(path, ["regolith", "bedrock", "rock", "crater", "shadow"])



    def test_class_support_gate_rejects_missing_classes_before_training(self):
        with tempfile.TemporaryDirectory() as td:
            root = Path(td)
            mask = root / "mask.png"
            Image.fromarray(np.zeros((4, 4), dtype=np.uint8), mode="L").save(mask)
            records = {split: [{"mask": str(mask)}] for split in ("train", "validation", "test")}
            with self.assertRaisesRegex(ValueError, "missing class IDs by split"):
                audit_class_support(records)

    def test_class_support_gate_counts_all_five_and_ignores_255(self):
        with tempfile.TemporaryDirectory() as td:
            root = Path(td)
            mask = root / "mask.png"
            values = np.array([[0, 1, 2], [3, 4, 255]], dtype=np.uint8)
            Image.fromarray(values, mode="L").save(mask)
            records = {split: [{"mask": str(mask)}] for split in ("train", "validation", "test")}
            support = audit_class_support(records)
            self.assertEqual([1, 1, 1, 1, 1], support["validation"])


class PerceptionManifestTrainingTests(unittest.TestCase):
    def _write_image_pair(self, root: Path, name: str):
        Image.new("RGB", (8, 8), color=(80, 90, 100)).save(root / f"{name}.png")
        Image.fromarray(np.zeros((8, 8), dtype=np.uint8), mode="L").save(root / f"{name}-mask.png")

    def test_manifest_enforces_scene_split_and_resolves_files(self):
        with tempfile.TemporaryDirectory() as td:
            root = Path(td)
            rows = []
            for terrain, split in (("terrain01", "train"), ("terrain09", "validation"), ("terrain10", "test")):
                stem = terrain
                self._write_image_pair(root, stem)
                rows.append({"image": f"{stem}.png", "mask": f"{stem}-mask.png", "source_id": "polar-sim-labels", "terrain_id": terrain, "split": split})
            manifest = root / "manifest.csv"
            with manifest.open("w", newline="", encoding="utf-8") as stream:
                writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
                writer.writeheader()
                writer.writerows(rows)
            records = load_records(manifest, root, ROOT / "config/perception/splits.json", ROOT / "config/perception/sources.json")
            self.assertEqual({"train": 1, "validation": 1, "test": 1}, {key: len(value) for key, value in records.items()})
            self.assertTrue(Path(records["test"][0]["image"]).is_absolute())

    def test_manifest_rejects_validation_terrain_leak(self):
        with tempfile.TemporaryDirectory() as td:
            root = Path(td)
            self._write_image_pair(root, "terrain01")
            manifest = root / "manifest.csv"
            with manifest.open("w", newline="", encoding="utf-8") as stream:
                writer = csv.DictWriter(stream, fieldnames=["image", "mask", "source_id", "terrain_id", "split"])
                writer.writeheader()
                writer.writerow({"image": "terrain01.png", "mask": "terrain01-mask.png", "source_id": "polar-sim-labels", "terrain_id": "terrain01", "split": "validation"})
                writer.writerow({"image": "terrain01.png", "mask": "terrain01-mask.png", "source_id": "polar-sim-labels", "terrain_id": "terrain01", "split": "train"})
                writer.writerow({"image": "terrain09.png", "mask": "terrain09-mask.png", "source_id": "polar-sim-labels", "terrain_id": "terrain09", "split": "validation"})
                writer.writerow({"image": "terrain10.png", "mask": "terrain10-mask.png", "source_id": "polar-sim-labels", "terrain_id": "terrain10", "split": "test"})
            with self.assertRaisesRegex(ValueError, "belongs to"):
                load_records(manifest, root, ROOT / "config/perception/splits.json", ROOT / "config/perception/sources.json")


if __name__ == "__main__":
    unittest.main()
