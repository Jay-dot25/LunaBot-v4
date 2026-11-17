from __future__ import annotations
import json
import importlib.util
import subprocess
import sys
import tempfile
import unittest
from io import BytesIO
from pathlib import Path

PYARROW_AVAILABLE = importlib.util.find_spec("pyarrow") is not None
import numpy as np
if PYARROW_AVAILABLE:
    import pyarrow as pa
    import pyarrow.parquet as pq
else:
    pa = pq = None
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
from materialize_marsseg_train import remap_mask


class MarsSegMaterializerTests(unittest.TestCase):
    def test_source_classes_map_and_unknowns_stay_ignore(self):
        source = np.array([[0, 1, 2, 3], [4, 5, 6, 2]], dtype=np.uint8)
        expected = np.array([[255, 1, 0, 2], [4, 255, 255, 0]], dtype=np.uint8)
        np.testing.assert_array_equal(remap_mask(source), expected)

    def test_unknown_source_class_fails_closed(self):
        with self.assertRaisesRegex(ValueError, "unsupported source mask IDs"):
            remap_mask(np.array([[7]], dtype=np.uint8))

    @unittest.skipUnless(PYARROW_AVAILABLE, "pyarrow is required to read Parquet inputs")
    def test_cli_materializes_only_train_with_provenance(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            image_bytes, mask_bytes = BytesIO(), BytesIO()
            Image.new("RGB", (7, 1), color=(10, 20, 30)).save(image_bytes, format="PNG")
            Image.fromarray(np.array([[0, 1, 2, 3, 4, 5, 6]], dtype=np.uint8), mode="L").save(mask_bytes, format="PNG")
            source = root / "train.parquet"
            pq.write_table(pa.table({
                "image": [{"bytes": image_bytes.getvalue(), "path": "sample.png"}],
                "mask": [{"bytes": mask_bytes.getvalue(), "path": "sample-mask.png"}],
                "width": [7], "height": [1], "class_labels": [["Bedrock", "Gravel / Sand / Soil", "Rock", "Shadow"]],
            }), source)
            mapping = root / "mapping.json"
            mapping.write_text(json.dumps({"0":"Background", "1":"Bedrock", "2":"Gravel / Sand / Soil", "3":"Rock", "4":"Shadow", "5":"Sky / Distant Mountains", "6":"Track"}), encoding="utf-8")
            readme = root / "README.md"; readme.write_text("CC-BY-4.0", encoding="utf-8")
            output = root / "output"
            subprocess.run([sys.executable, str(ROOT / "tools/materialize_marsseg_train.py"),
                            "--parquet", str(source), "--mapping", str(mapping),
                            "--source-readme", str(readme), "--revision", "test-revision",
                            "--output", str(output)], check=True, capture_output=True, text=True)
            report = json.loads((output / "materialization-report.json").read_text(encoding="utf-8"))
            self.assertEqual(report["source_rows"], 1)
            self.assertEqual(report["split"], "train")
            self.assertTrue(report["train_only"])
            self.assertFalse(report["training_ready"])
            self.assertEqual(report["validation_or_test_records"], 0)
            with Image.open(output / "masks/mars_msl_00000.png") as result:
                np.testing.assert_array_equal(np.asarray(result), [[255, 1, 0, 2, 4, 255, 255]])
            with (output / "manifest.csv").open(encoding="utf-8", newline="") as stream:
                import csv
                row = next(csv.DictReader(stream))
            self.assertEqual((row["source_id"], row["split"]), ("marsseg-msl-bedrock-pretrain", "train"))


if __name__ == "__main__":
    unittest.main()
