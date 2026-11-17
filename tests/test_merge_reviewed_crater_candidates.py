import contextlib
import csv
import io
import json
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
import merge_reviewed_crater_candidates as tool


class MergeReviewedCraterCandidatesTests(unittest.TestCase):
    def test_merges_only_train_candidates_and_preserves_sources(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            rows = []
            candidates = []
            for terrain, split in (("terrain05", "train"), ("terrain11", "validation")):
                image = root / "images" / terrain / f"{terrain}.png"
                image.parent.mkdir(parents=True)
                Image.new("RGB", (8, 8), (80, 90, 100)).save(image)
                mask = root / "masks" / terrain / f"{terrain}.png"
                mask.parent.mkdir(parents=True)
                source = np.zeros((8, 8), dtype=np.uint8)
                source[1, 1] = 2
                source[1, 2] = 4
                Image.fromarray(source, mode="L").save(mask)
                rows.append({"image": str(image), "mask": str(mask), "source_id": "polar-sim-labels", "terrain_id": terrain, "split": split})

                candidate_mask = root / "candidate" / terrain / f"{terrain}.png"
                candidate_mask.parent.mkdir(parents=True)
                proposed = np.full((8, 8), 255, dtype=np.uint8)
                proposed[3, 3] = 3
                Image.fromarray(proposed, mode="L").save(candidate_mask)
                candidates.append({
                    "terrain_id": terrain, "split": split, "image": str(image),
                    "candidate_mask": str(candidate_mask), "candidate_class_id": 3,
                    "requires_human_review": True, "excluded_non_regolith_pixels": 2,
                })

            manifest = root / "manifest.csv"
            # Exercise the materializer's relative paths: the output CSV moves
            # to another directory and must still resolve every image/mask.
            for row in rows:
                for field in ("image", "mask"):
                    row[field] = str(Path(row[field]).relative_to(root))
            with manifest.open("w", encoding="utf-8", newline="") as stream:
                writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
                writer.writeheader()
                writer.writerows(rows)
            report_path = root / "candidate-report.json"
            report_path.write_text(json.dumps({
                "training_ready": False,
                "roi_definitions": [{"terrain_id": terrain, "position": "A", "annotator_id": "delegated"} for terrain in ("terrain05", "terrain11")],
                "candidates": candidates,
            }), encoding="utf-8")
            output = root / "merged"
            args = ["merge_reviewed_crater_candidates", "--manifest", str(manifest),
                    "--candidate-report", str(report_path), "--output", str(output),
                    "--accept-annotator-id", "delegated"]
            with patch("sys.argv", args), contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(0, tool.main())

            with Image.open(output / "merged_masks/terrain05/terrain05-mask.png") as image:
                merged = np.asarray(image)
            self.assertEqual(3, int(merged[3, 3]))
            self.assertEqual(2, int(merged[1, 1]))
            self.assertEqual(4, int(merged[1, 2]))
            with Image.open(root / rows[0]["mask"]) as image:
                self.assertEqual(0, int(np.asarray(image)[3, 3]))
            with Image.open(root / rows[1]["mask"]) as image:
                self.assertNotIn(3, set(np.asarray(image).reshape(-1)))
            with (output / "manifest.csv").open(newline="", encoding="utf-8") as stream:
                merged_rows = list(csv.DictReader(stream))
            self.assertNotEqual(rows[0]["mask"], merged_rows[0]["mask"])
            self.assertEqual(str(root / rows[1]["mask"]), merged_rows[1]["mask"])
            for row in merged_rows:
                self.assertTrue(Path(row["image"]).is_file())
                self.assertTrue(Path(row["mask"]).is_file())
            summary = json.loads((output / "merge-report.json").read_text())
            self.assertEqual(1, summary["accepted_train_images"])
            self.assertEqual(1, summary["ignored_validation_test_candidates"])
            self.assertFalse(summary["source_manifests_and_masks_modified"])

    def test_refuses_candidate_overwriting_known_rock(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            image = root / "image.png"
            mask = root / "mask.png"
            candidate_mask = root / "candidate.png"
            Image.new("RGB", (4, 4)).save(image)
            source = np.zeros((4, 4), dtype=np.uint8)
            source[0, 0] = 2
            Image.fromarray(source, mode="L").save(mask)
            proposed = np.full((4, 4), 255, dtype=np.uint8)
            proposed[0, 0] = 3
            Image.fromarray(proposed, mode="L").save(candidate_mask)
            manifest = root / "manifest.csv"
            with manifest.open("w", newline="", encoding="utf-8") as stream:
                writer = csv.DictWriter(stream, fieldnames=["image", "mask", "source_id", "terrain_id", "split"])
                writer.writeheader()
                writer.writerow({"image": str(image), "mask": str(mask), "source_id": "polar-sim-labels", "terrain_id": "terrain05", "split": "train"})
            report = root / "candidate-report.json"
            report.write_text(json.dumps({
                "training_ready": False,
                "roi_definitions": [{"terrain_id": "terrain05", "position": "A", "annotator_id": "delegated"}],
                "candidates": [{"terrain_id": "terrain05", "split": "train", "image": str(image), "candidate_mask": str(candidate_mask), "candidate_class_id": 3, "requires_human_review": True}],
            }), encoding="utf-8")
            args = ["merge_reviewed_crater_candidates", "--manifest", str(manifest),
                    "--candidate-report", str(report), "--output", str(root / "out"),
                    "--accept-annotator-id", "delegated"]
            with patch("sys.argv", args), contextlib.redirect_stderr(io.StringIO()):
                self.assertEqual(2, tool.main())


if __name__ == "__main__":
    unittest.main()
