import sys
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
from project_crater_candidates import (
    locate_lidar_members,
    parse_calibration,
    points_in_polygon,
    project_lidar_points,
    validate_rois,
)


class ProjectCraterCandidatesTests(unittest.TestCase):
    def test_polygon_membership_includes_edges(self):
        x = np.array([0.5, 0.0, 1.0, 1.5])
        y = np.array([0.5, 0.5, 0.5, 0.5])
        inside = points_in_polygon(x, y, [[0, 0], [1, 0], [1, 1], [0, 1]])
        self.assertEqual([True, True, True, False], inside.tolist())

    def test_camera_projection_returns_unique_in_frame_pixels(self):
        matrix = np.array([[10.0, 0, 5.0], [0, 10.0, 5.0], [0, 0, 1]])
        points = np.array([[0, 0, 1], [0, 0, 1], [0.1, 0, 1], [10, 0, 1]], dtype=np.float32)
        pixels, in_front = project_lidar_points(
            points, matrix, np.zeros(4), np.eye(3), np.zeros(3), (10, 10))
        self.assertEqual(4, in_front)
        self.assertEqual({(5, 5), (6, 5)}, set(map(tuple, pixels.tolist())))

    def test_archive_root_discovery_accepts_real_terrain_folder_names(self):
        import tempfile
        import zipfile
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "Terrain11_FreshCrater.zip"
            with zipfile.ZipFile(path, "w") as archive:
                archive.writestr("Terrain11_FreshCrater/GroundTruth_ascii/PosA.xyz", "")
                archive.writestr("Terrain11_FreshCrater/stereo.calibration", "")
            with zipfile.ZipFile(path) as archive:
                self.assertEqual(
                    ("Terrain11_FreshCrater/GroundTruth_ascii/PosA.xyz",
                     "Terrain11_FreshCrater/stereo.calibration"),
                    locate_lidar_members(archive, "A"),
                )

    def test_calibration_parser_checks_required_fields(self):
        text = (
            "CAMERA_MATRIX_LEFT: 10 0 5 0 10 5 0 0 1\n"
            "DISTORTION_COEFFICIENTS_LEFT: 0 0 0 0\n"
            "CAM_LIDAR_ROTATION: 1 0 0 0 1 0 0 0 1\n"
            "CAM_LIDAR_TRANSLATION: 0 0 0\n"
            "IMAGE_WIDTH_HEIGHT: 10 10\n"
        )
        matrix, distortion, rotation, translation, size = parse_calibration(text)
        self.assertEqual((10, 10), size)
        self.assertEqual((3, 3), matrix.shape)
        self.assertEqual((4,), distortion.shape)
        self.assertTrue(np.array_equal(rotation, np.eye(3)))
        with self.assertRaisesRegex(ValueError, "CAMERA_MATRIX_LEFT"):
            parse_calibration("CAM_LIDAR_TRANSLATION: 0 0 0")

    def test_roi_schema_requires_finite_xy_vertices_and_known_positions(self):
        good = {
            "schema_version": 1,
            "rois": [{"terrain_id": "terrain11", "position": "A", "annotator_id": "test",
                      "polygon_xy_m": [[0, 0], [1, 0], [0, 1]]}],
        }
        self.assertEqual({("terrain11", "A"): [[[0.0, 0.0], [1.0, 0.0], [0.0, 1.0]]]},
                         validate_rois(good))
        bad = {
            "schema_version": 1,
            "rois": [{"terrain_id": "terrain11", "position": "D", "annotator_id": "test",
                      "polygon_xy_m": [[0, 0], [1, 0], [0, 1]]}],
        }
        with self.assertRaisesRegex(ValueError, "position"):
            validate_rois(bad)
        bad["rois"][0]["position"] = "A"
        bad["rois"][0]["polygon_xy_m"][0][0] = float("nan")
        with self.assertRaisesRegex(ValueError, "finite"):
            validate_rois(bad)


    def test_cli_writes_separate_review_only_candidate_and_preserves_input(self):
        import contextlib
        import csv
        import io
        import json
        import tempfile
        import zipfile
        from unittest.mock import patch
        from PIL import Image
        import project_crater_candidates as tool

        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            nasa = root / "nasa"
            nasa.mkdir()
            source = root / "images" / "terrain01" / "01_A_off_180_L_0016.png"
            source.parent.mkdir(parents=True)
            Image.new("RGB", (10, 10), (30, 40, 50)).save(source)
            original_mask = root / "masks" / "terrain01" / source.name
            original_mask.parent.mkdir(parents=True)
            Image.new("L", (10, 10), 0).save(original_mask)
            with Image.open(original_mask) as image:
                source_mask = image.copy()
            source_mask.putpixel((4, 5), 2)  # known rock: do not turn it into crater
            source_mask.putpixel((6, 5), 4)  # known shadow: do not turn it into crater
            source_mask.save(original_mask)

            calibration = (
                "CAMERA_MATRIX_LEFT: 10 0 5 0 10 5 0 0 1\n"
                "DISTORTION_COEFFICIENTS_LEFT: 0 0 0 0\n"
                "CAM_LIDAR_ROTATION: 1 0 0 0 1 0 0 0 1\n"
                "CAM_LIDAR_TRANSLATION: 0 0 0\n"
                "IMAGE_WIDTH_HEIGHT: 10 10\n"
            )
            points = "0 0 -0.1 0 1 100\n0 1 0.1 0 1 100\n1 0 0 0.1 1 100\n"
            archive_path = nasa / "Terrain01_Test.zip"
            with zipfile.ZipFile(archive_path, "w") as archive:
                archive.writestr("Terrain01/GroundTruth_ascii/PosA.xyz", points)
                archive.writestr("Terrain01/stereo.calibration", calibration)

            manifest = root / "manifest.csv"
            with manifest.open("w", newline="", encoding="utf-8") as stream:
                writer = csv.DictWriter(stream, fieldnames=[
                    "image", "mask", "source_id", "terrain_id", "split"])
                writer.writeheader()
                writer.writerow({
                    "image": "images/terrain01/" + source.name,
                    "mask": "masks/terrain01/" + source.name,
                    "source_id": "polar-sim-labels", "terrain_id": "terrain01",
                    "split": "train",
                })
            annotations = root / "rois.json"
            annotations.write_text(json.dumps({
                "schema_version": 1,
                "rois": [{"terrain_id": "terrain01", "position": "A", "annotator_id": "fixture",
                          "polygon_xy_m": [[-1, -1], [1, -1], [1, 1], [-1, 1]]}],
            }), encoding="utf-8")
            output = root / "candidate-run"
            args = ["project_crater_candidates", "--manifest", str(manifest),
                    "--nasa-dir", str(nasa), "--annotations", str(annotations),
                    "--output", str(output)]
            with patch("sys.argv", args), contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(0, tool.main())

            candidate = Image.open(output / "candidate_masks/terrain01" / source.name)
            self.assertEqual({3, 255}, set(candidate.getdata()))
            self.assertEqual(1, sum(value == 3 for value in candidate.getdata()))
            report = json.loads((output / "candidate-report.json").read_text())
            self.assertFalse(report["training_ready"])
            self.assertTrue(report["candidates"][0]["requires_human_review"])
            self.assertEqual(1, report["candidates"][0]["candidate_crater_pixels"])
            self.assertEqual(2, report["candidates"][0]["excluded_non_regolith_pixels"])
            self.assertFalse((output / "manifest.csv").exists())
            with Image.open(original_mask) as preserved:
                self.assertEqual({0, 2, 4}, set(preserved.getdata()))


if __name__ == "__main__":
    unittest.main()
