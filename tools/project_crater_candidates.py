#!/usr/bin/env python3
"""Project explicit POLAR LIDAR XY crater regions into left-camera images.

Outputs sparse, diagnostic candidate masks and overlays only. It never changes
source masks or creates a trainer manifest. Unprojected pixels remain ignore
(255); every candidate requires visual review before training use.

Annotations are JSON:
{"schema_version":1,"rois":[
  {"terrain_id":"terrain11","position":"A",
   "polygon_xy_m":[[-5,-1],[-4,-1],[-4,1],[-5,1]]}
]}
Coordinates are in the terrain's LIDAR XYZ frame, in meters.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import re
import sys
from pathlib import Path
from zipfile import BadZipFile, ZipFile

import numpy as np
from PIL import Image

IGNORE_ID = 255
CRATER_ID = 3
TERRAIN_RE = re.compile(r"^terrain\d{2}$")
IMAGE_STEM_RE = re.compile(r"^\d{2}_([ABC])_(on|off)_(no|\d+)_([LR])_(\d+)\.png$", re.I)


def locate_lidar_members(archive: ZipFile, position: str) -> tuple[str, str]:
    """Find point-cloud and calibration members regardless of archive root name."""
    suffix = f"/GroundTruth_ascii/Pos{position}.xyz"
    matches = [name for name in archive.namelist() if name.endswith(suffix)]
    if len(matches) != 1:
        raise ValueError(f"expected one GroundTruth_ascii/Pos{position}.xyz member; found {len(matches)}")
    xyz_name = matches[0]
    terrain_root = xyz_name.rsplit("/GroundTruth_ascii/", 1)[0]
    calibration_name = f"{terrain_root}/stereo.calibration"
    if calibration_name not in archive.namelist():
        raise ValueError(f"missing archive member {calibration_name}")
    return xyz_name, calibration_name


def parse_calibration(text: str):
    values = {}
    for line in text.splitlines():
        if ":" in line:
            key, raw = line.split(":", 1)
            values[key.strip()] = np.fromstring(raw, sep=" ")
    sizes = {
        "CAMERA_MATRIX_LEFT": 9,
        "DISTORTION_COEFFICIENTS_LEFT": 4,
        "CAM_LIDAR_ROTATION": 9,
        "CAM_LIDAR_TRANSLATION": 3,
        "IMAGE_WIDTH_HEIGHT": 2,
    }
    for key, size in sizes.items():
        if key not in values or values[key].size != size or not np.isfinite(values[key]).all():
            raise ValueError(f"invalid or missing calibration field {key}")
    width, height = map(int, values["IMAGE_WIDTH_HEIGHT"])
    if width <= 0 or height <= 0:
        raise ValueError("calibrated image dimensions must be positive")
    return (values["CAMERA_MATRIX_LEFT"].reshape(3, 3),
            values["DISTORTION_COEFFICIENTS_LEFT"],
            values["CAM_LIDAR_ROTATION"].reshape(3, 3),
            values["CAM_LIDAR_TRANSLATION"], (width, height))


def points_in_polygon(x, y, polygon):
    """Vectorized even-odd test; points on the polygon edge are included."""
    inside = np.zeros(x.shape, dtype=bool)
    edge = np.zeros(x.shape, dtype=bool)
    for i, (x1, y1) in enumerate(polygon):
        x2, y2 = polygon[(i + 1) % len(polygon)]
        cross = (x - x1) * (y2 - y1) - (y - y1) * (x2 - x1)
        edge |= ((np.abs(cross) <= 1e-9)
                 & (x >= min(x1, x2) - 1e-9) & (x <= max(x1, x2) + 1e-9)
                 & (y >= min(y1, y2) - 1e-9) & (y <= max(y1, y2) + 1e-9))
        crossing = ((y1 > y) != (y2 > y)) & (
            x < (x2 - x1) * (y - y1) / ((y2 - y1) if y2 != y1 else 1.0) + x1)
        inside ^= crossing
    return inside | edge


def project_lidar_points(points, matrix, distortion, rotation, translation, image_size):
    """Project LIDAR XYZ into the left camera; return unique in-frame pixels."""
    # CAM_LIDAR calibration is camera-to-LIDAR; invert it for LIDAR-to-camera.
    camera = (points.astype(np.float64, copy=False) - translation) @ rotation
    front = camera[:, 2] > 0.05
    p = camera[front]
    x, y = p[:, 0] / p[:, 2], p[:, 1] / p[:, 2]
    k1, k2, p1, p2 = distortion
    r2 = x * x + y * y
    radial = 1 + k1 * r2 + k2 * r2 * r2
    xd = x * radial + 2 * p1 * x * y + p2 * (r2 + 2 * x * x)
    yd = y * radial + p1 * (r2 + 2 * y * y) + 2 * p2 * x * y
    u = np.rint(matrix[0, 0] * xd + matrix[0, 2]).astype(np.int32)
    v = np.rint(matrix[1, 1] * yd + matrix[1, 2]).astype(np.int32)
    width, height = image_size
    valid = (u >= 0) & (u < width) & (v >= 0) & (v < height)
    pixels = np.unique(np.column_stack((u[valid], v[valid])), axis=0)
    return pixels, int(front.sum())


def validate_rois(document):
    if document.get("schema_version") != 1 or not isinstance(document.get("rois"), list):
        raise ValueError("annotations require schema_version=1 and a rois array")
    grouped = {}
    for index, roi in enumerate(document["rois"]):
        if not isinstance(roi, dict):
            raise ValueError(f"roi {index}: expected an object")
        terrain, position = roi.get("terrain_id"), roi.get("position")
        if not isinstance(terrain, str) or not TERRAIN_RE.fullmatch(terrain):
            raise ValueError(f"roi {index}: terrain_id must look like terrain11")
        if position not in ("A", "B", "C"):
            raise ValueError(f"roi {index}: position must be A, B, or C")
        if not isinstance(roi.get("annotator_id"), str) or not roi["annotator_id"].strip():
            raise ValueError(f"roi {index}: annotator_id is required")
        polygon = roi.get("polygon_xy_m")
        if not isinstance(polygon, list) or len(polygon) < 3:
            raise ValueError(f"roi {index}: polygon_xy_m requires at least three points")
        checked = []
        for point in polygon:
            if (not isinstance(point, list) or len(point) != 2
                    or any(isinstance(v, bool) or not isinstance(v, (int, float))
                           or not math.isfinite(v) for v in point)):
                raise ValueError(f"roi {index}: polygon points must be finite [x, y] pairs")
            checked.append([float(point[0]), float(point[1])])
        grouped.setdefault((terrain, position), []).append(checked)
    if not grouped:
        raise ValueError("annotations contain no crater ROIs")
    return grouped


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, required=True,
                        help="completed materialize_perception manifest.csv")
    parser.add_argument("--nasa-dir", type=Path, required=True)
    parser.add_argument("--annotations", type=Path, required=True,
                        help="explicit crater ROI polygons in the LIDAR XY frame")
    parser.add_argument("--output", type=Path, required=True,
                        help="new directory for candidate masks and overlays")
    parser.add_argument("--overlay-limit", type=int, default=12,
                        help="maximum number of full-size review overlays (default: 12)")
    args = parser.parse_args()
    try:
        if args.overlay_limit < 0:
            raise ValueError("--overlay-limit must be non-negative")
        annotation_bytes = args.annotations.read_bytes()
        annotation_hash = hashlib.sha256(annotation_bytes).hexdigest()
        document = json.loads(annotation_bytes)
        rois = validate_rois(document)
        if args.output.exists():
            raise FileExistsError(f"output already exists; choose a new path: {args.output}")
        with args.manifest.open("r", encoding="utf-8-sig", newline="") as stream:
            reader = csv.DictReader(stream)
            required = {"image", "mask", "source_id", "terrain_id", "split"}
            if not reader.fieldnames or not required.issubset(reader.fieldnames):
                raise ValueError(f"manifest columns must include {sorted(required)}")
            records = list(reader)

        archives = {}
        for path in args.nasa_dir.glob("Terrain*.zip"):
            match = re.match(r"^Terrain(\d{2})_.*\.zip$", path.name)
            if match:
                archives["terrain" + match.group(1)] = path
        args.output.mkdir(parents=True)
        masks_dir, overlays_dir = args.output / "candidate_masks", args.output / "overlays"
        masks_dir.mkdir()
        overlays_dir.mkdir()
        results, missing = [], []

        for (terrain, position), polygons in sorted(rois.items()):
            archive_path = archives.get(terrain)
            if archive_path is None:
                raise FileNotFoundError(f"NASA archive missing for {terrain}")
            selected_records = []
            for row in records:
                stem = Path(row["image"]).name
                match = IMAGE_STEM_RE.fullmatch(stem)
                if (row["terrain_id"] == terrain and match
                        and match.group(1).upper() == position
                        and match.group(4).upper() == "L"):
                    selected_records.append(row)
            if not selected_records:
                missing.append({"terrain_id": terrain, "position": position,
                                "reason": "no left-camera records in input manifest"})
                continue

            with ZipFile(archive_path) as archive:
                try:
                    xyz_name, calibration_name = locate_lidar_members(archive, position)
                    with archive.open(xyz_name) as stream:
                        all_points = np.loadtxt(stream, usecols=(2, 3, 4), dtype=np.float32)
                    calibration = archive.read(calibration_name).decode("utf-8")
                except KeyError as exc:
                    raise ValueError(f"{archive_path.name}: missing archive member {exc}") from exc
            matrix, distortion, rotation, translation, image_size = parse_calibration(calibration)
            xy = all_points[:, :2]
            selected = np.zeros(len(all_points), dtype=bool)
            for polygon in polygons:
                selected |= points_in_polygon(xy[:, 0], xy[:, 1], polygon)
            if not selected.any():
                raise ValueError(f"{terrain}/Pos{position}: supplied polygons select no LIDAR points")
            pixels, points_in_front = project_lidar_points(
                all_points[selected], matrix, distortion, rotation, translation, image_size)
            if not len(pixels):
                raise ValueError(f"{terrain}/Pos{position}: selected points project outside the image")
            u, v = pixels[:, 0], pixels[:, 1]

            for row in selected_records:
                image_path = Path(row["image"])
                if not image_path.is_absolute():
                    image_path = args.manifest.parent / image_path
                if not image_path.is_file():
                    raise FileNotFoundError(f"manifest image missing: {image_path}")
                with Image.open(image_path) as source:
                    image = source.convert("RGB")
                width, height = image.size
                if image.size != image_size:
                    raise ValueError(f"{image_path}: image size {image.size} differs from calibration {image_size}")
                source_mask_path = Path(row["mask"])
                if not source_mask_path.is_absolute():
                    source_mask_path = args.manifest.parent / source_mask_path
                if not source_mask_path.is_file():
                    raise FileNotFoundError(f"manifest mask missing: {source_mask_path}")
                with Image.open(source_mask_path) as source_mask_image:
                    source_mask = np.asarray(source_mask_image.convert("L"), dtype=np.uint8)
                if source_mask.shape != (height, width):
                    raise ValueError(
                        f"{source_mask_path}: mask size {source_mask.shape[::-1]} differs from image {(width, height)}")

                # Crater ROIs describe a geomorphic region, but the region can
                # contain rocks, bedrock, shadows, or unannotated pixels. Only
                # relabel pixels already known to be regolith; preserve every
                # other source class and keep unknown pixels ignored.
                eligible = source_mask[v, u] == 0
                candidate_pixels = int(eligible.sum())
                excluded_pixels = int(len(pixels) - candidate_pixels)
                candidate = np.full((height, width), IGNORE_ID, dtype=np.uint8)
                candidate[v[eligible], u[eligible]] = CRATER_ID
                rel = Path(terrain) / image_path.name
                mask_path = masks_dir / rel
                mask_path.parent.mkdir(parents=True, exist_ok=True)
                Image.fromarray(candidate, mode="L").save(mask_path, format="PNG")
                overlay_path = None
                if len(results) < args.overlay_limit:
                    overlay = np.asarray(image).copy()
                    overlay[v[eligible], u[eligible]] = (255, 0, 255)
                    overlay_path = overlays_dir / rel
                    overlay_path.parent.mkdir(parents=True, exist_ok=True)
                    Image.fromarray(overlay, mode="RGB").save(overlay_path, format="PNG")
                results.append({
                    "terrain_id": terrain, "position": position, "split": row["split"],
                    "image": str(image_path), "candidate_mask": str(mask_path),
                    "overlay": str(overlay_path) if overlay_path else None,
                    "lidar_points_in_front": points_in_front,
                    "unique_projected_pixels": int(len(pixels)),
                    "candidate_crater_pixels": candidate_pixels,
                    "excluded_non_regolith_pixels": excluded_pixels,
                    "candidate_class_id": CRATER_ID, "requires_human_review": True,
                })

        report = {
            "result": "CANDIDATES_WRITTEN_REVIEW_REQUIRED",
            "training_ready": False,
            "source": "NASA POLAR GroundTruth_ascii XYZ and stereo.calibration",
            "transform": "camera_point = (lidar_point - CAM_LIDAR_TRANSLATION) @ CAM_LIDAR_ROTATION",
            "label_rule": "Projected points inside explicitly supplied LIDAR-XY polygons become class 3 only where the source mask already labels regolith (0); known rock/bedrock/shadow and unknown pixels are not relabeled.",
            "warning": "Sparse projection candidates only. Do not merge into training masks until boundaries, coverage and occlusion are reviewed.",
            "input_manifest_unchanged": str(args.manifest),
            "annotations_file": str(args.annotations),
            "annotations_sha256": annotation_hash,
            "roi_definitions": document["rois"],
            "candidate_count": len(results),
            "missing_positions": missing,
            "candidates": results,
        }
        (args.output / "candidate-report.json").write_text(
            json.dumps(report, indent=2) + "\n", encoding="utf-8")
        print(json.dumps({"result": report["result"], "output": str(args.output),
                          "candidate_count": len(results), "missing_positions": missing,
                          "training_ready": False}, sort_keys=True))
        return 0
    except (OSError, BadZipFile, KeyError, json.JSONDecodeError, ValueError) as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
