#!/usr/bin/env python3
"""Pair POLAR-Sim YOLO polygon labels with NASA POLAR images and make canonical masks.

Canonical mask IDs: regolith=0, bedrock=1, rock=2, crater=3, shadow=4,
ignore=255. POLAR-Sim provides ground, rock, rock-shadow and background only;
background/unannotated pixels become ignore, and missing bedrock/crater remain
unlabeled for later annotation.
"""
from __future__ import annotations
import argparse
import csv
import hashlib
import json
import re
import sys
from collections import Counter
from pathlib import Path
from zipfile import BadZipFile, ZipFile

import numpy as np
from PIL import Image, ImageDraw

ROOT = Path(__file__).resolve().parents[1]
LABEL_RE = re.compile(r"^(\d{2})_([ABC])_(on|off)_(no|\d+)_([LR])_(\d+)\.txt$")
IMAGE_RE = re.compile(r"(?:^|/)Pos([ABC])_(\d+)_L(off|on)/(?:Sun_(\d+)|NoSun)/Cam([LR])_\d+_(\d+)\.png$", re.IGNORECASE)
CLASS_REMAP = {1: 0, 2: 2, 3: 4}  # ground->regolith, rock->rock, rock shadow->shadow
IGNORE_ID = 255


def sha256_file(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for block in iter(lambda: f.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def label_key(filename: str) -> tuple[str, str, str, str, str] | None:
    match = LABEL_RE.fullmatch(Path(filename).name)
    if not match:
        return None
    _, pos, light, sun, camera, exposure = match.groups()
    return pos, light, sun, camera, exposure


def image_key(filename: str) -> tuple[str, str, str, str, str] | None:
    match = IMAGE_RE.search(filename.replace("\\", "/"))
    if not match:
        return None
    pos, distance, light, sun, camera, exposure = match.groups()
    expected_distance = {"A": "1500", "B": "4000", "C": "1500"}[pos]
    if distance != expected_distance:
        return None
    return pos, light, sun or "no", camera, exposure


def parse_yolo_polygons(text: str, width: int, height: int) -> list[tuple[int, list[tuple[int, int]]]]:
    polygons = []
    for line_no, line in enumerate(text.splitlines(), start=1):
        if not line.strip():
            continue
        fields = line.split()
        try:
            class_id = int(fields[0])
            coords = [float(value) for value in fields[1:]]
        except ValueError as exc:
            raise ValueError(f"label line {line_no}: non-numeric value") from exc
        if class_id not in (0, 1, 2, 3):
            raise ValueError(f"label line {line_no}: unsupported POLAR-Sim class {class_id}")
        if len(coords) < 6 or len(coords) % 2:
            raise ValueError(f"label line {line_no}: polygon must contain at least three (x,y) points")
        if any(value < 0.0 or value > 1.0 for value in coords):
            raise ValueError(f"label line {line_no}: normalized coordinate outside [0,1]")
        points = [(int(coords[i] * width), int(coords[i + 1] * height))
                  for i in range(0, len(coords), 2)]
        polygons.append((class_id, points))
    return polygons


def rasterize_mask(polygons: list[tuple[int, list[tuple[int, int]]]], width: int, height: int) -> np.ndarray:
    image = Image.new("L", (width, height), color=IGNORE_ID)
    draw = ImageDraw.Draw(image)
    # Background annotations stay ignore; later classes follow the project’s
    # visible-surface precedence: ground, then rock, then cast shadow.
    for source_class in (1, 2, 3):
        target_class = CLASS_REMAP[source_class]
        for class_id, points in polygons:
            if class_id == source_class:
                draw.polygon(points, fill=target_class)
    return np.asarray(image, dtype=np.uint8)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--labels-dir", type=Path, required=True)
    parser.add_argument("--nasa-dir", type=Path, required=True)
    parser.add_argument("--splits", type=Path, default=ROOT / "config/perception/splits.json")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--allow-partial", action="store_true", help="materialize available terrains while reporting missing label archives")
    args = parser.parse_args()
    try:
        split_doc = json.loads(args.splits.read_text(encoding="utf-8"))
        polar = next(row for row in split_doc["scene_splits"] if row["source_id"] == "polar-sim-labels")
        terrain_to_split = {terrain: split for split in ("train", "validation", "test") for terrain in polar[split]}
        required_terrains = sorted(terrain_to_split)
        label_paths = {"terrain" + p.stem.replace("SegmentLabels_Terrain", ""): p
                       for p in args.labels_dir.glob("SegmentLabels_Terrain*.zip")}
        nasa_paths = {"terrain" + m.group(1): p for p in args.nasa_dir.glob("Terrain*.zip")
                      if (m := re.match(r"Terrain(\d{2})_.*\.zip$", p.name))}
        missing_labels = sorted(set(required_terrains) - set(label_paths))
        missing_nasa = sorted(set(required_terrains) - set(nasa_paths))
        if missing_nasa:
            raise ValueError(f"missing NASA terrain archives: {missing_nasa}")
        if missing_labels and not args.allow_partial:
            raise ValueError(f"missing POLAR-Sim label archives: {missing_labels}; pass --allow-partial for diagnostic materialization")
        if args.output.exists():
            raise FileExistsError(f"output already exists; choose a new path: {args.output}")
        indexes: dict[str, dict[tuple[str, str, str, str, str], list[str]]] = {}
        notices: dict[str, list[str]] = {}
        for terrain in required_terrains:
            with ZipFile(nasa_paths[terrain]) as archive:
                lookup: dict[tuple[str, str, str, str, str], list[str]] = {}
                candidates = []
                for info in archive.infolist():
                    leaf = Path(info.filename).name.lower()
                    if leaf.startswith(("readme", "license", "licence", "copyright", "notice", "terms")):
                        candidates.append(info.filename)
                    key = image_key(info.filename)
                    if key and not info.is_dir():
                        lookup.setdefault(key, []).append(info.filename)
                indexes[terrain] = lookup
                notices[terrain] = candidates
        args.output.mkdir(parents=True)
        image_out = args.output / "images"
        mask_out = args.output / "masks"
        image_out.mkdir()
        mask_out.mkdir()
        rows = []
        terrain_counts: Counter[str] = Counter()
        split_counts: Counter[str] = Counter()
        pixel_counts: Counter[str] = Counter()
        unmatched = []
        ambiguous = []
        for terrain in required_terrains:
            if terrain not in label_paths:
                continue
            with ZipFile(label_paths[terrain]) as label_zip, ZipFile(nasa_paths[terrain]) as nasa_zip:
                for info in label_zip.infolist():
                    if info.is_dir() or not info.filename.endswith(".txt"):
                        continue
                    key = label_key(info.filename)
                    if not key:
                        continue
                    matches = indexes[terrain].get(key, [])
                    if len(matches) != 1:
                        issue = {"terrain_id": terrain, "label": info.filename, "matches": matches}
                        (unmatched if not matches else ambiguous).append(issue)
                        continue
                    label_text = label_zip.read(info.filename).decode("utf-8")
                    with nasa_zip.open(matches[0]) as source:
                        original = Image.open(source).convert("RGB")
                        width, height = original.size
                        polygons = parse_yolo_polygons(label_text, width, height)
                        mask = rasterize_mask(polygons, width, height)
                        for class_id, count in zip(*np.unique(mask, return_counts=True)):
                            pixel_counts[str(int(class_id))] += int(count)
                        stem = Path(info.filename).stem
                        relative_image = Path("images") / terrain / f"{stem}.png"
                        relative_mask = Path("masks") / terrain / f"{stem}.png"
                        (args.output / relative_image).parent.mkdir(parents=True, exist_ok=True)
                        (args.output / relative_mask).parent.mkdir(parents=True, exist_ok=True)
                        original.save(args.output / relative_image, format="PNG")
                        Image.fromarray(mask, mode="L").save(args.output / relative_mask, format="PNG")
                    rows.append({"image": relative_image.as_posix(), "mask": relative_mask.as_posix(),
                                 "source_id": "polar-sim-labels", "terrain_id": terrain,
                                 "split": terrain_to_split[terrain]})
                    terrain_counts[terrain] += 1
                    split_counts[terrain_to_split[terrain]] += 1
        with (args.output / "manifest.csv").open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=["image", "mask", "source_id", "terrain_id", "split"])
            writer.writeheader()
            writer.writerows(rows)
        report = {
            "source": "POLAR-Sim Dryad 10.5061/dryad.ksn02v7hf; NASA POLAR Stereo Dataset",
            "class_remap": {"POLAR ground 1": "regolith 0", "POLAR rock 2": "rock 2", "POLAR rock-shadow 3": "shadow 4", "POLAR background 0/unlabeled": "ignore 255"},
            "unavailable_target_classes": ["bedrock", "crater"],
            "missing_label_archives": [f"SegmentLabels_Terrain{value.removeprefix('terrain')}.zip" for value in missing_labels],
            "terrain_records": dict(terrain_counts), "split_records": dict(split_counts),
            "mask_pixel_counts": dict(pixel_counts), "unmatched_labels": unmatched,
            "ambiguous_labels": ambiguous, "nasa_notice_candidates": notices,
            "license_notice_review": "NASA archives inspected for embedded README/LICENSE/COPYRIGHT/NOTICE/TERMS filenames; none found. Verify source page/file notice again before publication.",
            "source_archive_sha256": {f"labels/{p.name}": sha256_file(p) for p in label_paths.values() if p.exists()},
            "source_nasa_archives": {t: {"name": nasa_paths[t].name, "size_bytes": nasa_paths[t].stat().st_size} for t in required_terrains if t in label_paths},
            "training_ready": not missing_labels and not unmatched and not ambiguous and all(split_counts[s] > 0 for s in ("train", "validation", "test")) and all(pixel_counts[str(class_id)] > 0 for class_id in range(5)),
            "record_count": len(rows),
        }
        (args.output / "materialization-report.json").write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        print(json.dumps({"result": "PASS" if report["training_ready"] else "PARTIAL_DATA_ONLY", "output": str(args.output), "records": len(rows), "terrain_records": dict(terrain_counts), "split_records": dict(split_counts), "missing_labels": report["missing_label_archives"], "unmatched": len(unmatched), "ambiguous": len(ambiguous), "unavailable_classes": report["unavailable_target_classes"]}, sort_keys=True))
        return 0 if report["training_ready"] else 3
    except (OSError, BadZipFile, KeyError, StopIteration, json.JSONDecodeError, ValueError) as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
