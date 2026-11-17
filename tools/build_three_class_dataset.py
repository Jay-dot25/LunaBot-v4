#!/usr/bin/env python3
"""Derive P5-02's three-class masks from an audited canonical five-class corpus.

Mapping: regolith stays regolith; bedrock and discrete rock merge to rock;
shadow stays shadow; crater and existing ignore pixels become ignore (255).
Source masks are read-only and preserved. The output manifest reuses source
images and points at newly derived masks.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
from collections import Counter, defaultdict
from pathlib import Path

import numpy as np
from PIL import Image

MAPPING = np.full(256, 255, dtype=np.uint8)
MAPPING[0] = 0  # regolith
MAPPING[1] = 1  # bedrock -> rock
MAPPING[2] = 1  # discrete rock
MAPPING[4] = 2  # shadow
MAPPING[255] = 255  # ignore
IGNORE_ID = 255


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    manifest = args.manifest.resolve()
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    counts: dict[str, Counter[int]] = defaultdict(Counter)
    source_counts: Counter[str] = Counter()
    split_counts: Counter[str] = Counter()
    allowed = {"image", "mask", "source_id", "terrain_id", "split"}
    with manifest.open("r", encoding="utf-8-sig", newline="") as stream:
        reader = csv.DictReader(stream)
        if not reader.fieldnames or not allowed.issubset(reader.fieldnames):
            raise ValueError(f"manifest columns must include {sorted(allowed)}")
        rows = list(reader)
    if not rows:
        raise ValueError("input manifest is empty")

    output_manifest = out / "manifest.csv"
    with output_manifest.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=["image", "mask", "source_id", "terrain_id", "split"])
        writer.writeheader()
        for index, row in enumerate(rows):
            image_path, source_mask = Path(row["image"]).resolve(), Path(row["mask"]).resolve()
            if not image_path.is_file() or not source_mask.is_file():
                raise FileNotFoundError(f"manifest row {index + 2} has missing image or mask")
            with Image.open(source_mask) as image:
                if image.mode not in ("L", "P"):
                    raise ValueError(f"mask must be grayscale/palette PNG: {source_mask} ({image.mode})")
                original = np.asarray(image, dtype=np.uint8)
            values = set(np.unique(original).tolist())
            if not values.issubset({0, 1, 2, 3, 4, 255}):
                raise ValueError(f"unexpected source class IDs in {source_mask}: {sorted(values)}")
            derived = MAPPING[original]
            split = row["split"]
            target_dir = out / "masks" / split
            target_dir.mkdir(parents=True, exist_ok=True)
            mask_name = f"{index:05d}-{hashlib.sha256(str(source_mask).encode()).hexdigest()[:16]}.png"
            target_mask = target_dir / mask_name
            Image.fromarray(derived, mode="L").save(target_mask, optimize=False)
            values, pixel_counts = np.unique(derived, return_counts=True)
            for value, count in zip(values.tolist(), pixel_counts.tolist()):
                counts[split][int(value)] += int(count)
            source_counts[f"{split}/{row['source_id']}"] += 1
            split_counts[split] += 1
            writer.writerow({"image": str(image_path), "mask": str(target_mask),
                             "source_id": row["source_id"], "terrain_id": row["terrain_id"],
                             "split": split})

    if set(split_counts) != {"train", "validation", "test"}:
        raise ValueError(f"expected train/validation/test splits, found {dict(split_counts)}")
    missing = {split: [cid for cid in range(3) if counts[split][cid] == 0]
               for split in split_counts if any(counts[split][cid] == 0 for cid in range(3))}
    if missing:
        raise ValueError(f"three-class pixel support missing: {missing}")
    report = {
        "schema_version": 1,
        "mapping": {"0": "regolith", "1": "bedrock merged into rock", "2": "rock",
                    "3": "ignore (crater is outside three-class scope)", "4": "shadow", "255": "ignore"},
        "class_ids": {"0": "regolith", "1": "rock", "2": "shadow", "255": "ignore"},
        "source_manifest": str(manifest),
        "source_manifest_sha256": sha256_file(manifest),
        "derived_manifest": str(output_manifest),
        "records_by_split": dict(split_counts),
        "records_by_split_source": dict(sorted(source_counts.items())),
        "pixels_by_split_id": {split: {str(cid): count for cid, count in sorted(values.items())}
                               for split, values in sorted(counts.items())},
        "records": len(rows),
    }
    (out / "build-report.json").write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps({"result": "PASS", "output": str(out), "records": len(rows),
                      "records_by_split": dict(split_counts), "pixels_by_split_id": report["pixels_by_split_id"]}, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
