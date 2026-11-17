#!/usr/bin/env python3
"""Materialize only the approved MarsSeg MSL training shard for bedrock pretraining.

Source masks map 1 Bedrock->1, 2 Gravel/Sand/Soil->0 (regolith),
3 Rock->2, and 4 Shadow->4. Background, sky/mountains and track map to
IGNORE=255. This source is always train-only and never supports lunar/analog
validation or test metrics. Outputs are kept outside the repository.
"""
from __future__ import annotations
import argparse
import csv
import hashlib
import json
from collections import Counter
from datetime import datetime, timezone
from io import BytesIO
from pathlib import Path
import shutil

import numpy as np
from PIL import Image

SOURCE_ID = "marsseg-msl-bedrock-pretrain"
IGNORE_ID = 255
REMAP = {0: IGNORE_ID, 1: 1, 2: 0, 3: 2, 4: 4, 5: IGNORE_ID, 6: IGNORE_ID}

def remap_mask(mask: np.ndarray) -> np.ndarray:
    """Map audited source IDs into canonical IDs; all other source IDs fail."""
    ids = set(map(int, np.unique(mask)))
    unknown = ids - set(REMAP)
    if unknown:
        raise ValueError(f"unsupported source mask IDs: {sorted(unknown)}")
    output = np.full(mask.shape, IGNORE_ID, dtype=np.uint8)
    for source_id, target_id in REMAP.items():
        output[mask == source_id] = target_id
    return output


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--parquet", type=Path, required=True)
    parser.add_argument("--mapping", type=Path, required=True)
    parser.add_argument("--source-readme", type=Path, required=True)
    parser.add_argument("--revision", required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    try:
        import pyarrow.parquet as pq
    except ImportError as exc:
        parser.error(f"pyarrow is required to read the source Parquet file: {exc}")

    if args.output.exists():
        parser.error(f"output already exists; choose a new path: {args.output}")
    for source in (args.parquet, args.mapping, args.source_readme):
        if not source.is_file() or source.stat().st_size == 0:
            parser.error(f"missing or empty input file: {source}")
    source_map = json.loads(args.mapping.read_text(encoding="utf-8"))
    expected_map = {"0": "Background", "1": "Bedrock", "2": "Gravel / Sand / Soil",
                    "3": "Rock", "4": "Shadow", "5": "Sky / Distant Mountains", "6": "Track"}
    if source_map != expected_map:
        parser.error("source mapping differs from the audited seven-class mapping")

    table = pq.ParquetFile(args.parquet)
    required_columns = {"image", "mask", "width", "height", "class_labels"}
    schema_names = set(pq.read_schema(args.parquet).names)
    if not required_columns.issubset(schema_names):
        parser.error(f"Parquet schema missing columns: {sorted(required_columns - schema_names)}")
    args.output.mkdir(parents=True)
    image_dir, mask_dir = args.output / "images", args.output / "masks"
    image_dir.mkdir(); mask_dir.mkdir()
    manifest_path = args.output / "manifest.csv"
    rows = 0
    pixel_counts: Counter[int] = Counter()
    class_image_counts: Counter[str] = Counter()
    try:
        with manifest_path.open("w", encoding="utf-8", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=("image", "mask", "source_id", "terrain_id", "split", "source_sample"))
            writer.writeheader()
            for batch in table.iter_batches(columns=sorted(required_columns), batch_size=32):
                for row in batch.to_pylist():
                    source_image = Image.open(BytesIO(row["image"]["bytes"])).convert("RGB")
                    with Image.open(BytesIO(row["mask"]["bytes"])) as source_mask:
                        mask = np.asarray(source_mask.convert("L"), dtype=np.uint8)
                    if source_image.size != (row["width"], row["height"]) or mask.shape != (row["height"], row["width"]):
                        raise ValueError(f"dimension mismatch at source row {rows}: {row['image'].get('path')}")
                    try:
                        mapped = remap_mask(mask)
                    except ValueError as exc:
                        raise ValueError(f"source row {rows}: {exc}") from exc
                    stem = f"mars_msl_{rows:05d}"
                    image_rel = Path("images") / f"{stem}.png"
                    mask_rel = Path("masks") / f"{stem}.png"
                    source_image.save(args.output / image_rel, format="PNG")
                    Image.fromarray(mapped, mode="L").save(args.output / mask_rel, format="PNG")
                    ids, counts = np.unique(mapped, return_counts=True)
                    for class_id, count in zip(ids, counts):
                        pixel_counts[int(class_id)] += int(count)
                    class_image_counts.update(row["class_labels"] or [])
                    writer.writerow({"image": image_rel.as_posix(), "mask": mask_rel.as_posix(),
                                     "source_id": SOURCE_ID, "terrain_id": "marsseg-msl-train", "split": "train",
                                     "source_sample": row["image"].get("path") or f"row-{rows}"})
                    rows += 1
        if rows != table.metadata.num_rows:
            raise ValueError(f"row count mismatch: wrote {rows} of {table.metadata.num_rows}")
        report = {
            "schema_version": 1, "source_id": SOURCE_ID,
            "repository": "gremlin97/mars-seg_msl", "revision": args.revision,
            "license": "CC-BY-4.0", "attribution": "Credit gremlin97/mars-seg_msl; cite the AI4Mars / Mars-Bench source as documented by the dataset authors.",
            "split": "train", "train_only": True, "validation_or_test_records": 0,
            "source_rows": rows, "materialized_images": rows, "materialized_masks": rows,
            "source_mask_id_mapping": {str(k): v for k, v in REMAP.items()},
            "pixel_counts_by_canonical_id": {str(k): v for k, v in sorted(pixel_counts.items())},
            "source_class_image_counts": dict(sorted(class_image_counts.items())),
            "source_files": {p.name: {"size_bytes": p.stat().st_size, "sha256": sha256(p)}
                             for p in (args.parquet, args.mapping, args.source_readme)},
            "training_ready": False, "requires_lunar_analog_validation": True,
            "note": "Mars-only training aid; no crater supervision; not a held-out evaluation source. Ignore means unknown/unlabeled, not a surface class.",
            "created_at_utc": datetime.now(timezone.utc).isoformat(),
        }
        (args.output / "materialization-report.json").write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    except Exception:
        # This converter exclusively creates its chosen, previously absent output.
        shutil.rmtree(args.output, ignore_errors=True)
        raise
    print(json.dumps({"result": "PASS_TRAIN_ONLY_MATERIALIZATION", "output": str(args.output),
                      "records": rows, "pixel_counts_by_canonical_id": report["pixel_counts_by_canonical_id"],
                      "training_ready": False}, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
