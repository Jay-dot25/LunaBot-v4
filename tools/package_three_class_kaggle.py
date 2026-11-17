#!/usr/bin/env python3
"""Build a Kaggle upload ZIP by replacing masks in the frozen corpus bundle."""
from __future__ import annotations

import argparse
import csv
import hashlib
import io
import json
import shutil
import zipfile
from pathlib import Path


def file_key(path_text: str) -> str:
    path = Path(path_text)
    return "data/" + hashlib.sha256(str(path).encode()).hexdigest() + path.suffix


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open("r", encoding="utf-8-sig", newline="") as stream:
        return list(csv.DictReader(stream))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-archive", type=Path, required=True,
                        help="Existing audited Kaggle ZIP containing the source images")
    parser.add_argument("--source-manifest", type=Path, required=True,
                        help="Full canonical five-class CSV used to derive the masks")
    parser.add_argument("--derived-manifest", type=Path, required=True,
                        help="Manifest written by build_three_class_dataset.py")
    parser.add_argument("--derived-report", type=Path, required=True)
    parser.add_argument("--classes", type=Path, required=True)
    parser.add_argument("--sources", type=Path, required=True)
    parser.add_argument("--splits", type=Path, required=True)
    parser.add_argument("--gates", type=Path, required=True)
    parser.add_argument("--repo", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    source_rows = read_csv(args.source_manifest)
    derived_rows = read_csv(args.derived_manifest)
    if len(source_rows) != len(derived_rows):
        raise ValueError("source and derived manifests have different record counts")
    expected_fields = ("source_id", "terrain_id", "split")
    remap: dict[str, Path] = {}
    for i, (source, derived) in enumerate(zip(source_rows, derived_rows), 1):
        if any(source[name] != derived[name] for name in expected_fields):
            raise ValueError(f"source/split assignment changed at row {i}")
        # The source manifest is known-good from the completed derivation. Do
        # not resolve tens of thousands of image paths here: source images can
        # sit on a slow mounted Windows drive, and resolution adds a stat RPC
        # for every row before packaging even starts.
        if Path(source["image"]) != Path(derived["image"]):
            raise ValueError(f"source image changed at row {i}")
        archive_mask = file_key(source["mask"])
        new_mask = Path(derived["mask"])
        if not new_mask.is_file():
            raise FileNotFoundError(new_mask)
        previous = remap.get(archive_mask)
        if previous and sha256_file(previous) != sha256_file(new_mask):
            raise ValueError(f"one package mask path maps to different three-class data: {archive_mask}")
        remap[archive_mask] = new_mask

    fields = ["image", "mask", "source_id", "terrain_id", "split"]
    portable_rows = [{"image": file_key(row["image"]), "mask": file_key(source["mask"]),
                      **{name: row[name] for name in expected_fields}}
                     for source, row in zip(source_rows, derived_rows)]
    manifest_stream = io.StringIO(newline="")
    writer = csv.DictWriter(manifest_stream, fieldnames=fields)
    writer.writeheader()
    writer.writerows(portable_rows)
    expected_data = {row[field] for row in portable_rows for field in ("image", "mask")}

    args.output.parent.mkdir(parents=True, exist_ok=True)
    partial = args.output.with_suffix(args.output.suffix + ".partial")
    if partial.exists():
        partial.unlink()
    with zipfile.ZipFile(args.source_archive, "r") as source_zip, \
            zipfile.ZipFile(partial, "w", compression=zipfile.ZIP_STORED, allowZip64=True) as output_zip:
        names = {info.filename for info in source_zip.infolist()}
        if not expected_data.issubset(names):
            missing = sorted(expected_data - names)[:5]
            raise ValueError(f"source archive is missing expected image/mask members: {missing}")
        output_zip.writestr("manifest.csv", manifest_stream.getvalue())
        for file_name, archive_name in [
            (args.classes, "config/classes.json"),
            (args.sources, "config/sources.json"),
            (args.splits, "config/splits.json"),
            (args.gates, "config/gates.json"),
            (args.derived_report, "config/three-class-dataset-report.json"),
            (args.source_manifest.parent / "materialization.json", "config/source-materialization.json"),
            (args.source_manifest.parent / "audit.json", "config/source-corpus-audit.json"),
        ]:
            if file_name.is_file():
                output_zip.write(file_name, archive_name)
        for name in ["train_perception.py", "perception_metrics.py"]:
            output_zip.write(args.repo / "tools" / name, "tools/" + name)
        readme = f"""# P5-02 three-class Kaggle dataset

This private upload bundle has {len(portable_rows):,} rows. Its masks contain
only regolith (0), rock (1), shadow (2), and ignore (255). Original bedrock
labels were merged into rock; crater and prior ignore pixels map to ignore.
Crater risk remains the geometry layer's responsibility.

The scene-isolated row assignments are preserved: 32,104 train, 746
validation, 749 test. Use `notebooks/p502-three-class-kaggle.ipynb` from the
project repository. Enable a Kaggle GPU. The notebook includes chunked resume
instructions and leaves test scoring disabled until the final training chunk.

This archive contains the data and trainer only. `config/` contains the class,
source, split, gate, and derivation reports. All images and masks are under
`data/` and referenced by `manifest.csv`.
"""
        output_zip.writestr("README.md", readme)
        data_names = sorted(name for name in names if name.startswith("data/"))
        if set(data_names) != expected_data:
            extra = set(data_names) - expected_data
            missing = expected_data - set(data_names)
            raise ValueError(f"source archive data members differ from manifest (extra={len(extra)}, missing={len(missing)})")
        for i, name in enumerate(data_names, 1):
            replacement = remap.get(name)
            with output_zip.open(name, "w") as target:
                if replacement:
                    with replacement.open("rb") as source:
                        shutil.copyfileobj(source, target, 1024 * 1024)
                else:
                    with source_zip.open(name, "r") as source:
                        shutil.copyfileobj(source, target, 8 * 1024 * 1024)
            if i % 5000 == 0:
                print(f"Copied {i:,}/{len(data_names):,} data files", flush=True)

    partial.replace(args.output)
    summary = {
        "archive": str(args.output),
        "archive_sha256": sha256_file(args.output),
        "archive_bytes": args.output.stat().st_size,
        "records": len(portable_rows),
        "unique_data_files": len(expected_data),
        "unique_masks_remapped": len(remap),
        "source_archive_sha256": sha256_file(args.source_archive),
        "source_manifest_sha256": sha256_file(args.source_manifest),
        "derived_manifest_sha256": sha256_file(args.derived_manifest),
        "three_class_report_sha256": sha256_file(args.derived_report),
    }
    args.output.with_suffix(args.output.suffix + ".json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(summary, sort_keys=True), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
