#!/usr/bin/env python3
"""Merge explicitly accepted crater candidates into new trainer masks.

This writes a new manifest and copies modified source masks into a separate
output tree. It never overwrites source masks or changes the input manifest.
Only train-split candidates are accepted by this first qualification workflow.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import sys
from pathlib import Path

import numpy as np
from PIL import Image

IGNORE_ID = 255
CRATER_ID = 3


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def resolve(base: Path, value: str) -> Path:
    path = Path(value)
    return path.resolve() if path.is_absolute() else (base / path).resolve()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--candidate-report", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--accept-annotator-id", required=True,
                        help="exact delegated reviewer ID recorded in the ROI report")
    args = parser.parse_args()
    try:
        manifest = args.manifest.resolve()
        report_path = args.candidate_report.resolve()
        report = json.loads(report_path.read_text(encoding="utf-8"))
        if report.get("training_ready") is not False:
            raise ValueError("candidate report must remain review-only")
        rois = report.get("roi_definitions", [])
        if not rois or any(roi.get("annotator_id") != args.accept_annotator_id for roi in rois):
            raise ValueError("every ROI must carry the explicitly accepted annotator ID")
        if args.output.exists():
            raise FileExistsError(f"output already exists; choose a new path: {args.output}")

        with manifest.open("r", encoding="utf-8-sig", newline="") as stream:
            reader = csv.DictReader(stream)
            if not reader.fieldnames or not {"image", "mask", "source_id", "terrain_id", "split"}.issubset(reader.fieldnames):
                raise ValueError("manifest must include image, mask, source_id, terrain_id, and split")
            fieldnames = list(reader.fieldnames)
            rows = list(reader)
        by_image = {}
        for row in rows:
            image = resolve(manifest.parent, row["image"])
            if image in by_image:
                raise ValueError(f"duplicate manifest image path: {image}")
            by_image[image] = row

        accepted = {}
        ignored_nontraining = 0
        for candidate in report.get("candidates", []):
            if candidate.get("split") != "train":
                ignored_nontraining += 1
                continue
            if candidate.get("candidate_class_id") != CRATER_ID or not candidate.get("requires_human_review"):
                raise ValueError("candidate report contains an unexpected class or review state")
            image = Path(candidate["image"]).resolve()
            row = by_image.get(image)
            if row is None:
                raise ValueError(f"candidate image is absent from input manifest: {image}")
            if row["split"] != "train" or row["terrain_id"] != candidate["terrain_id"]:
                raise ValueError(f"candidate split/terrain disagrees with manifest: {image}")
            if candidate["terrain_id"] not in {roi["terrain_id"] for roi in rois}:
                raise ValueError(f"candidate terrain has no accepted ROI: {candidate['terrain_id']}")
            accepted[image] = candidate
        if not accepted:
            raise ValueError("no train-split crater candidates found")

        args.output.mkdir(parents=True)
        merged_dir = args.output / "merged_masks"
        merged_dir.mkdir()
        merged_pixels = excluded_pixels = 0
        output_rows = []
        for row in rows:
            image = resolve(manifest.parent, row["image"])
            candidate = accepted.get(image)
            updated = dict(row)
            # The output manifest lives in a different directory. Rebase both
            # unchanged paths as well as paths for images with modified masks.
            updated["image"] = str(image)
            updated["mask"] = str(resolve(manifest.parent, row["mask"]))
            if candidate is not None:
                source_mask_path = resolve(manifest.parent, row["mask"])
                with Image.open(source_mask_path) as source_image:
                    if source_image.mode not in ("L", "P"):
                        raise ValueError(f"source mask must be indexed grayscale/palette: {source_mask_path}")
                    source = np.asarray(source_image, dtype=np.uint8).copy()
                candidate_path = Path(candidate["candidate_mask"]).resolve()
                with Image.open(candidate_path) as candidate_image:
                    if candidate_image.mode not in ("L", "P"):
                        raise ValueError(f"candidate mask must be indexed grayscale/palette: {candidate_path}")
                    proposed = np.asarray(candidate_image, dtype=np.uint8)
                if proposed.shape != source.shape:
                    raise ValueError(f"candidate/source mask dimensions differ: {candidate_path}")
                unexpected = set(np.unique(proposed).tolist()) - {CRATER_ID, IGNORE_ID}
                if unexpected:
                    raise ValueError(f"candidate mask contains unexpected IDs: {sorted(unexpected)}")
                proposed_crater = proposed == CRATER_ID
                if np.any(source[proposed_crater] != 0):
                    raise ValueError("candidate attempts to overwrite a non-regolith source pixel")
                merged_pixels += int(proposed_crater.sum())
                excluded_pixels += int(candidate.get("excluded_non_regolith_pixels", 0))
                source[proposed_crater] = CRATER_ID
                relative = Path(row["terrain_id"]) / f"{image.stem}-mask.png"
                destination = merged_dir / relative
                destination.parent.mkdir(parents=True, exist_ok=True)
                Image.fromarray(source, mode="L").save(destination, format="PNG")
                updated["mask"] = str(destination.resolve())
            output_rows.append(updated)

        out_manifest = args.output / "manifest.csv"
        with out_manifest.open("w", encoding="utf-8", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(output_rows)
        summary = {
            "result": "TRAIN_CANDIDATES_MERGED",
            "training_ready": False,
            "input_manifest": str(manifest),
            "input_manifest_sha256": sha256_file(manifest),
            "candidate_report": str(report_path),
            "candidate_report_sha256": sha256_file(report_path),
            "accepted_annotator_id": args.accept_annotator_id,
            "accepted_train_images": len(accepted),
            "ignored_validation_test_candidates": ignored_nontraining,
            "merged_crater_pixels": merged_pixels,
            "excluded_known_non_regolith_pixels": excluded_pixels,
            "output_manifest": str(out_manifest),
            "source_manifests_and_masks_modified": False,
            "qualification_note": "Held-out candidates are intentionally not merged. Run class-support preflight before any model training.",
        }
        (args.output / "merge-report.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
        print(json.dumps(summary, sort_keys=True))
        return 0
    except (OSError, KeyError, json.JSONDecodeError, ValueError) as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
