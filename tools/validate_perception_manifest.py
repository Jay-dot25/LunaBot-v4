#!/usr/bin/env python3
"""Validate five-class ontology, source rights gates, and scene-level split policy."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

EXPECTED = ["regolith", "bedrock", "rock", "crater", "shadow"]
ALLOWED_LICENSES = {"CC0-1.0", "CC-BY-4.0"}
SPLITS = ("train", "validation", "test")


def read_json(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"{path}: top level must be a JSON object")
    return value


def validate(classes: dict[str, Any], sources: dict[str, Any],
             split_policy: dict[str, Any]) -> list[str]:
    errors: list[str] = []
    rows = classes.get("classes", [])
    names = [row.get("name") for row in rows]
    ids = [row.get("id") for row in rows]
    if names != EXPECTED:
        errors.append(f"class names/order must equal {EXPECTED}")
    if ids != list(range(5)):
        errors.append("class ids must be exactly 0..4")
    if classes.get("ignore_id") != 255:
        errors.append("ignore_id must be 255")

    source_rows = sources.get("selected_sources", [])
    source_ids = [row.get("source_id") for row in source_rows]
    if len(source_ids) != len(set(source_ids)):
        errors.append("duplicate selected source_id")
    source_map = {row.get("source_id"): row for row in source_rows}
    for row in source_rows:
        if row.get("license") not in ALLOWED_LICENSES:
            errors.append(f"{row.get('source_id')}: unsupported or missing license")
        if row.get("status") not in {
                "approved", "approved_with_file_notice_check",
                "approved_for_training_only"}:
            errors.append(f"{row.get('source_id')}: source not approved")
        if row.get("file_notice_check_required") and not row.get("license_basis"):
            errors.append(f"{row.get('source_id')}: required file notice basis missing")
        if not row.get("url") or not row.get("attribution"):
            errors.append(f"{row.get('source_id')}: URL and attribution required")
    if sources.get("target_classes") != EXPECTED:
        errors.append("source manifest target_classes must list all five classes")

    train_only = set(split_policy.get("train_only_sources", []))
    evaluation_only = set(split_policy.get("evaluation_only_sources", []))
    if train_only & evaluation_only:
        errors.append("a source cannot be both training-only and evaluation-only")
    for source_id in evaluation_only:
        if source_id not in source_map:
            errors.append(f"evaluation-only source {source_id}: not selected")
    for source_id in train_only:
        if source_id not in source_map:
            errors.append(f"train-only source {source_id}: not selected")
        elif source_map[source_id].get("status") not in {
                "approved_for_training_only", "approved_with_file_notice_check"}:
            errors.append(f"train-only source {source_id}: missing training-only rights boundary")

    seen: dict[tuple[str, str], str] = {}
    for scene_set in split_policy.get("scene_splits", []):
        source_id = scene_set.get("source_id")
        if source_id not in source_map:
            errors.append(f"split source {source_id}: not selected")
            continue
        namespace = scene_set.get(
            "group_namespace", source_map[source_id].get("group_namespace", source_id))
        for split in SPLITS:
            groups = scene_set.get(split, [])
            if not groups and source_id not in evaluation_only:
                errors.append(f"{source_id}: {split} split is empty")
            if groups and split == "train" and source_id in evaluation_only:
                errors.append(f"evaluation-only source {source_id} appears in train")
            for group in groups:
                key = (namespace, group)
                if key in seen and seen[key] != split:
                    errors.append(
                        f"scene leakage: {namespace}/{group} appears in "
                        f"{seen[key]} and {split}")
                else:
                    seen[key] = split
                if source_id in train_only and split != "train":
                    errors.append(
                        f"training-only source {source_id} appears in {split}")
        if source_id in evaluation_only and not (scene_set.get("validation") or scene_set.get("test")):
            errors.append(f"evaluation-only source {source_id}: no held-out scenes")
    if not split_policy.get("split_unit"):
        errors.append("split_unit must identify the scene grouping key")
    if not split_policy.get("materialization_state"):
        errors.append("materialization_state must distinguish plan from acquired data")
    return errors


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--classes", type=Path, required=True)
    parser.add_argument("--sources", type=Path, required=True)
    parser.add_argument("--splits", type=Path, required=True)
    args = parser.parse_args()
    try:
        errors = validate(read_json(args.classes), read_json(args.sources),
                          read_json(args.splits))
    except (OSError, json.JSONDecodeError, ValueError) as exc:
        print(f"FAIL: {exc}")
        return 1
    if errors:
        for error in errors:
            print(f"FAIL: {error}")
        return 1
    print("PASS: five classes, approved source terms, and scene-disjoint split plan")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
