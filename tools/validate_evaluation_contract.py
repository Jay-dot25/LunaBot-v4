#!/usr/bin/env python3
"""Validate evaluation metrics and enforce the simulator-truth boundary."""
from __future__ import annotations

import argparse
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
REQUIRED_METRICS = {
    "localization_position_error", "localization_yaw_error", "trajectory_length_error",
    "path_length", "travel_time", "mission_success", "collision_count", "minimum_clearance",
    "unsafe_terrain_entry", "terrain_exposure", "perception_iou", "replanning", "recovery",
    "resource_usage",
}


def validate_contract(contract: dict, root: Path) -> list[str]:
    errors: list[str] = []
    if contract.get("schema_version") != 1:
        errors.append("schema_version must be 1")
    if not contract.get("contract_id"):
        errors.append("contract_id is required")
    boundary = contract.get("truth_boundary", {})
    for key in ("simulator_source", "capture_program", "analysis_programs", "truth_policy"):
        if not boundary.get(key):
            errors.append(f"truth_boundary.{key} is required")
    if boundary.get("ros_bridge_enabled") is not False:
        errors.append("simulator truth must not be bridged into ROS")
    if boundary.get("operational_consumers") != []:
        errors.append("operational_consumers must be empty")
    for rel in [boundary.get("capture_program"), *boundary.get("analysis_programs", [])]:
        if not rel or not (root / rel).is_file():
            errors.append(f"evaluation truth tool is missing: {rel}")

    metrics = contract.get("metrics")
    if not isinstance(metrics, list):
        errors.append("metrics must be an array")
        metrics = []
    ids = [item.get("id") for item in metrics if isinstance(item, dict)]
    if len(ids) != len(metrics):
        errors.append("every metric must be an object with an id")
    if len(ids) != len(set(ids)):
        errors.append("metric IDs must be unique")
    missing = sorted(REQUIRED_METRICS - set(ids))
    if missing:
        errors.append(f"required metrics missing: {', '.join(missing)}")
    for item in metrics:
        if not isinstance(item, dict):
            continue
        metric_id = item.get("id", "<unknown>")
        for field in ("unit", "definition", "reference", "aggregation", "gate", "evidence_source"):
            if not item.get(field):
                errors.append(f"{metric_id}: {field} is required")
        gate = item.get("gate")
        if isinstance(gate, dict) and gate.get("state") not in {
            "frozen", "report-only", "scenario-defined", "pending"
        }:
            errors.append(f"{metric_id}: unknown gate state")

    markers = boundary.get("forbidden_operational_markers", [])
    extensions = {".py", ".sh", ".cc", ".cpp", ".h", ".hpp", ".xml", ".yaml", ".yml", ".json", ".sdf", ".urdf", ".xacro"}
    for relative in boundary.get("operational_paths", []):
        folder = root / relative
        if not folder.exists():
            continue
        candidates = [folder] if folder.is_file() else folder.rglob("*")
        for path in candidates:
            if not path.is_file() or path.suffix.lower() not in extensions:
                continue
            try:
                text = path.read_text(encoding="utf-8")
            except (OSError, UnicodeDecodeError):
                errors.append(f"cannot inspect operational file: {path.relative_to(root)}")
                continue
            for marker in markers:
                if marker in text:
                    errors.append(f"forbidden truth interface marker {marker!r} in {path.relative_to(root)}")
    return errors


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, default=ROOT)
    parser.add_argument("--contract", type=Path)
    args = parser.parse_args()
    root = args.root.resolve()
    contract_path = args.contract or (root / "config" / "evaluation" / "metrics.json")
    try:
        contract = json.loads(contract_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        print(f"FAIL: cannot load evaluation contract: {exc}")
        return 1
    errors = validate_contract(contract, root)
    if errors:
        for error in errors:
            print(f"FAIL: {error}")
        return 1
    print(f"PASS: {len(contract['metrics'])} metric contracts; truth isolated from operational paths")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
