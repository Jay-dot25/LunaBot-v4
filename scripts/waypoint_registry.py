#!/usr/bin/env python3
"""Fail-closed loader for map-bound semantic rover waypoints."""
from __future__ import annotations
import argparse
import hashlib
import json
import math
import re
import sys
from pathlib import Path
from typing import Any

class RegistryError(ValueError):
    pass

_FRAME = re.compile(r"^[A-Za-z][A-Za-z0-9_/]*$")

def _sha256(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()

def load_registry(path: str | Path, *, map_id: str, frame_id: str,
                  require_bound: bool = False) -> dict[str, Any]:
    """Load only when caller map identity and frame match the pinned registry."""
    registry_path = Path(path).resolve()
    try:
        data = json.loads(registry_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise RegistryError(f"cannot read registry {registry_path}: {exc}") from exc
    if not isinstance(data, dict) or data.get("schema_version") != 1:
        raise RegistryError("unsupported waypoint registry schema")
    map_data = data.get("map")
    if not isinstance(map_data, dict):
        raise RegistryError("registry map identity is missing")
    expected_map = map_data.get("id")
    expected_frame = map_data.get("frame_id")
    if not isinstance(expected_map, str) or not expected_map:
        raise RegistryError("registry map ID is invalid")
    if not isinstance(expected_frame, str) or not _FRAME.fullmatch(expected_frame):
        raise RegistryError("registry map frame is invalid")
    if map_id != expected_map:
        raise RegistryError(f"map ID mismatch: expected {expected_map}, got {map_id}")
    if frame_id != expected_frame:
        raise RegistryError(f"map frame mismatch: expected {expected_frame}, got {frame_id}")
    metadata_ref = map_data.get("metadata_path")
    if not isinstance(metadata_ref, str) or not metadata_ref:
        raise RegistryError("map metadata path is missing")
    metadata_path = (registry_path.parent / metadata_ref).resolve()
    if not metadata_path.is_file():
        raise RegistryError(f"map metadata does not exist: {metadata_path}")
    if _sha256(metadata_path) != map_data.get("metadata_sha256"):
        raise RegistryError("map metadata hash mismatch")
    try:
        image_ref = next(line.split(":", 1)[1].strip()
                         for line in metadata_path.read_text(encoding="utf-8").splitlines()
                         if line.startswith("image:"))
    except (OSError, StopIteration) as exc:
        raise RegistryError("map metadata image entry is missing") from exc
    image_path = (metadata_path.parent / image_ref).resolve()
    if not image_path.is_file():
        raise RegistryError(f"map image does not exist: {image_path}")
    if _sha256(image_path) != map_data.get("image_sha256"):
        raise RegistryError("map image hash mismatch")
    entries = data.get("waypoints")
    if not isinstance(entries, list):
        raise RegistryError("waypoints must be a list")
    by_id: dict[str, dict[str, Any]] = {}
    for item in entries:
        if not isinstance(item, dict):
            raise RegistryError("waypoint entry must be an object")
        ident = item.get("id")
        if not isinstance(ident, str) or not re.fullmatch(r"[a-z][a-z0-9_]*", ident):
            raise RegistryError(f"invalid waypoint ID: {ident!r}")
        if ident in by_id:
            raise RegistryError(f"duplicate waypoint ID: {ident}")
        if item.get("map_id") != expected_map:
            raise RegistryError(f"{ident}: waypoint map ID mismatch")
        if item.get("frame_id") != expected_frame:
            raise RegistryError(f"{ident}: waypoint frame mismatch")
        pose = item.get("pose")
        if pose is None:
            if item.get("binding_status") != "unbound" or not item.get("binding_reason"):
                raise RegistryError(f"{ident}: unbound waypoint needs an explicit reason")
            if require_bound:
                raise RegistryError(f"{ident}: waypoint has no verified pose")
        else:
            if not isinstance(pose, dict) or not all(
                isinstance(pose.get(k), (int, float)) and math.isfinite(pose[k])
                for k in ("x", "y", "yaw")
            ):
                raise RegistryError(f"{ident}: pose requires finite x, y and yaw")
            if item.get("binding_status") != "bound":
                raise RegistryError(f"{ident}: posed waypoint must be marked bound")
        by_id[ident] = item
    missing = {"habitat", "science"} - by_id.keys()
    if missing:
        raise RegistryError(f"required site IDs missing: {sorted(missing)}")
    data["waypoints_by_id"] = by_id
    return data

def resolve_waypoint(registry: dict[str, Any], waypoint_id: str) -> dict[str, Any]:
    item = registry.get("waypoints_by_id", {}).get(waypoint_id)
    if item is None:
        raise RegistryError(f"unknown waypoint ID: {waypoint_id}")
    if item.get("binding_status") != "bound" or item.get("pose") is None:
        raise RegistryError(f"{waypoint_id}: refusing to dispatch an unbound goal")
    return item

def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--registry", type=Path,
                        default=Path(__file__).resolve().parents[1] / "config/waypoints.json")
    parser.add_argument("--map-id", required=True)
    parser.add_argument("--frame-id", required=True)
    parser.add_argument("--require-bound", action="store_true")
    args = parser.parse_args(argv)
    try:
        data = load_registry(args.registry, map_id=args.map_id,
                             frame_id=args.frame_id, require_bound=args.require_bound)
    except RegistryError as exc:
        print(f"REJECT: {exc}", file=sys.stderr)
        return 2
    ids = sorted(data["waypoints_by_id"])
    unbound = sorted(k for k,v in data["waypoints_by_id"].items()
                     if v.get("binding_status") != "bound")
    print(f"PASS: map={args.map_id} frame={args.frame_id} registered={ids}")
    if unbound:
        print(f"UNBOUND (not dispatchable): {unbound}")
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
