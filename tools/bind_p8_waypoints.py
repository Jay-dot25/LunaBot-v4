#!/usr/bin/env python3
"""Freeze map-frame P8 site poses only from a successful map-qualification run."""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def read_pgm(path: Path) -> tuple[int, int, list[int]]:
    data = path.read_bytes()
    tokens: list[bytes] = []
    index = 0
    while len(tokens) < 4:
        while index < len(data) and data[index] in b" \t\r\n":
            index += 1
        if index < len(data) and data[index] == ord("#"):
            while index < len(data) and data[index] not in b"\r\n":
                index += 1
            continue
        start = index
        while index < len(data) and data[index] not in b" \t\r\n":
            index += 1
        tokens.append(data[start:index])
    magic, width_b, height_b, maximum_b = tokens
    width, height, maximum = int(width_b), int(height_b), int(maximum_b)
    if width <= 0 or height <= 0 or maximum <= 0 or maximum > 255:
        raise ValueError("only 8-bit nonempty PGM maps are supported")
    if magic == b"P5":
        while index < len(data) and data[index] in b" \t\r\n":
            index += 1
        pixels = list(data[index:index + width * height])
    elif magic == b"P2":
        rest = data[index:].decode("ascii").split()
        pixels = [int(value) for value in rest[:width * height]]
    else:
        raise ValueError(f"unsupported PGM encoding {magic!r}")
    if len(pixels) != width * height:
        raise ValueError("PGM pixel count does not match its dimensions")
    return width, height, pixels


def read_map_yaml(path: Path) -> tuple[str, float, float, float, float]:
    lines = path.read_text(encoding="utf-8").splitlines()
    image = next((line.split(":", 1)[1].strip() for line in lines
                  if line.startswith("image:")), None)
    resolution = next((float(line.split(":", 1)[1].strip()) for line in lines
                      if line.startswith("resolution:")), None)
    origin_line = next((line.split(":", 1)[1].strip() for line in lines
                        if line.startswith("origin:")), None)
    if image is None or resolution is None or origin_line is None:
        raise ValueError("map YAML is missing image, resolution or origin")
    origin = json.loads(origin_line)
    if len(origin) != 3 or not all(math.isfinite(float(value)) for value in origin):
        raise ValueError("map origin must be three finite values")
    return image, resolution, float(origin[0]), float(origin[1]), float(origin[2])


def map_pose(world: dict, start: dict, transform: dict) -> dict:
    tx = float(transform["x"])
    ty = float(transform["y"])
    tyaw = float(transform["yaw"])
    c0, s0 = math.cos(float(start["yaw"])), math.sin(float(start["yaw"]))
    c, s = math.cos(tyaw), math.sin(tyaw)
    dx, dy = float(world["x"]) - float(start["x"]), float(world["y"]) - float(start["y"])
    odom_x, odom_y = c0 * dx + s0 * dy, -s0 * dx + c0 * dy
    return {"x": tx + c * odom_x - s * odom_y,
            "y": ty + s * odom_x + c * odom_y,
            "yaw": float(world["yaw"]) - float(start["yaw"]) + tyaw}


def require_free_cell(pose: dict, *, width: int, height: int,
                      pixels: list[int], resolution: float,
                      origin_x: float, origin_y: float,
                      origin_yaw: float) -> None:
    dx, dy = pose["x"] - origin_x, pose["y"] - origin_y
    gx = math.floor((math.cos(origin_yaw) * dx + math.sin(origin_yaw) * dy) / resolution)
    gy = math.floor((-math.sin(origin_yaw) * dx + math.cos(origin_yaw) * dy) / resolution)
    if gx < 0 or gy < 0 or gx >= width or gy >= height:
        raise ValueError(f"pose {pose} is outside the saved map")
    pixel = pixels[(height - 1 - gy) * width + gx]
    if pixel < 250:
        raise ValueError(f"pose {pose} is not a known free map cell (PGM={pixel})")


def bind(map_yaml: Path, qualification: Path, scenario_path: Path,
         registry_path: Path, map_id: str) -> dict:
    result = json.loads(qualification.read_text(encoding="utf-8"))
    if result.get("result") != "QUALIFICATION_PASS" or result.get("completed"):
        raise ValueError("qualification summary is not a successful non-acceptance pilot")
    if [leg.get("site") for leg in result.get("legs", [])] != [
            "habitat", "science", "habitat"]:
        raise ValueError("qualification did not complete all three site legs")
    transform_data = result.get("map_transform_at_start", {}).get("map_to_odom")
    if not isinstance(transform_data, dict):
        raise ValueError("qualification summary has no measured map->odom transform")
    scenario = json.loads(scenario_path.read_text(encoding="utf-8"))
    start = scenario["start_pose_world"]
    by_id = {item["id"]: item["pose_world"] for item in scenario["waypoints"]}
    poses = {name: map_pose(by_id[name], start, transform_data)
             for name in ("habitat", "science")}
    image_ref, resolution, ox, oy, oyaw = read_map_yaml(map_yaml)
    image_path = (map_yaml.parent / image_ref).resolve()
    width, height, pixels = read_pgm(image_path)
    for pose in poses.values():
        require_free_cell(pose, width=width, height=height, pixels=pixels,
                          resolution=resolution, origin_x=ox, origin_y=oy,
                          origin_yaw=oyaw)

    registry_path = registry_path.resolve()
    metadata_rel = Path(__import__("os").path.relpath(map_yaml.resolve(),
                                                       registry_path.parent.resolve())).as_posix()
    output = {
        "schema_version": 1,
        "map": {"id": map_id, "frame_id": "map",
                "metadata_path": metadata_rel,
                "metadata_sha256": sha256(map_yaml),
                "image_sha256": sha256(image_path),
                "qualification": "P8-03 live map pilot; site cells known and free",
                "qualification_run": result.get("run_id"),
                "reference_map_to_odom": transform_data},
        # Keep dispatch targets fixed in the SLAM map frame. Converting to an
        # odom snapshot makes them drift as SLAM updates map->odom during a run.
        "navigation_frame": "map",
        "waypoints": [
            {"id": name, "name": name.title(), "map_id": map_id,
             "frame_id": "map", "pose": poses[name],
             "binding_status": "bound",
             "binding_reason": "Derived from configured world site and measured map->odom transform; saved-map goal cell is known free."}
            for name in ("habitat", "science")],
    }
    registry_path.write_text(json.dumps(output, indent=2) + "\n", encoding="utf-8")
    return {"map_id": map_id, "frame_id": "map", "metadata": str(map_yaml),
            "image": str(image_path), "width": width, "height": height,
            "resolution": resolution, "waypoints": poses,
            "metadata_sha256": output["map"]["metadata_sha256"],
            "image_sha256": output["map"]["image_sha256"],
            "registry": str(registry_path)}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map-yaml", type=Path, required=True)
    parser.add_argument("--qualification", type=Path, required=True)
    parser.add_argument("--scenario", type=Path, default=ROOT / "config/p8_roundtrip.json")
    parser.add_argument("--registry", type=Path, default=ROOT / "config/waypoints.json")
    parser.add_argument("--map-id", default="lunar_p803_roundtrip")
    args = parser.parse_args()
    try:
        report = bind(args.map_yaml, args.qualification, args.scenario,
                      args.registry, args.map_id)
    except (OSError, ValueError, KeyError, TypeError, json.JSONDecodeError) as exc:
        parser.error(str(exc))
    print("P8_WAYPOINT_BIND_PASS " + json.dumps(report, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
