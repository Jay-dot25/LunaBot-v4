#!/usr/bin/env python3
"""Run repeatable occupancy-grid baselines against the existing A* planner.

Every attempt is serialized, including planner failures and exceptions. These
scenarios measure grid planning behavior only; they are not Gazebo run claims.
"""
from __future__ import annotations

import argparse
import json
import math
import statistics
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
from astar_navigation import AStarNavigation
from nav_msgs.msg import OccupancyGrid


def load_scenarios(path: Path) -> dict[str, Any]:
    data = json.loads(path.read_text(encoding="utf-8"))
    if data.get("schema_version") != 1 or not data.get("scenarios"):
        raise ValueError("scenario file must have schema_version 1 and scenarios")
    ids = [row.get("route_id") for row in data["scenarios"]]
    if any(not value for value in ids) or len(ids) != len(set(ids)):
        raise ValueError("route_id values must be non-empty and unique")
    return data


def make_planner(config: dict[str, Any], scenario: dict[str, Any]) -> AStarNavigation:
    width, height = int(scenario["width"]), int(scenario["height"])
    resolution = float(config["resolution_m"])
    if width < 2 or height < 2 or not math.isfinite(resolution) or resolution <= 0:
        raise ValueError(f"{scenario['route_id']}: invalid map dimensions/resolution")
    grid = OccupancyGrid()
    grid.info.width = width
    grid.info.height = height
    grid.info.resolution = resolution
    grid.info.origin.orientation.w = 1.0
    cells = [0] * (width * height)
    for rectangle in scenario.get("obstacles", []):
        if len(rectangle) != 4:
            raise ValueError(f"{scenario['route_id']}: obstacle must be [x0,y0,x1,y1]")
        x0, y0, x1, y1 = map(int, rectangle)
        if x0 > x1 or y0 > y1 or x0 < 0 or y0 < 0 or x1 >= width or y1 >= height:
            raise ValueError(f"{scenario['route_id']}: obstacle outside map")
        for y in range(y0, y1 + 1):
            for x in range(x0, x1 + 1):
                cells[y * width + x] = 100
    grid.data = cells
    planner = AStarNavigation.__new__(AStarNavigation)
    planner.map_msg = grid
    planner.inflation_radius = float(config["inflation_radius_m"])
    planner.occupied_threshold = int(config["occupied_threshold"])
    planner.unknown_is_obstacle = bool(config["unknown_is_obstacle"])
    return planner


def min_clearance_m(planner: AStarNavigation, path: list[tuple[int, int]]) -> float | None:
    grid = planner.map_msg
    occupied = [(i % grid.info.width, i // grid.info.width, value)
                for i, value in enumerate(grid.data)
                if value >= planner.occupied_threshold]
    if not path or not occupied:
        return None
    resolution = grid.info.resolution
    # Subtract a cell half-diagonal to estimate clearance to occupied cell area.
    cell_edge = math.sqrt(2.0) * resolution / 2.0
    return max(0.0, min(
        math.hypot(px - ox, py - oy) * resolution - cell_edge
        for px, py in path for ox, oy, _ in occupied))


def run_attempt(config: dict[str, Any], scenario: dict[str, Any],
                trial_index: int) -> dict[str, Any]:
    record: dict[str, Any] = {
        "route_id": scenario["route_id"],
        "trial_index": trial_index,
        "start_cell": scenario["start"],
        "goal_cell": scenario["goal"],
        "expected_success": bool(scenario.get("expected_success", True)),
        "status": "ERROR",
        "success": False,
        "path_length_m": None,
        "path_cells": 0,
        "min_clearance_m": None,
        "planning_time_ms": None,
        "error": None,
    }
    started = time.perf_counter()
    try:
        planner = make_planner(config, scenario)
        start = tuple(map(int, scenario["start"]))
        goal = tuple(map(int, scenario["goal"]))
        if not planner._cell_in_bounds(start) or not planner._cell_in_bounds(goal):
            raise ValueError("start or goal lies outside the grid")
        path = planner._astar(start, goal)
        record["success"] = bool(path)
        record["status"] = "PASS" if bool(path) == record["expected_success"] else "FAIL"
        record["path_cells"] = len(path)
        if path:
            record["path_length_m"] = round(sum(
                math.hypot(b[0] - a[0], b[1] - a[1]) * planner.map_msg.info.resolution
                for a, b in zip(path, path[1:])), 6)
            clearance = min_clearance_m(planner, path)
            record["min_clearance_m"] = (
                round(clearance, 6) if clearance is not None else None)
    except Exception as exc:
        record["error"] = f"{type(exc).__name__}: {exc}"
        record["status"] = "ERROR"
    record["planning_time_ms"] = round((time.perf_counter() - started) * 1000.0, 6)
    return record


def evaluate(config: dict[str, Any], trials_per_route: int) -> dict[str, Any]:
    if trials_per_route < 1:
        raise ValueError("trials_per_route must be at least one")
    attempts = []
    route_summaries = []
    for scenario in config["scenarios"]:
        rows = [run_attempt(config, scenario, trial)
                for trial in range(1, trials_per_route + 1)]
        attempts.extend(rows)
        times = [row["planning_time_ms"] for row in rows
                 if row["planning_time_ms"] is not None]
        lengths = [row["path_length_m"] for row in rows
                   if row["path_length_m"] is not None]
        clearances = [row["min_clearance_m"] for row in rows
                      if row["min_clearance_m"] is not None]
        route_summaries.append({
            "route_id": scenario["route_id"],
            "attempts": len(rows),
            "passes": sum(row["status"] == "PASS" for row in rows),
            "failures": sum(row["status"] != "PASS" for row in rows),
            "mean_path_length_m": round(statistics.mean(lengths), 6) if lengths else None,
            "minimum_clearance_m": min(clearances) if clearances else None,
            "mean_planning_time_ms": round(statistics.mean(times), 6) if times else None,
            "trial_indices_retained": [row["trial_index"] for row in rows],
        })
    return {
        "schema_version": 1,
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "measurement_scope": "existing A* planner on deterministic occupancy grids; not Gazebo motion",
        "trials_per_route": trials_per_route,
        "route_count": len(route_summaries),
        "attempt_count": len(attempts),
        "all_attempts_retained": len(attempts) == len(route_summaries) * trials_per_route,
        "result": "PASS" if all(row["status"] == "PASS" for row in attempts) else "FAIL",
        "routes": route_summaries,
        "attempts": attempts,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--scenarios", type=Path,
                        default=ROOT / "config/navigation_baselines.json")
    parser.add_argument("--trials-per-route", type=int, default=3)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    config = load_scenarios(args.scenarios)
    result = evaluate(config, args.trials_per_route)
    if args.output.exists():
        parser.error(f"refusing to overwrite existing evidence: {args.output}")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(f"{result['result']}: {result['route_count']} routes, "
          f"{result['attempt_count']} attempts; all attempts retained")
    for route in result["routes"]:
        print(f"{route['route_id']}: {route['passes']}/{route['attempts']} pass, "
              f"path={route['mean_path_length_m']} m, "
              f"clearance={route['minimum_clearance_m']} m, "
              f"plan={route['mean_planning_time_ms']} ms")
    return 0 if result["result"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
