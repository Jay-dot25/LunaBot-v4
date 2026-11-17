#!/usr/bin/env python3
"""Run seeded, deterministic P10 scenario fixtures against both grid planners.

This is a planner-level scenario harness. It does not simulate rover motion or
claim Gazebo mission, collision, localization, or perception performance.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import random
import statistics
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
import platform

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
from terrain_plan import path_is_traversable, weighted_astar


def _fill_rect(grid: list[int], width: int, height: int,
               rect: list[int], value: int) -> None:
    if len(rect) != 4:
        raise ValueError("grid rectangle must contain x0,y0,x1,y1")
    x0, y0, x1, y1 = map(int, rect)
    if not (0 <= x0 <= x1 < width and 0 <= y0 <= y1 < height):
        raise ValueError(f"rectangle outside grid: {rect}")
    for y in range(y0, y1 + 1):
        for x in range(x0, x1 + 1):
            grid[y * width + x] = value


def build_grids(config: dict, scenario: dict, seed: int,
                extra_obstacles: list[list[int]] | None = None
                ) -> tuple[list[int], list[int]]:
    width, height = int(config["grid"]["width"]), int(config["grid"]["height"])
    if width < 2 or height < 2:
        raise ValueError("grid dimensions must be at least 2x2")
    occupancy = [0] * (width * height)
    terrain = [0] * (width * height)
    for rect in [*scenario.get("obstacles", []), *(extra_obstacles or [])]:
        _fill_rect(occupancy, width, height, rect, 100)
        _fill_rect(terrain, width, height, rect, 100)
    for region in scenario.get("terrain_regions", []):
        value = int(region["cost"])
        if not 0 <= value < 100:
            raise ValueError(f"terrain region cost must be 0..99: {region}")
        _fill_rect(terrain, width, height, region["rect"], value)
    roughness = config.get("seeded_roughness", {})
    if roughness.get("enabled"):
        rng = random.Random(seed)
        for _ in range(int(roughness.get("samples", 0))):
            x, y = rng.randrange(width), rng.randrange(height)
            index = y * width + x
            if occupancy[index] < 100 and terrain[index] == 0:
                terrain[index] = rng.randint(0, int(roughness["maximum_cost"]))
    return occupancy, terrain


def _plan(grid: list[int], config: dict, start: list[int], goal: list[int]):
    width, height = int(config["grid"]["width"]), int(config["grid"]["height"])
    start_cell, goal_cell = tuple(map(int, start)), tuple(map(int, goal))
    started = time.perf_counter()
    path, cost = weighted_astar(
        grid, width, height, start_cell, goal_cell,
        cost_weight=float(config["planner"]["cost_weight"]),
        blocked_cost=int(config["planner"]["blocked_cost"]),
        unknown_cost=int(config["planner"]["unknown_cost"]),
        unknown_is_blocked=False,
    )
    elapsed_ms = (time.perf_counter() - started) * 1000.0
    resolution = float(config["grid"]["resolution_m"])
    length = sum(math.hypot(b[0] - a[0], b[1] - a[1]) * resolution
                 for a, b in zip(path, path[1:])) if path else None
    return {
        "success": bool(path), "path": path, "path_cells": len(path),
        "path_length_m": round(length, 6) if length is not None else None,
        "weighted_cost": round(cost, 6) if math.isfinite(cost) else None,
        "planning_time_ms": round(elapsed_ms, 6),
    }


def _intersects(path: list[tuple[int, int]], rect: list[int]) -> bool:
    x0, y0, x1, y1 = rect
    return any(x0 <= x <= x1 and y0 <= y <= y1 for x, y in path)


def run_attempt(config: dict, scenario: dict, trial_index: int) -> dict:
    scenario_offset = int.from_bytes(
        hashlib.sha256(scenario["scenario_id"].encode("utf-8")).digest()[:4], "big")
    seed = (int(config["seed"]) + scenario_offset + trial_index - 1) % (2 ** 32)
    result = {
        "scenario_id": scenario["scenario_id"], "scenario_type": scenario["type"],
        "trial_index": trial_index, "seed": seed, "status": "ERROR",
        "occupancy": {"legs": []}, "terrain": {"legs": []},
        "replanning_count": 0, "delivery_complete": False,
        "collision_count": None,
        "collision_count_unavailable_reason": "planner fixture has no contact simulator",
        "localization_error_m": None,
        "localization_error_unavailable_reason": "planner fixture produces no rover pose samples",
    }
    try:
        occupancy, terrain = build_grids(config, scenario, seed)
        width, height = int(config["grid"]["width"]), int(config["grid"]["height"])
        start = list(map(int, scenario["start"]))
        if not (0 <= start[0] < width and 0 <= start[1] < height):
            raise ValueError("start outside grid")
        current = start
        expected = scenario.get("expected", {})
        legs_ok = True
        for leg_index, goal in enumerate(scenario["goals"], start=1):
            occ = _plan(occupancy, config, current, goal)
            terr = _plan(terrain, config, current, goal)
            for row, grid in ((occ, occupancy), (terr, terrain)):
                row["traversable"] = bool(row["success"] and path_is_traversable(
                    grid, width, height, row["path"],
                    blocked_cost=int(config["planner"]["blocked_cost"])))
                row["path"] = [list(cell) for cell in row["path"]]
            result["occupancy"]["legs"].append(occ)
            result["terrain"]["legs"].append(terr)
            legs_ok = legs_ok and occ["success"] and terr["success"] and occ["traversable"] and terr["traversable"]
            current = list(map(int, goal))

        scenario_regions = {item["id"]: item["rect"] for item in scenario.get("terrain_regions", [])}
        for name, side in (("occupancy", result["occupancy"]), ("terrain", result["terrain"])):
            paths = [tuple(map(tuple, leg["path"])) for leg in side["legs"]]
            side["region_intersections"] = {
                region_id: _intersects([cell for path in paths for cell in path], rect)
                for region_id, rect in scenario_regions.items()
            }
            side["total_path_length_m"] = round(sum(
                leg["path_length_m"] or 0.0 for leg in side["legs"]), 6)

        if scenario["type"] == "dynamic_obstacle":
            first_path = [tuple(cell) for cell in result["terrain"]["legs"][0]["path"]]
            updated_occupancy, updated_terrain = build_grids(
                config, scenario, seed, scenario.get("obstacle_update", []))
            obstacle_rects = scenario["obstacle_update"]
            intersects_update = any(_intersects(first_path, rect) for rect in obstacle_rects)
            updated = _plan(updated_terrain, config, start, scenario["goals"][0])
            updated_path = updated.pop("path")
            result["replanning_count"] = 1 if intersects_update and updated["success"] and updated_path != first_path else 0
            result["dynamic_replan"] = {
                "original_path_intersected_new_obstacle": intersects_update,
                "updated_plan_success": updated["success"],
                "updated_path_changed": updated_path != first_path,
                "updated_path_traversable": path_is_traversable(
                    updated_terrain, width, height, updated_path,
                    blocked_cost=int(config["planner"]["blocked_cost"])),
                "updated_path_length_m": updated["path_length_m"],
                "updated_planning_time_ms": updated["planning_time_ms"],
            }

        checks = [legs_ok]
        if expected.get("delivery_complete"):
            checks.append(current == list(map(int, scenario["goals"][-1])))
            result["delivery_complete"] = current == list(map(int, scenario["goals"][-1]))
        if expected.get("return_to_start"):
            checks.append(current == start)
        for region_id in expected.get("terrain_path_avoids_regions", []):
            checks.append(not result["terrain"]["region_intersections"].get(region_id, True))
        occupancy_region = expected.get("occupancy_path_intersects_region")
        if occupancy_region:
            checks.append(result["occupancy"]["region_intersections"].get(occupancy_region, False))
        if expected.get("require_changed_replan"):
            checks.append(result.get("replanning_count") == 1)
            checks.append(result.get("dynamic_replan", {}).get("updated_path_traversable", False))
        required_legs = expected.get("required_legs")
        if required_legs is not None:
            checks.append(len(result["terrain"]["legs"]) == int(required_legs))
        result["acceptance_checks_pass"] = all(checks)
        result["status"] = "PASS" if result["acceptance_checks_pass"] else "FAIL"
    except Exception as exc:
        result["error"] = f"{type(exc).__name__}: {exc}"
    return result


def evaluate(config: dict) -> dict:
    if config.get("schema_version") != 1 or not config.get("scenarios"):
        raise ValueError("scenario config must use schema_version 1 and include scenarios")
    trials = int(config.get("trials_per_scenario", 0))
    if trials < 3:
        raise ValueError("P10-02 requires at least three trials per scenario")
    ids = [item.get("scenario_id") for item in config["scenarios"]]
    if len(ids) != 5 or len(ids) != len(set(ids)):
        raise ValueError("P10-02 requires exactly five unique scenarios")
    attempts = [run_attempt(config, scenario, trial)
                for scenario in config["scenarios"]
                for trial in range(1, trials + 1)]
    summaries = []
    for scenario in config["scenarios"]:
        rows = [item for item in attempts if item["scenario_id"] == scenario["scenario_id"]]
        times = [leg["planning_time_ms"] for row in rows
                 for side in (row["occupancy"], row["terrain"])
                 for leg in side["legs"] if leg["planning_time_ms"] is not None]
        summaries.append({
            "scenario_id": scenario["scenario_id"], "trial_count": len(rows),
            "passes": sum(row["status"] == "PASS" for row in rows),
            "failures": sum(row["status"] != "PASS" for row in rows),
            "median_planning_time_ms": round(statistics.median(times), 6) if times else None,
            "attempt_indices_retained": [row["trial_index"] for row in rows],
        })
    return {
        "schema_version": 1,
        "suite_id": config["suite_id"],
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "config_sha256": hashlib.sha256(json.dumps(config, sort_keys=True).encode()).hexdigest(),
        "runner_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        "python_version": platform.python_version(),
        "measurement_scope": config["measurement_scope"],
        "seed": config["seed"], "trials_per_scenario": trials,
        "scenario_count": len(summaries), "attempt_count": len(attempts),
        "all_attempts_retained": len(attempts) == len(summaries) * trials,
        "runtime_metrics_unavailable": {
            "reason": "This harness evaluates planner fixtures without ROS/Gazebo motion.",
            "metrics": ["collisions", "clearance to physical rover footprint", "travel time", "localization error", "perception IoU", "CPU/RAM/VRAM", "simulation real-time factor"],
        },
        "corroborating_runtime_evidence": {
            "task": "P8-03",
            "path": "evidence/runs/p803-acceptance-final2-20261113/P8-03/summary.json",
            "result": "PASS",
            "independent_roundtrips": 3,
            "completed_legs": 9,
            "durable_deliveries": 3,
            "scope_note": "Previously recorded live Habitat-Science-Habitat mission acceptance; separate from this P10 grid-fixture run and not rescored under the P10 schema."
        },
        "result": "PASS" if all(row["status"] == "PASS" for row in attempts) else "FAIL",
        "scenarios": summaries, "attempts": attempts,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--scenarios", type=Path, default=ROOT / "config/evaluation/scenarios.json")
    parser.add_argument("--trials", type=int)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    config = json.loads(args.scenarios.read_text(encoding="utf-8"))
    if args.trials is not None:
        config["trials_per_scenario"] = args.trials
    if args.output.exists():
        parser.error(f"refusing to overwrite evidence: {args.output}")
    result = evaluate(config)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(f"{result['result']}: {result['scenario_count']} scenarios, {result['attempt_count']} attempts; scope={result['measurement_scope']}")
    for row in result["scenarios"]:
        print(f"{row['scenario_id']}: {row['passes']}/{row['trial_count']} pass")
    return 0 if result["result"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
