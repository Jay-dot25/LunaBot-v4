#!/usr/bin/env python3
"""Matched dynamic replan benchmark for weighted A* and D* Lite.

The D* Lite implementation is a benchmark candidate. This tool does not
change the production planner or imply Gazebo/runtime performance.
"""
from __future__ import annotations

import argparse
import hashlib
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
from dstar_lite import DStarLite
from terrain_plan import weighted_astar


def _load_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def _grid(config: dict[str, Any], scenario: dict[str, Any],
          rectangles: list[list[int]]) -> list[int]:
    width, height = int(scenario["width"]), int(scenario["height"])
    cells = [0] * (width * height)
    for rectangle in rectangles:
        if len(rectangle) != 4:
            raise ValueError("obstacle rectangle must be [x0,y0,x1,y1]")
        x0, y0, x1, y1 = map(int, rectangle)
        if (x0 > x1 or y0 > y1 or x0 < 0 or y0 < 0 or
                x1 >= width or y1 >= height):
            raise ValueError(f"{scenario['route_id']}: obstacle outside map")
        for y in range(y0, y1 + 1):
            for x in range(x0, x1 + 1):
                cells[y * width + x] = 100
    return cells


def _astar(costs: list[int], base: dict[str, Any], scenario: dict[str, Any]):
    return weighted_astar(
        costs, int(scenario["width"]), int(scenario["height"]),
        tuple(map(int, scenario["start"])), tuple(map(int, scenario["goal"])),
        cost_weight=float(base.get("cost_weight", 2.0)),
        blocked_cost=int(base["occupied_threshold"]),
        unknown_cost=min(80, int(base["occupied_threshold"]) - 1),
        unknown_is_blocked=bool(base["unknown_is_obstacle"]))


def _cost(path: list[tuple[int, int]], costs: list[int], width: int,
          weight: float, unknown_cost: int = 80) -> float:
    total = 0.0
    for (x0, y0), (x1, y1) in zip(path, path[1:]):
        value = costs[y1 * width + x1]
        if value < 0:
            value = unknown_cost
        distance = math.sqrt(2.0) if x0 != x1 and y0 != y1 else 1.0
        total += distance * (1.0 + weight * value / 100.0)
    return total


def _attempt(base: dict[str, Any], scenario: dict[str, Any], updates: list[list[int]],
             trial_index: int, mode: str) -> dict[str, Any]:
    width, height = int(scenario["width"]), int(scenario["height"])
    start, goal = tuple(map(int, scenario["start"])), tuple(map(int, scenario["goal"]))
    occupied = int(base["occupied_threshold"])
    weight = float(base.get("cost_weight", 2.0))
    initial = _grid(base, scenario, scenario.get("obstacles", []))
    updated = _grid(base, scenario,
                    list(scenario.get("obstacles", [])) + updates)
    row: dict[str, Any] = {
        "route_id": scenario["route_id"], "trial_index": trial_index,
        "mode": mode, "start_cell": list(start), "goal_cell": list(goal),
        "status": "ERROR", "initial_success": False, "replan_success": False,
        "initial_path_cost": None, "replan_path_cost": None,
        "initial_path_cells": 0, "replan_path_cells": 0,
        "initial_planning_ms": None, "replanning_ms": None,
        "total_planning_ms": None, "error": None,
    }
    try:
        if mode == "weighted_astar":
            started = time.perf_counter()
            initial_path, initial_cost = _astar(initial, base, scenario)
            row["initial_planning_ms"] = (time.perf_counter() - started) * 1000
            row["initial_success"] = bool(initial_path)
            row["initial_path_cost"] = initial_cost if initial_path else None
            row["initial_path_cells"] = len(initial_path)
            started = time.perf_counter()
            path, path_cost = _astar(updated, base, scenario)
            row["replanning_ms"] = (time.perf_counter() - started) * 1000
            row["replan_success"] = bool(path)
            row["replan_path_cost"] = path_cost if path else None
            row["replan_path_cells"] = len(path)
        elif mode == "dstar_lite":
            planner = DStarLite(
                initial, width, height, start, goal, cost_weight=weight,
                blocked_cost=occupied,
                unknown_cost=min(80, occupied - 1),
                unknown_is_blocked=bool(base["unknown_is_obstacle"]))
            started = time.perf_counter()
            initial_path, initial_cost = planner.extract_path()
            row["initial_planning_ms"] = (time.perf_counter() - started) * 1000
            row["initial_success"] = bool(initial_path)
            row["initial_path_cost"] = initial_cost if initial_path else None
            row["initial_path_cells"] = len(initial_path)
            changed = [(x, y) for y in range(height) for x in range(width)
                       if initial[y * width + x] != updated[y * width + x]]
            expanded_before_replan = planner.expanded
            started = time.perf_counter()
            planner.update_costs(updated, changed)
            path, path_cost = planner.extract_path()
            row["replanning_ms"] = (time.perf_counter() - started) * 1000
            row["replan_success"] = bool(path)
            row["replan_path_cost"] = path_cost if path else None
            row["replan_path_cells"] = len(path)
            row["replan_expansions"] = planner.expanded - expanded_before_replan
        else:
            raise ValueError(f"unknown planner mode: {mode}")
        row["total_planning_ms"] = (
            row["initial_planning_ms"] + row["replanning_ms"])
        row["status"] = "PASS" if row["initial_success"] and row["replan_success"] else "FAIL"
    except Exception as exc:
        row["error"] = f"{type(exc).__name__}: {exc}"
        row["status"] = "ERROR"
    for name in ("initial_planning_ms", "replanning_ms", "total_planning_ms"):
        if row[name] is not None:
            row[name] = round(float(row[name]), 6)
    for name in ("initial_path_cost", "replan_path_cost"):
        if row[name] is not None:
            row[name] = round(float(row[name]), 9)
    return row


def run_benchmark(base: dict[str, Any], benchmark: dict[str, Any]) -> dict[str, Any]:
    trials = int(benchmark["trials_per_route_per_mode"])
    if trials < 3:
        raise ValueError("P7-03 requires at least three matched trials per route and mode")
    scenarios = base["scenarios"]
    route_ids = {row["route_id"] for row in scenarios}
    if set(benchmark["updates"]) != route_ids:
        raise ValueError("every baseline route must have exactly one update definition")
    attempts = []
    for scenario in scenarios:
        for trial in range(1, trials + 1):
            # Alternate execution order to limit systematic warm-up bias.
            modes = (("weighted_astar", "dstar_lite") if trial % 2 else
                     ("dstar_lite", "weighted_astar"))
            for mode in modes:
                attempts.append(_attempt(
                    base, scenario, benchmark["updates"][scenario["route_id"]],
                    trial, mode))
    per_mode = {}
    for mode in ("weighted_astar", "dstar_lite"):
        rows = [row for row in attempts if row["mode"] == mode]
        per_mode[mode] = {
            "attempts": len(rows),
            "passes": sum(row["status"] == "PASS" for row in rows),
            "failures": sum(row["status"] != "PASS" for row in rows),
            "median_replanning_ms": round(statistics.median(
                row["replanning_ms"] for row in rows
                if row["replanning_ms"] is not None), 6),
            "median_total_planning_ms": round(statistics.median(
                row["total_planning_ms"] for row in rows
                if row["total_planning_ms"] is not None), 6),
        }
    route_comparisons = []
    for scenario in scenarios:
        route_id = scenario["route_id"]
        astar_rows = [row for row in attempts if row["route_id"] == route_id
                      and row["mode"] == "weighted_astar"]
        dstar_rows = [row for row in attempts if row["route_id"] == route_id
                      and row["mode"] == "dstar_lite"]
        astar_median = statistics.median(row["replanning_ms"] for row in astar_rows)
        dstar_median = statistics.median(row["replanning_ms"] for row in dstar_rows)
        astar_cost = statistics.median(row["replan_path_cost"] for row in astar_rows
                                       if row["replan_path_cost"] is not None)
        dstar_cost = statistics.median(row["replan_path_cost"] for row in dstar_rows
                                       if row["replan_path_cost"] is not None)
        route_comparisons.append({
            "route_id": route_id,
            "matched_trials": trials,
            "weighted_astar_median_replanning_ms": round(astar_median, 6),
            "dstar_lite_median_replanning_ms": round(dstar_median, 6),
            "replan_speedup_fraction": round(
                (astar_median - dstar_median) / astar_median
                if astar_median > 0 else 0.0, 6),
            "weighted_astar_median_cost": round(astar_cost, 9),
            "dstar_lite_median_cost": round(dstar_cost, 9),
            "cost_regression_fraction": round(
                (dstar_cost - astar_cost) / astar_cost if astar_cost > 0 else 0.0, 9),
        })
    gate = benchmark["decision_gate"]
    improved_routes = sum(row["replan_speedup_fraction"] > 0
                          for row in route_comparisons)
    overall_speedup = ((per_mode["weighted_astar"]["median_replanning_ms"] -
                        per_mode["dstar_lite"]["median_replanning_ms"]) /
                       per_mode["weighted_astar"]["median_replanning_ms"]
                       if per_mode["weighted_astar"]["median_replanning_ms"] > 0 else 0.0)
    maximum_regression = max(row["cost_regression_fraction"]
                             for row in route_comparisons)
    all_pass = all(row["status"] == "PASS" for row in attempts)
    adopt = (
        (all_pass or not gate["require_all_attempts_successful"]) and
        overall_speedup >= float(gate["minimum_median_replan_speedup_fraction"]) and
        improved_routes >= int(gate["minimum_route_medians_improved"]) and
        maximum_regression <= float(gate["maximum_weighted_cost_regression_fraction"]))
    return {
        "schema_version": 1,
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "measurement_scope": "matched deterministic occupancy-grid dynamic replans; not Gazebo motion or ROS end-to-end latency",
        "trials_per_route_per_mode": trials,
        "route_count": len(scenarios),
        "paired_trial_count": len(scenarios) * trials,
        "mode_count": 2,
        "attempt_count": len(attempts),
        "all_attempts_retained": len(attempts) == len(scenarios) * trials * 2,
        "result": "PASS" if all_pass else "FAIL",
        "decision_gate": gate,
        "overall_replan_speedup_fraction": round(overall_speedup, 6),
        "route_medians_improved": improved_routes,
        "maximum_cost_regression_fraction": round(maximum_regression, 9),
        "decision": "ADOPT_DSTAR_LITE" if adopt else "KEEP_WEIGHTED_ASTAR",
        "modes": per_mode,
        "route_comparisons": route_comparisons,
        "attempts": attempts,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--base-scenarios", type=Path,
                        default=ROOT / "config/navigation_baselines.json")
    parser.add_argument("--benchmark", type=Path,
                        default=ROOT / "config/p7_replan_benchmark.json")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error(f"refusing to overwrite existing evidence: {args.output}")
    base_bytes = args.base_scenarios.read_bytes()
    benchmark_bytes = args.benchmark.read_bytes()
    result = run_benchmark(json.loads(base_bytes), json.loads(benchmark_bytes))
    result["provenance"] = {
        "base_scenarios": str(args.base_scenarios.relative_to(ROOT)),
        "base_scenarios_sha256": hashlib.sha256(base_bytes).hexdigest(),
        "benchmark_config": str(args.benchmark.relative_to(ROOT)),
        "benchmark_config_sha256": hashlib.sha256(benchmark_bytes).hexdigest(),
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(f"{result['result']}: {result['route_count']} routes, "
          f"{result['paired_trial_count']} matched trials, "
          f"{result['attempt_count']} planner attempts")
    print(f"decision={result['decision']} "
          f"median_replan_speedup={result['overall_replan_speedup_fraction']:.1%}")
    for row in result["route_comparisons"]:
        print(f"{row['route_id']}: weighted A*={row['weighted_astar_median_replanning_ms']:.6f} ms, "
              f"D* Lite={row['dstar_lite_median_replanning_ms']:.6f} ms, "
              f"speedup={row['replan_speedup_fraction']:.1%}")
    return 0 if result["result"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
