#!/usr/bin/env python3
"""Build the provenance-linked P10 cross-trial report and static SVG plots."""
from __future__ import annotations

import argparse
import hashlib
import html
import json
import statistics
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[1]

INPUTS = {
    "p10_scenarios": "evidence/runs/20261113-p1002/P10-02/scenario-results.json",
    "p5_heldout": "evidence/runs/20261112-p502-threeclass/P5-02/heldout-three-class-evaluation.json",
    "p5_inference": "evidence/runs/20261112-p503/P5-03/summary.json",
    "p7_benchmark": "evidence/runs/20261112-p703/P7-03/benchmark.json",
    "p7_summary": "evidence/runs/20261112-p703/P7-03/summary.json",
    "p8_roundtrip": "evidence/runs/p803-acceptance-final2-20261113/P8-03/summary.json",
    "p3_localization": "evidence/runs/20261103-p302/localization-loop-final/localization_truth_comparison.txt",
    "metric_catalog": "config/evaluation/metrics.json",
    "scenario_config": "config/evaluation/scenarios.json",
    "report_schema": "config/evaluation/report.schema.json",
}


def _read_json(root: Path, key: str) -> tuple[dict[str, Any], Path]:
    path = root / INPUTS[key]
    return json.loads(path.read_text(encoding="utf-8")), path


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _metric(metric_id: str, status: str, value: Any, sources: list[str],
            limitations: list[str], unavailable_reason: str | None = None) -> dict:
    return {
        "id": metric_id, "status": status, "value": value,
        "unavailable_reason": unavailable_reason,
        "source_paths": sources, "limitations": limitations,
    }


def _metric_report(root: Path, scenario_data: dict, heldout: dict,
                   inference: dict, benchmark: dict, p8: dict,
                   localization_text: str) -> list[dict]:
    scenario_path = INPUTS["p10_scenarios"]
    p5_path = INPUTS["p5_heldout"]
    p5_inf_path = INPUTS["p5_inference"]
    p7_path = INPUTS["p7_benchmark"]
    p8_path = INPUTS["p8_roundtrip"]
    p3_path = INPUTS["p3_localization"]
    loc = {}
    for line in localization_text.splitlines():
        if "=" in line:
            key, value = line.split("=", 1)
            loc[key.strip()] = value.strip()
    loc_pos = float(loc["position_error_m"])
    loc_yaw = float(loc["absolute_yaw_error_rad"])

    attempts = scenario_data["attempts"]
    scenario_rows = []
    for summary in scenario_data["scenarios"]:
        rows = [row for row in attempts if row["scenario_id"] == summary["scenario_id"]]
        occ_lengths = [row["occupancy"]["total_path_length_m"] for row in rows]
        terrain_lengths = [row["terrain"]["total_path_length_m"] for row in rows]
        scenario_rows.append({
            "scenario_id": summary["scenario_id"],
            "trials": summary["trial_count"], "passes": summary["passes"],
            "occupancy_median_grid_path_length_m": round(statistics.median(occ_lengths), 6),
            "terrain_median_grid_path_length_m": round(statistics.median(terrain_lengths), 6),
        })

    hazard_rows = [row for row in attempts if row["scenario_type"] == "hazard_avoidance"]
    occupancy_hazard_hits = sum(
        any(row["occupancy"]["region_intersections"].values()) for row in hazard_rows)
    terrain_hazard_hits = sum(
        any(row["terrain"]["region_intersections"].values()) for row in hazard_rows)
    dynamic_rows = [row for row in attempts if row["scenario_type"] == "dynamic_obstacle"]
    dynamic_passes = sum(
        row.get("replanning_count") == 1 and
        row.get("dynamic_replan", {}).get("updated_path_traversable") is True
        for row in dynamic_rows)

    per_class = heldout["metrics"]["per_class"]
    p5 = {
        "scope": heldout["scope"], "records": heldout["records"],
        "miou": heldout["metrics"]["miou"],
        "per_class_iou": {name: item["iou"] for name, item in per_class.items()},
        "class_support_pixels": {name: item["support_pixels"] for name, item in per_class.items()},
        "gate_pass": heldout["qualification_gate"]["pass"],
        "checkpoint_sha256": heldout["checkpoint_sha256"],
        "limitations": heldout["limitation"],
    }
    weighted = benchmark["modes"]["weighted_astar"]
    dstar = benchmark["modes"]["dstar_lite"]
    p7 = {
        "paired_attempts": benchmark["paired_trial_count"],
        "weighted_astar_median_replanning_ms": weighted["median_replanning_ms"],
        "dstar_lite_median_replanning_ms": dstar["median_replanning_ms"],
        "dstar_lite_speedup_fraction": benchmark["overall_replan_speedup_fraction"],
        "weighted_astar_passes": weighted["passes"], "dstar_lite_passes": dstar["passes"],
        "decision": benchmark["decision"],
        "scope": benchmark["measurement_scope"],
    }
    provider = {row["provider"]: row for row in inference["providers"]}
    inf = {
        "scope": "single-stream P5-03 inference package benchmark; not whole-stack usage",
        "cpu_p50_ms": provider["cpu"]["latency_ms"]["p50"],
        "cpu_p95_ms": provider["cpu"]["latency_ms"]["p95"],
        "cuda_p50_ms": provider["cuda"]["latency_ms"]["p50"],
        "cuda_p95_ms": provider["cuda"]["latency_ms"]["p95"],
        "cuda_peak_allocated_mb": provider["cuda"]["gpu_memory_mb"]["peak_allocated_mb"],
        "cuda_peak_reserved_mb": provider["cuda"]["gpu_memory_mb"]["peak_reserved_mb"],
        "cpu_process_rss_after_benchmark_mb": provider["cpu"]["process_rss_mb"]["after_benchmark"],
        "cuda_process_rss_after_benchmark_mb": provider["cuda"]["process_rss_mb"]["after_benchmark"],
        "limitations": inference["limitations"],
    }
    p8_summary = {
        "result": p8["result"],
        "independent_roundtrips": p8["checks"]["independent_roundtrips"],
        "legs_per_roundtrip": p8["checks"]["legs_per_roundtrip"],
        "completed_legs": p8["checks"]["independent_roundtrips"] * p8["checks"]["legs_per_roundtrip"],
        "durable_deliveries": p8["checks"]["durable_deliveries"],
        "map_id": p8["map_id"], "map_metadata_sha256": p8["map_metadata_sha256"],
        "map_image_sha256": p8["map_image_sha256"],
    }

    return [
        _metric("localization_position_error", "limited",
                {"p3_02_short_route_error_m": loc_pos, "qualification_bound_m": 0.25,
                 "qualification_result": loc.get("result")}, [p3_path],
                ["One short-route qualification sample, not a P10 scenario aggregate.",
                 "Position error was near the 0.25 m bound; broader-route localization remains unverified."]),
        _metric("localization_yaw_error", "limited",
                {"p3_02_short_route_error_rad": loc_yaw, "qualification_bound_rad": 0.15,
                 "qualification_result": loc.get("result")}, [p3_path],
                ["One short-route qualification sample, not a P10 scenario aggregate."]),
        _metric("trajectory_length_error", "unavailable", None, [scenario_path, p3_path],
                ["No common five-scenario timestamp-aligned reference trajectory was recorded."],
                "The P10 suite contains grid paths without rover trajectories; P3-02 evidence is a bounded pose comparison, not an arc-length comparison."),
        _metric("path_length", "limited",
                {"planner_fixture_grid_path_median_by_scenario": scenario_rows,
                 "scope": "planned grid route length, not rover-traveled distance"}, [scenario_path],
                ["No estimated-versus-reference motion path is available for four scenarios."]),
        _metric("travel_time", "unavailable", None, [scenario_path, p8_path],
                ["Scenario fixture elapsed time is planner computation time, not mission travel time."],
                "P8-03 evidence records mission completion but no common simulation start/end duration for all scenarios."),
        _metric("mission_success", "limited",
                {"p10_fixture_scenario_passes": scenario_data["result"],
                 "fixture_scenario_count": scenario_data["scenario_count"],
                 "fixture_attempts": scenario_data["attempt_count"],
                 "fixture_passes": sum(row["status"] == "PASS" for row in attempts),
                 "separate_live_p8_roundtrips": p8_summary}, [scenario_path, p8_path],
                ["P10 fixture completion verifies planned routes, not mission execution.",
                 "Only the P8 full round-trip has live mission evidence; it was not rescored using the P10 schema."]),
        _metric("collision_count", "unavailable", None, [scenario_path, p8_path],
                ["No Gazebo contact stream or per-trial contact records are present in these sources."],
                "Collision instrumentation is absent; no-contact cannot be inferred from successful plans or missions."),
        _metric("minimum_clearance", "unavailable", None, [scenario_path],
                ["No timestamped rover reference poses are paired with obstacle geometry for P10 scenarios."],
                "The planner fixtures do not model the rover footprint or physical clearance."),
        _metric("unsafe_terrain_entry", "limited",
                {"hazard_fixture_trials": len(hazard_rows),
                 "occupancy_planned_paths_intersecting_risk_region": occupancy_hazard_hits,
                 "terrain_planned_paths_intersecting_risk_region": terrain_hazard_hits,
                 "physical_rover_entries": None}, [scenario_path],
                ["This counts planned grid-path intersections in crater/unknown cost fixtures, not rover footprint entries."]),
        _metric("terrain_exposure", "limited",
                {"planned_hazard_intersections": {"occupancy": occupancy_hazard_hits,
                                                   "terrain_aware": terrain_hazard_hits},
                 "physical_distance_by_terrain_class_m": None}, [scenario_path],
                ["Planner-cell intersection is a route-planning proxy; no physical footprint trajectory or terrain labels were recorded."]),
        _metric("perception_iou", "limited", p5, [p5_path],
                ["One frozen held-out analog-corpus checkpoint result; not per-scenario or deployment performance."]),
        _metric("replanning", "limited",
                {"p10_unexpected_obstacle_fixture": {"trials": len(dynamic_rows),
                    "changed_safe_replans": dynamic_passes},
                 "p7_03_benchmark": p7}, [scenario_path, p7_path, INPUTS["p7_summary"]],
                ["P7 timing is Python grid-planner time, not ROS/Gazebo end-to-end latency.",
                 "P10 dynamic replan counts are fixtures, not sensor-triggered rover replans."]),
        _metric("recovery", "unavailable", None, [scenario_path, p8_path],
                ["No P10 scenario triggered and measured mission recovery behavior."],
                "Recovery attempts/outcomes were not instrumented in the included scenario result sources."),
        _metric("resource_usage", "limited", inf, [p5_inf_path],
                ["Reports model-only CPU/CUDA inference measurements from P5-03.",
                 "Whole-stack CPU/RAM, mission GPU use, and simulation real-time factor remain unavailable."]),
    ]


def build_report(root: Path = ROOT, report_id: str = "p10-cross-trial-20261113") -> dict:
    scenario_data, _ = _read_json(root, "p10_scenarios")
    heldout, _ = _read_json(root, "p5_heldout")
    inference, _ = _read_json(root, "p5_inference")
    benchmark, _ = _read_json(root, "p7_benchmark")
    p8, _ = _read_json(root, "p8_roundtrip")
    localization_path = root / INPUTS["p3_localization"]
    localization_text = localization_path.read_text(encoding="utf-8")
    metrics_path = root / INPUTS["metric_catalog"]
    metric_catalog = json.loads(metrics_path.read_text(encoding="utf-8"))
    metrics = _metric_report(root, scenario_data, heldout, inference, benchmark, p8, localization_text)
    metric_ids = [item["id"] for item in metrics]
    catalog_ids = [item["id"] for item in metric_catalog["metrics"]]
    if set(metric_ids) != set(catalog_ids) or len(metric_ids) != len(catalog_ids):
        raise ValueError("report must contain exactly one result for every metric catalog entry")
    for item in metrics:
        if item["status"] == "unavailable" and not item["unavailable_reason"]:
            raise ValueError(f"{item['id']}: unavailable metric requires a reason")
    input_rows = []
    for key, rel in INPUTS.items():
        path = root / rel
        if not path.is_file():
            raise FileNotFoundError(f"missing report input {rel}")
        input_rows.append({"key": key, "path": rel, "sha256": _sha256(path)})
    return {
        "schema_version": 1,
        "report_id": report_id,
        "generated_utc": datetime.now(timezone.utc).isoformat(),
        "result": "COMPLETE_WITH_LIMITATIONS",
        "metric_catalog_sha256": _sha256(metrics_path),
        "inputs": input_rows,
        "trial_summary": {
            "p10_suite_id": scenario_data["suite_id"],
            "p10_seed": scenario_data["seed"],
            "p10_trials_per_scenario": scenario_data["trials_per_scenario"],
            "p10_scenario_count": scenario_data["scenario_count"],
            "p10_attempt_count": scenario_data["attempt_count"],
            "p10_all_attempts_retained": scenario_data["all_attempts_retained"],
            "p10_measurement_scope": scenario_data["measurement_scope"],
            "p8_live_roundtrips": p8["checks"]["independent_roundtrips"],
            "p8_live_legs": p8["checks"]["independent_roundtrips"] * p8["checks"]["legs_per_roundtrip"],
            "p8_durable_deliveries": p8["checks"]["durable_deliveries"],
            "p7_matched_pairs": benchmark["paired_trial_count"],
            "p5_heldout_records": heldout["records"],
        },
        "metrics": metrics,
        "plots": ["scenario-pass-rates.svg", "planner-path-lengths.svg", "perception-iou.svg", "replan-latency.svg"],
        "limitations": [
            "P10-02 scenario trials are deterministic weighted-grid fixtures, not full live robot trials.",
            "Only P8-03 has three live outbound-return missions; those were not rescored under the P10 metric schema.",
            "Unavailable metric values are null and include explicit reasons; null does not mean zero or pass.",
            "P5-02 IoU applies to one frozen three-class analog held-out set; the historical five-class model remains unqualified.",
            "P7-03 replan times are Python planner measurements without ROS/Gazebo transport latency.",
        ],
    }


def validate_report(report: dict, schema: dict, expected_metric_ids: set[str]) -> list[str]:
    errors = []
    for key in schema["required"]:
        if key not in report:
            errors.append(f"missing report field: {key}")
    ids = [item.get("id") for item in report.get("metrics", [])]
    if set(ids) != expected_metric_ids or len(ids) != len(expected_metric_ids):
        errors.append("metric IDs do not exactly match the catalog")
    for metric in report.get("metrics", []):
        if metric.get("status") == "unavailable" and not metric.get("unavailable_reason"):
            errors.append(f"{metric.get('id')}: unavailable reason missing")
        if not metric.get("source_paths"):
            errors.append(f"{metric.get('id')}: source paths missing")
    if report.get("result") not in schema["properties"]["result"]["enum"]:
        errors.append("invalid report result")
    return errors


def _svg_bar_chart(title: str, labels: list[str], series: list[tuple[str, list[float], str]],
                   y_max: float, y_label: str) -> str:
    width, height = 960, 520
    left, top, plot_w, plot_h = 82, 80, 820, 330
    colors = [item[2] for item in series]
    group_w = plot_w / max(1, len(labels))
    bar_w = min(42, group_w / (len(series) + 1))
    parts = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
             '<rect width="100%" height="100%" fill="#ffffff"/>',
             f'<text x="{width/2}" y="34" text-anchor="middle" font-size="22" font-family="sans-serif" font-weight="700">{html.escape(title)}</text>',
             f'<text x="20" y="{top+plot_h/2}" transform="rotate(-90 20 {top+plot_h/2})" text-anchor="middle" font-size="14" font-family="sans-serif">{html.escape(y_label)}</text>']
    ticks = 4
    for i in range(ticks + 1):
        value = y_max * i / ticks
        y = top + plot_h - plot_h * i / ticks
        parts.append(f'<line x1="{left}" y1="{y:.1f}" x2="{left+plot_w}" y2="{y:.1f}" stroke="#d9dee7"/>')
        parts.append(f'<text x="{left-12}" y="{y+4:.1f}" text-anchor="end" font-size="12" fill="#526071" font-family="sans-serif">{value:.2g}</text>')
    for group, label in enumerate(labels):
        cx = left + (group + 0.5) * group_w
        parts.append(f'<text x="{cx:.1f}" y="{top+plot_h+24}" text-anchor="middle" font-size="11" font-family="sans-serif">{html.escape(label.replace("_", " "))}</text>')
        for series_i, (name, values, color) in enumerate(series):
            value = values[group]
            x = cx + (series_i - (len(series)-1)/2) * (bar_w + 6) - bar_w/2
            bh = plot_h * value / y_max if y_max else 0
            y = top + plot_h - bh
            parts.append(f'<rect x="{x:.1f}" y="{y:.1f}" width="{bar_w:.1f}" height="{bh:.1f}" rx="3" fill="{color}"/>')
            parts.append(f'<text x="{x+bar_w/2:.1f}" y="{y-6:.1f}" text-anchor="middle" font-size="11" fill="#172033" font-family="sans-serif">{value:.2f}</text>')
    lx = left
    for name, _, color in series:
        parts.append(f'<rect x="{lx}" y="{height-46}" width="13" height="13" fill="{color}"/>')
        parts.append(f'<text x="{lx+19}" y="{height-35}" font-size="12" font-family="sans-serif">{html.escape(name)}</text>')
        lx += 180
    parts.append('</svg>')
    return "\n".join(parts) + "\n"


def render_markdown(report: dict) -> str:
    by_id = {item["id"]: item for item in report["metrics"]}
    lines = [
        "# P10 Cross-Trial Evaluation Report", "",
        f"Report ID: `{report['report_id']}`  ",
        f"Generated UTC: `{report['generated_utc']}`  ",
        f"Result: **{report['result']}**", "",
        "## Trial summary", "",
        f"- P10: {report['trial_summary']['p10_scenario_count']} scenarios, {report['trial_summary']['p10_attempt_count']} retained attempts; scope: {report['trial_summary']['p10_measurement_scope']}",
        f"- P8-03 live reference: {report['trial_summary']['p8_live_roundtrips']} round trips, {report['trial_summary']['p8_live_legs']} completed legs, {report['trial_summary']['p8_durable_deliveries']} durable deliveries.",
        f"- P7-03: {report['trial_summary']['p7_matched_pairs']} matched planner pairs; P5 held-out: {report['trial_summary']['p5_heldout_records']} records.",
        "",
        "## Metric accounting", "",
        "| Metric | Status | Result or unavailable reason | Sources |", "|---|---|---|---|",
    ]
    for metric in report["metrics"]:
        result = metric["value"] if metric["value"] is not None else metric["unavailable_reason"]
        result_text = json.dumps(result, sort_keys=True, ensure_ascii=False)
        lines.append(f"| `{metric['id']}` | {metric['status']} | `{result_text}` | {', '.join(f'`{x}`' for x in metric['source_paths'])} |")
    lines += ["", "## Plots", ""]
    for plot in report["plots"]:
        lines.append(f"![{plot}](./{plot})")
        lines.append("")
    lines += ["## Limitations", ""]
    lines.extend(f"- {item}" for item in report["limitations"])
    lines += ["", "## Input provenance", "", "| Input | SHA-256 |", "|---|---|"]
    lines.extend(f"| `{item['path']}` | `{item['sha256']}` |" for item in report["inputs"])
    lines += ["", f"Metric catalog SHA-256: `{report['metric_catalog_sha256']}`", ""]
    return "\n".join(lines)


def render_plots(report: dict, root: Path) -> dict[str, str]:
    scenario_data, _ = _read_json(root, "p10_scenarios")
    heldout, _ = _read_json(root, "p5_heldout")
    benchmark, _ = _read_json(root, "p7_benchmark")
    labels = [row["scenario_id"] for row in scenario_data["scenarios"]]
    success = [row["passes"] / row["trial_count"] for row in scenario_data["scenarios"]]
    occupancy_len, terrain_len = [], []
    for row in scenario_data["scenarios"]:
        attempts = [item for item in scenario_data["attempts"] if item["scenario_id"] == row["scenario_id"]]
        occupancy_len.append(statistics.median(item["occupancy"]["total_path_length_m"] for item in attempts))
        terrain_len.append(statistics.median(item["terrain"]["total_path_length_m"] for item in attempts))
    ious = heldout["metrics"]["per_class"]
    perception_svg = _svg_bar_chart("P5-02 held-out IoU (one analog corpus)",
                                    ["regolith", "rock", "shadow", "mean IoU"],
                                    [("IoU", [ious["regolith"]["iou"], ious["rock"]["iou"], ious["shadow"]["iou"], heldout["metrics"]["miou"]], "#3973ac")],
                                    1.0, "Intersection over union")
    p7 = benchmark["modes"]
    return {
        "scenario-pass-rates.svg": _svg_bar_chart("P10 planner-fixture scenario pass rates", labels,
            [("Pass rate", success, "#32815a")], 1.0, "Pass fraction"),
        "planner-path-lengths.svg": _svg_bar_chart("Median planned grid path length", labels,
            [("Occupancy", occupancy_len, "#5577aa"), ("Terrain-aware", terrain_len, "#d18a32")],
            max(max(occupancy_len), max(terrain_len)) * 1.15, "Path length (m)"),
        "perception-iou.svg": perception_svg,
        "replan-latency.svg": _svg_bar_chart("P7-03 median planner replan time", ["weighted A*", "D* Lite"],
            [("Median", [p7["weighted_astar"]["median_replanning_ms"], p7["dstar_lite"]["median_replanning_ms"]], "#7955a3")],
            max(p7["weighted_astar"]["median_replanning_ms"], p7["dstar_lite"]["median_replanning_ms"]) * 1.15,
            "Planner time (ms)"),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, default=ROOT)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--report-id", default="p10-cross-trial-20261113")
    args = parser.parse_args()
    report = build_report(args.root.resolve(), args.report_id)
    schema = json.loads((args.root / INPUTS["report_schema"]).read_text(encoding="utf-8"))
    catalog = json.loads((args.root / INPUTS["metric_catalog"]).read_text(encoding="utf-8"))
    errors = validate_report(report, schema, {row["id"] for row in catalog["metrics"]})
    if errors:
        raise SystemExit("FAIL: " + "; ".join(errors))
    out = args.output_dir.resolve()
    out.mkdir(parents=True, exist_ok=False)
    (out / "report.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    (out / "report.md").write_text(render_markdown(report), encoding="utf-8")
    for name, svg in render_plots(report, args.root.resolve()).items():
        (out / name).write_text(svg, encoding="utf-8")
    print(f"PASS: report covers {len(report['metrics'])} metrics with provenance; outputs={out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
