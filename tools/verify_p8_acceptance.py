#!/usr/bin/env python3
"""Verify three independent P8 round-trip runs and their SQLite proofs."""
from __future__ import annotations

import argparse
import json
import sqlite3
import sys
from datetime import datetime, timezone
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
from waypoint_registry import RegistryError, load_registry


def verify_trial(path: Path, trial_number: str) -> dict:
    summary_path = path / "p8_roundtrip.json"
    db_path = path / "mission_events.sqlite3"
    if not summary_path.is_file() or not db_path.is_file():
        raise ValueError(f"trial {trial_number}: missing summary or mission database")
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    if summary.get("task_id") != "P8-03" or summary.get("result") != "PASS":
        raise ValueError(f"trial {trial_number}: P8 round-trip result is not PASS")
    if summary.get("completed") is not True:
        raise ValueError(f"trial {trial_number}: summary does not mark completion")
    legs = summary.get("legs", [])
    if [leg.get("site") for leg in legs] != ["habitat", "science", "habitat"]:
        raise ValueError(f"trial {trial_number}: expected completed Habitat-Science-Habitat legs")
    if any(leg.get("result") != "MISSION_DEMO_PASS" or not leg.get("goal_id")
           for leg in legs):
        raise ValueError(f"trial {trial_number}: a leg lacks matching goal completion evidence")
    delivery = summary.get("delivery_id")
    if not delivery or summary.get("delivery_proofs") != 1:
        raise ValueError(f"trial {trial_number}: expected exactly one delivery proof")
    with sqlite3.connect(db_path) as connection:
        rows = connection.execute(
            "SELECT event_id, goal_id, payload_json FROM mission_events "
            "WHERE event_type='DELIVERY_COMPLETED'").fetchall()
    matching = []
    for event_id, goal_id, payload_json in rows:
        payload = json.loads(payload_json)
        if payload.get("delivery_id") == delivery:
            matching.append({"event_id": event_id, "goal_id": goal_id,
                             "payload": payload})
    if len(rows) != 1 or len(matching) != 1:
        raise ValueError(f"trial {trial_number}: durable delivery count is {len(rows)}, expected 1")
    if matching[0]["goal_id"] != legs[-1]["goal_id"]:
        raise ValueError(f"trial {trial_number}: delivery is not attached to the return goal")
    return {"trial": trial_number, "run_id": summary["run_id"],
            "legs": legs, "delivery_id": delivery,
            "durable_event": matching[0],
            "map_id": summary.get("map_id"),
            "map_transform_at_start": summary.get("map_transform_at_start")}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trial", type=Path)
    parser.add_argument("--trial-number", default="01")
    parser.add_argument("--validate-only", action="store_true")
    parser.add_argument("--run-root", type=Path)
    parser.add_argument("--registry", type=Path, default=ROOT / "config/waypoints.json")
    args = parser.parse_args()
    try:
        if args.trial:
            result = verify_trial(args.trial, args.trial_number)
            print(f"P8_TRIAL_VERIFIED trial={args.trial_number} "
                  f"run_id={result['run_id']} delivery_id={result['delivery_id']}")
            if args.validate_only:
                return 0
        if not args.run_root:
            parser.error("--run-root is required unless --validate-only is used")
        registry = load_registry(args.registry, map_id="lunar_p803_roundtrip",
                                 frame_id="map", require_bound=True)
        trials = [verify_trial(args.run_root / f"trial-{index:02d}",
                               f"{index:02d}") for index in range(1, 4)]
        if len({item["delivery_id"] for item in trials}) != 3:
            raise ValueError("delivery IDs are not unique across independent trials")
        if any(item["map_id"] != registry["map"]["id"] for item in trials):
            raise ValueError("a trial did not use the pinned P8 map identity")
        summary = {
            "task_id": "P8-03", "result": "PASS", "completed": True,
            "verified_at_utc": datetime.now(timezone.utc).isoformat(),
            "map_id": registry["map"]["id"],
            "map_metadata_sha256": registry["map"]["metadata_sha256"],
            "map_image_sha256": registry["map"]["image_sha256"],
            "checks": {"independent_roundtrips": 3,
                       "legs_per_roundtrip": 3,
                       "durable_deliveries": 3,
                       "delivery_attached_to_return_leg": True},
            "trials": trials,
        }
        args.run_root.mkdir(parents=True, exist_ok=True)
        (args.run_root / "summary.json").write_text(
            json.dumps(summary, indent=2) + "\n", encoding="utf-8")
        lines = ["# P8-03 Habitat-Science-Habitat acceptance",
                 "", "**Result: PASS**", "",
                 "Three independent headless Phase L runs completed the bound ",
                 "Habitat → Science → Habitat itinerary. Each leg has its own ",
                 "mission goal ID and completion status. Each run recorded one ",
                 "durable SQLite delivery event attached to its return goal.", "",
                 f"Pinned map: `{registry['map']['id']}` (frame `map`).",
                 f"Metadata SHA-256: `{registry['map']['metadata_sha256']}`.",
                 f"Image SHA-256: `{registry['map']['image_sha256']}`.", ""]
        for item in trials:
            lines.extend([f"## Trial {item['trial']}", "",
                          f"Run ID: `{item['run_id']}`; delivery ID: `{item['delivery_id']}`.", "",
                          "| Leg | Site | Goal ID | Result |",
                          "|---:|---|---|---|"])
            for index, leg in enumerate(item["legs"], start=1):
                lines.append(f"| {index} | {leg['site']} | `{leg['goal_id']}` | {leg['result']} |")
            lines.extend(["", f"Durable event: `{item['durable_event']['event_id']}` attached to return goal `{item['durable_event']['goal_id']}`.", ""])
        (args.run_root / "report.md").write_text("\n".join(lines), encoding="utf-8")
        print("P8_ACCEPTANCE_VERIFIED independent_roundtrips=3 "
              "legs=9 durable_deliveries=3")
    except (OSError, ValueError, KeyError, sqlite3.Error, RegistryError,
            json.JSONDecodeError) as exc:
        print(f"P8_ACCEPTANCE_FAIL {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
