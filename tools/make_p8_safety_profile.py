#!/usr/bin/env python3
"""Write a bounded llvmpipe profile without changing shared safety defaults."""
from __future__ import annotations

import argparse
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def make_profile(source: Path, destination: Path, *,
                 health_report_timeout_wall_sec: float = 0.5,
                 input_timeout_wall_sec: float = 3.0) -> dict:
    settings = json.loads(source.read_text(encoding="utf-8"))
    health = settings["health"]
    if health["clock_timeout_wall_sec"] != 0.5:
        raise ValueError("unexpected baseline stalled-clock limit")
    if health["max_stamp_age_sim_sec"] != 0.25:
        raise ValueError("unexpected baseline simulation-time stamp-age limit")
    # Software-rendered WSL can deliver sensor updates several wall seconds
    # apart while simulation time advances slowly. Keep a bounded wall-age
    # stop and retain the independent sim-time age and clock-stall gates.
    if input_timeout_wall_sec <= 0:
        raise ValueError("sensor input timeout must be positive")
    health["input_timeout_wall_sec"] = float(input_timeout_wall_sec)
    health["max_stamp_age_sim_sec"] = 3.0
    # The software-rendered simulation periodically stalls planner updates;
    # retain a safe path longer while the planner revalidates it per revision.
    health["path_timeout_sec"] = 18.0
    # Allow bounded scheduler gaps in llvmpipe while preserving a fail-stop
    # clock watchdog; a simulation clock stopped for two wall seconds still
    # latches the controller. Sensor wall age remains separately bounded.
    health["clock_timeout_wall_sec"] = 2.0
    # slam_toolbox emits map->odom stamps up to 0.2 s ahead of /clock because
    # of its 0.1 s publish batching. Keep that allowance local to this TF;
    # raw sensor stamps retain their existing global future tolerance.
    health["map_tf_stamp_offset_sec"] = 0.0
    health["map_tf_future_tolerance_sim_sec"] = 0.25
    health["map_tf_motion_grace_wall_sec"] = 30.0
    if health_report_timeout_wall_sec <= 0:
        raise ValueError("health report timeout must be positive")
    health["health_report_timeout_wall_sec"] = float(
        health_report_timeout_wall_sec)
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(json.dumps(settings, indent=2) + "\n",
                           encoding="utf-8")
    return settings


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path,
                        default=ROOT / "config/safety/control.json")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--health-report-timeout-wall-sec", type=float, default=0.5)
    parser.add_argument("--input-timeout-wall-sec", type=float, default=3.0)
    args = parser.parse_args()
    try:
        settings = make_profile(
            args.source, args.output,
            health_report_timeout_wall_sec=args.health_report_timeout_wall_sec,
            input_timeout_wall_sec=args.input_timeout_wall_sec)
    except (OSError, KeyError, ValueError, json.JSONDecodeError) as exc:
        parser.error(str(exc))
    print(f"P8_SAFETY_PROFILE_PASS input_timeout_wall_sec="
          f"{settings['health']['input_timeout_wall_sec']:.1f} "
          f"max_stamp_age_sim_sec={settings['health']['max_stamp_age_sim_sec']:.2f} "
          f"clock_timeout_wall_sec={settings['health']['clock_timeout_wall_sec']:.1f} "
          f"health_report_timeout_wall_sec={settings['health']['health_report_timeout_wall_sec']:.1f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
