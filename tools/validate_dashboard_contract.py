#!/usr/bin/env python3
"""Validate P9 dashboard candidate comparison and six-view ROS contract."""
from __future__ import annotations

import argparse
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
REQUIRED_CANDIDATES = {
    "browser_ros_gateway", "foxglove_bridge_desktop", "native_qt_rviz_rqt"
}
REQUIRED_VIEWS = {
    "system_health", "sensors_and_perception", "map_and_localization",
    "terrain_layers", "planning_and_evaluation", "mission_and_safety_controls",
}
REQUIRED_CANDIDATE_FIELDS = {
    "ros2_support", "streaming", "maps", "controls", "latency_resources",
    "wsl_fit", "license", "maintenance", "evidence_urls",
}


def validate(options: dict, contract: dict, adapter: dict | None = None,
             controls: dict | None = None) -> list[str]:
    errors: list[str] = []
    candidates = options.get("candidates", [])
    candidate_ids = [row.get("stack_id") for row in candidates]
    if set(candidate_ids) != REQUIRED_CANDIDATES or len(candidate_ids) != 3:
        errors.append("exactly the three required dashboard candidates must be compared")
    if options.get("preferred_stack_id") not in REQUIRED_CANDIDATES:
        errors.append("preferred_stack_id must name a compared candidate")
    for candidate in candidates:
        for field in REQUIRED_CANDIDATE_FIELDS:
            if not candidate.get(field):
                errors.append(f"{candidate.get('stack_id', '<candidate>')}: missing {field}")
    if not options.get("benchmarks_pending"):
        errors.append("pending latency/resource benchmark list is required")

    views = contract.get("views", [])
    view_ids = [row.get("view_id") for row in views]
    if set(view_ids) != REQUIRED_VIEWS or len(view_ids) != 6:
        errors.append("dashboard contract must contain exactly the six required views")
    read_only = set(contract.get("read_only_topics", []))
    for view in views:
        view_id = view.get("view_id", "<view>")
        if not view.get("topics"):
            errors.append(f"{view_id}: at least one read topic is required")
        for topic in view.get("topics", []):
            if topic not in read_only:
                errors.append(f"{view_id}: unknown or writable telemetry topic {topic}")
        if not all(view.get(flag) for flag in ("show_source_stamp", "show_receive_age", "show_stale_state")):
            errors.append(f"{view_id}: source stamp, receive age, and stale state must be visible")
    for view in views[:5]:
        if view.get("metadata_topic") != "/lunabot/dashboard/telemetry":
            errors.append(f"{view.get('view_id')}: telemetry metadata topic is required")
    control_view = next((view for view in views
                         if view.get("view_id") == "mission_and_safety_controls"), {})
    if control_view.get("write_topics") != ["/lunabot/dashboard/command_request"]:
        errors.append("controls view must write only to the guarded dashboard request topic")
    if control_view.get("call_services") != []:
        errors.append("controls view must call services through the guarded gateway")
    if control_view.get("metadata_topic") != "/lunabot/dashboard/telemetry":
        errors.append("controls view requires health and mission freshness metadata")
    if not {"CANCEL", "ESTOP", "RESET_ESTOP"}.issubset(set(control_view.get("confirm_commands", []))):
        errors.append("cancel, e-stop, and e-stop reset require explicit confirmation")
    if contract.get("map_frame") != "map" or contract.get("telemetry_contract", {}).get("map_and_plan_frame") != "map":
        errors.append("map and planner views must use the map frame")

    writes = contract.get("write_contract", {})
    if writes.get("velocity_publish_allowed") is not False:
        errors.append("dashboard must not publish velocity commands")
    if writes.get("operational_truth_access") is not False:
        errors.append("dashboard must not receive simulator truth")
    if set(writes.get("forbidden_write_topics", [])) != {
        "/cmd_vel", "/cmd_vel_in", "/lunabot/actuator_request", "/world/lunar_world/pose/info"
    }:
        errors.append("forbidden velocity and truth interfaces must be listed")
    if "/lunabot/mission/command" not in writes.get("topic_types", {}):
        errors.append("mission command topic type is required")
    if writes.get("topic_types", {}).get("/lunabot/dashboard/command_request") != "std_msgs/msg/String":
        errors.append("guarded dashboard command request must use std_msgs/msg/String")
    for service in ("/lunabot/safety/emergency_stop", "/lunabot/safety/reset", "/lunabot/safety/arm_startup"):
        if service not in writes.get("service_types", {}):
            errors.append(f"missing safety service contract: {service}")
    if writes.get("require_fresh_mission_status_seconds") != 1.5:
        errors.append("mission controls must require fresh status within the follower's 1.5s timeout")
    if writes.get("require_fresh_safety_health_seconds") != 0.5:
        errors.append("safety controls must require fresh health within the 0.5s monitor contract")

    if controls is not None:
        if controls.get("request_topic") != "/lunabot/dashboard/command_request":
            errors.append("controls gateway request topic must match the guarded view interface")
        if controls.get("status_topic") not in read_only:
            errors.append("controls gateway result topic must be read-only dashboard telemetry")
        if controls.get("fresh_mission_status_seconds") != writes.get("require_fresh_mission_status_seconds"):
            errors.append("controls gateway mission freshness must match the dashboard contract")
        if controls.get("fresh_safety_health_seconds") != writes.get("require_fresh_safety_health_seconds"):
            errors.append("controls gateway safety freshness must match the dashboard contract")
        if controls.get("safety_services") != {
            "ESTOP": "/lunabot/safety/emergency_stop",
            "RESET_ESTOP": "/lunabot/safety/reset",
            "ARM_STARTUP": "/lunabot/safety/arm_startup",
        }:
            errors.append("controls gateway must use the three existing safety services")
        if set(controls.get("confirmed_actions", [])) != set(control_view.get("confirm_commands", [])):
            errors.append("controls gateway and view must require the same confirmations")
        if set(controls.get("mission_commands", [])) != {
            "PAUSE", "RESUME", "CANCEL", "RECOVER", "DELIVER"
        }:
            errors.append("controls gateway mission action allowlist is incomplete or broader than contract")

    if adapter is not None:
        adapter_rows = adapter.get("topics", [])
        adapter_topics = [row.get("topic") for row in adapter_rows]
        required_view_topics = set().union(*(set(row.get("topics", [])) for row in views[:5])) if len(views) >= 5 else set()
        if set(adapter_topics) != required_view_topics or len(adapter_topics) != len(required_view_topics):
            errors.append("telemetry adapter must cover each views 1-5 topic exactly once")
        if adapter.get("output_topic") != "/lunabot/dashboard/telemetry":
            errors.append("telemetry adapter must publish only its dedicated metadata topic")
        if adapter.get("output_topic") not in read_only or adapter.get("output_topic") in writes.get("forbidden_write_topics", []):
            errors.append("telemetry metadata output must be an allowlisted read-only dashboard topic")
        if adapter.get("output_topic") in adapter_topics:
            errors.append("telemetry metadata output cannot be an adapter input")
        for row in adapter_rows:
            topic = row.get("topic", "<topic>")
            if topic not in read_only:
                errors.append(f"adapter input is not an allowlisted dashboard topic: {topic}")
            view_ids = set(row.get("views", [row.get("view")]))
            if not row.get("type") or not view_ids or not view_ids.issubset({view.get("view_id") for view in views[:5]}):
                errors.append(f"{topic}: adapter type and views 1-5 provenance are required")
            try:
                if float(row.get("stale_after_seconds", 0)) <= 0:
                    errors.append(f"{topic}: stale_after_seconds must be positive")
                if "source_age_limit_seconds" in row and float(row["source_age_limit_seconds"]) <= 0:
                    errors.append(f"{topic}: source_age_limit_seconds must be positive")
            except (TypeError, ValueError):
                errors.append(f"{topic}: freshness limits must be numeric")
    return errors


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--options", type=Path, default=ROOT / "config/dashboard/options.json")
    parser.add_argument("--contract", type=Path, default=ROOT / "config/dashboard/contracts.json")
    parser.add_argument("--adapter", type=Path, default=ROOT / "config/dashboard/adapter.json")
    parser.add_argument("--controls", type=Path, default=ROOT / "config/dashboard/controls.json")
    args = parser.parse_args()
    try:
        options = json.loads(args.options.read_text(encoding="utf-8"))
        contract = json.loads(args.contract.read_text(encoding="utf-8"))
        adapter = json.loads(args.adapter.read_text(encoding="utf-8"))
        controls = json.loads(args.controls.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        print(f"FAIL: cannot load dashboard contract: {exc}")
        return 1
    errors = validate(options, contract, adapter, controls)
    if errors:
        for error in errors:
            print(f"FAIL: {error}")
        return 1
    print("PASS: 3 stacks, 6 views, views 1-5 feeds, guarded view 6 controls, safe write boundary")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
