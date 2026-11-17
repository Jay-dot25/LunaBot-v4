"""Pure helpers for dashboard freshness and telemetry provenance."""
from __future__ import annotations

import json
import math
from typing import Any


def stamp_seconds(stamp: Any) -> float | None:
    """Convert a ROS builtin stamp to seconds, rejecting invalid values."""
    if stamp is None:
        return None
    try:
        value = float(stamp.sec) + float(stamp.nanosec) / 1_000_000_000.0
    except (AttributeError, TypeError, ValueError):
        return None
    return value if math.isfinite(value) and value >= 0 else None


def message_provenance(message: Any) -> dict[str, Any]:
    """Return an unmodified source stamp/frame when the message carries one."""
    header = getattr(message, "header", None)
    if header is not None:
        stamp = stamp_seconds(getattr(header, "stamp", None))
        return {
            "source_stamp_seconds": stamp,
            "source_clock": "ros",
            "frame_id": str(getattr(header, "frame_id", "")) or None,
            "stamp_origin": "message_header" if stamp is not None else "missing_header_stamp",
        }
    transforms = getattr(message, "transforms", None)
    if transforms is not None:
        stamps = [stamp_seconds(getattr(item, "header", None).stamp)
                  for item in transforms if getattr(item, "header", None) is not None]
        valid_stamps = [stamp for stamp in stamps if stamp is not None]
        frames = sorted({frame for item in transforms
                         for frame in (getattr(getattr(item, "header", None), "frame_id", ""),
                                       getattr(item, "child_frame_id", "")) if frame})
        return {
            "source_stamp_seconds": max(valid_stamps) if valid_stamps else None,
            "source_clock": "ros" if valid_stamps else None,
            "frame_id": None,
            "source_frame_ids": frames,
            "stamp_origin": "tf_transform_headers" if valid_stamps else "missing_transform_stamps",
        }

    # Many project status feeds are std_msgs/String JSON. Preserve a declared
    # source time if present, without treating unrelated local monotonic time as
    # ROS simulation time.
    payload = getattr(message, "data", None)
    if isinstance(payload, str):
        try:
            parsed = json.loads(payload)
        except (json.JSONDecodeError, TypeError):
            parsed = None
        if isinstance(parsed, dict):
            for key in ("source_stamp_seconds", "stamp_sec", "sim_time"):
                try:
                    value = float(parsed[key])
                except (KeyError, TypeError, ValueError):
                    continue
                if math.isfinite(value) and value >= 0:
                    return {
                        "source_stamp_seconds": value,
                        "source_clock": "ros_sim",
                        "frame_id": parsed.get("frame_id") or None,
                        "source_frame_ids": [],
                        "stamp_origin": f"status_json.{key}",
                    }
            try:
                value = float(parsed["created_monotonic"])
            except (KeyError, TypeError, ValueError):
                value = None
            if value is not None and math.isfinite(value) and value >= 0:
                return {
                    "source_stamp_seconds": value,
                    "source_clock": "host_monotonic",
                    "frame_id": None,
                    "source_frame_ids": [],
                    "stamp_origin": "status_json.created_monotonic",
                }
    return {
        "source_stamp_seconds": None,
        "source_clock": None,
        "frame_id": None,
        "source_frame_ids": [],
        "stamp_origin": "not_provided_by_message",
    }


def evaluate_topic(
    *, topic: str, message_type: str, last_receive_monotonic: float | None,
    now_monotonic: float, source_stamp_seconds: float | None,
    source_clock: str | None, frame_id: str | None, source_nodes: list[str],
    source_frame_ids: list[str] | None = None,
    stale_after_seconds: float, source_age_limit_seconds: float | None = None,
    freshness_mode: str = "stream", stamp_origin: str | None = None,
    now_ros_seconds: float | None = None, expected_frame: str | None = None,
    error: str | None = None,
) -> dict[str, Any]:
    """Build a bounded dashboard metadata row from measured receive/source times."""
    receive_age = None if last_receive_monotonic is None else max(
        0.0, now_monotonic - last_receive_monotonic
    )
    if error:
        state, reason = "unknown", "subscription_error"
    elif last_receive_monotonic is None:
        state, reason = "unknown", "no_sample_received"
    elif freshness_mode in {"latched", "event"}:
        state, reason = "fresh", freshness_mode
    elif receive_age is not None and receive_age > stale_after_seconds:
        state, reason = "stale", "receive_timeout"
    else:
        state, reason = "fresh", "within_receive_timeout"

    source_age = None
    if (source_stamp_seconds is not None and source_clock in {"ros", "ros_sim"}
            and now_ros_seconds is not None):
        source_age = now_ros_seconds - source_stamp_seconds
        if source_age < 0:
            state, reason = "unknown", "source_clock_ahead_of_adapter"
        elif (source_age_limit_seconds is not None
              and source_age > source_age_limit_seconds):
            state, reason = "stale", "source_stamp_timeout"
    if expected_frame and frame_id and frame_id != expected_frame:
        state, reason = "unknown", "expected_frame_mismatch"
    return {
        "topic": topic,
        "type": message_type,
        "state": state,
        "state_reason": reason,
        "source_stamp_seconds": source_stamp_seconds,
        "source_clock": source_clock,
        "source_age_seconds": source_age,
        "source_age_limit_seconds": source_age_limit_seconds,
        "receive_age_seconds": receive_age,
        "frame_id": frame_id,
        "expected_frame": expected_frame,
        "source_frame_ids": sorted(set(source_frame_ids or [])),
        "source_nodes": sorted(set(source_nodes)),
        "stamp_origin": stamp_origin or ("not_provided_by_message" if source_stamp_seconds is None else "source_message"),
        "freshness_mode": freshness_mode,
        "subscription_error": error,
    }
