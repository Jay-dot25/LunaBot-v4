"""Pure policy for safe mission/safety actions from dashboard view 6."""
from __future__ import annotations

import math
import json
import re
from typing import Any


MISSION_COMMANDS = {"PAUSE", "RESUME", "CANCEL", "RECOVER"}
CONFIRMED_ACTIONS = {"CANCEL", "ESTOP", "RESET_ESTOP"}
DELIVERY_ID = re.compile(r"[A-Za-z0-9._:-]{1,128}\Z")


def _fresh(age: float | None, limit: float) -> bool:
    return age is not None and math.isfinite(age) and 0.0 <= age <= limit


def safety_health_created_monotonic(payload: str) -> float | None:
    """Read the source clock required by the local safety-health protocol."""
    try:
        message = json.loads(payload)
        if not isinstance(message, dict) or message.get("schema") != 1:
            return None
        stamp = float(message["created_monotonic"])
    except (json.JSONDecodeError, KeyError, TypeError, ValueError):
        return None
    return stamp if math.isfinite(stamp) and stamp >= 0 else None


def decide_request(
    request: Any, *, mission_status_age: float | None,
    safety_health_age: float | None, fresh_mission_seconds: float = 1.5,
    fresh_safety_seconds: float = 0.5,
) -> dict[str, Any]:
    """Validate an action and return its only permitted relay route."""
    if not isinstance(request, dict):
        return {"accepted": False, "reason": "REQUEST_MUST_BE_OBJECT"}
    action = request.get("action")
    if not isinstance(action, str) or action.upper() != action:
        return {"accepted": False, "reason": "ACTION_MUST_BE_UPPERCASE_STRING"}
    if action not in MISSION_COMMANDS | CONFIRMED_ACTIONS | {"DELIVER", "ARM_STARTUP"}:
        return {"accepted": False, "reason": "ACTION_NOT_ALLOWED"}
    if action in CONFIRMED_ACTIONS and request.get("confirmed") is not True:
        return {"accepted": False, "reason": "EXPLICIT_CONFIRMATION_REQUIRED"}

    if action == "ESTOP":
        # Emergency stop intentionally remains available during telemetry loss.
        return {"accepted": True, "route": "emergency_stop", "command": "ESTOP"}
    if action == "ARM_STARTUP":
        if not _fresh(safety_health_age, fresh_safety_seconds):
            return {"accepted": False, "reason": "SAFETY_HEALTH_STALE_OR_UNKNOWN"}
        return {"accepted": True, "route": "arm_startup", "command": action}
    if action == "RESET_ESTOP":
        if not _fresh(mission_status_age, fresh_mission_seconds):
            return {"accepted": False, "reason": "MISSION_STATUS_STALE_OR_UNKNOWN"}
        if not _fresh(safety_health_age, fresh_safety_seconds):
            return {"accepted": False, "reason": "SAFETY_HEALTH_STALE_OR_UNKNOWN"}
        return {"accepted": True, "route": "reset_then_mission", "command": action}

    if not _fresh(mission_status_age, fresh_mission_seconds):
        return {"accepted": False, "reason": "MISSION_STATUS_STALE_OR_UNKNOWN"}
    if action == "DELIVER":
        delivery_id = request.get("delivery_id")
        if not isinstance(delivery_id, str) or not DELIVERY_ID.fullmatch(delivery_id):
            return {"accepted": False, "reason": "INVALID_DELIVERY_ID"}
        return {"accepted": True, "route": "mission", "command": f"DELIVER {delivery_id}"}
    return {"accepted": True, "route": "mission", "command": action}
