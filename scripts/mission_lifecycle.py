"""Pure lifecycle and input-validation contracts for the Phase L supervisor."""
from __future__ import annotations

import hashlib
import json
import math


TRANSITIONS = {
    "WAITING_FOR_MAP": {"WAITING_FOR_GOAL", "FAILED", "CANCELLED", "ESTOPPED"},
    # A planner can publish a valid path before the supervisor receives the
    # goal (for example, when a goal was published during ROS discovery). In
    # that ordering the goal completes the inputs needed for ACTIVE directly.
    "WAITING_FOR_GOAL": {"WAITING_FOR_MAP", "READY", "ACTIVE", "FAILED",
                          "CANCELLED", "ESTOPPED"},
    "READY": {"WAITING_FOR_MAP", "WAITING_FOR_GOAL", "ACTIVE", "PAUSED",
              "BLOCKED", "FAILED", "CANCELLED", "ESTOPPED"},
    "ACTIVE": {"WAITING_FOR_MAP", "WAITING_FOR_GOAL", "READY",
               "COMPLETED", "PAUSED", "BLOCKED", "FAILED", "CANCELLED",
               "ESTOPPED"},
    "PAUSED": {"READY", "ACTIVE", "BLOCKED", "FAILED", "CANCELLED",
               "ESTOPPED"},
    "BLOCKED": {"READY", "ACTIVE", "FAILED", "CANCELLED", "ESTOPPED"},
    "COMPLETED": {"WAITING_FOR_MAP", "WAITING_FOR_GOAL", "READY", "ACTIVE",
                  "PAUSED", "BLOCKED", "FAILED", "CANCELLED", "ESTOPPED"},
    "FAILED": {"READY", "ACTIVE", "CANCELLED", "ESTOPPED"},
    "CANCELLED": {"ESTOPPED"},
    "ESTOPPED": {"READY", "ACTIVE", "CANCELLED", "FAILED"},
}


class MissionLifecycle:
    """Small explicit state machine; restart starts a fresh WAITING_FOR_MAP."""

    def __init__(self) -> None:
        self.state = "WAITING_FOR_MAP"

    def transition(self, target: str) -> bool:
        if target == self.state:
            return False
        if target not in TRANSITIONS.get(self.state, set()):
            raise ValueError(f"invalid mission lifecycle transition: {self.state}->{target}")
        self.state = target
        return True


def validate_occupancy_grid(grid, expected_frame: str = "map") -> tuple[bool, str]:
    """Reject malformed grids before the mission supervisor treats them as valid."""
    info = grid.info
    width, height = int(info.width), int(info.height)
    if width <= 0 or height <= 0:
        return False, "NONPOSITIVE_DIMENSIONS"
    if len(grid.data) != width * height:
        return False, "DATA_LENGTH_MISMATCH"
    if grid.header.frame_id != expected_frame:
        return False, "FRAME_MISMATCH"
    if not math.isfinite(float(info.resolution)) or float(info.resolution) <= 0:
        return False, "INVALID_RESOLUTION"
    origin = info.origin
    coordinates = (origin.position.x, origin.position.y, origin.position.z)
    quaternion = (origin.orientation.x, origin.orientation.y,
                  origin.orientation.z, origin.orientation.w)
    if not all(math.isfinite(float(value)) for value in coordinates + quaternion):
        return False, "NONFINITE_ORIGIN"
    qnorm = math.sqrt(sum(float(value) ** 2 for value in quaternion))
    if not math.isfinite(qnorm) or qnorm < 1e-12:
        return False, "INVALID_ORIGIN_ORIENTATION"
    if any(int(value) < -1 or int(value) > 100 for value in grid.data):
        return False, "INVALID_OCCUPANCY_VALUE"
    return True, "VALID"


def validate_goal_pose(message) -> tuple[bool, str]:
    pose = message.pose
    values = (pose.position.x, pose.position.y, pose.position.z,
              pose.orientation.x, pose.orientation.y,
              pose.orientation.z, pose.orientation.w)
    if not all(math.isfinite(float(value)) for value in values):
        return False, "NONFINITE_POSE"
    qnorm = math.sqrt(sum(float(value) ** 2 for value in values[3:]))
    if qnorm < 1e-12:
        return False, "INVALID_ORIENTATION"
    if not message.header.frame_id:
        return False, "MISSING_FRAME"
    return True, "VALID"


def goal_identifier(message) -> str:
    """Create a stable opaque identifier from a stamped goal request."""
    pose = message.pose
    stamp = message.header.stamp
    payload = {
        "frame": message.header.frame_id,
        "stamp": [int(stamp.sec), int(stamp.nanosec)],
        "pose": [float(pose.position.x), float(pose.position.y),
                 float(pose.position.z), float(pose.orientation.x),
                 float(pose.orientation.y), float(pose.orientation.z),
                 float(pose.orientation.w)],
    }
    digest = hashlib.sha256(json.dumps(
        payload, sort_keys=True, separators=(",", ":"),
        allow_nan=False).encode("utf-8")).hexdigest()[:16]
    return f"goal-{digest}"
