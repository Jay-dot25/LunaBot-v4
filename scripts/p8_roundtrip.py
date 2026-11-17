#!/usr/bin/env python3
"""Run a fail-closed Habitat -> Science -> Habitat delivery itinerary.

The runner never publishes velocity. Each leg is a normal Phase L goal and is
counted only when the mission supervisor reports MISSION_DEMO_PASS for the
exact goal ID submitted by this runner. A delivery is recorded only after the
return leg has completed.
"""
from __future__ import annotations

import argparse
import json
import math
import re
import sys
import time
import uuid
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
from waypoint_registry import RegistryError, load_registry, resolve_waypoint
from mission_lifecycle import goal_identifier, validate_occupancy_grid
from terrain_plan import merge_obstacle_grid, world_to_grid


ITINERARY = ("habitat", "science", "habitat")
MAX_SAFE_RECOVERIES_PER_LEG = 8
_GOAL_ID = re.compile(r"(?:^|\s)goal_id=([A-Za-z0-9_-]+)(?:\s|$)")


def parse_goal_id(status: str) -> str | None:
    match = _GOAL_ID.search(status)
    return match.group(1) if match else None


def delivery_id(run_id: str, roundtrip: int) -> str:
    safe = re.sub(r"[^A-Za-z0-9._-]", "-", run_id)[:70].strip("-")
    if not safe:
        raise ValueError("run ID must contain at least one safe character")
    if roundtrip < 1:
        raise ValueError("roundtrip index must be positive")
    return f"p803-{safe}-{roundtrip:02d}"


def transform_matches(actual: tuple[float, float, float], expected: dict,
                      *, position_tolerance: float = 0.25,
                      yaw_tolerance: float = 0.15) -> bool:
    return (math.hypot(actual[0] - float(expected["x"]),
                       actual[1] - float(expected["y"])) <= position_tolerance
            and abs(math.atan2(math.sin(actual[2] - float(expected["yaw"])),
                               math.cos(actual[2] - float(expected["yaw"]))))
            <= yaw_tolerance)


def waypoint_poses(registry: dict) -> list[tuple[str, dict]]:
    result = []
    navigation_frame = registry.get("navigation_frame", "map")
    for ident in ITINERARY:
        item = registry.get("waypoints_by_id", {}).get(ident)
        if item is not None and item.get("binding_status") == "scenario":
            pose = item.get("pose")
        else:
            pose = resolve_waypoint(registry, ident)["pose"]
        if navigation_frame == "odom":
            reference = (registry.get("map", {}).get("reference_map_to_odom")
                         or registry.get("map_transform_at_start", {}).get(
                             "map_to_odom"))
            if not isinstance(reference, dict):
                raise RegistryError(
                    "odom navigation requires a qualified map->odom reference")
            tx, ty, yaw = (float(reference["x"]), float(reference["y"]),
                           float(reference["yaw"]))
            c, s = math.cos(yaw), math.sin(yaw)
            dx, dy = float(pose["x"]) - tx, float(pose["y"]) - ty
            pose = {"x": c * dx + s * dy,
                    "y": -s * dx + c * dy,
                    "yaw": float(pose["yaw"]) - yaw}
        result.append((ident, pose))
    return result


def planner_goal_cell_cost(cost_grid, obstacle_grid, pose: dict, *,
                           map_to_odom: tuple[float, float, float] | None = None,
                           obstacle_threshold: int = 65,
                           obstacle_inflation: float = 0.35) -> int:
    """Return a goal's cost after applying the planner's obstacle overlay.

    Phase H plans on a merged terrain/obstacle grid. Checking the terrain map
    alone lets a goal appear traversable to this runner while the planner
    rejects it as occupied. Keep the dispatch preflight equivalent to Phase H.
    """
    if cost_grid is None or obstacle_grid is None:
        raise RuntimeError("terrain and obstacle maps are both required")
    frame = cost_grid.header.frame_id
    if not frame or obstacle_grid.header.frame_id != frame:
        raise RuntimeError("terrain and obstacle map frames do not match")
    ci, oi = cost_grid.info, obstacle_grid.info
    for grid, info, label in ((cost_grid, ci, "terrain"),
                              (obstacle_grid, oi, "obstacle")):
        if (info.width <= 0 or info.height <= 0
                or not math.isfinite(info.resolution) or info.resolution <= 0
                or len(grid.data) != info.width * info.height):
            raise RuntimeError(f"live {label} map is malformed")

    def origin(info):
        q = info.origin.orientation
        yaw = math.atan2(2.0 * q.w * q.z,
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return info.origin.position.x, info.origin.position.y, yaw

    cx, cy, cyaw = origin(ci)
    ox, oy, oyaw = origin(oi)
    merged = merge_obstacle_grid(
        cost_grid.data, int(ci.width), int(ci.height),
        origin_x=cx, origin_y=cy, origin_yaw=cyaw,
        resolution=float(ci.resolution), obstacles=obstacle_grid.data,
        obstacle_width=int(oi.width), obstacle_height=int(oi.height),
        obstacle_origin_x=ox, obstacle_origin_y=oy,
        obstacle_origin_yaw=oyaw, obstacle_resolution=float(oi.resolution),
        obstacle_threshold=obstacle_threshold,
        inflation_radius=obstacle_inflation)

    x, y = float(pose["x"]), float(pose["y"])
    if map_to_odom is not None:
        tx, ty, tf_yaw = map_to_odom
        c, s = math.cos(tf_yaw), math.sin(tf_yaw)
        x, y = tx + c * x - s * y, ty + s * x + c * y
    gx, gy = world_to_grid(x, y, origin_x=cx, origin_y=cy,
                           origin_yaw=cyaw, resolution=float(ci.resolution))
    if not (0 <= gx < ci.width and 0 <= gy < ci.height):
        raise RuntimeError(
            f"waypoint ({pose['x']:.2f},{pose['y']:.2f}) is outside "
            "the live terrain cost map")
    return int(merged[gy * ci.width + gx])


def write_preflight_failure(path: Path, run_id: str, error: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps({
        "task_id": "P8-03", "run_id": run_id, "result": "PREFLIGHT_FAIL",
        "completed": False, "legs": [], "delivery_proofs": 0,
        "error": error,
    }, indent=2) + "\n", encoding="utf-8")


def scenario_registry(path: Path, map_transform: tuple[float, float, float]) -> dict:
    """Resolve world-site coordinates into map using the measured map->odom TF.

    The scenario start pose is the rover's known world spawn. At startup the
    rover odometry origin is checked against that spawn, and the current
    map->odom transform supplies the remaining rotation/translation.
    """
    scenario = json.loads(path.read_text(encoding="utf-8"))
    start = scenario["start_pose_world"]
    tx, ty, tyaw = map_transform
    c, s = math.cos(tyaw), math.sin(tyaw)
    c0, s0 = math.cos(float(start["yaw"])), math.sin(float(start["yaw"]))
    poses = {}
    for item in scenario["waypoints"]:
        world = item["pose_world"]
        dx = float(world["x"]) - float(start["x"])
        dy = float(world["y"]) - float(start["y"])
        odom_x = c0 * dx + s0 * dy
        odom_y = -s0 * dx + c0 * dy
        poses[item["id"]] = {
            "x": tx + c * odom_x - s * odom_y,
            "y": ty + s * odom_x + c * odom_y,
            "yaw": float(world["yaw"]) - float(start["yaw"]) + tyaw,
        }
    reference = {"x": tx, "y": ty, "yaw": tyaw}
    return {"map": {"id": scenario["scenario_id"], "frame_id": "map",
                     "reference_map_to_odom": reference},
            # Dispatch site goals in the stable SLAM map frame. Sending their
            # startup odom equivalents lets later map->odom corrections move
            # the effective goal across the occupancy grid during a leg.
            "navigation_frame": "map",
            "waypoints_by_id": {
                key: {"id": key, "pose": pose, "binding_status": "scenario"}
                for key, pose in poses.items()},
            "scenario": scenario, "map_transform_at_start": {
                "map_to_odom": {"x": tx, "y": ty, "yaw": tyaw},
                "world_to_odom_origin": {"x": float(start["x"]),
                                          "y": float(start["y"]),
                                          "yaw": float(start["yaw"])}}}


class RoundtripRunner:
    def __init__(self, *, registry: dict, run_id: str, summary_path: Path,
                 leg_timeout_s: float, input_timeout_s: float,
                 qualification_mode: bool = False) -> None:
        import rclpy
        from geometry_msgs.msg import PoseStamped
        from nav_msgs.msg import OccupancyGrid, Path
        from rclpy.node import Node
        from rclpy.qos import (DurabilityPolicy, QoSProfile,
                               ReliabilityPolicy)
        from std_msgs.msg import String
        from tf2_msgs.msg import TFMessage

        self.rclpy = rclpy
        self.PoseStamped = PoseStamped
        self.String = String
        self.node = Node("lunabot_p8_roundtrip")
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.node.create_subscription(String, "/lunabot/mission/status",
                                      self.on_status, qos)
        self.node.create_subscription(String, "/lunabot/mission/events",
                                      self.on_event, qos)
        self.node.create_subscription(OccupancyGrid, "/map", self.on_map, qos)
        self.node.create_subscription(OccupancyGrid, "/lunabot/terrain/cost_map",
                                      self.on_cost_map, qos)
        self.node.create_subscription(OccupancyGrid, "/lunabot/obstacles/map",
                                      self.on_obstacle_map, qos)
        self.node.create_subscription(Path, "/lunabot/terrain/plan",
                                      self.on_plan, qos)
        self.node.create_subscription(String, "/lunabot/terrain/planner/status",
                                      self.on_planner_status, qos)
        self.node.create_subscription(TFMessage, "/tf", self.on_tf, 10)
        self.goal_pub = self.node.create_publisher(PoseStamped, "/goal_pose", 10)
        self.command_pub = self.node.create_publisher(
            String, "/lunabot/mission/command", 10)
        self.registry = registry
        self.run_id = run_id
        self.summary_path = summary_path
        self.leg_timeout_s = leg_timeout_s
        self.input_timeout_s = input_timeout_s
        self.qualification_mode = qualification_mode
        self.map = None
        self.cost_map = None
        self.obstacle_map = None
        self.map_to_odom = None
        self.nonempty_plan_count = 0
        self.latest_planner_status = ""
        self.statuses: list[str] = []
        self.events: list[str] = []
        self.legs: list[dict] = []
        self.safety_recoveries: list[dict] = []
        self.current_expected_goal = ""
        self.current_goal_message = None
        self.delivery_recorded = False
        self.delivery = ""

    def on_status(self, msg) -> None:
        text = msg.data.strip()
        self.statuses.append(text)
        if len(self.statuses) > 1000:
            del self.statuses[:500]

    def on_event(self, msg) -> None:
        text = msg.data.strip()
        self.events.append(text)
        if len(self.events) > 1000:
            del self.events[:500]

    def on_map(self, msg) -> None:
        self.map = msg

    def on_cost_map(self, msg) -> None:
        self.cost_map = msg

    def on_obstacle_map(self, msg) -> None:
        self.obstacle_map = msg

    def goal_cell_cost(self, pose: dict) -> int:
        grid = self.cost_map
        if grid is None or self.obstacle_map is None:
            raise RuntimeError("no live terrain and obstacle maps")
        if grid.header.frame_id != self.registry["map"]["frame_id"]:
            raise RuntimeError(
                f"cost map frame {grid.header.frame_id!r} does not match "
                f"waypoint frame {self.registry['map']['frame_id']!r}")
        map_to_odom = None
        if self.registry.get("navigation_frame", "map") == "odom":
            if self.map_to_odom is None:
                raise RuntimeError("no live map->odom transform for odom goal")
            map_to_odom = self.map_to_odom
        return planner_goal_cell_cost(
            grid, self.obstacle_map, pose, map_to_odom=map_to_odom,
            obstacle_threshold=65, obstacle_inflation=0.35)

    def on_plan(self, msg) -> None:
        if msg.poses:
            self.nonempty_plan_count += 1

    def on_planner_status(self, msg) -> None:
        self.latest_planner_status = msg.data.strip()

    def on_tf(self, msg) -> None:
        for transform in msg.transforms:
            if (transform.header.frame_id, transform.child_frame_id) != ("map", "odom"):
                continue
            t, q = transform.transform.translation, transform.transform.rotation
            yaw = math.atan2(2.0 * (q.w*q.z + q.x*q.y),
                             1.0 - 2.0 * (q.y*q.y + q.z*q.z))
            if all(math.isfinite(v) for v in (t.x, t.y, yaw)):
                self.map_to_odom = (t.x, t.y, yaw)

    def spin_until(self, predicate, deadline: float) -> bool:
        while self.rclpy.ok() and time.monotonic() < deadline:
            self.rclpy.spin_once(self.node, timeout_sec=0.1)
            if predicate():
                return True
        return False

    def send_command(self, text: str) -> None:
        self.command_pub.publish(self.String(data=text))

    def submit_goal(self, site: str, pose: dict) -> str:
        message = self.PoseStamped()
        message.header.frame_id = self.registry.get(
            "navigation_frame", self.registry["map"]["frame_id"])
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.pose.position.x = float(pose["x"])
        message.pose.position.y = float(pose["y"])
        message.pose.orientation.z = math.sin(float(pose["yaw"]) / 2.0)
        message.pose.orientation.w = math.cos(float(pose["yaw"]) / 2.0)
        expected = goal_identifier(message)
        self.current_expected_goal = expected
        self.current_goal_message = message
        self.goal_pub.publish(message)
        print(f"P8_LEG_SUBMITTED site={site} goal_id={expected}", flush=True)
        return expected

    def wait_for_goal(self, expected: str, deadline: float) -> str:
        next_goal_refresh = time.monotonic() + 0.5
        blocked_plan_count = None
        recovery_pending = False
        recoveries_for_leg = 0
        while self.rclpy.ok() and time.monotonic() < deadline:
            self.rclpy.spin_once(self.node, timeout_sec=0.1)
            # Repeat the identical stamped goal so every volatile subscriber
            # sees it even if DDS discovery completes just after the first send.
            if (self.current_goal_message is not None
                    and time.monotonic() >= next_goal_refresh):
                self.goal_pub.publish(self.current_goal_message)
                next_goal_refresh = time.monotonic() + 1.0
            status = next((item for item in reversed(self.statuses[-40:])
                           if parse_goal_id(item) == expected), "")
            if "MISSION_DEMO_PASS" in status and "lifecycle=COMPLETED" in status:
                return "PASS"
            if "lifecycle=FAILED" in status:
                raise RuntimeError(f"mission stopped for {expected}: {status}")
            if "lifecycle=BLOCKED" in status:
                if "reason=FOLLOWER_SAFETY_STOP" not in status:
                    raise RuntimeError(f"mission stopped for {expected}: {status}")
                if recoveries_for_leg >= MAX_SAFE_RECOVERIES_PER_LEG:
                    raise RuntimeError(
                        f"mission remained blocked after {recoveries_for_leg} safe replans")
                if blocked_plan_count is None:
                    blocked_plan_count = self.nonempty_plan_count
                    print("P8_RECOVERY_WAITING_FOR_FRESH_TERRAIN_PLAN "
                          f"goal_id={expected}", flush=True)
                elif (not recovery_pending
                      and self.nonempty_plan_count > blocked_plan_count
                      and self.latest_planner_status.startswith("TERRAIN_PLAN_PASS")):
                    self.send_command("RECOVER")
                    recoveries_for_leg += 1
                    self.safety_recoveries.append({
                        "goal_id": expected, "attempt": recoveries_for_leg,
                        "planner_status": self.latest_planner_status,
                    })
                    blocked_plan_count = self.nonempty_plan_count
                    recovery_pending = True
                    print("P8_RECOVERY_SUBMITTED "
                          f"goal_id={expected} attempt={recoveries_for_leg}",
                          flush=True)
            else:
                blocked_plan_count = None
                recovery_pending = False
        raise TimeoutError(f"goal {expected} did not complete before timeout")

    def write_summary(self, *, result: str, error: str = "") -> None:
        self.summary_path.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            "task_id": "P8-03", "run_id": self.run_id,
            "result": ("QUALIFICATION_PASS" if result == "PASS"
                       and self.qualification_mode else result),
            "completed": result == "PASS" and not self.qualification_mode,
            "map_id": self.registry["map"]["id"],
            "map_frame": self.registry["map"]["frame_id"],
            "map_transform_at_start": self.registry.get("map_transform_at_start"),
            "navigation_frame": self.registry.get(
                "navigation_frame", self.registry["map"]["frame_id"]),
            "itinerary": list(ITINERARY), "legs": self.legs,
            "safety_recoveries": self.safety_recoveries,
            "delivery_id": self.delivery,
            "delivery_events": [e for e in self.events if "DELIVERY_RECORDED" in e],
            "mission_status_tail": self.statuses[-20:],
            "delivery_proofs": int(self.delivery_recorded),
        }
        if error:
            payload["error"] = error
        self.summary_path.write_text(json.dumps(payload, indent=2) + "\n",
                                     encoding="utf-8")

    def run(self) -> int:
        poses = waypoint_poses(self.registry)
        deadline = time.monotonic() + self.input_timeout_s
        if not self.spin_until(lambda: self.map is not None and self.cost_map is not None
                               and self.obstacle_map is not None and self.statuses
                               and self.map_to_odom is not None,
                               deadline):
            raise TimeoutError("no live map, map->odom TF, and mission status received")
        valid, reason = validate_occupancy_grid(self.map, "map")
        if not valid:
            raise RuntimeError(f"live map is invalid: {reason}")
        expected_tf = (self.registry.get("map", {}).get("reference_map_to_odom")
                       or self.registry.get("map_transform_at_start", {}).get("map_to_odom"))
        if expected_tf and not transform_matches(self.map_to_odom, expected_tf):
            raise RuntimeError(
                "live map->odom does not match the qualified spawn alignment")
        print(f"P8_PREFLIGHT_PASS map={self.registry['map']['id']} "
              f"frame={self.registry['map']['frame_id']} "
              f"navigation_frame={self.registry.get('navigation_frame', 'map')}",
              flush=True)

        for leg_index, (site, pose) in enumerate(poses, start=1):
            self.rclpy.spin_once(self.node, timeout_sec=0.05)
            cost = self.goal_cell_cost(pose)
            if cost >= 100:
                raise RuntimeError(
                    f"{site} goal cell is blocked by the live terrain cost map "
                    f"(cost={cost})")
            print(f"P8_GOAL_CELL_PASS site={site} cost={cost}", flush=True)
            expected = self.submit_goal(site, pose)
            self.wait_for_goal(expected, time.monotonic() + self.leg_timeout_s)
            self.legs.append({"leg": leg_index, "site": site,
                              "goal_id": expected, "result": "MISSION_DEMO_PASS"})
            print(f"P8_LEG_PASS leg={leg_index} site={site} goal_id={expected}",
                  flush=True)

        self.delivery = delivery_id(self.run_id, 1)
        self.send_command(f"DELIVER {self.delivery}")
        if not self.spin_until(
                lambda: any(f"DELIVERY_RECORDED delivery_id={self.delivery} " in e
                            for e in self.events),
                time.monotonic() + 10.0):
            raise RuntimeError("final leg passed but durable delivery was not recorded")
        self.delivery_recorded = True
        self.write_summary(result="PASS")
        marker = "P8_MAP_QUALIFICATION_PASS" if self.qualification_mode else "P8_ROUNDTRIP_PASS"
        print(f"{marker} run_id={self.run_id} delivery_id={self.delivery} "
              f"legs={len(self.legs)}", flush=True)
        return 0

    def close(self) -> None:
        self.node.destroy_node()


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--registry", type=Path, default=ROOT / "config/waypoints.json")
    parser.add_argument("--scenario", type=Path,
                        help="mapping qualification mode using world coordinates")
    parser.add_argument("--map-id", default="lunar_p803_roundtrip")
    parser.add_argument("--frame-id", default="map")
    parser.add_argument("--run-id", required=True)
    parser.add_argument("--summary", type=Path, required=True)
    parser.add_argument("--leg-timeout", type=float, default=300.0)
    parser.add_argument("--input-timeout", type=float, default=60.0)
    args = parser.parse_args(argv)
    import rclpy
    rclpy.init()
    if args.scenario:
        # A short bootstrap subscription captures map->odom before goals are
        # transformed. This mode is explicitly for map qualification and does
        # not produce qualified P8-03 acceptance evidence.
        from rclpy.node import Node
        from tf2_msgs.msg import TFMessage
        probe = Node("lunabot_p8_map_transform_probe")
        transform = {"value": None}
        def capture_tf(message):
            for entry in message.transforms:
                if entry.header.frame_id == "map" and entry.child_frame_id == "odom":
                    t, q = entry.transform.translation, entry.transform.rotation
                    transform["value"] = (
                        t.x, t.y,
                        math.atan2(2.0 * (q.w*q.z + q.x*q.y),
                                   1.0 - 2.0 * (q.y*q.y + q.z*q.z)))
        probe.create_subscription(TFMessage, "/tf", capture_tf, 10)
        deadline = time.monotonic() + args.input_timeout
        while rclpy.ok() and transform["value"] is None and time.monotonic() < deadline:
            rclpy.spin_once(probe, timeout_sec=0.1)
        probe.destroy_node()
        if transform["value"] is None:
            error = "no map->odom transform"
            print(f"P8_PREFLIGHT_FAIL {error}", file=sys.stderr)
            write_preflight_failure(args.summary, args.run_id, error)
            rclpy.shutdown()
            return 2
        registry = scenario_registry(args.scenario, transform["value"])
    else:
        try:
            registry = load_registry(args.registry, map_id=args.map_id,
                                     frame_id=args.frame_id, require_bound=True)
        except RegistryError as exc:
            print(f"P8_PREFLIGHT_FAIL {exc}", file=sys.stderr)
            write_preflight_failure(args.summary, args.run_id, str(exc))
            rclpy.shutdown()
            return 2
    runner = RoundtripRunner(registry=registry, run_id=args.run_id,
                             summary_path=args.summary,
                             leg_timeout_s=args.leg_timeout,
                             input_timeout_s=args.input_timeout,
                             qualification_mode=bool(args.scenario))
    try:
        result = runner.run()
    except Exception as exc:  # keep a machine-readable failed trial record
        runner.write_summary(result="FAIL", error=f"{type(exc).__name__}: {exc}")
        print(f"P8_ROUNDTRIP_FAIL {type(exc).__name__}: {exc}", file=sys.stderr)
        result = 1
    finally:
        runner.close()
        if rclpy.ok():
            rclpy.shutdown()
    return result


if __name__ == "__main__":
    raise SystemExit(main())
