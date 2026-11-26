#!/usr/bin/env python3
"""LunaBot V4 Phase I: terrain-aware autonomous path follower.

The follower consumes the Phase H terrain-aware path and sends conservative
commands through the approved Phase B `/cmd_vel_in` boundary. It is the first
phase that integrates the terrain-aware plan with rover motion. The existing
controller/watchdog remains responsible for clamping, acceleration limiting,
and safe `/cmd_vel` output.
"""

from __future__ import annotations

import math
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.clock import Clock, ClockType
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from std_msgs.msg import String
import tf2_ros


def angle_error(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class TerrainPathFollower(Node):
    """Follow the terrain-aware path while preserving the controller boundary."""

    def __init__(self) -> None:
        super().__init__("lunabot_terrain_path_follower")
        self.declare_parameter("path_topic", "/lunabot/terrain/plan")
        self.declare_parameter("goal_topic", "/goal_pose")
        self.declare_parameter("odom_topic", "/lunabot/odom")
        self.declare_parameter("cmd_topic", "/cmd_vel_in")
        self.declare_parameter(
            "status_topic", "/lunabot/autonomy/status")
        self.declare_parameter("mission_status_topic", "")
        self.declare_parameter("mission_status_timeout", 1.5)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "chassis")
        self.declare_parameter("goal_tolerance", 0.35)
        self.declare_parameter("max_linear", 0.20)
        self.declare_parameter("max_angular", 0.60)
        self.declare_parameter("lookahead_cells", 2)
        self.declare_parameter("control_rate", 10.0)
        self.declare_parameter("path_timeout", 2.5)
        self.declare_parameter("obstacle_map_topic", "/lunabot/obstacles/map")
        self.declare_parameter("obstacle_map_timeout", 1.0)
        self.declare_parameter("require_obstacle_map", False)
        self.declare_parameter("obstacle_threshold", 65)
        self.declare_parameter("obstacle_clearance", 0.35)

        get = self.get_parameter
        self.path_topic = str(get("path_topic").value)
        self.goal_topic = str(get("goal_topic").value)
        self.odom_topic = str(get("odom_topic").value)
        self.cmd_topic = str(get("cmd_topic").value)
        self.status_topic = str(get("status_topic").value)
        self.mission_status_topic = str(get("mission_status_topic").value)
        self.mission_gate_enabled = bool(self.mission_status_topic)
        self.mission_motion_allowed = not self.mission_gate_enabled
        self.mission_lifecycle_state = "UNBOUND"
        self.mission_status_timeout = max(
            0.3, float(get("mission_status_timeout").value))
        self.mission_status_received_monotonic: Optional[float] = None
        self.map_frame = str(get("map_frame").value)
        self.base_frame = str(get("base_frame").value)
        self.goal_tolerance = max(0.05, float(get("goal_tolerance").value))
        self.max_linear = max(0.02, float(get("max_linear").value))
        self.max_angular = max(0.1, float(get("max_angular").value))
        self.lookahead_cells = max(0, int(get("lookahead_cells").value))
        self.path_timeout = max(0.2, float(get("path_timeout").value))
        self.obstacle_map_topic = str(get("obstacle_map_topic").value)
        self.obstacle_map_timeout = max(
            0.2, float(get("obstacle_map_timeout").value))
        self.require_obstacle_map = bool(get("require_obstacle_map").value)
        self.obstacle_threshold = int(get("obstacle_threshold").value)
        self.obstacle_clearance = max(
            0.05, float(get("obstacle_clearance").value))

        path_qos = QoSProfile(depth=1)
        path_qos.reliability = ReliabilityPolicy.RELIABLE
        path_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.path_sub = self.create_subscription(
            Path, self.path_topic, self._path_callback, path_qos)
        self.goal_sub = self.create_subscription(
            PoseStamped, self.goal_topic, self._goal_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, self.odom_topic, self._odom_callback,
            qos_profile_sensor_data)
        obstacle_qos = QoSProfile(depth=1)
        obstacle_qos.reliability = ReliabilityPolicy.RELIABLE
        obstacle_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.obstacle_sub = self.create_subscription(
            OccupancyGrid, self.obstacle_map_topic,
            self._obstacle_map_callback, obstacle_qos)
        if self.mission_gate_enabled:
            self.mission_status_sub = self.create_subscription(
                String, self.mission_status_topic,
                self._mission_status_callback, obstacle_qos)
        self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)
        status_qos = QoSProfile(depth=1)
        status_qos.reliability = ReliabilityPolicy.RELIABLE
        status_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.status_pub = self.create_publisher(
            String, self.status_topic, status_qos)
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.path: list[tuple[float, float]] = []
        self.path_received_monotonic: Optional[float] = None
        self.obstacle_map: Optional[OccupancyGrid] = None
        self.obstacle_points_map: list[tuple[float, float]] = []
        self.obstacle_map_received_monotonic: Optional[float] = None
        self.obstacle_map_stamp_ns: Optional[int] = None
        self.goal: Optional[PoseStamped] = None
        self.odom: Optional[Odometry] = None
        self.reached = False
        self.last_waypoint = -1
        self.last_status = ""
        # After a local obstacle invalidates a route, repeated publications of
        # that same planner route must not re-arm motion. Only a changed path
        # can release this latch.
        self.invalidated_path_signature: Optional[tuple] = None
        self.path_invalidation_monotonic: Optional[float] = None
        self.publish_status("INTEGRATION_WAITING_FOR_TERRAIN_PLAN")
        # This callback must keep feeding zero while mission motion is gated.
        # Simulation time can advance much slower than wall time in headless
        # WSL, but the controller's command watchdog is wall-clock based.
        # Drive the command heartbeat from steady time so scheduler slowdown
        # cannot turn a safe idle into a latched actuator timeout.
        self.timer = self.create_timer(
            1.0 / max(1.0, float(get("control_rate").value)), self._tick,
            clock=Clock(clock_type=ClockType.STEADY_TIME))

    def publish_status(self, text: str) -> None:
        if text == self.last_status:
            return
        self.last_status = text
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        self.get_logger().info(text)

    def _path_callback(self, msg: Path) -> None:
        if not msg.poses:
            self.path = []
            self.path_received_monotonic = time.monotonic()
            self.publish_status("INTEGRATION_EMPTY_TERRAIN_PLAN")
            self._publish_stop()
            return
        candidate = [(pose.pose.position.x, pose.pose.position.y)
                     for pose in msg.poses]
        signature = tuple((round(x, 4), round(y, 4)) for x, y in candidate)
        if (self.invalidated_path_signature is not None and
                signature == self.invalidated_path_signature):
            # The planner can republish its latched output before its next
            # cost-map cycle. Keep the follower stopped until a new route is
            # actually available.
            self._publish_stop()
            return
        # Terrain planner publishes at a higher rate than the map changes.
        # Refresh the watchdog for an identical route without resetting
        # waypoint progress or bouncing the follower status back to RECEIVED.
        if (self.path and signature == self._path_signature(self.path) and
                self.invalidated_path_signature is None):
            self.path_received_monotonic = time.monotonic()
            return
        replan_latency_ms = None
        if self.path_invalidation_monotonic is not None:
            replan_latency_ms = max(
                0.0, (time.monotonic() - self.path_invalidation_monotonic) * 1000.0)
            self.invalidated_path_signature = None
            self.path_invalidation_monotonic = None
        self.path = candidate
        self.path_received_monotonic = time.monotonic()
        # The terrain planner republishes its latched/replanned path. Do not
        # turn a completed goal back into an active one just because an
        # unchanged path arrived on the next planning tick.
        was_reached = self.reached
        self.last_waypoint = -1
        if not was_reached:
            if replan_latency_ms is None:
                self.publish_status(
                    f"INTEGRATION_PATH_RECEIVED cells={len(self.path)}")
            else:
                self.publish_status(
                    f"INTEGRATION_REPLAN_ACCEPTED cells={len(self.path)} "
                    f"replan_latency_ms={replan_latency_ms:.1f}")

    @staticmethod
    def _path_signature(path: list[tuple[float, float]]) -> tuple:
        return tuple((round(x, 4), round(y, 4)) for x, y in path)

    def _invalidate_path_for_obstacle(self) -> None:
        """Latch a blocked route invalid before any new command is published."""
        now = time.monotonic()
        detected_at = self.obstacle_map_received_monotonic
        stop_latency_ms = (max(0.0, (now - detected_at) * 1000.0)
                           if detected_at is not None else 0.0)
        signature = self._path_signature(self.path)
        self.invalidated_path_signature = signature
        if self.path_invalidation_monotonic is None:
            self.path_invalidation_monotonic = now
        self.path = []
        self.path_received_monotonic = now
        self._publish_stop()
        self.publish_status(
            "INTEGRATION_PATH_INVALIDATED_OBSTACLE "
            f"detection_to_stop_ms={stop_latency_ms:.1f}")

    @staticmethod
    def _same_goal(first: PoseStamped, second: PoseStamped) -> bool:
        if (first.header.frame_id or "") != (second.header.frame_id or ""):
            return False
        a, b = first.pose, second.pose
        return (
            abs(a.position.x - b.position.x) < 1e-6 and
            abs(a.position.y - b.position.y) < 1e-6 and
            abs(a.position.z - b.position.z) < 1e-6 and
            abs(a.orientation.x - b.orientation.x) < 1e-6 and
            abs(a.orientation.y - b.orientation.y) < 1e-6 and
            abs(a.orientation.z - b.orientation.z) < 1e-6 and
            abs(a.orientation.w - b.orientation.w) < 1e-6
        )

    def _goal_callback(self, msg: PoseStamped) -> None:
        # A* republishes its active goal for late observers. Treat an identical
        # message as state, not as a new command, so it cannot reset progress.
        if self.goal is not None and self._same_goal(msg, self.goal):
            return
        self.goal = msg
        self.reached = False
        self.last_waypoint = -1
        self.publish_status(
            f"INTEGRATION_GOAL_RECEIVED frame={msg.header.frame_id or self.map_frame}")

    def _odom_callback(self, msg: Odometry) -> None:
        self.odom = msg

    def _mission_status_callback(self, msg: String) -> None:
        state = ""
        for token in msg.data.split():
            if token.startswith("lifecycle="):
                state = token.partition("=")[2]
                break
        self.mission_lifecycle_state = state or "UNKNOWN"
        self.mission_status_received_monotonic = time.monotonic()
        # Only ACTIVE authorizes commands. Unknown, paused, cancelled, failed,
        # blocked, completed, and startup states all fail closed.
        self.mission_motion_allowed = state == "ACTIVE"
        if not self.mission_motion_allowed:
            self._publish_stop()

    def _obstacle_map_callback(self, msg: OccupancyGrid) -> None:
        if (msg.info.width <= 0 or msg.info.height <= 0 or
                not math.isfinite(msg.info.resolution) or
                msg.info.resolution <= 0.0 or
                len(msg.data) != msg.info.width * msg.info.height):
            self.obstacle_map = None
            self.obstacle_points_map = []
            self.obstacle_map_received_monotonic = time.monotonic()
            self.obstacle_map_stamp_ns = None
            self.publish_status("INTEGRATION_INVALID_OBSTACLE_MAP")
            self._publish_stop()
            return
        self.obstacle_map = msg
        origin = msg.info.origin
        q = origin.orientation
        yaw = math.atan2(2.0 * q.w * q.z,
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        cosine, sine = math.cos(yaw), math.sin(yaw)
        self.obstacle_points_map = []
        for index, value in enumerate(msg.data):
            if value < self.obstacle_threshold:
                continue
            gx, gy = index % msg.info.width, index // msg.info.width
            local_x = (gx + 0.5) * msg.info.resolution
            local_y = (gy + 0.5) * msg.info.resolution
            self.obstacle_points_map.append((
                origin.position.x + cosine * local_x - sine * local_y,
                origin.position.y + sine * local_x + cosine * local_y))
        self.obstacle_map_received_monotonic = time.monotonic()
        self.obstacle_map_stamp_ns = Time.from_msg(msg.header.stamp).nanoseconds

    def _obstacle_map_age(self) -> Optional[float]:
        if self.obstacle_map is None:
            return None
        clock = self.get_clock()
        if clock.ros_time_is_active and self.obstacle_map_stamp_ns is not None:
            return max(0.0, (clock.now().nanoseconds -
                             self.obstacle_map_stamp_ns) / 1e9)
        if self.obstacle_map_received_monotonic is None:
            return None
        return time.monotonic() - self.obstacle_map_received_monotonic

    @staticmethod
    def _grid_cell(grid: OccupancyGrid, x: float, y: float) -> tuple[int, int]:
        info = grid.info
        q = info.origin.orientation
        yaw = math.atan2(2.0 * q.w * q.z,
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        dx, dy = x - info.origin.position.x, y - info.origin.position.y
        gx = math.cos(yaw) * dx + math.sin(yaw) * dy
        gy = -math.sin(yaw) * dx + math.cos(yaw) * dy
        return (math.floor(gx / info.resolution),
                math.floor(gy / info.resolution))

    def _point_blocked(self, x: float, y: float) -> bool:
        grid = self.obstacle_map
        if grid is None:
            return False
        cx, cy = self._grid_cell(grid, x, y)
        # Occupancy cells represent areas, not points; include the worst-case
        # cell-center quantization in the footprint margin.
        effective_clearance = (
            self.obstacle_clearance + math.sqrt(2.0) * grid.info.resolution)
        radius = int(math.ceil(effective_clearance / grid.info.resolution))
        radius_sq = effective_clearance ** 2
        for oy in range(-radius, radius + 1):
            for ox in range(-radius, radius + 1):
                if (ox * ox + oy * oy) * grid.info.resolution ** 2 > radius_sq:
                    continue
                gx, gy = cx + ox, cy + oy
                if not (0 <= gx < grid.info.width and
                        0 <= gy < grid.info.height):
                    continue
                if (grid.data[gy * grid.info.width + gx] >=
                        self.obstacle_threshold):
                    return True
        return False

    def _nearest_obstacle_distance(self, x: float, y: float) -> float:
        if not self.obstacle_points_map:
            return math.inf
        return min(math.hypot(x - ox, y - oy)
                   for ox, oy in self.obstacle_points_map)

    def _path_intersects_obstacle(self, start: tuple[float, float],
                                  start_index: int) -> bool:
        if self.obstacle_map is None:
            return False
        points = [start, *self.path[start_index:]]
        step = max(0.02, self.obstacle_map.info.resolution * 0.5)
        effective_clearance = (
            self.obstacle_clearance +
            math.sqrt(2.0) * self.obstacle_map.info.resolution)
        previous_clearance = self._nearest_obstacle_distance(*start)
        escaping_existing_clearance = previous_clearance < effective_clearance
        for first, second in zip(points, points[1:]):
            distance = math.hypot(second[0] - first[0], second[1] - first[1])
            count = max(1, int(math.ceil(distance / step)))
            for index in range(count + 1):
                fraction = index / count
                x = first[0] + fraction * (second[0] - first[0])
                y = first[1] + fraction * (second[1] - first[1])
                if escaping_existing_clearance:
                    current_clearance = self._nearest_obstacle_distance(x, y)
                    if current_clearance + 0.02 < previous_clearance:
                        return True
                    if current_clearance >= effective_clearance:
                        escaping_existing_clearance = False
                    previous_clearance = current_clearance
                elif self._point_blocked(x, y):
                    return True
        return False

    @staticmethod
    def _yaw(q) -> float:
        return math.atan2(2.0 * q.w * q.z,
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def _pose(self):
        if self.odom is not None:
            pose = self.odom.pose.pose
            x, y, yaw = pose.position.x, pose.position.y, self._yaw(pose.orientation)
            source = self.odom.header.frame_id or "odom"
        else:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, Time())
            t = transform.transform.translation
            return t.x, t.y, self._yaw(transform.transform.rotation)
        if source == self.map_frame:
            return x, y, yaw
        transform = self.tf_buffer.lookup_transform(self.map_frame, source, Time())
        t = transform.transform.translation
        tf_yaw = self._yaw(transform.transform.rotation)
        c, s = math.cos(tf_yaw), math.sin(tf_yaw)
        return t.x + c * x - s * y, t.y + s * x + c * y, angle_error(yaw + tf_yaw)

    def _goal_map(self):
        if self.goal is None:
            return None
        pose = self.goal.pose
        yaw = self._yaw(pose.orientation)
        source = self.goal.header.frame_id or self.map_frame
        if source == self.map_frame:
            return pose.position.x, pose.position.y, yaw
        transform = self.tf_buffer.lookup_transform(self.map_frame, source, Time())
        t = transform.transform.translation
        tf_yaw = self._yaw(transform.transform.rotation)
        c, s = math.cos(tf_yaw), math.sin(tf_yaw)
        return t.x + c * pose.position.x - s * pose.position.y, \
            t.y + s * pose.position.x + c * pose.position.y, \
            angle_error(yaw + tf_yaw)

    def _publish_stop(self) -> None:
        self.cmd_pub.publish(Twist())

    def _tick(self) -> None:
        # Preserve the retained GOAL_REACHED status after the mission gate
        # transitions to COMPLETED. Motion stays stopped, and late acceptance
        # subscribers can still observe the completion event.
        if getattr(self, "reached", False):
            self._publish_stop()
            return
        mission_gate = getattr(self, "mission_gate_enabled", False)
        received = getattr(self, "mission_status_received_monotonic", None)
        status_stale = (
            mission_gate and
            (received is None or time.monotonic() - received >
             getattr(self, "mission_status_timeout", 1.5)))
        if (mission_gate and (status_stale or
                              not getattr(self, "mission_motion_allowed", False))):
            reason = "STALE" if status_stale else getattr(
                self, "mission_lifecycle_state", "UNKNOWN")
            self.publish_status(
                "INTEGRATION_MISSION_HOLD_STOP "
                f"lifecycle={reason}")
            self._publish_stop()
            return
        if not self.path:
            self.publish_status("INTEGRATION_NO_TERRAIN_PLAN_STOP")
            self._publish_stop()
            return
        if (self.path_received_monotonic is None or
                time.monotonic() - self.path_received_monotonic >
                self.path_timeout):
            self.publish_status("INTEGRATION_STALE_TERRAIN_PLAN_STOP")
            self._publish_stop()
            return
        if (self.require_obstacle_map and
                self.obstacle_map_received_monotonic is None):
            self.publish_status("INTEGRATION_WAITING_FOR_OBSTACLE_MAP_STOP")
            self._publish_stop()
            return
        if self.obstacle_map_received_monotonic is not None:
            obstacle_map_age = self._obstacle_map_age()
            if (obstacle_map_age is None or
                    obstacle_map_age > self.obstacle_map_timeout):
                self.publish_status("INTEGRATION_STALE_OBSTACLE_MAP_STOP")
                self._publish_stop()
                return
        try:
            current_x, current_y, current_yaw = self._pose()
            goal = self._goal_map()
        except (RuntimeError, tf2_ros.TransformException) as exc:
            self.publish_status(f"INTEGRATION_WAITING_FOR_TF {exc}")
            self._publish_stop()
            return
        if goal is None:
            self.publish_status("INTEGRATION_WAITING_FOR_GOAL")
            self._publish_stop()
            return
        goal_distance = math.hypot(goal[0] - current_x, goal[1] - current_y)
        path_end_distance = math.hypot(
            self.path[-1][0] - current_x, self.path[-1][1] - current_y)
        if goal_distance <= self.goal_tolerance:
            self._publish_stop()
            if not self.reached:
                self.reached = True
                self.publish_status(
                    f"INTEGRATION_GOAL_REACHED distance={goal_distance:.2f}")
            return
        if path_end_distance <= self.goal_tolerance:
            # The planner can terminate at a nearby clear cell when the
            # requested goal itself is occupied. Stop at that safe approach
            # point instead of driving toward the blocked cell.
            self._publish_stop()
            if not self.reached:
                self.reached = True
                self.publish_status(
                    "INTEGRATION_GOAL_REACHED_SAFE_APPROACH "
                    f"distance={goal_distance:.2f} "
                    f"offset={math.hypot(self.path[-1][0] - goal[0], self.path[-1][1] - goal[1]):.2f}")
            return
        nearest = min(range(len(self.path)),
                      key=lambda i: math.hypot(self.path[i][0] - current_x,
                                               self.path[i][1] - current_y))
        target_index = min(len(self.path) - 1, nearest + self.lookahead_cells)
        if self._path_intersects_obstacle((current_x, current_y), nearest):
            self._invalidate_path_for_obstacle()
            return
        target_x, target_y = self.path[target_index]
        heading = math.atan2(target_y - current_y, target_x - current_x)
        error = angle_error(heading - current_yaw)
        cmd = Twist()
        cmd.angular.z = max(-self.max_angular, min(self.max_angular, 2.0 * error))
        if abs(error) <= 0.9:
            cmd.linear.x = min(self.max_linear, 0.45 * goal_distance)
            cmd.linear.x *= max(0.2, math.cos(error))
        self.cmd_pub.publish(cmd)
        if target_index != self.last_waypoint:
            self.last_waypoint = target_index
            self.publish_status(
                f"INTEGRATION_FOLLOWING waypoint={target_index}/{len(self.path)} "
                f"pose=({current_x:.3f},{current_y:.3f}) "
                f"goal_distance={goal_distance:.3f} "
                f"target=({target_x:.3f},{target_y:.3f}) "
                f"heading_error={error:.3f}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TerrainPathFollower()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        try:
            node._publish_stop()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
