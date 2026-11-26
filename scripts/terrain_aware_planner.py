#!/usr/bin/env python3
"""LunaBot V4 Phase H: terrain-aware weighted A* path planning.

This node consumes the Phase G terrain cost map and the existing navigation goal
and publishes a cost-aware path. It deliberately does not publish velocity:
Phase I will integrate terrain-aware planning into autonomous motion after this
path-planning contract is independently validated.
"""

from __future__ import annotations

import math
import time
from typing import Optional

import rclpy
from rclpy.clock import Clock, ClockType
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from std_msgs.msg import String
import tf2_ros
from terrain_plan import (compute_map_revision, grid_to_world,
                          merge_obstacle_grid, path_is_traversable,
                          weighted_astar, world_to_grid)


class TerrainAwarePlanner(Node):
    """Plan through traversable cells while minimizing terrain cost."""

    def __init__(self) -> None:
        super().__init__("lunabot_terrain_aware_planner")
        self.declare_parameter("cost_map_topic", "/lunabot/terrain/cost_map")
        self.declare_parameter("obstacle_map_topic", "/lunabot/obstacles/map")
        self.declare_parameter("obstacle_threshold", 65)
        self.declare_parameter("obstacle_inflation", 0.35)
        self.declare_parameter("goal_topic", "/goal_pose")
        self.declare_parameter("odom_topic", "/lunabot/odom")
        self.declare_parameter("plan_topic", "/lunabot/terrain/plan")
        self.declare_parameter(
            "status_topic", "/lunabot/terrain/planner/status")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "chassis")
        self.declare_parameter("blocked_cost", 100)
        self.declare_parameter("cost_weight", 2.0)
        self.declare_parameter("unknown_cost", 80)
        self.declare_parameter("unknown_is_blocked", False)
        self.declare_parameter("goal_snap_radius", 0.80)
        self.declare_parameter("start_escape_radius", 1.50)
        self.declare_parameter("replan_period", 1.0)

        get = self.get_parameter
        self.cost_map_topic = str(get("cost_map_topic").value)
        self.obstacle_map_topic = str(get("obstacle_map_topic").value)
        self.obstacle_threshold = max(1, min(100, int(get("obstacle_threshold").value)))
        self.obstacle_inflation = max(0.0, float(get("obstacle_inflation").value))
        if not math.isfinite(self.obstacle_inflation):
            raise ValueError("obstacle_inflation must be finite")
        self.goal_topic = str(get("goal_topic").value)
        self.odom_topic = str(get("odom_topic").value)
        self.plan_topic = str(get("plan_topic").value)
        self.status_topic = str(get("status_topic").value)
        self.map_frame = str(get("map_frame").value)
        self.base_frame = str(get("base_frame").value)
        self.blocked_cost = max(1, min(100, int(get("blocked_cost").value)))
        self.cost_weight = float(get("cost_weight").value)
        if not math.isfinite(self.cost_weight) or self.cost_weight < 0:
            raise ValueError("cost_weight must be finite and nonnegative")
        self.unknown_cost = max(0, min(self.blocked_cost - 1,
                                       int(get("unknown_cost").value)))
        self.unknown_is_blocked = bool(get("unknown_is_blocked").value)
        self.goal_snap_radius = max(0.0, float(get("goal_snap_radius").value))
        if not math.isfinite(self.goal_snap_radius):
            raise ValueError("goal_snap_radius must be finite")
        self.start_escape_radius = max(
            0.0, float(get("start_escape_radius").value))
        if not math.isfinite(self.start_escape_radius):
            raise ValueError("start_escape_radius must be finite")
        self.replan_period = max(0.2, float(get("replan_period").value))

        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.cost_sub = self.create_subscription(
            OccupancyGrid, self.cost_map_topic, self._cost_callback, map_qos)
        self.obstacle_sub = self.create_subscription(
            OccupancyGrid, self.obstacle_map_topic,
            self._obstacle_callback, map_qos)
        self.goal_sub = self.create_subscription(
            PoseStamped, self.goal_topic, self._goal_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, self.odom_topic, self._odom_callback,
            qos_profile_sensor_data)
        self.plan_pub = self.create_publisher(Path, self.plan_topic, map_qos)
        self.status_pub = self.create_publisher(String, self.status_topic, map_qos)
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.cost_map: Optional[OccupancyGrid] = None
        self.obstacle_map: Optional[OccupancyGrid] = None
        self.planning_data: list[int] = []
        self.cost_revision = ""
        self.planned_revision = ""
        self.last_plan_world: list[tuple[float, float]] = []
        self.plan_requested = True
        self.goal: Optional[PoseStamped] = None
        self.odom: Optional[Odometry] = None
        # Simulation time starts at zero; subtracting a warm-up duration can
        # produce a negative rclpy Time. The first timer tick will plan after
        # the configured replan period instead.
        self.last_plan_time = time.monotonic()
        self.last_status = ""
        self.publish_status("TERRAIN_PLANNER_WAITING_FOR_COST_MAP")
        self.timer = self.create_timer(0.1, self._tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def publish_status(self, text: str) -> None:
        if text == self.last_status:
            return
        self.last_status = text
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        self.get_logger().info(text)

    def _cost_callback(self, msg: OccupancyGrid) -> None:
        if (msg.info.width <= 0 or msg.info.height <= 0
                or len(msg.data) != int(msg.info.width) * int(msg.info.height)):
            self._reject_cost_map("INVALID_COST_MAP")
            return
        if msg.header.frame_id != self.map_frame:
            self._reject_cost_map("INVALID_COST_MAP_FRAME")
            return
        self.cost_map = msg
        self._refresh_planning_grid()

    def _obstacle_callback(self, msg: OccupancyGrid) -> None:
        if (msg.header.frame_id != self.map_frame or msg.info.width <= 0
                or msg.info.height <= 0
                or len(msg.data) != int(msg.info.width) * int(msg.info.height)):
            self.publish_status("TERRAIN_PLANNER_INVALID_OBSTACLE_MAP")
            return
        self.obstacle_map = msg
        if self.cost_map is not None:
            self._refresh_planning_grid()

    def _refresh_planning_grid(self) -> None:
        if self.cost_map is None:
            return
        msg = self.cost_map
        try:
            if self.obstacle_map is None:
                planning_data = [int(value) for value in msg.data]
            else:
                obstacle = self.obstacle_map
                base_origin = msg.info.origin
                obstacle_origin = obstacle.info.origin
                planning_data = merge_obstacle_grid(
                    msg.data, int(msg.info.width), int(msg.info.height),
                    origin_x=base_origin.position.x,
                    origin_y=base_origin.position.y,
                    origin_yaw=self._yaw(base_origin.orientation),
                    resolution=float(msg.info.resolution),
                    obstacles=obstacle.data,
                    obstacle_width=int(obstacle.info.width),
                    obstacle_height=int(obstacle.info.height),
                    obstacle_origin_x=obstacle_origin.position.x,
                    obstacle_origin_y=obstacle_origin.position.y,
                    obstacle_origin_yaw=self._yaw(obstacle_origin.orientation),
                    obstacle_resolution=float(obstacle.info.resolution),
                    obstacle_threshold=self.obstacle_threshold,
                    inflation_radius=self.obstacle_inflation)
            origin = msg.info.origin
            q = origin.orientation
            revision = compute_map_revision(
                frame_id=msg.header.frame_id, width=int(msg.info.width),
                height=int(msg.info.height), resolution=float(msg.info.resolution),
                origin=(origin.position.x, origin.position.y, origin.position.z,
                        q.x, q.y, q.z, q.w), data=planning_data)
        except (ValueError, TypeError, OverflowError):
            self._reject_cost_map("INVALID_COST_MAP")
            return
        revision_changed = revision != self.cost_revision
        self.planning_data = planning_data
        if revision_changed:
            path_clear = bool(self.last_plan_world) and self._existing_path_clear()
            self.cost_revision = revision
            self.plan_requested = True
            if self.planned_revision and not path_clear:
                self._invalidate_plan("MAP_REVISION_BLOCKED_PATH")
            else:
                self.publish_status(
                    "TERRAIN_PLAN_REPLANNING reason=MAP_OR_OBSTACLE_CHANGED")

    def _existing_path_clear(self) -> bool:
        if self.cost_map is None or not self.last_plan_world:
            return False
        new_cells = [self._world_to_grid(x, y)
                     for x, y in self.last_plan_world]
        return path_is_traversable(
            self.planning_data, int(self.cost_map.info.width),
            int(self.cost_map.info.height), new_cells,
            blocked_cost=self.blocked_cost,
            unknown_is_blocked=self.unknown_is_blocked)

    def _reject_cost_map(self, reason: str) -> None:
        self.cost_map = None
        self.cost_revision = ""
        self.planning_data = []
        self.plan_requested = False
        self.last_plan_world = []
        self._invalidate_plan(reason)
        self.publish_status(f"TERRAIN_PLANNER_{reason}")

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
        # A* republishes its active goal for late observers. Do not reset the
        # terrain planner's goal state or status for an identical message.
        if self.goal is not None and self._same_goal(msg, self.goal):
            return
        self.goal = msg
        self.plan_requested = True
        self.last_plan_world = []
        self._invalidate_plan("GOAL_CHANGED")
        self.publish_status(
            f"TERRAIN_GOAL_RECEIVED frame={msg.header.frame_id or self.map_frame}")

    def _odom_callback(self, msg: Odometry) -> None:
        self.odom = msg

    @staticmethod
    def _yaw(q) -> float:
        norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
        if not math.isfinite(norm) or norm < 1e-12:
            raise ValueError("map pose quaternion is invalid")
        x, y, z, w = q.x/norm, q.y/norm, q.z/norm, q.w/norm
        return math.atan2(2.0 * (w * z + x * y),
                          1.0 - 2.0 * (y * y + z * z))

    def _transform_xy(self, x: float, y: float, yaw: float,
                      source: str, target: str):
        source = source or target
        if source == target:
            return x, y, yaw
        transform = self.tf_buffer.lookup_transform(target, source, Time())
        t = transform.transform.translation
        tf_yaw = self._yaw(transform.transform.rotation)
        c, s = math.cos(tf_yaw), math.sin(tf_yaw)
        return t.x + c * x - s * y, t.y + s * x + c * y, yaw + tf_yaw

    def _current_pose(self):
        if self.odom is not None:
            pose = self.odom.pose.pose
            return self._transform_xy(
                pose.position.x, pose.position.y, self._yaw(pose.orientation),
                self.odom.header.frame_id or "odom", self.map_frame)
        transform = self.tf_buffer.lookup_transform(
            self.map_frame, self.base_frame, Time())
        t = transform.transform.translation
        return t.x, t.y, self._yaw(transform.transform.rotation)

    def _world_to_grid(self, x: float, y: float):
        info = self.cost_map.info
        origin = info.origin
        oyaw = self._yaw(origin.orientation)
        return world_to_grid(x, y, origin_x=origin.position.x,
                             origin_y=origin.position.y, origin_yaw=oyaw,
                             resolution=info.resolution)

    def _grid_to_world(self, cell):
        info = self.cost_map.info
        origin = info.origin
        oyaw = self._yaw(origin.orientation)
        return grid_to_world(cell, origin_x=origin.position.x,
                             origin_y=origin.position.y, origin_yaw=oyaw,
                             resolution=info.resolution)

    def _cell_cost(self, cell) -> Optional[int]:
        info = self.cost_map.info
        x, y = cell
        if x < 0 or y < 0 or x >= info.width or y >= info.height:
            return None
        index = y * info.width + x
        if index >= len(self.cost_map.data):
            return None
        value = int(self.planning_data[index])
        if value < 0:
            if self.unknown_is_blocked:
                return None
            value = self.unknown_cost
        if value >= self.blocked_cost:
            return None
        return max(0, min(100, value))

    def _weighted_astar(self, start, goal):
        info = self.cost_map.info
        return weighted_astar(
            self.planning_data, int(info.width), int(info.height), start, goal,
            cost_weight=self.cost_weight, blocked_cost=self.blocked_cost,
            unknown_cost=self.unknown_cost, unknown_is_blocked=self.unknown_is_blocked)

    def _safe_goal_path(self, start_cell, blocked_goal_cell):
        """Find the nearest reachable, traversable endpoint near a blocked goal."""
        if self.goal_snap_radius <= 0.0:
            return None
        resolution = float(self.cost_map.info.resolution)
        max_cells = int(math.ceil(self.goal_snap_radius / resolution))
        goal_x, goal_y = blocked_goal_cell
        candidates = []
        for dy in range(-max_cells, max_cells + 1):
            for dx in range(-max_cells, max_cells + 1):
                distance = math.hypot(dx, dy) * resolution
                if distance > self.goal_snap_radius:
                    continue
                cell = (goal_x + dx, goal_y + dy)
                if cell == start_cell or self._cell_cost(cell) is None:
                    continue
                candidates.append((distance, cell))
        for distance, cell in sorted(candidates, key=lambda item: item[0]):
            cells, total_cost = self._weighted_astar(start_cell, cell)
            if cells:
                return cell, cells, total_cost, distance
        return None

    def _safe_start_cells(self, blocked_start_cell):
        """Find nearby free cells when only the inflated rover cell is blocked."""
        if self.start_escape_radius <= 0.0:
            return []
        resolution = float(self.cost_map.info.resolution)
        max_cells = int(math.ceil(self.start_escape_radius / resolution))
        start_x, start_y = blocked_start_cell
        candidates = []
        for dy in range(-max_cells, max_cells + 1):
            for dx in range(-max_cells, max_cells + 1):
                distance = math.hypot(dx, dy) * resolution
                if distance <= self.start_escape_radius:
                    cell = (start_x + dx, start_y + dy)
                    if self._cell_cost(cell) is not None:
                        candidates.append((distance, cell))
        return [cell for _, cell in sorted(candidates, key=lambda item: item[0])]

    def _segment_clear_of_obstacles(self, start_xy, end_xy) -> bool:
        """Match the follower's raw-LiDAR clearance check for start escapes."""
        grid = self.obstacle_map
        if grid is None:
            return True
        info = grid.info
        origin = info.origin
        effective_clearance = (
            self.obstacle_inflation + math.sqrt(2.0) * info.resolution)
        q = info.origin.orientation
        obstacle_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                  1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        occupied = []
        for index, value in enumerate(grid.data):
            if value < self.obstacle_threshold:
                continue
            gx, gy = index % info.width, index // info.width
            lx, ly = (gx + 0.5) * info.resolution, (gy + 0.5) * info.resolution
            occupied.append((
                origin.position.x + math.cos(obstacle_yaw) * lx - math.sin(obstacle_yaw) * ly,
                origin.position.y + math.sin(obstacle_yaw) * lx + math.cos(obstacle_yaw) * ly))
        if not occupied:
            return True

        def clearance(point) -> float:
            return min(math.hypot(point[0] - ox, point[1] - oy)
                       for ox, oy in occupied)

        step = max(0.02, info.resolution * 0.5)
        distance = math.hypot(end_xy[0] - start_xy[0], end_xy[1] - start_xy[1])
        samples = max(1, int(math.ceil(distance / step)))
        initial_clearance = clearance(start_xy)
        escaping_existing_clearance = initial_clearance < effective_clearance
        previous_clearance = initial_clearance
        for sample in range(samples + 1):
            fraction = sample / samples
            x = start_xy[0] + fraction * (end_xy[0] - start_xy[0])
            y = start_xy[1] + fraction * (end_xy[1] - start_xy[1])
            current_clearance = clearance((x, y))
            if escaping_existing_clearance:
                if current_clearance + 0.02 < previous_clearance:
                    return False
                if current_clearance >= effective_clearance:
                    escaping_existing_clearance = False
            elif current_clearance < effective_clearance:
                return False
            previous_clearance = current_clearance
        return True

    def _invalidate_plan(self, reason: str, *, reset_revision: bool = True) -> None:
        self.last_plan_world = []
        path = Path()
        path.header.frame_id = self.map_frame
        path.header.stamp = self.get_clock().now().to_msg()
        self.plan_pub.publish(path)
        if reset_revision:
            self.planned_revision = ""
        self.publish_status(f"TERRAIN_PLAN_INVALIDATED reason={reason}")

    def _plan(self) -> None:
        if self.cost_map is None or self.goal is None:
            return
        try:
            current_x, current_y, current_yaw = self._current_pose()
            goal_pose = self.goal.pose
            goal_yaw = self._yaw(goal_pose.orientation)
            goal_x, goal_y, goal_yaw = self._transform_xy(
                goal_pose.position.x, goal_pose.position.y, goal_yaw,
                self.goal.header.frame_id or self.map_frame, self.map_frame)
        except (RuntimeError, ValueError, tf2_ros.TransformException) as exc:
            self.publish_status(f"TERRAIN_PLANNER_WAITING_FOR_TF {exc}")
            return
        try:
            start_cell = self._world_to_grid(current_x, current_y)
            goal_cell = self._world_to_grid(goal_x, goal_y)
        except (TypeError, ValueError, ZeroDivisionError) as exc:
            self._invalidate_plan("INVALID_POSE_OR_GEOMETRY", reset_revision=False)
            self.publish_status(f"TERRAIN_PLANNER_INVALID_POSE_OR_GEOMETRY {exc}")
            return
        start_cost = self._cell_cost(start_cell)
        goal_cost = self._cell_cost(goal_cell)
        if start_cost is not None:
            start_candidates = [start_cell]
        else:
            start_candidates = self._safe_start_cells(start_cell)
        if not start_candidates:
            start_raw = (self.planning_data[start_cell[1] * int(self.cost_map.info.width)
                                           + start_cell[0]]
                         if 0 <= start_cell[0] < int(self.cost_map.info.width)
                         and 0 <= start_cell[1] < int(self.cost_map.info.height)
                         else "OUT_OF_BOUNDS")
            self._invalidate_plan("OCCUPANCY_CONSTRAINT", reset_revision=False)
            self.publish_status(
                f"TERRAIN_PLANNER_START_OCCUPIED_NO_CLEAR_ESCAPE "
                f"start={start_cell} start_cost={start_raw} goal={goal_cell} "
                f"revision={self.cost_revision[:12]}")
            return
        safe_approach_offset = 0.0
        start_escape_offset = 0.0
        clear_start_escape_found = start_cost is not None
        cells, total_cost = [], math.inf
        for candidate_start in start_candidates:
            start_escape_offset = math.hypot(
                candidate_start[0] - start_cell[0],
                candidate_start[1] - start_cell[1]) * float(self.cost_map.info.resolution)
            if (start_cost is None and not self._segment_clear_of_obstacles(
                    (current_x, current_y), self._grid_to_world(candidate_start))):
                continue
            clear_start_escape_found = True
            if goal_cost is None:
                safe_approach = self._safe_goal_path(candidate_start, goal_cell)
                if safe_approach is None:
                    continue
                candidate_goal, candidate_cells, candidate_cost, offset = safe_approach
            else:
                candidate_goal = goal_cell
                candidate_cells, candidate_cost = self._weighted_astar(
                    candidate_start, candidate_goal)
                offset = 0.0
            if candidate_cells:
                goal_cell = candidate_goal
                cells, total_cost = candidate_cells, candidate_cost
                safe_approach_offset = offset
                break
        if not cells:
            if start_cost is None and not clear_start_escape_found:
                self._invalidate_plan("START_OCCUPIED_NO_CLEAR_ESCAPE",
                                      reset_revision=False)
                self.publish_status(
                    f"TERRAIN_PLANNER_START_OCCUPIED_NO_CLEAR_ESCAPE "
                    f"start={start_cell} goal={goal_cell} "
                    f"start_escape_radius={self.start_escape_radius:.2f} "
                    f"revision={self.cost_revision[:12]}")
            elif goal_cost is None:
                self._invalidate_plan("GOAL_OCCUPIED_NO_SAFE_APPROACH",
                                      reset_revision=False)
                self.publish_status(
                    f"TERRAIN_PLANNER_GOAL_OCCUPIED_NO_SAFE_APPROACH "
                    f"start={start_cell} goal={goal_cell} "
                    f"goal_snap_radius={self.goal_snap_radius:.2f} "
                    f"revision={self.cost_revision[:12]}")
            else:
                self._invalidate_plan("NO_PATH", reset_revision=False)
                self.publish_status("TERRAIN_PLANNER_NO_PATH")
            return
        path = Path()
        path.header.frame_id = self.map_frame
        path.header.stamp = self.get_clock().now().to_msg()
        for index, cell in enumerate(cells):
            x, y = self._grid_to_world(cell)
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            if index + 1 < len(cells):
                nx, ny = self._grid_to_world(cells[index + 1])
                heading = math.atan2(ny - y, nx - x)
            else:
                heading = goal_yaw
            pose.pose.orientation.z = math.sin(heading / 2.0)
            pose.pose.orientation.w = math.cos(heading / 2.0)
            path.poses.append(pose)
        self.plan_pub.publish(path)
        self.last_plan_world = [self._grid_to_world(cell) for cell in cells]
        self.planned_revision = self.cost_revision
        self.plan_requested = False
        self.publish_status(
            f"TERRAIN_PLAN_PASS cells={len(cells)} weighted_cost={total_cost:.2f} "
            f"start_escape_offset={start_escape_offset:.2f} "
            f"safe_approach_offset={safe_approach_offset:.2f} "
            f"frame={self.map_frame} cost_weight={self.cost_weight:.2f} "
            f"map_revision={self.cost_revision}")

    def _tick(self) -> None:
        now = time.monotonic()
        revision_changed = self.cost_revision != self.planned_revision
        if (not self.plan_requested and not revision_changed
                and now - self.last_plan_time < self.replan_period):
            return
        self.last_plan_time = now
        self.planned_revision = self.cost_revision
        self.plan_requested = False
        self._plan()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TerrainAwarePlanner()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
