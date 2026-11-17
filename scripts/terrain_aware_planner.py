#!/usr/bin/env python3
"""LunaBot V4 Phase H: terrain-aware weighted A* path planning.

This node consumes the Phase G terrain cost map and the existing navigation goal
and publishes a cost-aware path. It deliberately does not publish velocity:
Phase I will integrate terrain-aware planning into autonomous motion after this
path-planning contract is independently validated.
"""

from __future__ import annotations

import math
from typing import Optional

import rclpy
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
        self.last_plan_time = self.get_clock().now()
        self.last_status = ""
        self.publish_status("TERRAIN_PLANNER_WAITING_FOR_COST_MAP")
        self.timer = self.create_timer(0.1, self._tick)

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
        if start_cost is None or goal_cost is None:
            start_raw = (self.planning_data[start_cell[1] * int(self.cost_map.info.width)
                                           + start_cell[0]]
                         if 0 <= start_cell[0] < int(self.cost_map.info.width)
                         and 0 <= start_cell[1] < int(self.cost_map.info.height)
                         else "OUT_OF_BOUNDS")
            goal_raw = (self.planning_data[goal_cell[1] * int(self.cost_map.info.width)
                                          + goal_cell[0]]
                        if 0 <= goal_cell[0] < int(self.cost_map.info.width)
                        and 0 <= goal_cell[1] < int(self.cost_map.info.height)
                        else "OUT_OF_BOUNDS")
            self._invalidate_plan("OCCUPANCY_CONSTRAINT", reset_revision=False)
            self.publish_status(
                f"TERRAIN_PLANNER_OCCUPANCY_CONSTRAINT start={start_cell} goal={goal_cell} "
                f"start_cost={start_cost if start_cost is not None else start_raw} "
                f"goal_cost={goal_cost if goal_cost is not None else goal_raw} "
                f"revision={self.cost_revision[:12]}")
            return
        cells, total_cost = self._weighted_astar(start_cell, goal_cell)
        if not cells:
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
        self.last_plan_time = self.get_clock().now()
        self.planned_revision = self.cost_revision
        self.plan_requested = False
        self.publish_status(
            f"TERRAIN_PLAN_PASS cells={len(cells)} weighted_cost={total_cost:.2f} "
            f"frame={self.map_frame} cost_weight={self.cost_weight:.2f} "
            f"map_revision={self.cost_revision}")

    def _tick(self) -> None:
        revision_changed = self.cost_revision != self.planned_revision
        if (not self.plan_requested and not revision_changed
                and (self.get_clock().now() - self.last_plan_time).nanoseconds
                < int(self.replan_period * 1e9)):
            return
        self.last_plan_time = self.get_clock().now()
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
