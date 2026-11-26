#!/usr/bin/env python3
"""Publish a short goal only when the live terrain and obstacle maps show a route."""
from __future__ import annotations

import math
import sys
import time
from pathlib import Path

import rclpy
import tf2_ros
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy)
from rclpy.time import Time

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from terrain_plan import merge_obstacle_grid, select_safe_goal_along_heading


class LiveGoalSelector(Node):
    def __init__(self) -> None:
        super().__init__("phase_l_safe_goal_selector")
        self.cost_map: OccupancyGrid | None = None
        self.obstacle_map: OccupancyGrid | None = None
        self.map_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            OccupancyGrid, "/lunabot/terrain/cost_map",
            lambda msg: setattr(self, "cost_map", msg), self.map_qos)
        self.create_subscription(
            OccupancyGrid, "/lunabot/obstacles/map",
            lambda msg: setattr(self, "obstacle_map", msg), self.map_qos)
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.goal_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)

    @staticmethod
    def _yaw(q) -> float:
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def choose_and_publish(self) -> bool:
        deadline = time.monotonic() + 12.0
        transform = None
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.cost_map is None or self.obstacle_map is None:
                continue
            try:
                transform = self.tf_buffer.lookup_transform(
                    "map", "chassis", Time())
                break
            except tf2_ros.TransformException:
                continue
        if transform is None or self.cost_map is None or self.obstacle_map is None:
            self.get_logger().error(
                "SAFE_GOAL_UNAVAILABLE: waiting for cost map, obstacle map, or map->chassis TF")
            return False

        cost, obstacles = self.cost_map, self.obstacle_map
        cost_origin, obstacle_origin = cost.info.origin, obstacles.info.origin
        merged = merge_obstacle_grid(
            cost.data, int(cost.info.width), int(cost.info.height),
            origin_x=cost_origin.position.x, origin_y=cost_origin.position.y,
            origin_yaw=self._yaw(cost_origin.orientation),
            resolution=float(cost.info.resolution),
            obstacles=obstacles.data,
            obstacle_width=int(obstacles.info.width),
            obstacle_height=int(obstacles.info.height),
            obstacle_origin_x=obstacle_origin.position.x,
            obstacle_origin_y=obstacle_origin.position.y,
            obstacle_origin_yaw=self._yaw(obstacle_origin.orientation),
            obstacle_resolution=float(obstacles.info.resolution),
            obstacle_threshold=65, inflation_radius=0.35)

        t = transform.transform.translation
        q = transform.transform.rotation
        heading = self._yaw(q)
        selected = select_safe_goal_along_heading(
            merged, int(cost.info.width), int(cost.info.height),
            origin_x=cost_origin.position.x,
            origin_y=cost_origin.position.y,
            origin_yaw=self._yaw(cost_origin.orientation),
            resolution=float(cost.info.resolution),
            start_x=t.x, start_y=t.y, heading=heading,
            max_distance=0.75, min_distance=0.30,
            distance_step=0.10,
            lateral_offsets=(0.0, 0.25, -0.25, 0.50, -0.50),
            cost_weight=2.0, blocked_cost=100, unknown_cost=80,
            unknown_is_blocked=False)
        if selected is None:
            self.get_logger().error(
                "SAFE_GOAL_UNAVAILABLE: no reachable point within 0.30-0.75 m")
            return False

        x, y, distance, lateral = selected
        goal_yaw = math.atan2(y - t.y, x - t.x)
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.orientation.z = math.sin(goal_yaw / 2.0)
        msg.pose.orientation.w = math.cos(goal_yaw / 2.0)
        self.goal_pub.publish(msg)
        self.get_logger().info(
            f"SAFE_GOAL_PUBLISHED x={x:.3f} y={y:.3f} "
            f"distance={distance:.2f} lateral={lateral:.2f}")
        return True


def main() -> int:
    rclpy.init()
    node = LiveGoalSelector()
    try:
        return 0 if node.choose_and_publish() else 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
