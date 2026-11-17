#!/usr/bin/env python3
"""Turn in place so startup SLAM can settle a usable map->odom transform."""
from __future__ import annotations

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from tf2_msgs.msg import TFMessage
from std_msgs.msg import String


class MapPrime(Node):
    def __init__(self) -> None:
        super().__init__("p8_map_prime")
        self.started = time.monotonic()
        self.initial_yaw: float | None = None
        self.yaw: float | None = None
        self.has_map_tf = False
        self.map_transform: tuple[float, float, float] | None = None
        self.settle_started: float | None = None
        self.settle_transform: tuple[float, float, float] | None = None
        self.result: bool | None = None
        self.controller_armed = False
        self.publisher = self.create_publisher(Twist, "/cmd_vel_in", 10)
        tf_qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                            durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(Odometry, "/lunabot/odom", self.on_odom,
                                 qos_profile_sensor_data)
        self.create_subscription(TFMessage, "/tf", self.on_tf, tf_qos)
        self.create_subscription(String, "/lunabot/control/status",
                                 self.on_control_status, 10)
        self.last_progress_log = self.started

    def on_odom(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        self.yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                              1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        if self.initial_yaw is None:
            self.initial_yaw = self.yaw

    def on_tf(self, msg: TFMessage) -> None:
            for item in msg.transforms:
                if item.header.frame_id != "map" or item.child_frame_id != "odom":
                    continue
                t, q = item.transform.translation, item.transform.rotation
                yaw = math.atan2(2.0 * (q.w*q.z + q.x*q.y),
                                 1.0 - 2.0 * (q.y*q.y + q.z*q.z))
                if all(math.isfinite(v) for v in (t.x, t.y, yaw)):
                    self.has_map_tf = True
                    self.map_transform = (t.x, t.y, yaw)

    def tick(self) -> None:
        command = Twist()
        elapsed = time.monotonic() - self.started
        rotated = (0.0 if self.yaw is None or self.initial_yaw is None else
                   abs(math.atan2(math.sin(self.yaw - self.initial_yaw),
                                  math.cos(self.yaw - self.initial_yaw))))
        if (self.controller_armed and self.settle_started is None
                and rotated >= 0.30):
            self.settle_started = time.monotonic()
            self.settle_transform = self.map_transform
        if self.settle_started is not None:
            self.publisher.publish(command)
            if time.monotonic() - self.settle_started >= 2.0:
                stable = self.has_map_tf and self.map_transform is not None
                if stable and self.settle_transform is not None:
                    x, y, yaw = self.map_transform
                    x0, y0, yaw0 = self.settle_transform
                    dyaw = math.atan2(math.sin(yaw-yaw0), math.cos(yaw-yaw0))
                    stable = math.hypot(x-x0, y-y0) <= 0.25 and abs(dyaw) <= 0.15
                self.result = stable
                level = self.get_logger().info if stable else self.get_logger().error
                level(
                    f"P8_MAP_PRIME_{'PASS' if stable else 'FAIL'} "
                    f"yaw_change={rotated:.3f} rad map->odom_stable={stable}")
            return
        if elapsed >= 300.0:
            self.result = False
            self.publisher.publish(command)
            self.get_logger().error(
                f"P8_MAP_PRIME_FAIL elapsed={elapsed:.1f}s "
                f"yaw_change={rotated:.3f} rad map_tf={self.has_map_tf} "
                f"controller_armed={self.controller_armed}")
            return
        if self.controller_armed and self.yaw is not None and rotated < 0.30:
            command.angular.z = 0.15
        self.publisher.publish(command)
        if elapsed - (self.last_progress_log - self.started) >= 5.0:
            self.get_logger().info(
                f"P8_MAP_PRIME_WAIT control_active={self.controller_armed} "
                f"odom_seen={self.yaw is not None} yaw_change={rotated:.3f} rad")
            self.last_progress_log = time.monotonic()

    def on_control_status(self, msg: String) -> None:
        self.controller_armed = msg.data.startswith("ACTIVE ")


def main() -> int:
    rclpy.init()
    node = MapPrime()
    try:
        # Publish from the steady-clock main loop rather than an rclpy Timer:
        # this process runs during simulated-time startup, before /clock can be
        # trusted, and every iteration also services controller-status/odometry.
        while rclpy.ok() and node.result is None:
            rclpy.spin_once(node, timeout_sec=0.02)
            node.tick()
            time.sleep(0.08)
    except KeyboardInterrupt:
        node.result = False
    finally:
        node.publisher.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if node.result else 1


if __name__ == "__main__":
    raise SystemExit(main())
