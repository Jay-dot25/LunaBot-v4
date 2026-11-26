#!/usr/bin/env python3
"""LunaBot V4 Phase J: monitor live terrain-plan replanning.

The Phase H terrain-aware planner already replans from the live cost map and
odometry. Phase J makes that behavior an explicit, auditable contract: this
node observes the active terrain path, shared goal, and real odometry, counts
changed path revisions after rover motion, and publishes a latched status.
It never publishes velocity and cannot compete with the Phase I follower.
"""

from __future__ import annotations

import math
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String


class DynamicReplanMonitor(Node):
    """Prove that live motion produces changed terrain-aware path revisions."""

    def __init__(self) -> None:
        super().__init__("lunabot_dynamic_replan_monitor")
        self.declare_parameter("plan_topic", "/lunabot/terrain/plan")
        self.declare_parameter("goal_topic", "/goal_pose")
        self.declare_parameter("odom_topic", "/lunabot/odom")
        self.declare_parameter("planner_status_topic", "/lunabot/terrain/planner/status")
        self.declare_parameter("obstacle_topic", "/lunabot/obstacles/status")
        self.declare_parameter("status_topic", "/lunabot/autonomy/replan_status")
        self.declare_parameter("require_obstacle", False)
        self.declare_parameter("minimum_revisions", 2)
        self.declare_parameter("minimum_motion", 0.05)

        get = self.get_parameter
        self.plan_topic = str(get("plan_topic").value)
        self.goal_topic = str(get("goal_topic").value)
        self.odom_topic = str(get("odom_topic").value)
        self.planner_status_topic = str(get("planner_status_topic").value)
        self.obstacle_topic = str(get("obstacle_topic").value)
        self.status_topic = str(get("status_topic").value)
        self.require_obstacle = bool(get("require_obstacle").value)
        self.minimum_revisions = max(1, int(get("minimum_revisions").value))
        self.minimum_motion = max(0.01, float(get("minimum_motion").value))

        plan_qos = QoSProfile(depth=1)
        plan_qos.reliability = ReliabilityPolicy.RELIABLE
        plan_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.plan_sub = self.create_subscription(
            Path, self.plan_topic, self._plan_callback, plan_qos)
        self.goal_sub = self.create_subscription(
            PoseStamped, self.goal_topic, self._goal_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, self.odom_topic, self._odom_callback,
            qos_profile_sensor_data)
        self.planner_status_sub = self.create_subscription(
            String, self.planner_status_topic, self._planner_status_callback,
            plan_qos)
        self.obstacle_sub = self.create_subscription(
            String, self.obstacle_topic, self._obstacle_callback, plan_qos)
        status_qos = QoSProfile(depth=1)
        status_qos.reliability = ReliabilityPolicy.RELIABLE
        status_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.status_pub = self.create_publisher(String, self.status_topic, status_qos)

        self.goal_seen = False
        self.active_goal_key: Optional[tuple] = None
        self.last_signature: Optional[tuple] = None
        self.revisions = 0
        self.total_motion = 0.0
        self.last_x: Optional[float] = None
        self.last_y: Optional[float] = None
        self.last_planner_status = ""
        self.obstacle_seen = False
        self.obstacle_detected_monotonic: Optional[float] = None
        self.obstacle_replan_latency_ms: Optional[float] = None
        self.last_status = ""
        self.passed = False
        self.publish_status("DYNAMIC_REPLAN_WAITING_FOR_PLAN")
        self.timer = self.create_timer(0.2, self._evaluate)

    def publish_status(self, text: str) -> None:
        if text == self.last_status:
            return
        self.last_status = text
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        self.get_logger().info(text)

    @staticmethod
    def _goal_key(msg: PoseStamped) -> tuple:
        pose = msg.pose
        return (
            msg.header.frame_id or "",
            round(float(pose.position.x), 5),
            round(float(pose.position.y), 5),
            round(float(pose.position.z), 5),
            round(float(pose.orientation.x), 5),
            round(float(pose.orientation.y), 5),
            round(float(pose.orientation.z), 5),
            round(float(pose.orientation.w), 5),
        )

    def _goal_callback(self, msg: PoseStamped) -> None:
        key = self._goal_key(msg)
        if key != self.active_goal_key:
            # AStar echoes the active goal on /goal_pose. Pose-based identity
            # ignores that echo's stamp but starts a fresh replan measurement
            # for each new operator-selected destination.
            self.active_goal_key = key
            self.last_signature = None
            self.revisions = 0
            self.total_motion = 0.0
            self.last_x = None
            self.last_y = None
            self.last_planner_status = ""
            self.passed = False
            self.publish_status("DYNAMIC_REPLAN_NEW_GOAL")
        self.goal_seen = True
        self._evaluate()

    def _planner_status_callback(self, msg: String) -> None:
        self.last_planner_status = msg.data
        # A previous successful replan does not mean the current plan is still
        # usable. Reopen the gate whenever the planner starts replanning or
        # invalidates its active path.
        if not msg.data.startswith("TERRAIN_PLAN_PASS"):
            self.passed = False
        self._evaluate()

    def _obstacle_callback(self, msg: String) -> None:
        if "OBSTACLE_DETECTED" in msg.data:
            self.obstacle_seen = True
            if self.obstacle_detected_monotonic is None:
                self.obstacle_detected_monotonic = time.monotonic()
        self._evaluate()

    def _odom_callback(self, msg: Odometry) -> None:
        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)
        if self.last_x is not None and self.last_y is not None:
            self.total_motion += math.hypot(x - self.last_x, y - self.last_y)
        self.last_x, self.last_y = x, y
        self._evaluate()

    @staticmethod
    def _signature(msg: Path) -> tuple:
        return tuple(
            (round(pose.pose.position.x, 2), round(pose.pose.position.y, 2))
            for pose in msg.poses
        )

    def _plan_callback(self, msg: Path) -> None:
        if not msg.poses:
            self.passed = False
            self.publish_status("DYNAMIC_REPLAN_EMPTY_PLAN")
            return
        signature = self._signature(msg)
        if self.last_signature is None:
            self.last_signature = signature
            self.publish_status(
                f"DYNAMIC_REPLAN_PLAN_RECEIVED cells={len(signature)}")
            return
        if signature != self.last_signature:
            self.passed = False
            self.revisions += 1
            self.last_signature = signature
            if (self.obstacle_detected_monotonic is not None and
                    self.obstacle_replan_latency_ms is None):
                self.obstacle_replan_latency_ms = max(
                    0.0, (time.monotonic() -
                          self.obstacle_detected_monotonic) * 1000.0)
            self.publish_status(
                f"DYNAMIC_REPLAN_UPDATE revision={self.revisions} "
                f"cells={len(signature)} motion={self.total_motion:.2f}" +
                (f" obstacle_replan_latency_ms="
                 f"{self.obstacle_replan_latency_ms:.1f}"
                 if self.obstacle_replan_latency_ms is not None else ""))
        self._evaluate()

    def _evaluate(self) -> None:
        if self.passed:
            return
        if self.last_signature is None:
            self.publish_status("DYNAMIC_REPLAN_WAITING_FOR_PLAN")
            return
        if not self.goal_seen:
            self.publish_status("DYNAMIC_REPLAN_WAITING_FOR_GOAL")
            return
        if self.require_obstacle and not self.obstacle_seen:
            self.publish_status("DYNAMIC_REPLAN_WAITING_FOR_OBSTACLE")
            return
        if (self.require_obstacle and
                self.obstacle_replan_latency_ms is None):
            self.publish_status("DYNAMIC_REPLAN_WAITING_FOR_OBSTACLE_REPLAN")
            return
        if self.revisions < self.minimum_revisions:
            self.publish_status(
                f"DYNAMIC_REPLAN_WAITING_FOR_UPDATE revisions={self.revisions}/"
                f"{self.minimum_revisions}")
            return
        if self.total_motion < self.minimum_motion:
            self.publish_status(
                f"DYNAMIC_REPLAN_WAITING_FOR_MOTION motion={self.total_motion:.2f}")
            return
        if not self.last_planner_status.startswith("TERRAIN_PLAN_PASS"):
            self.publish_status(
                "DYNAMIC_REPLAN_WAITING_FOR_VALID_PLAN "
                f"planner={self.last_planner_status or 'UNKNOWN'}")
            return
        self.passed = True
        self.publish_status(
            f"DYNAMIC_REPLAN_PASS revisions={self.revisions} "
            f"motion={self.total_motion:.2f} obstacle={int(self.obstacle_seen)} "
            + (f"obstacle_replan_latency_ms="
               f"{self.obstacle_replan_latency_ms:.1f} "
               if self.obstacle_replan_latency_ms is not None else "")
            + f"planner={self.last_planner_status}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DynamicReplanMonitor()
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
