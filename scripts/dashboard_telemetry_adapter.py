#!/usr/bin/env python3
"""Read-only ROS adapter publishing age and provenance for dashboard views 1-5."""
from __future__ import annotations

import json
import time
from pathlib import Path

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from rosidl_runtime_py.utilities import get_message
from std_msgs.msg import String

from dashboard_telemetry import evaluate_topic, message_provenance


class DashboardTelemetryAdapter(Node):
    def __init__(self) -> None:
        super().__init__("lunabot_dashboard_telemetry_adapter")
        self.declare_parameter("config_path", "")
        configured = self.get_parameter("config_path").get_parameter_value().string_value
        config_path = Path(configured) if configured else Path(__file__).resolve().parents[1] / "config/dashboard/adapter.json"
        self.config = json.loads(config_path.read_text(encoding="utf-8"))
        self.entries = {entry["topic"]: entry for entry in self.config["topics"]}
        self.states = {topic: {"last_receive": None, "provenance": {}, "error": None}
                       for topic in self.entries}
        # rclpy.Node exposes a read-only ``subscriptions`` property.
        self._dashboard_subscriptions = {}
        self.publisher = self.create_publisher(String, self.config["output_topic"], 10)
        self.create_timer(
            float(self.config.get("publish_period_seconds", 0.25)), self.publish_snapshot,
            clock=Clock(clock_type=ClockType.STEADY_TIME),
        )
        self.create_timer(0.5, self.discover_topics, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.discover_topics()

    def discover_topics(self) -> None:
        discovered = dict(self.get_topic_names_and_types())
        for topic, entry in self.entries.items():
            if topic in self._dashboard_subscriptions:
                continue
            types = discovered.get(topic, [])
            if not types:
                continue
            expected = entry["type"]
            if expected not in types or len(types) != 1:
                self.states[topic]["error"] = f"expected {expected}; graph reports {types}"
                continue
            try:
                message_type = get_message(expected)
                qos = self.subscription_qos(topic, entry)
                callback = lambda msg, name=topic: self.on_message(name, msg)
                self._dashboard_subscriptions[topic] = self.create_subscription(
                    message_type, topic, callback, qos,
                )
                self.states[topic]["error"] = None
            except Exception as exc:  # Keep the adapter alive and report the exact feed failure.
                self.states[topic]["error"] = f"subscription setup failed: {type(exc).__name__}: {exc}"

    def subscription_qos(self, topic: str, entry: dict) -> QoSProfile:
        if entry.get("sensor_data_qos"):
            return QoSProfile(
                history=HistoryPolicy.KEEP_LAST, depth=5,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
            )
        infos = self.get_publishers_info_by_topic(topic)
        all_transient = entry.get("transient_local", False) or (bool(infos) and all(
            info.qos_profile.durability == DurabilityPolicy.TRANSIENT_LOCAL
            for info in infos
        ))
        return QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=(DurabilityPolicy.TRANSIENT_LOCAL if all_transient
                        else DurabilityPolicy.VOLATILE),
        )

    def on_message(self, topic: str, message) -> None:
        state = self.states[topic]
        state["last_receive"] = time.monotonic()
        state["provenance"] = message_provenance(message)
        state["error"] = None

    def publisher_nodes(self, topic: str) -> list[str]:
        return sorted({f"{info.node_namespace.rstrip('/')}/{info.node_name}"
                       for info in self.get_publishers_info_by_topic(topic)})

    def publish_snapshot(self) -> None:
        mono_now = time.monotonic()
        ros_now = self.get_clock().now().nanoseconds / 1_000_000_000.0
        rows = []
        for topic, entry in self.entries.items():
            state = self.states[topic]
            provenance = state["provenance"]
            row = evaluate_topic(
                topic=topic, message_type=entry["type"],
                last_receive_monotonic=state["last_receive"], now_monotonic=mono_now,
                source_stamp_seconds=provenance.get("source_stamp_seconds"),
                source_clock=provenance.get("source_clock"),
                frame_id=provenance.get("frame_id"), source_nodes=self.publisher_nodes(topic),
                source_frame_ids=provenance.get("source_frame_ids", []),
                stale_after_seconds=float(entry.get("stale_after_seconds", self.config["default_stale_after_seconds"])),
                source_age_limit_seconds=entry.get("source_age_limit_seconds"),
                freshness_mode=entry.get("freshness_mode", "stream"),
                stamp_origin=provenance.get("stamp_origin"), error=state["error"],
                now_ros_seconds=ros_now,
                expected_frame=entry.get("frame"),
            )
            row["views"] = entry.get("views", [entry.get("view")])
            rows.append(row)
        msg = String()
        msg.data = json.dumps({
            "schema": 1,
            "adapter_monotonic_seconds": mono_now,
            "adapter_ros_time_seconds": ros_now,
            "adapter_ros_clock_type": "sim_or_configured_ros_clock",
            "provenance_note": "Input topics are read-only and are not rewritten; status rows without source stamps are explicitly identified.",
            "topics": rows,
        }, separators=(",", ":"), sort_keys=True)
        self.publisher.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DashboardTelemetryAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
