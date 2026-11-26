#!/usr/bin/env python3
"""Sample command topics at a bounded wall rate for motion evidence."""

from __future__ import annotations

import argparse
import math
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy


class TwistCapture(Node):
    def __init__(self, input_topic: str, input_path: Path,
                 output_topic: str, output_path: Path, rate: float) -> None:
        super().__init__("lunabot_motion_evidence_capture")
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.records = {
            input_topic: {"path": input_path, "latest": None},
            output_topic: {"path": output_path, "latest": None},
        }
        for topic in self.records:
            self.create_subscription(
                Twist, topic, lambda msg, topic=topic: self._receive(topic, msg), qos)
        for record in self.records.values():
            record["path"].parent.mkdir(parents=True, exist_ok=True)
            record["path"].write_text("")
        self.timer = self.create_timer(
            1.0 / rate, self._sample,
            clock=Clock(clock_type=ClockType.STEADY_TIME))

    def _receive(self, topic: str, msg: Twist) -> None:
        self.records[topic]["latest"] = msg

    def _sample(self) -> None:
        for record in self.records.values():
            msg = record["latest"]
            if msg is None:
                continue
            values = (msg.linear.x, msg.angular.z)
            if not all(math.isfinite(value) for value in values):
                continue
            with record["path"].open("a", encoding="utf-8") as stream:
                stream.write(
                    f"linear:\n  x: {msg.linear.x:.6f}\n"
                    f"  y: {msg.linear.y:.6f}\n  z: {msg.linear.z:.6f}\n"
                    f"angular:\n  x: {msg.angular.x:.6f}\n"
                    f"  y: {msg.angular.y:.6f}\n  z: {msg.angular.z:.6f}\n---\n")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input-topic", default="/cmd_vel_in")
    parser.add_argument("--input-path", required=True, type=Path)
    parser.add_argument("--output-topic", default="/cmd_vel")
    parser.add_argument("--output-path", required=True, type=Path)
    parser.add_argument("--rate", type=float, default=2.0)
    args = parser.parse_args()
    if not math.isfinite(args.rate) or args.rate <= 0.0:
        parser.error("--rate must be finite and positive")

    rclpy.init()
    node = TwistCapture(args.input_topic, args.input_path,
                        args.output_topic, args.output_path, args.rate)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
