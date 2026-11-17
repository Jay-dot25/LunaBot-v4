#!/usr/bin/env python3
"""Guarded dashboard command relay; never publishes velocity commands."""
from __future__ import annotations

import json
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger

from dashboard_control_policy import decide_request, safety_health_created_monotonic


class DashboardControlGateway(Node):
    def __init__(self) -> None:
        super().__init__("lunabot_dashboard_control_gateway")
        self.declare_parameter("config_path", "")
        configured = self.get_parameter("config_path").get_parameter_value().string_value
        config_path = Path(configured) if configured else Path(__file__).resolve().parents[1] / "config/dashboard/controls.json"
        self.config = json.loads(config_path.read_text(encoding="utf-8"))
        self.last_mission_receive = None
        self.last_safety_receive = None
        self.last_safety_source_monotonic = None
        self.latest_mission_status = ""
        self.latest_safety_health = ""
        self.pending_services = set()

        mission_qos = QoSProfile(depth=1)
        mission_qos.reliability = ReliabilityPolicy.RELIABLE
        mission_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            String, self.config["mission_status_topic"], self.on_mission_status,
            mission_qos,
        )
        self.create_subscription(
            String, self.config["safety_health_topic"], self.on_safety_health, 10,
        )
        self.request_sub = self.create_subscription(
            String, self.config["request_topic"], self.on_request, 10,
        )
        self.mission_pub = self.create_publisher(
            String, self.config["mission_command_topic"], 10,
        )
        self.status_pub = self.create_publisher(String, self.config["status_topic"], 10)
        services = self.config["safety_services"]
        self.service_clients = {
            action: self.create_client(Trigger, service)
            for action, service in services.items()
        }

    def on_mission_status(self, message: String) -> None:
        self.last_mission_receive = time.monotonic()
        self.latest_mission_status = message.data

    def on_safety_health(self, message: String) -> None:
        self.last_safety_receive = time.monotonic()
        self.last_safety_source_monotonic = safety_health_created_monotonic(message.data)
        self.latest_safety_health = message.data

    def ages(self) -> tuple[float | None, float | None]:
        now = time.monotonic()
        mission = None if self.last_mission_receive is None else max(
            0.0, now - self.last_mission_receive)
        safety = None if self.last_safety_receive is None else max(
            0.0, now - self.last_safety_receive)
        if safety is not None:
            if self.last_safety_source_monotonic is None:
                safety = None
            else:
                source_age = now - self.last_safety_source_monotonic
                safety = max(safety, source_age) if source_age >= 0.0 else None
        return mission, safety

    def publish_status(self, *, request_id: str, action: str, accepted: bool,
                       reason: str, command: str | None = None) -> None:
        mission_age, safety_age = self.ages()
        payload = {
            "schema": 1,
            "gateway_monotonic_seconds": time.monotonic(),
            "request_id": request_id,
            "action": action,
            "accepted": accepted,
            "reason": reason,
            "mission_status_age_seconds": mission_age,
            "safety_health_age_seconds": safety_age,
            "latest_mission_status": self.latest_mission_status or None,
            "latest_safety_health": self.latest_safety_health or None,
            "command": command,
        }
        message = String()
        message.data = json.dumps(payload, separators=(",", ":"), sort_keys=True)
        self.status_pub.publish(message)

    def on_request(self, message: String) -> None:
        request_id = ""
        action = "UNKNOWN"
        try:
            request = json.loads(message.data)
            if isinstance(request, dict):
                request_id = str(request.get("request_id", ""))[:80]
                action = str(request.get("action", "UNKNOWN"))[:40]
        except (json.JSONDecodeError, TypeError):
            request = None
        mission_age, safety_age = self.ages()
        decision = decide_request(
            request,
            mission_status_age=mission_age,
            safety_health_age=safety_age,
            fresh_mission_seconds=float(self.config["fresh_mission_status_seconds"]),
            fresh_safety_seconds=float(self.config["fresh_safety_health_seconds"]),
        )
        if not decision.get("accepted"):
            self.publish_status(request_id=request_id, action=action, accepted=False,
                                reason=decision["reason"])
            return

        route = decision["route"]
        command = decision.get("command")
        if route == "mission":
            outgoing = String()
            outgoing.data = command
            self.mission_pub.publish(outgoing)
            self.publish_status(
                request_id=request_id, action=action, accepted=True,
                reason="PUBLISHED_AWAIT_MISSION_STATUS", command=command,
            )
            return

        service_action = "RESET_ESTOP" if route == "reset_then_mission" else (
            "ARM_STARTUP" if route == "arm_startup" else "ESTOP")
        client = self.service_clients[service_action]
        if self.pending_services and service_action != "ESTOP":
            self.publish_status(request_id=request_id, action=action, accepted=False,
                                reason="SAFETY_SERVICE_REQUEST_ALREADY_PENDING")
            return
        if not client.service_is_ready():
            self.publish_status(request_id=request_id, action=action, accepted=False,
                                reason="SAFETY_SERVICE_UNAVAILABLE")
            return

        self.pending_services.add(service_action)
        self.publish_status(request_id=request_id, action=action, accepted=True,
                            reason="SAFETY_SERVICE_PENDING")
        future = client.call_async(Trigger.Request())
        future.add_done_callback(
            lambda completed, key=service_action, req_id=request_id, act=action,
            cmd=command: self.on_service_result(completed, key, req_id, act, cmd)
        )

    def on_service_result(self, future, service_action: str, request_id: str,
                          action: str, command: str | None) -> None:
        self.pending_services.discard(service_action)
        try:
            response = future.result()
        except Exception as exc:
            self.publish_status(
                request_id=request_id, action=action, accepted=False,
                reason=f"SERVICE_CALL_FAILED:{type(exc).__name__}", command=command,
            )
            return
        if not response.success:
            self.publish_status(
                request_id=request_id, action=action, accepted=False,
                reason=f"SAFETY_SERVICE_REJECTED:{response.message}", command=command,
            )
            return
        if service_action == "RESET_ESTOP":
            # Controller reset is authoritative first; then align the mission
            # lifecycle. The mission node still checks its own map/goal gates.
            reset_message = String()
            reset_message.data = "RESET_ESTOP"
            self.mission_pub.publish(reset_message)
            result = "SAFETY_RESET_ACCEPTED_MISSION_RESET_PUBLISHED"
        else:
            result = f"SAFETY_SERVICE_ACCEPTED:{response.message}"
        self.publish_status(request_id=request_id, action=action, accepted=True,
                            reason=result, command=command)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DashboardControlGateway()
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
