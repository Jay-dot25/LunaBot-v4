#!/usr/bin/env python3
"""LunaBot V4 Phase L: final mission demonstration monitor.

This node watches the already validated Phase K outputs, accepts lifecycle
commands, and journals mission events. It never publishes velocity or goals.
"""

from __future__ import annotations

import re
import uuid

import rclpy
from rclpy.clock import Clock, ClockType
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger
from mission_lifecycle import (MissionLifecycle, goal_identifier,
                               validate_goal_pose, validate_occupancy_grid)
from mission_event_store import MissionEventStore


class PhaseLMission(Node):
    """Aggregate the final autonomous mission's retained runtime evidence."""

    def __init__(self) -> None:
        super().__init__("lunabot_phase_l_mission")
        self.declare_parameter("plan_topic", "/lunabot/terrain/plan")
        self.declare_parameter("replan_topic", "/lunabot/autonomy/replan_status")
        self.declare_parameter("autonomy_topic", "/lunabot/autonomy/status")
        self.declare_parameter("evaluation_topic", "/lunabot/evaluation/status")
        self.declare_parameter("navigation_topic", "/lunabot/navigation/status")
        self.declare_parameter("obstacle_topic", "/lunabot/obstacles/status")
        self.declare_parameter("goal_topic", "/goal_pose")
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("status_topic", "/lunabot/mission/status")
        self.declare_parameter("command_topic", "/lunabot/mission/command")
        self.declare_parameter("event_topic", "/lunabot/mission/events")
        self.declare_parameter(
            "event_store_path", "evidence/phase-l-launch-l/mission_events.sqlite3")
        self.declare_parameter("control_topic", "/lunabot/control/status")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("require_manual_goal", True)
        self.declare_parameter("retain_runtime_gates_between_goals", False)

        get = self.get_parameter
        status_qos = QoSProfile(depth=1)
        status_qos.reliability = ReliabilityPolicy.RELIABLE
        status_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        retained_qos = QoSProfile(depth=1)
        retained_qos.reliability = ReliabilityPolicy.RELIABLE
        retained_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.plan_sub = self.create_subscription(
            Path, str(get("plan_topic").value), self._plan_callback, retained_qos)
        self.replan_sub = self.create_subscription(
            String, str(get("replan_topic").value), self._replan_callback,
            retained_qos)
        self.autonomy_sub = self.create_subscription(
            String, str(get("autonomy_topic").value), self._autonomy_callback,
            retained_qos)
        self.evaluation_sub = self.create_subscription(
            String, str(get("evaluation_topic").value),
            self._evaluation_callback, retained_qos)
        self.goal_sub = self.create_subscription(
            PoseStamped, str(get("goal_topic").value), self._goal_callback, 10)
        self.navigation_sub = self.create_subscription(
            String, str(get("navigation_topic").value),
            self._navigation_callback, retained_qos)
        self.obstacle_sub = self.create_subscription(
            String, str(get("obstacle_topic").value),
            self._obstacle_callback, retained_qos)
        self.map_sub = self.create_subscription(
            OccupancyGrid, str(get("map_topic").value), self._map_callback,
            retained_qos)
        self.require_manual_goal = bool(get("require_manual_goal").value)
        self.retain_runtime_gates_between_goals = bool(
            get("retain_runtime_gates_between_goals").value)
        self.map_frame = str(get("map_frame").value)
        self.command_topic = str(get("command_topic").value)
        self.event_topic = str(get("event_topic").value)
        self.control_topic = str(get("control_topic").value)
        self.event_store = MissionEventStore(str(get("event_store_path").value))
        self.status_pub = self.create_publisher(
            String, str(get("status_topic").value), status_qos)
        self.event_pub = self.create_publisher(String, self.event_topic, status_qos)
        self.emergency_stop_client = self.create_client(
            Trigger, "/lunabot/safety/emergency_stop")
        self.command_sub = self.create_subscription(
            String, self.command_topic, self._command_callback, 10)
        self.control_sub = self.create_subscription(
            String, self.control_topic, self._control_callback, 10)

        self.plan_seen = False
        self.replan_pass = False
        self.goal_reached = False
        self.evaluation_pass = False
        self.goal_seen = False
        self.manual_goal_seen = False
        self.obstacle_detected = False
        self.map_seen = False
        self.map_valid = False
        self.map_updates = 0
        self.active_goal_id = ""
        self.lifecycle = MissionLifecycle()
        self.resume_state = "ACTIVE"
        self.failure_reason = ""
        self.last_status = ""
        self.passed = False
        self.event_store.record(
            f"process-start:{uuid.uuid4().hex}", "MISSION_PROCESS_STARTED",
            payload={"lifecycle": self.lifecycle.state})
        self.publish_status(
            "MISSION_WAITING_FOR_DATA lifecycle=WAITING_FOR_MAP goal_id=none")
        self.timer = self.create_timer(0.2, self._evaluate)
        # The follower judges supervisor freshness against wall time. Keep
        # the heartbeat on a steady clock too; a slow Gazebo simulation must
        # not make a healthy mission appear stale.
        heartbeat_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.heartbeat_timer = self.create_timer(
            0.5, self._heartbeat, clock=heartbeat_clock)

    def publish_status(self, text: str) -> None:
        if text == self.last_status:
            return
        self.last_status = text
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        self.get_logger().info(text)

    def _heartbeat(self) -> None:
        if self.last_status:
            self.status_pub.publish(String(data=self.last_status))

    def _transition(self, target: str, reason: str) -> bool:
        previous = self.lifecycle.state
        changed = self.lifecycle.transition(target)
        if changed:
            self.event_store.record(
                f"transition:{uuid.uuid4().hex}", "LIFECYCLE_TRANSITION",
                self.active_goal_id,
                {"from": previous, "to": target, "reason": reason})
        return changed

    def _publish_event(self, text: str) -> None:
        self.event_pub.publish(String(data=text))

    def _request_control_estop(self) -> str:
        client = getattr(self, "emergency_stop_client", None)
        if client is None or not client.service_is_ready():
            return "mission_gate_only"
        client.call_async(Trigger.Request())
        return "controller_latch_requested"

    def _command_result(self, command: str, accepted: bool, reason: str = "") -> None:
        outcome = "ACCEPTED" if accepted else "REJECTED"
        self.publish_status(
            f"MISSION_COMMAND_{outcome} command={command} "
            f"lifecycle={self.lifecycle.state} goal_id="
            f"{self.active_goal_id or 'none'}" +
            (f" reason={reason}" if reason else ""))

    def _reset_goal_evidence(self, *, retain_runtime_gates: bool = False,
                             retain_evaluation: bool = False) -> None:
        self.plan_seen = False
        self.goal_reached = False
        if not retain_runtime_gates:
            self.replan_pass = False
        if not retain_evaluation:
            self.evaluation_pass = False
        self.passed = False

    def _recovery_target(self) -> str | None:
        if not self.map_valid or not self.goal_seen:
            return None
        self._reset_goal_evidence()
        return "READY"

    def _command_callback(self, msg: String) -> None:
        parts = msg.data.strip().split()
        if not parts:
            self._command_result("EMPTY", False, "EMPTY_COMMAND")
            return
        command = parts[0].upper()
        state = self.lifecycle.state
        if command == "PAUSE" and len(parts) == 1 and state in ("READY", "ACTIVE"):
            self.resume_state = state
            self._transition("PAUSED", "OPERATOR_PAUSE")
            self._command_result(command, True)
        elif command == "RESUME" and len(parts) == 1 and state == "PAUSED":
            target = ("ACTIVE" if self.resume_state == "ACTIVE" and self.plan_seen
                      else "READY")
            self._transition(target, "OPERATOR_RESUME")
            self._command_result(command, True)
            self._evaluate()
        elif command == "CANCEL" and len(parts) == 1 and state not in (
                "CANCELLED", "ESTOPPED"):
            self._transition("CANCELLED", "OPERATOR_CANCEL")
            self._command_result(command, True)
        elif command in ("ESTOP", "E_STOP") and len(parts) == 1:
            self._transition("ESTOPPED", "OPERATOR_ESTOP")
            self._command_result(
                "ESTOP", True,
                f"control={self._request_control_estop()}")
        elif command == "RESET_ESTOP" and len(parts) == 1 and state == "ESTOPPED":
            target = self._recovery_target()
            if target is None:
                self._command_result(command, False, "VALID_MAP_AND_GOAL_REQUIRED")
            else:
                self._transition(target, "EXPLICIT_ESTOP_RESET")
                self._command_result(command, True)
                self._evaluate()
        elif command == "RECOVER" and len(parts) == 1 and state in ("BLOCKED", "FAILED"):
            target = self._recovery_target()
            if target is None:
                self._command_result(command, False, "VALID_MAP_AND_GOAL_REQUIRED")
            else:
                self.failure_reason = ""
                self._transition(target, "EXPLICIT_RECOVERY")
                self._command_result(command, True)
                self._evaluate()
        elif command in ("FAIL", "FAILED") and state not in ("CANCELLED", "ESTOPPED"):
            self.failure_reason = "OPERATOR_FAILURE"
            self._transition("FAILED", self.failure_reason)
            self._command_result("FAIL", True)
        elif command == "DELIVER" and len(parts) == 2:
            delivery_id = parts[1]
            if not re.fullmatch(r"[A-Za-z0-9._:-]{1,128}", delivery_id):
                self._command_result(command, False, "INVALID_DELIVERY_ID")
            elif not self.goal_reached or state != "COMPLETED":
                self._command_result(command, False, "MISSION_NOT_COMPLETED")
            else:
                recorded = self.event_store.record(
                    f"delivery:{delivery_id}", "DELIVERY_COMPLETED",
                    self.active_goal_id,
                    {"delivery_id": delivery_id})
                self._publish_event(
                    f"DELIVERY_RECORDED delivery_id={delivery_id} "
                    f"goal_id={recorded['goal_id']} "
                    f"duplicate={int(not recorded['inserted'])}")
                self._command_result(
                    command, True,
                    f"delivery_id={delivery_id} duplicate={int(not recorded['inserted'])}")
        else:
            self._command_result(command, False, "INVALID_STATE_OR_SYNTAX")

    def _control_callback(self, msg: String) -> None:
        text = msg.data.strip()
        if text.startswith("EMERGENCY_STOP"):
            self._transition("ESTOPPED", "CONTROL_ESTOP")
            self._command_result("ESTOP", True)
        elif text.startswith(("HEALTH_", "INVALID_COMMAND", "OUTPUT_AUTHORITY_CONFLICT")):
            self.failure_reason = text.split()[0]
            if self.lifecycle.state not in ("CANCELLED", "ESTOPPED"):
                self._transition("FAILED", self.failure_reason)
                self._command_result("FAIL", True, self.failure_reason)

    def _plan_callback(self, msg: Path) -> None:
        self.plan_seen = self.plan_seen or bool(msg.poses)
        self._evaluate()

    def _replan_callback(self, msg: String) -> None:
        self.replan_pass = self.replan_pass or "DYNAMIC_REPLAN_PASS" in msg.data
        self._evaluate()

    def _autonomy_callback(self, msg: String) -> None:
        text = msg.data
        if any(token in text for token in (
                "INTEGRATION_LOCAL_OBSTACLE_STOP",
                "INTEGRATION_STALE_TERRAIN_PLAN_STOP",
                "INTEGRATION_STALE_OBSTACLE_MAP_STOP")):
            if self.lifecycle.state in ("READY", "ACTIVE", "PAUSED"):
                self._transition("BLOCKED", "FOLLOWER_SAFETY_STOP")
                self._command_result("BLOCKED", True, "FOLLOWER_SAFETY_STOP")
        self.goal_reached = self.goal_reached or "INTEGRATION_GOAL_REACHED" in msg.data
        self._evaluate()

    def _evaluation_callback(self, msg: String) -> None:
        self.evaluation_pass = self.evaluation_pass or "EVALUATION_PASS" in msg.data
        self._evaluate()

    def _navigation_callback(self, msg: String) -> None:
        self.manual_goal_seen = self.manual_goal_seen or "MANUAL_GOAL_SELECTED" in msg.data
        self._evaluate()

    def _obstacle_callback(self, msg: String) -> None:
        self.obstacle_detected = self.obstacle_detected or "OBSTACLE_DETECTED" in msg.data
        self._evaluate()

    def _goal_callback(self, msg: PoseStamped) -> None:
        if self.lifecycle.state == "CANCELLED":
            return
        valid, reason = validate_goal_pose(msg)
        if not valid:
            if self.lifecycle.state not in ("FAILED", "ESTOPPED"):
                self.failure_reason = reason
                self._transition("FAILED", reason)
            self.publish_status(
                f"MISSION_GOAL_REJECTED reason={reason} "
                f"lifecycle={self.lifecycle.state} "
                f"goal_id={self.active_goal_id or 'none'}")
            return
        new_goal_id = goal_identifier(msg)
        if self.active_goal_id and new_goal_id != self.active_goal_id:
            # A new request starts a new goal-scoped mission attempt. A fresh
            # plan and reached-goal event are required for every leg. The
            # evaluator's controller qualification is launch-wide and remains
            # valid after its first successful physical run.
            self._reset_goal_evidence(
                retain_runtime_gates=self.retain_runtime_gates_between_goals,
                retain_evaluation=True)
        self.active_goal_id = new_goal_id
        self.goal_seen = True
        # In this launch mode AUTO_GOAL is disabled and this topic is fed by
        # RViz's manual goal tool. The navigation status topic is retained for
        # display, but a late QoS match can miss its MANUAL_GOAL_SELECTED edge;
        # the received goal itself is authoritative in manual-only mode.
        if self.require_manual_goal:
            self.manual_goal_seen = True
        if self.lifecycle.state in ("FAILED", "ESTOPPED"):
            self.publish_status(
                f"MISSION_RECOVERY_INPUT_UPDATED lifecycle={self.lifecycle.state} "
                f"goal_id={self.active_goal_id}")
            return
        self._evaluate()

    def _map_callback(self, msg: OccupancyGrid) -> None:
        if self.lifecycle.state == "CANCELLED":
            return
        valid, reason = validate_occupancy_grid(msg, self.map_frame)
        if not valid:
            # slam_toolbox can publish a zero-sized placeholder grid before
            # its first usable scan. Keep the mission in WAITING_FOR_MAP until
            # SLAM has data; malformed populated grids still fail closed.
            if (reason == "NONPOSITIVE_DIMENSIONS" and
                    (int(msg.info.width) <= 0 or int(msg.info.height) <= 0) and
                    self.lifecycle.state == "WAITING_FOR_MAP"):
                self.publish_status(
                    "MISSION_MAP_WAITING_FOR_SLAM "
                    f"width={int(msg.info.width)} height={int(msg.info.height)}")
                return
            self.map_valid = False
            self.map_seen = False
            if self.lifecycle.state not in ("FAILED", "ESTOPPED"):
                self.failure_reason = reason
                self._transition("FAILED", reason)
            self.publish_status(
                f"MISSION_MAP_REJECTED reason={reason} "
                f"lifecycle={self.lifecycle.state} "
                f"goal_id={self.active_goal_id or 'none'}")
            return
        self.map_valid = True
        self.map_seen = True
        self.map_updates += 1
        if self.lifecycle.state in ("FAILED", "ESTOPPED"):
            self.publish_status(
                f"MISSION_RECOVERY_INPUT_UPDATED lifecycle={self.lifecycle.state} "
                f"goal_id={self.active_goal_id or 'none'} map=valid")
            return
        self._evaluate()

    def _evaluate(self) -> None:
        if self.passed or self.lifecycle.state in (
                "FAILED", "PAUSED", "BLOCKED", "CANCELLED", "ESTOPPED"):
            return
        if not self.map_valid:
            target = "WAITING_FOR_MAP"
        elif not self.goal_seen:
            target = "WAITING_FOR_GOAL"
        elif not self.plan_seen:
            target = "READY"
        else:
            target = "ACTIVE"
        if target != self.lifecycle.state:
            self._transition(target, "MISSION_INPUTS_UPDATED")
        criteria = (
            self.plan_seen,
            self.replan_pass,
            self.goal_reached,
            self.evaluation_pass,
            self.goal_seen,
            self.map_seen,
            self.obstacle_detected,
            (self.manual_goal_seen or not self.require_manual_goal),
        )
        if all(criteria):
            if self.lifecycle.state != "COMPLETED":
                self._transition("COMPLETED", "MISSION_ACCEPTANCE_MET")
            self.passed = True
            self.publish_status(
                "MISSION_DEMO_PASS lifecycle=COMPLETED "
                f"goal_id={self.active_goal_id} plan=1 replan=1 goal=1 evaluation=1 "
                f"goal_pose=1 map=1 obstacle=1 manual_goal="
                f"{int(self.manual_goal_seen)} map_updates={self.map_updates}")
            return
        names = ("plan", "replan", "goal", "evaluation", "goal_pose", "map",
                 "obstacle", "manual_goal")
        missing = [name for name, valid in zip(names, criteria) if not valid]
        self.publish_status(
            "MISSION_WAITING_FOR_" + ",".join(missing).upper() +
            f" lifecycle={self.lifecycle.state} "
            f"goal_id={self.active_goal_id or 'none'}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PhaseLMission()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        node.event_store.close()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
