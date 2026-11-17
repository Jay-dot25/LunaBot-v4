import math
import sys
import tempfile
import unittest
from pathlib import Path

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path as NavPath
from std_msgs.msg import String

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
from mission_lifecycle import (MissionLifecycle, goal_identifier,
                               validate_occupancy_grid)
from mission_event_store import MissionEventStore
from phase_l_mission import PhaseLMission


def valid_map():
    grid = OccupancyGrid()
    grid.header.frame_id = "map"
    grid.info.width = 2
    grid.info.height = 2
    grid.info.resolution = 0.1
    grid.info.origin.orientation.w = 1.0
    grid.data = [0, 0, -1, 100]
    return grid


def goal(stamp=1, x=1.0):
    message = PoseStamped()
    message.header.frame_id = "map"
    message.header.stamp.sec = stamp
    message.pose.position.x = x
    message.pose.orientation.w = 1.0
    return message


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


def mission_node(event_store=None):
    node = PhaseLMission.__new__(PhaseLMission)
    node.lifecycle = MissionLifecycle()
    node.event_store = event_store or MissionEventStore(":memory:")
    node.event_pub = Publisher()
    node.resume_state = "ACTIVE"
    node.failure_reason = ""
    node.map_frame = "map"
    node.plan_seen = False
    node.replan_pass = False
    node.goal_reached = False
    node.evaluation_pass = False
    node.goal_seen = False
    node.manual_goal_seen = False
    node.obstacle_detected = False
    node.map_seen = False
    node.map_valid = False
    node.map_updates = 0
    node.active_goal_id = ""
    node.require_manual_goal = False
    node.retain_runtime_gates_between_goals = False
    node.last_status = ""
    node.passed = False
    statuses = []
    node.publish_status = statuses.append
    node._test_statuses = statuses
    return node


def active_mission(node, request=None):
    node._map_callback(valid_map())
    node._goal_callback(request or goal())
    path = NavPath()
    path.poses.append(PoseStamped())
    node._plan_callback(path)


class MissionLifecycleTests(unittest.TestCase):
    def test_normal_mission_advances_through_lifecycle_to_goal_id_pass(self):
        node = mission_node()
        node._map_callback(valid_map())
        self.assertEqual("WAITING_FOR_GOAL", node.lifecycle.state)
        request = goal()
        expected_id = goal_identifier(request)
        node._goal_callback(request)
        self.assertEqual("READY", node.lifecycle.state)
        self.assertEqual(expected_id, node.active_goal_id)
        path = NavPath()
        path.poses.append(PoseStamped())
        node._plan_callback(path)
        self.assertEqual("ACTIVE", node.lifecycle.state)
        node.replan_pass = True
        node.goal_reached = True
        node.evaluation_pass = True
        node.obstacle_detected = True
        node._evaluate()
        self.assertEqual("COMPLETED", node.lifecycle.state)
        self.assertTrue(node.passed)
        self.assertIn(f"goal_id={expected_id}", node._test_statuses[-1])
        self.assertIn("MISSION_DEMO_PASS", node._test_statuses[-1])

    def test_goal_after_preexisting_plan_enters_active_without_crashing(self):
        node = mission_node()
        node._map_callback(valid_map())
        path = NavPath()
        path.poses.append(PoseStamped())
        node._plan_callback(path)
        self.assertEqual("WAITING_FOR_GOAL", node.lifecycle.state)

        node._goal_callback(goal())

        self.assertEqual("ACTIVE", node.lifecycle.state)
        self.assertTrue(node.goal_seen)
        self.assertTrue(node.plan_seen)
        self.assertIn("MISSION_WAITING_FOR_REPLAN", node._test_statuses[-1])

    def test_invalid_map_fails_and_is_not_counted_as_seen(self):
        node = mission_node()
        malformed = valid_map()
        malformed.data.pop()
        node._map_callback(malformed)
        self.assertEqual("FAILED", node.lifecycle.state)
        self.assertFalse(node.map_seen)
        self.assertFalse(node.map_valid)
        self.assertEqual(0, node.map_updates)
        self.assertIn("MISSION_MAP_REJECTED reason=DATA_LENGTH_MISMATCH",
                      node._test_statuses[-1])

    def test_invalid_goal_pose_is_rejected_into_failed_lifecycle(self):
        node = mission_node()
        malformed = goal()
        malformed.pose.orientation.w = 0.0
        node._goal_callback(malformed)
        self.assertEqual("FAILED", node.lifecycle.state)
        self.assertFalse(node.goal_seen)
        self.assertEqual("", node.active_goal_id)
        self.assertIn("MISSION_GOAL_REJECTED reason=INVALID_ORIENTATION",
                      node._test_statuses[-1])

    def test_map_validator_rejects_wrong_frame_nonfinite_and_bad_values(self):
        grid = valid_map()
        grid.header.frame_id = "odom"
        self.assertEqual((False, "FRAME_MISMATCH"), validate_occupancy_grid(grid))
        grid = valid_map()
        grid.info.resolution = math.nan
        self.assertEqual((False, "INVALID_RESOLUTION"), validate_occupancy_grid(grid))
        grid = valid_map()
        grid.data[0] = 101
        self.assertEqual((False, "INVALID_OCCUPANCY_VALUE"),
                         validate_occupancy_grid(grid))

    def test_goal_identifier_is_stable_and_changes_for_new_goal_request(self):
        first = goal(stamp=11, x=1.25)
        same = goal(stamp=11, x=1.25)
        changed = goal(stamp=12, x=1.25)
        self.assertEqual(goal_identifier(first), goal_identifier(same))
        self.assertNotEqual(goal_identifier(first), goal_identifier(changed))

    def test_new_goal_invalidates_goal_scoped_completion_evidence(self):
        node = mission_node()
        node._map_callback(valid_map())
        node._goal_callback(goal(stamp=1))
        path = NavPath()
        path.poses.append(PoseStamped())
        node._plan_callback(path)
        node.replan_pass = True
        node.goal_reached = True
        node.evaluation_pass = True
        old_id = node.active_goal_id
        node._goal_callback(goal(stamp=2, x=2.0))
        self.assertNotEqual(old_id, node.active_goal_id)
        self.assertFalse(node.plan_seen)
        self.assertFalse(node.replan_pass)
        self.assertFalse(node.goal_reached)
        self.assertFalse(node.evaluation_pass)
        self.assertFalse(node.passed)
        self.assertEqual("READY", node.lifecycle.state)

    def test_itinerary_mode_keeps_launch_gates_but_requires_each_leg_plan_and_goal(self):
        node = mission_node()
        node.retain_runtime_gates_between_goals = True
        active_mission(node, goal(stamp=30, x=0.0))
        node.replan_pass = True
        node.evaluation_pass = True
        node.obstacle_detected = True
        node.goal_reached = True
        node._evaluate()
        self.assertEqual("COMPLETED", node.lifecycle.state)

        next_goal = goal(stamp=31, x=4.8)
        node._goal_callback(next_goal)
        self.assertFalse(node.plan_seen)
        self.assertFalse(node.goal_reached)
        self.assertTrue(node.replan_pass)
        self.assertTrue(node.evaluation_pass)
        self.assertEqual("READY", node.lifecycle.state)

        path = NavPath()
        path.poses.append(PoseStamped())
        node._plan_callback(path)
        self.assertEqual("ACTIVE", node.lifecycle.state)
        self.assertFalse(node.passed)
        node.goal_reached = True
        node._evaluate()
        self.assertEqual("COMPLETED", node.lifecycle.state)
        self.assertTrue(node.passed)

    def test_pause_resume_and_cancel_are_explicit_and_fail_closed(self):
        node = mission_node()
        active_mission(node)
        node._command_callback(String(data="PAUSE"))
        self.assertEqual("PAUSED", node.lifecycle.state)
        node._command_callback(String(data="RESUME"))
        self.assertEqual("ACTIVE", node.lifecycle.state)
        node._command_callback(String(data="CANCEL"))
        self.assertEqual("CANCELLED", node.lifecycle.state)
        node._command_callback(String(data="RESUME"))
        self.assertEqual("CANCELLED", node.lifecycle.state)
        self.assertIn("MISSION_COMMAND_REJECTED command=RESUME",
                      node._test_statuses[-1])

    def test_blocked_and_failed_missions_require_explicit_recovery(self):
        node = mission_node()
        active_mission(node)
        node._autonomy_callback(String(data="INTEGRATION_LOCAL_OBSTACLE_STOP"))
        self.assertEqual("BLOCKED", node.lifecycle.state)
        changed_path = NavPath()
        changed_path.poses.append(PoseStamped())
        node._plan_callback(changed_path)
        self.assertEqual("BLOCKED", node.lifecycle.state)
        node._command_callback(String(data="RECOVER"))
        self.assertEqual("READY", node.lifecycle.state)
        self.assertFalse(node.plan_seen)
        node._command_callback(String(data="FAIL"))
        self.assertEqual("FAILED", node.lifecycle.state)

    def test_initial_obstacle_map_wait_does_not_block_active_mission(self):
        node = mission_node()
        active_mission(node)
        node._autonomy_callback(String(
            data="INTEGRATION_WAITING_FOR_OBSTACLE_MAP_STOP"))
        self.assertEqual("ACTIVE", node.lifecycle.state)

    def test_path_invalidation_for_replan_keeps_mission_active(self):
        node = mission_node()
        active_mission(node)
        node._autonomy_callback(String(
            data="INTEGRATION_PATH_INVALIDATED_OBSTACLE "
                 "detection_to_stop_ms=20.0"))
        self.assertEqual("ACTIVE", node.lifecycle.state)
        node._autonomy_callback(String(data="INTEGRATION_REPLAN_ACCEPTED cells=40"))
        self.assertEqual("ACTIVE", node.lifecycle.state)

    def test_local_or_stale_obstacle_safety_stop_blocks_mission(self):
        for status in (
                "INTEGRATION_LOCAL_OBSTACLE_STOP",
                "INTEGRATION_STALE_TERRAIN_PLAN_STOP",
                "INTEGRATION_STALE_OBSTACLE_MAP_STOP"):
            node = mission_node()
            active_mission(node)
            node._autonomy_callback(String(data=status))
            self.assertEqual("BLOCKED", node.lifecycle.state, status)

    def test_e_stop_requires_explicit_reset_and_fresh_plan(self):
        node = mission_node()
        active_mission(node)
        node._control_callback(String(data="EMERGENCY_STOP input=/cmd_vel_in"))
        self.assertEqual("ESTOPPED", node.lifecycle.state)
        node._command_callback(String(data="RESET_ESTOP"))
        self.assertEqual("READY", node.lifecycle.state)
        self.assertFalse(node.plan_seen)
        node._command_callback(String(data="ESTOP"))
        self.assertEqual("ESTOPPED", node.lifecycle.state)

    def test_failed_map_can_recover_only_after_valid_inputs_and_command(self):
        node = mission_node()
        malformed = valid_map()
        malformed.data.pop()
        node._map_callback(malformed)
        self.assertEqual("FAILED", node.lifecycle.state)
        node._map_callback(valid_map())
        node._goal_callback(goal(stamp=77))
        self.assertEqual("FAILED", node.lifecycle.state)
        node._command_callback(String(data="RECOVER"))
        self.assertEqual("READY", node.lifecycle.state)
        self.assertTrue(node.map_valid)
        self.assertTrue(node.goal_seen)

    def test_delivery_event_is_idempotent_after_process_restart(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            path = Path(temp_dir) / "mission.sqlite3"
            first_store = MissionEventStore(path)
            first = mission_node(first_store)
            active_mission(first, goal(stamp=50, x=3.0))
            first.replan_pass = True
            first.goal_reached = True
            first.evaluation_pass = True
            first.obstacle_detected = True
            first._evaluate()
            first_goal_id = first.active_goal_id
            first._command_callback(String(data="DELIVER cargo-50"))
            self.assertEqual(1, first_store.count("DELIVERY_COMPLETED"))
            first_store.close()

            restarted_store = MissionEventStore(path)
            restarted = mission_node(restarted_store)
            active_mission(restarted, goal(stamp=51, x=4.0))
            restarted.replan_pass = True
            restarted.goal_reached = True
            restarted.evaluation_pass = True
            restarted.obstacle_detected = True
            restarted._evaluate()
            self.assertNotEqual(first_goal_id, restarted.active_goal_id)
            restarted._command_callback(String(data="DELIVER cargo-50"))
            self.assertEqual(1, restarted_store.count("DELIVERY_COMPLETED"))
            stored = restarted_store.get("delivery:cargo-50")
            self.assertEqual(first_goal_id, stored["goal_id"])
            self.assertIn("duplicate=1", restarted.event_pub.messages[-1].data)
            restarted_store.close()


if __name__ == "__main__":
    unittest.main()
