import math
import sys
import time
import unittest
from pathlib import Path
from unittest.mock import patch

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
from terrain_path_follower import TerrainPathFollower


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


def make_grid():
    grid = OccupancyGrid()
    grid.info.width = 100
    grid.info.height = 100
    grid.info.resolution = 0.1
    grid.info.origin.position.x = -5.0
    grid.info.origin.position.y = -5.0
    grid.info.origin.orientation.w = 1.0
    grid.data = [-1] * 10000
    return grid


def make_follower():
    node = TerrainPathFollower.__new__(TerrainPathFollower)
    node.obstacle_map = make_grid()
    node.obstacle_threshold = 65
    node.obstacle_clearance = 0.35
    node.require_obstacle_map = False
    node.path = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)]
    node.invalidated_path_signature = None
    node.path_invalidation_monotonic = None
    return node


class TerrainFollowerSafetyTests(unittest.TestCase):
    def test_free_path_does_not_intersect_obstacle(self):
        node = make_follower()
        self.assertFalse(node._path_intersects_obstacle((0.0, 0.0), 0))

    def test_obstacle_within_path_clearance_is_detected(self):
        node = make_follower()
        gx, gy = node._grid_cell(node.obstacle_map, 1.0, 0.2)
        node.obstacle_map.data[gy * 100 + gx] = 100
        self.assertTrue(node._path_intersects_obstacle((0.0, 0.0), 0))

    def test_rotated_grid_cell_conversion(self):
        grid = make_grid()
        grid.info.origin.position.x = 0.0
        grid.info.origin.position.y = 0.0
        grid.info.origin.orientation.z = math.sin(math.pi / 4.0)
        grid.info.origin.orientation.w = math.cos(math.pi / 4.0)
        self.assertEqual((0, 10), TerrainPathFollower._grid_cell(grid, -1.05, 0.05))

    def test_stale_path_tick_publishes_zero_command(self):
        node = make_follower()
        node.path_received_monotonic = time.monotonic() - 3.0
        node.path_timeout = 2.5
        node.cmd_pub = Publisher()
        statuses = []
        node.publish_status = statuses.append
        node._tick()
        self.assertEqual(1, len(node.cmd_pub.messages))
        self.assertIsInstance(node.cmd_pub.messages[0], Twist)
        self.assertEqual(0.0, node.cmd_pub.messages[0].linear.x)
        self.assertEqual(0.0, node.cmd_pub.messages[0].angular.z)
        self.assertIn("INTEGRATION_STALE_TERRAIN_PLAN_STOP", statuses)

    def test_required_obstacle_map_waits_with_zero_command(self):
        node = make_follower()
        node.path_received_monotonic = time.monotonic()
        node.path_timeout = 2.5
        node.require_obstacle_map = True
        node.obstacle_map_received_monotonic = None
        node.cmd_pub = Publisher()
        statuses = []
        node.publish_status = statuses.append
        node._tick()
        self.assertEqual(1, len(node.cmd_pub.messages))
        self.assertEqual(0.0, node.cmd_pub.messages[0].linear.x)
        self.assertEqual(0.0, node.cmd_pub.messages[0].angular.z)
        self.assertIn("INTEGRATION_WAITING_FOR_OBSTACLE_MAP_STOP", statuses)

    def test_stale_obstacle_map_publishes_zero_command(self):
        node = make_follower()
        node.path_received_monotonic = time.monotonic()
        node.path_timeout = 2.5
        node.obstacle_map_received_monotonic = time.monotonic() - 2.0
        node.obstacle_map_timeout = 1.0
        node.cmd_pub = Publisher()
        statuses = []
        node.publish_status = statuses.append
        node._tick()
        self.assertEqual(1, len(node.cmd_pub.messages))
        self.assertEqual(0.0, node.cmd_pub.messages[0].linear.x)
        self.assertEqual(0.0, node.cmd_pub.messages[0].angular.z)
        self.assertIn("INTEGRATION_STALE_OBSTACLE_MAP_STOP", statuses)

    def test_empty_plan_clears_old_path_and_stops(self):
        node = make_follower()
        node.cmd_pub = Publisher()
        statuses = []
        node.publish_status = statuses.append
        from nav_msgs.msg import Path
        node._path_callback(Path())
        self.assertEqual([], node.path)
        self.assertEqual(1, len(node.cmd_pub.messages))
        self.assertEqual(0.0, node.cmd_pub.messages[0].linear.x)
        self.assertIn("INTEGRATION_EMPTY_TERRAIN_PLAN", statuses)

    def test_obstacle_stop_is_bounded_to_stops_and_never_reverses(self):
        node = make_follower()
        node.path_received_monotonic = time.monotonic()
        node.path_timeout = 2.5
        node.obstacle_map_received_monotonic = time.monotonic()
        node.obstacle_map_timeout = 1.0
        node.obstacle_map_received_monotonic = 10.0
        gx, gy = node._grid_cell(node.obstacle_map, 1.0, 0.0)
        node.obstacle_map.data[gy * 100 + gx] = 100
        node.cmd_pub = Publisher()
        node._pose = lambda: (0.0, 0.0, 0.0)
        node._goal_map = lambda: (2.0, 0.0, 0.0)
        node.goal = PoseStamped()
        node.goal_tolerance = 0.35
        node.lookahead_cells = 2
        node.max_linear = 0.20
        node.max_angular = 0.60
        node.last_waypoint = -1
        node.reached = False
        statuses = []
        node.publish_status = statuses.append
        with patch("terrain_path_follower.time.monotonic",
                   side_effect=[10.0, 10.0, 10.125]):
            for _ in range(5):
                node._tick()
        self.assertEqual(5, len(node.cmd_pub.messages))
        self.assertTrue(all(msg.linear.x == 0.0 and msg.angular.z == 0.0
                            for msg in node.cmd_pub.messages))
        self.assertNotIn("INTEGRATION_FOLLOWING waypoint=", statuses)
        self.assertTrue(any("INTEGRATION_PATH_INVALIDATED_OBSTACLE" in s
                            and "detection_to_stop_ms=" in s for s in statuses))
        self.assertTrue(any("detection_to_stop_ms=125.0" in s
                            for s in statuses))
        self.assertEqual([], node.path)

    def test_invalidated_path_cannot_rearm_until_changed_plan_arrives(self):
        from nav_msgs.msg import Path
        from geometry_msgs.msg import PoseStamped

        node = make_follower()
        original = list(node.path)
        node.path_received_monotonic = time.monotonic()
        node.path_timeout = 2.5
        node.obstacle_map_received_monotonic = time.monotonic()
        node.obstacle_map_timeout = 1.0
        gx, gy = node._grid_cell(node.obstacle_map, 1.0, 0.0)
        node.obstacle_map.data[gy * 100 + gx] = 100
        node.cmd_pub = Publisher()
        node._pose = lambda: (0.0, 0.0, 0.0)
        node._goal_map = lambda: (2.0, 0.0, 0.0)
        node.goal = PoseStamped()
        node.goal_tolerance = 0.35
        node.lookahead_cells = 2
        node.max_linear = 0.20
        node.max_angular = 0.60
        node.last_waypoint = -1
        node.reached = False
        statuses = []
        node.publish_status = statuses.append

        node._tick()
        stale = Path()
        for x, y in original:
            pose = PoseStamped()
            pose.pose.position.x, pose.pose.position.y = x, y
            stale.poses.append(pose)
        node._path_callback(stale)
        self.assertEqual([], node.path)
        self.assertEqual(0.0, node.cmd_pub.messages[-1].linear.x)

        replanned = Path()
        for x, y in [(0.0, 0.0), (0.5, 0.5), (1.0, 1.0), (2.0, 0.0)]:
            pose = PoseStamped()
            pose.pose.position.x, pose.pose.position.y = x, y
            replanned.poses.append(pose)
        with patch("terrain_path_follower.time.monotonic",
                   return_value=node.path_invalidation_monotonic + 0.275):
            node._path_callback(replanned)
        self.assertEqual(4, len(node.path))
        self.assertIsNone(node.invalidated_path_signature)
        self.assertTrue(any("INTEGRATION_REPLAN_ACCEPTED" in status and
                            "replan_latency_ms=275.0" in status
                            for status in statuses))

    def test_mission_hold_states_force_zero_at_command_boundary(self):
        node = TerrainPathFollower.__new__(TerrainPathFollower)
        node.mission_gate_enabled = True
        node.mission_status_timeout = 1.5
        node.cmd_pub = Publisher()
        statuses = []
        node.publish_status = statuses.append
        for state in ("PAUSED", "BLOCKED", "FAILED", "CANCELLED",
                      "ESTOPPED", "COMPLETED", "UNKNOWN"):
            node._mission_status_callback(String(data=f"MISSION_STATE lifecycle={state}"))
            node._tick()
            self.assertFalse(node.mission_motion_allowed, state)
        self.assertTrue(node.cmd_pub.messages)
        self.assertTrue(all(msg.linear.x == 0.0 and msg.angular.z == 0.0
                            for msg in node.cmd_pub.messages))
        node._mission_status_callback(String(data="MISSION_STATE lifecycle=ACTIVE"))
        self.assertTrue(node.mission_motion_allowed)

    def test_stale_mission_supervisor_heartbeat_stops_follower(self):
        node = TerrainPathFollower.__new__(TerrainPathFollower)
        node.mission_gate_enabled = True
        node.mission_motion_allowed = True
        node.mission_lifecycle_state = "ACTIVE"
        node.mission_status_timeout = 1.5
        node.mission_status_received_monotonic = time.monotonic() - 2.0
        node.cmd_pub = Publisher()
        statuses = []
        node.publish_status = statuses.append
        node._tick()
        self.assertEqual(1, len(node.cmd_pub.messages))
        self.assertEqual(0.0, node.cmd_pub.messages[0].linear.x)
        self.assertTrue(any("INTEGRATION_MISSION_HOLD_STOP lifecycle=STALE" in s
                            for s in statuses))

    def test_completed_mission_preserves_goal_reached_status_and_stops(self):
        node = TerrainPathFollower.__new__(TerrainPathFollower)
        node.reached = True
        node.mission_gate_enabled = True
        node.mission_motion_allowed = False
        node.mission_lifecycle_state = "COMPLETED"
        node.cmd_pub = Publisher()
        statuses = []
        node.publish_status = statuses.append
        node._tick()
        self.assertEqual(1, len(node.cmd_pub.messages))
        self.assertEqual(0.0, node.cmd_pub.messages[0].linear.x)
        self.assertEqual(0.0, node.cmd_pub.messages[0].angular.z)
        self.assertEqual([], statuses)


if __name__ == "__main__":
    unittest.main()
