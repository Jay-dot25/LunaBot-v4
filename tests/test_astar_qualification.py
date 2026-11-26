import math
from pathlib import Path
import sys
import unittest
from unittest.mock import Mock

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
from astar_navigation import AStarNavigation
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid

def make_planner(width=12, height=12, resolution=1.0, origin=(0.0, 0.0, 0.0),
                 occupied=(), inflation=0.0):
    nav = AStarNavigation.__new__(AStarNavigation)
    grid = OccupancyGrid()
    grid.info.width = width
    grid.info.height = height
    grid.info.resolution = resolution
    grid.info.origin.position.x = origin[0]
    grid.info.origin.position.y = origin[1]
    grid.info.origin.orientation.z = math.sin(origin[2] / 2.0)
    grid.info.origin.orientation.w = math.cos(origin[2] / 2.0)
    cells = [0] * (width * height)
    for x, y in occupied:
        cells[y * width + x] = 100
    grid.data = cells
    nav.map_msg = grid
    nav.inflation_radius = inflation
    nav.occupied_threshold = 65
    nav.unknown_is_obstacle = True
    nav.map_frame = "map"
    nav.auto_goal = False
    nav.goal_sent = True
    return nav

class AStarQualificationTests(unittest.TestCase):
    def test_node_starts_at_zero_simulation_time(self):
        import rclpy
        rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true'])
        nav = None
        try:
            nav = AStarNavigation()
            self.assertEqual(0, nav.get_clock().now().nanoseconds)
            self.assertIsNone(nav.last_plan)
        finally:
            if nav is not None:
                nav.destroy_node()
            rclpy.shutdown()

    def test_failed_plan_is_throttled_and_old_path_is_not_followed(self):
        from rclpy.time import Time
        nav = make_planner()
        nav.odom_msg = object()
        nav.goal_msg = PoseStamped()
        nav.reached = False
        nav.goal_pub = Mock()
        nav.path_points = [(1.0, 1.0)]
        nav.replan_period = 1.0
        nav.last_plan = Time(seconds=0)
        nav.get_clock = Mock()
        nav.schedule_clock = nav.get_clock.return_value
        nav.get_clock.return_value.now.return_value = Time(seconds=2)
        nav._current_map_pose = Mock(return_value=(0.0, 0.0, 0.0))
        nav._send_auto_goal = Mock()
        nav._plan = Mock(return_value=False)
        nav._publish_stop = Mock()
        nav._follow = Mock()
        nav._tick()
        nav.get_clock.return_value.now.return_value = Time(seconds=2.1)
        nav._tick()
        nav._plan.assert_called_once()
        nav._follow.assert_not_called()
        self.assertEqual([], nav.path_points)
        self.assertEqual(2, nav._publish_stop.call_count)

    def test_follower_waits_for_path_health_before_motion(self):
        nav = make_planner()
        nav.motion_health = Mock()
        nav._publish_stop = Mock()
        nav.publish_status = Mock()
        for reason in ('MISSING_PATH', 'STALE_PATH', 'STALE_HEALTH_MONITOR'):
            nav.motion_health.error.return_value = reason
            nav._follow((0.0, 0.0, 0.0))
            nav.motion_health.error.assert_called_with(check_path=True)
            nav.publish_status.assert_called_with('FOLLOWER_WAITING_FOR_HEALTH ' + reason)
        self.assertEqual(3, nav._publish_stop.call_count)

    def test_reaches_goal_around_wall_gap(self):
        wall = {(5, y) for y in range(12) if y != 6}
        nav = make_planner(12, 12, occupied=wall)
        path = nav._astar((2, 2), (9, 2))
        self.assertTrue(path)
        self.assertEqual((2, 2), path[0])
        self.assertEqual((9, 2), path[-1])
        self.assertTrue(all(nav._safe_cell(cell) for cell in path))

    def test_rotated_grid_world_cell_round_trip(self):
        nav = make_planner(width=20, height=20, resolution=0.25,
                           origin=(3.0, -2.0, math.pi / 2.0))
        for cell in [(0, 0), (4, 9), (18, 12)]:
            world = nav._grid_to_world(cell)
            self.assertEqual(cell, nav._world_to_grid(*world))

    def test_footprint_inflation_blocks_cells_within_radius(self):
        nav = make_planner(width=12, height=12, occupied={(5, 5)}, inflation=2.0)
        self.assertFalse(nav._safe_cell((7, 5)))
        self.assertTrue(nav._safe_cell((8, 5)))

    def test_diagonal_cannot_cut_between_blocked_cardinal_cells(self):
        nav = make_planner(width=3, height=3, occupied={(1, 0), (0, 1)})
        self.assertEqual([], nav._astar((0, 0), (1, 1)))

    def test_out_of_bounds_cell_is_not_snapped_inward(self):
        nav = make_planner(width=10, height=10)
        self.assertIsNone(nav._nearest_free((-1, 5)))
        self.assertIsNone(nav._nearest_free((10, 5)))

    def test_plan_rejects_out_of_bounds_goal(self):
        nav = make_planner(width=10, height=10)
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.pose.position.x = -0.1
        goal.pose.position.y = 4.5
        goal.pose.orientation.w = 1.0
        nav.goal_msg = goal
        nav._transform_pose = lambda x, y, yaw, source, target: (x, y, yaw)
        statuses = []
        nav.publish_status = statuses.append
        self.assertFalse(nav._plan((4.5, 4.5, 0.0)))
        self.assertIn("NO_SAFE_START_OR_GOAL_CELL", statuses)

if __name__ == "__main__":
    unittest.main()
