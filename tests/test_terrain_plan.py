import unittest
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from terrain_plan import (compute_map_revision, grid_to_world,
                          merge_obstacle_grid, path_is_traversable,
                          select_safe_goal_along_heading, weighted_astar,
                          world_to_grid)


class WeightedTerrainPlanTests(unittest.TestCase):
    def test_safe_goal_preflight_selects_clear_lateral_target(self):
        width = height = 24
        resolution = 0.25
        origin = (-1.0, -1.0)
        costs = [0] * (width * height)
        blocked_x, blocked_y = world_to_grid(
            1.0, 0.0, origin_x=origin[0], origin_y=origin[1],
            origin_yaw=0.0, resolution=resolution)
        costs[blocked_y * width + blocked_x] = 100

        selected = select_safe_goal_along_heading(
            costs, width, height, origin_x=origin[0], origin_y=origin[1],
            origin_yaw=0.0, resolution=resolution, start_x=0.0, start_y=0.0,
            heading=0.0, max_distance=1.0, min_distance=0.5,
            distance_step=0.25)

        self.assertIsNotNone(selected)
        self.assertEqual((1.0, 0.25, 1.0, 0.25), selected)

    def test_safe_goal_preflight_returns_none_when_a_wall_blocks_all_candidates(self):
        width = height = 16
        resolution = 0.25
        origin = (-2.0, -2.0)
        costs = [0] * (width * height)
        wall_x, _ = world_to_grid(
            0.5, 0.0, origin_x=origin[0], origin_y=origin[1],
            origin_yaw=0.0, resolution=resolution)
        for cell_y in range(height):
            costs[cell_y * width + wall_x] = 100

        selected = select_safe_goal_along_heading(
            costs, width, height, origin_x=origin[0], origin_y=origin[1],
            origin_yaw=0.0, resolution=resolution, start_x=0.0, start_y=0.0,
            heading=0.0, max_distance=1.0, min_distance=0.5,
            distance_step=0.25)

        self.assertIsNone(selected)

    def test_weighted_astar_prefers_longer_low_risk_route(self):
        width, height = 5, 3
        costs = [10] * (width * height)
        for x in (1, 2, 3):
            costs[1 * width + x] = 95
        path, total = weighted_astar(costs, width, height, (0, 1), (4, 1),
                                     cost_weight=2.0)
        self.assertTrue(path)
        self.assertTrue(any(y != 1 for _, y in path))
        self.assertLess(total, 10.0)

    def test_hard_occupancy_forces_detour_and_never_enters_blocked_cells(self):
        costs = [10] * 25
        costs[2 * 5 + 2] = 100
        path, _ = weighted_astar(costs, 5, 5, (0, 2), (4, 2))
        self.assertTrue(path)
        self.assertNotIn((2, 2), path)

    def test_diagonal_corner_cutting_is_forbidden(self):
        path, cost = weighted_astar([0, 100, 100, 0], 2, 2, (0, 0), (1, 1))
        self.assertEqual(path, [])
        self.assertEqual(cost, float("inf"))

    def test_existing_path_survives_safe_map_updates_but_not_new_blockers(self):
        path = [(0, 1), (1, 1), (2, 1)]
        costs = [20] * 9
        self.assertTrue(path_is_traversable(costs, 3, 3, path))
        costs[1 * 3 + 1] = 100
        self.assertFalse(path_is_traversable(costs, 3, 3, path))

    def test_existing_diagonal_path_rejects_new_corner_blocker(self):
        costs = [20] * 4
        costs[0 * 2 + 1] = 100
        self.assertFalse(path_is_traversable(costs, 2, 2, [(0, 0), (1, 1)]))

    def test_lidar_obstacle_grid_is_inflated_and_route_detours(self):
        terrain = [0] * 100
        obstacles = [100]
        merged = merge_obstacle_grid(
            terrain, 10, 10, origin_x=0.0, origin_y=0.0,
            origin_yaw=0.0, resolution=1.0, obstacles=obstacles,
            obstacle_width=1, obstacle_height=1,
            obstacle_origin_x=4.5, obstacle_origin_y=4.5,
            obstacle_origin_yaw=0.0, obstacle_resolution=1.0)
        self.assertEqual(terrain, [0] * 100)
        self.assertEqual(merged[5 * 10 + 5], 100)
        self.assertEqual(merged[5 * 10 + 6], 100)
        self.assertEqual(merged[0], 0)
        path, _ = weighted_astar(merged, 10, 10, (0, 5), (9, 5))
        self.assertTrue(path)
        self.assertNotIn((5, 5), path)

    def test_unknown_lidar_cells_do_not_block_the_terrain_map(self):
        merged = merge_obstacle_grid(
            [0] * 9, 3, 3, origin_x=0.0, origin_y=0.0,
            origin_yaw=0.0, resolution=1.0, obstacles=[-1],
            obstacle_width=1, obstacle_height=1,
            obstacle_origin_x=1.0, obstacle_origin_y=1.0,
            obstacle_origin_yaw=0.0, obstacle_resolution=1.0)
        self.assertEqual(merged, [0] * 9)

    def test_blocked_or_out_of_bounds_endpoints_are_rejected(self):
        costs = [0] * 9
        costs[0] = 100
        self.assertEqual(weighted_astar(costs, 3, 3, (0, 0), (2, 2))[0], [])
        self.assertEqual(weighted_astar(costs, 3, 3, (0, 1), (3, 2))[0], [])

    def test_unknown_is_caution_or_blocked_by_policy(self):
        costs = [0, -1, 0]
        path, _ = weighted_astar(costs, 3, 1, (0, 0), (2, 0),
                                 unknown_cost=80, unknown_is_blocked=False)
        self.assertEqual(path, [(0, 0), (1, 0), (2, 0)])
        blocked, _ = weighted_astar(costs, 3, 1, (0, 0), (2, 0),
                                    unknown_is_blocked=True)
        self.assertEqual(blocked, [])

    def test_map_revision_changes_with_data_and_geometry_not_time(self):
        args = dict(frame_id="map", width=2, height=2, resolution=0.1,
                    origin=(0, 0, 0, 0, 0, 0, 1), data=[0, 10, 80, 100])
        first = compute_map_revision(**args)
        later_same_map = compute_map_revision(**args)
        changed_data = compute_map_revision(**{**args, "data": [0, 10, 79, 100]})
        changed_origin = compute_map_revision(**{**args, "origin": (0.1, 0, 0, 0, 0, 0, 1)})
        same_rotation = compute_map_revision(**{**args, "origin": (0, 0, 0, 0, 0, 0, -1)})
        self.assertEqual(first, later_same_map)
        self.assertEqual(first, same_rotation)
        self.assertNotEqual(first, changed_data)
        self.assertNotEqual(first, changed_origin)

    def test_rotated_map_world_grid_round_trip(self):
        import math
        yaw = math.pi / 2
        cell = (3, 2)
        x, y = grid_to_world(cell, origin_x=-2.0, origin_y=4.0,
                             origin_yaw=yaw, resolution=0.5)
        self.assertEqual(world_to_grid(x, y, origin_x=-2.0, origin_y=4.0,
                                       origin_yaw=yaw, resolution=0.5), cell)

    def test_invalid_grid_values_and_dimensions_are_rejected(self):
        with self.assertRaises(ValueError):
            weighted_astar([0], 2, 2, (0, 0), (1, 1))
        with self.assertRaises(ValueError):
            weighted_astar([101], 1, 1, (0, 0), (0, 0))
        with self.assertRaises(ValueError):
            compute_map_revision(frame_id="map", width=1, height=1,
                                 resolution=0.1, origin=(0, 0, 0, 0, 0, 0, 0), data=[0])


if __name__ == "__main__":
    unittest.main()
