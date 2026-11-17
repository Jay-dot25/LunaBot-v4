#!/usr/bin/env python3
"""Print the live P8 goal's map-frame cost and obstacle neighborhood."""
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage


def grid_cell(grid, x, y):
    origin = grid.info.origin
    q = origin.orientation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                     1 - 2 * (q.y * q.y + q.z * q.z))
    dx, dy = x - origin.position.x, y - origin.position.y
    gx = math.floor((math.cos(yaw) * dx + math.sin(yaw) * dy) /
                    grid.info.resolution)
    gy = math.floor((-math.sin(yaw) * dx + math.cos(yaw) * dy) /
                    grid.info.resolution)
    if 0 <= gx < grid.info.width and 0 <= gy < grid.info.height:
        return gx, gy, grid.data[gy * grid.info.width + gx]
    return gx, gy, None


class Probe(Node):
    def __init__(self):
        super().__init__("p8_live_goal_probe")
        qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.grids = {}
        self.goal = None
        self.map_to_odom = None
        self.create_subscription(
            OccupancyGrid, "/lunabot/terrain/cost_map",
            lambda msg: self.grids.__setitem__("cost", msg), qos)
        self.create_subscription(
            OccupancyGrid, "/lunabot/obstacles/map",
            lambda msg: self.grids.__setitem__("obstacle", msg), qos)
        self.create_subscription(PoseStamped, "/goal_pose",
                                 lambda msg: setattr(self, "goal", msg), 10)
        self.create_subscription(TFMessage, "/tf", self.on_tf, 10)

    def on_tf(self, msg):
        for transform in msg.transforms:
            if (transform.header.frame_id, transform.child_frame_id) == (
                    "map", "odom"):
                q = transform.transform.rotation
                yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                                 1 - 2 * (q.y * q.y + q.z * q.z))
                self.map_to_odom = (transform.transform.translation.x,
                                    transform.transform.translation.y, yaw)


rclpy.init()
node = Probe()
deadline = time.monotonic() + 8
while time.monotonic() < deadline and (
        len(node.grids) < 2 or node.goal is None or node.map_to_odom is None):
    rclpy.spin_once(node, timeout_sec=0.1)
if node.goal is None or len(node.grids) < 2 or node.map_to_odom is None:
    print("P8_GOAL_GRID_PROBE_INCOMPLETE",
          "goal=", node.goal is not None,
          "grids=", sorted(node.grids),
          "map_to_odom=", node.map_to_odom)
else:
    x, y = node.goal.pose.position.x, node.goal.pose.position.y
    if node.goal.header.frame_id == "odom":
        tx, ty, yaw = node.map_to_odom
        x, y = tx + math.cos(yaw) * x - math.sin(yaw) * y, \
            ty + math.sin(yaw) * x + math.cos(yaw) * y
    print("goal_frame=", node.goal.header.frame_id,
          "goal_map=", (x, y), "map_to_odom=", node.map_to_odom)
    cells = {name: grid_cell(grid, x, y)
             for name, grid in node.grids.items()}
    print("goal_cells=", cells)
    for offset in (0.0, -0.5, -1.0, -1.5, 0.5):
        candidate_x, candidate_y = x, y + offset
        print("south_candidate_map=", offset,
              {name: grid_cell(grid, candidate_x, candidate_y)
               for name, grid in node.grids.items()})
    gx, gy, _ = cells["cost"]
    for row in range(gy - 3, gy + 4):
        values = []
        for col in range(gx - 3, gx + 4):
            cost = node.grids["cost"].data[
                row * node.grids["cost"].info.width + col]
            obstacle = node.grids["obstacle"].data[
                row * node.grids["obstacle"].info.width + col]
            values.append(f"{cost}/{obstacle}")
        print(" ".join(values))
node.destroy_node()
rclpy.shutdown()
