#!/usr/bin/env python3
import math
import time

import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage


class Probe(Node):
    def __init__(self):
        super().__init__("p8_grid_probe")
        qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.grids = {}
        self.odom = None
        self.tf = None
        self.map = None
        self.create_subscription(
            OccupancyGrid, "/lunabot/terrain/cost_map",
            lambda msg: self.grids.__setitem__("cost", msg), qos)
        self.create_subscription(
            OccupancyGrid, "/lunabot/obstacles/map",
            lambda msg: self.grids.__setitem__("obstacle", msg), qos)
        self.create_subscription(
            OccupancyGrid, "/map", lambda msg: setattr(self, "map", msg), qos)
        self.create_subscription(
            Odometry, "/lunabot/odom", lambda msg: setattr(self, "odom", msg), 10)
        self.create_subscription(TFMessage, "/tf", self.on_tf, 10)

    def on_tf(self, msg):
        for transform in msg.transforms:
            if (transform.header.frame_id, transform.child_frame_id) == ("map", "odom"):
                q = transform.transform.rotation
                yaw = math.atan2(2 * (q.w*q.z + q.x*q.y),
                                 1 - 2 * (q.y*q.y + q.z*q.z))
                self.tf = (transform.transform.translation.x,
                           transform.transform.translation.y, yaw)


def cell(grid, x, y):
    origin = grid.info.origin
    q = origin.orientation
    angle = math.atan2(2*q.w*q.z, 1 - 2*q.y*q.y - 2*q.z*q.z)
    dx, dy = x-origin.position.x, y-origin.position.y
    gx = math.floor((math.cos(angle)*dx + math.sin(angle)*dy) / grid.info.resolution)
    gy = math.floor((-math.sin(angle)*dx + math.cos(angle)*dy) / grid.info.resolution)
    if 0 <= gx < grid.info.width and 0 <= gy < grid.info.height:
        return gx, gy, grid.data[gy*grid.info.width+gx]
    return gx, gy, None


rclpy.init()
node = Probe()
deadline = time.monotonic() + 6
while (time.monotonic() < deadline and
       (len(node.grids) < 2 or node.map is None or node.odom is None or node.tf is None)):
    rclpy.spin_once(node, timeout_sec=0.1)
print("tf", node.tf, "grids", list(node.grids))
if node.odom is not None and node.tf is not None:
    pose = node.odom.pose.pose.position
    x = node.tf[0] + math.cos(node.tf[2])*pose.x - math.sin(node.tf[2])*pose.y
    y = node.tf[1] + math.sin(node.tf[2])*pose.x + math.cos(node.tf[2])*pose.y
    print("current_map", (x, y))
    for label, center in (("habitat_goal_cell", (82, 82)),
                          ("science_goal_cell", (88, 86)),
                          ("science_candidate_1_0_0", (87, 78)),
                          ("science_candidate_1_0_25", (87, 80)),
                          ("science_candidate_0_75_0", (85, 78)),
                          ("science_candidate_0_5_0", (82, 78)),
                          ("science_candidate_1_0_minus_0_5", (85, 73)),
                          ("science_candidate_1_0_minus_1_0", (85, 68)),
                          ("habitat_candidate_2_0_1_0", (96, 89)),
                          ("habitat_candidate_1_0_minus_1_0", (86, 69)),
                          ("science_candidate_2_0_minus_1_0", (96, 69)),
                          ("current_habitat_goal", (109, 27))):
        print(label, "cost/obstacle/map")
        for gy in range(center[1]-2, center[1]+3):
            row=[]
            for gx in range(center[0]-2, center[0]+3):
                def raw(grid):
                    if 0 <= gx < grid.info.width and 0 <= gy < grid.info.height:
                        return grid.data[gy*grid.info.width+gx]
                    return None
                row.append(f"{raw(node.grids['cost'])}/{raw(node.grids['obstacle'])}/{raw(node.map)}")
            print(" ".join(row))
    for label, point in (("current", (x, y)), ("habitat", (1.0, 2.0)),
                         ("science", (2.5, 1.5)),
                         ("habitat_map_goal", (1.7782026166, -7.1650537649)),
                         ("habitat_goal_west_1m", (2.45, -7.905)),
                         ("habitat_goal_east_1m", (1.105, -6.425)),
                         ("science_map_goal", (-5.82, 1.19)),
                         ("habitat_goal_south", (3.19, -7.23)),
                         ("science_goal_4_8", (-5.82, 1.19)),
                         ("science_goal_4_3", (-5.48, 1.53)),
                         ("science_goal_4_0", (-5.28, 1.75))):
        values = {key: cell(grid, *point) for key, grid in node.grids.items()}
        values["map"] = cell(node.map, *point)
        print(label, values)
    print("habitat_neighborhood x,y cost map obstacle")
    for yi in range(3, 11):
        row = []
        yy = yi * 0.5
        for xi in range(0, 5):
            xx = xi * 0.5
            cost = cell(node.grids["cost"], xx, yy)[2]
            occupancy = cell(node.map, xx, yy)[2]
            obstacle = cell(node.grids["obstacle"], xx, yy)[2]
            row.append(f"{xx:.1f},{yy:.1f}:{cost}/{occupancy}/{obstacle}")
        print(" ".join(row))
    print("science_neighborhood x,y cost map")
    for yi in range(3, 11):
        row = []
        yy = yi * 0.5
        for xi in range(5, 12):
            xx = xi * 0.5
            cost = cell(node.grids["cost"], xx, yy)[2]
            occupancy = cell(node.map, xx, yy)[2]
            obstacle = cell(node.grids["obstacle"], xx, yy)[2]
            row.append(f"{xx:.1f},{yy:.1f}:{cost}/{occupancy}/{obstacle}")
        print(" ".join(row))
node.destroy_node()
rclpy.shutdown()
