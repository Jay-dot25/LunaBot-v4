#!/usr/bin/env python3
"""Bounded live Phase D probe: retain map, health, controller and motion evidence."""
import json
import sys
import time
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import tf2_ros
from rclpy.time import Time
from astar_navigation import AStarNavigation

rclpy.init()
n = Node('phase_d_runtime_probe')
state = {}
events = []
def receive(name):
    def cb(msg):
        state[name] = msg
        if name in ('control', 'nav', 'health'):
            events.append({'wall': time.monotonic(), 'topic': name, 'data': msg.data})
    return cb
retained = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
subs = [n.create_subscription(OccupancyGrid, '/map', receive('map'), retained),
        n.create_subscription(Odometry, '/lunabot/odom', receive('odom'), qos_profile_sensor_data),
        n.create_subscription(String, '/lunabot/navigation/status', receive('nav'), retained),
        n.create_subscription(String, '/lunabot/control/status', receive('control'), 10),
        n.create_subscription(String, '/lunabot/safety/health', receive('health'), 1)]
buf = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(buf, n)
pub = n.create_publisher(PoseStamped, '/goal_pose', 10)
deadline = time.monotonic() + 120
goal = None
initial = None
minimum = float(sys.argv[2]) if len(sys.argv) > 2 else .45
maximum = float(sys.argv[3]) if len(sys.argv) > 3 else .65
reached_at = None
goal_accepted = False
while time.monotonic() < deadline:
    rclpy.spin_once(n, timeout_sec=.05)
    if goal is None and all(k in state for k in ('map', 'odom', 'health')):
        try:
            tf = buf.lookup_transform('map', 'chassis', Time())
        except tf2_ros.TransformException:
            continue
        nav = AStarNavigation.__new__(AStarNavigation)
        nav.map_msg = state['map']; nav.inflation_radius = .25
        nav.occupied_threshold = 65; nav.unknown_is_obstacle = True
        x, y = tf.transform.translation.x, tf.transform.translation.y
        start = nav._nearest_free(nav._world_to_grid(x, y))
        if start is None:
            continue
        reachable = {start}; todo = [start]
        for cell in todo:
            for dx, dy in ((1,0),(-1,0),(0,1),(0,-1)):
                nxt = (cell[0]+dx, cell[1]+dy)
                if nxt not in reachable and nav._safe_cell(nxt):
                    reachable.add(nxt); todo.append(nxt)
        candidates = [(c, nav._grid_to_world(c)) for c in reachable]
        candidates = [(c,p) for c,p in candidates if minimum < ((p[0]-x)**2+(p[1]-y)**2)**.5 < maximum]
        if not candidates:
            print('No reachable goal in requested distance interval', flush=True)
            break
        cell, point = min(candidates, key=lambda cp: abs(cp[1][1]-y))
        goal = PoseStamped(); goal.header.frame_id = 'map'
        deadline = time.monotonic() + 60
        goal.pose.position.x, goal.pose.position.y = point
        goal.pose.orientation.w = 1.
        initial = (state['odom'].pose.pose.position.x, state['odom'].pose.pose.position.y)
        print(json.dumps({'start': [x,y], 'snapped_start':nav._grid_to_world(start), 'goal':point, 'reachable_cells':len(reachable)}), flush=True)
        pub.publish(goal)
    if goal is not None and 'nav' in state and state['nav'].data == (
            f'MANUAL_GOAL_SELECTED frame=map x={goal.pose.position.x:.2f} y={goal.pose.position.y:.2f}'):
        goal_accepted = True
    if goal_accepted and state['nav'].data.startswith('GOAL_REACHED'):
        if reached_at is None:
            reached_at = time.monotonic()
    if reached_at is not None and time.monotonic() - reached_at > 1.0:
        break
    if goal is not None and ('nav' not in state or state['nav'].data.startswith('PLANNER_WAITING_FOR_GOAL')):
        pub.publish(goal)
out = {'goal': None if goal is None else [goal.pose.position.x, goal.pose.position.y], 'goal_accepted':goal_accepted, 'goal_reached':reached_at is not None, 'initial_odom': initial, 'events':events}
if 'odom' in state:
    p = state['odom'].pose.pose.position
    out['final_odom'] = [p.x,p.y]
Path(sys.argv[1]).write_text(json.dumps(out, indent=2))
print(json.dumps({k:v for k,v in out.items() if k != 'events'}), flush=True)
for key in ('nav','control','health'):
    print(key, state[key].data if key in state else 'MISSING', flush=True)
n.destroy_node(); rclpy.shutdown()
