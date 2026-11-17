#!/usr/bin/env python3
"""Wait for a non-empty OccupancyGrid and report its observed-cell bounds."""
import argparse
import time

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node


class MapVerifier(Node):
    def __init__(self, minimum_known):
        super().__init__('lunabot_map_content_verifier')
        self.minimum_known = minimum_known
        self.ready = False
        self.last = None
        self.create_subscription(OccupancyGrid, '/map', self.on_map, 10)

    def on_map(self, msg):
        values = list(msg.data)
        known = sum(value >= 0 for value in values)
        occupied = sum(value >= 50 for value in values)
        self.last = {
            'frame': msg.header.frame_id,
            'width': msg.info.width,
            'height': msg.info.height,
            'resolution': msg.info.resolution,
            'cells': len(values),
            'known': known,
            'occupied': occupied,
        }
        if self.last['frame'] == 'map' and known >= self.minimum_known:
            self.ready = True


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--minimum-known-cells', type=int, default=1)
    parser.add_argument('--timeout-sec', type=float, default=20.0)
    args = parser.parse_args()
    if args.minimum_known_cells < 1 or args.timeout_sec <= 0:
        parser.error('minimum-known-cells and timeout-sec must be positive')
    rclpy.init()
    node = MapVerifier(args.minimum_known_cells)
    deadline = time.monotonic() + args.timeout_sec
    try:
        while rclpy.ok() and not node.ready and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
        print(node.last, flush=True)
        if not node.ready:
            print(f'FAIL: no map with >= {args.minimum_known_cells} known cells', flush=True)
            return 1
        print('PASS: non-empty OccupancyGrid received', flush=True)
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
