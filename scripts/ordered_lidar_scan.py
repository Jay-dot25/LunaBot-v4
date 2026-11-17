#!/usr/bin/env python3
"""Keep delayed/out-of-order bridged scans out of the SLAM scan matcher."""
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class OrderedScan(Node):
    def __init__(self):
        super().__init__('lunabot_ordered_slam_scan')
        self.last_stamp = None
        self.dropped = 0
        self.publisher = self.create_publisher(
            LaserScan, '/lunabot/lidar/scan_slam', qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/lunabot/lidar/scan', self.on_scan,
                                 qos_profile_sensor_data)

    def on_scan(self, msg):
        stamp=(msg.header.stamp.sec, msg.header.stamp.nanosec)
        if self.last_stamp is not None and stamp <= self.last_stamp:
            self.dropped += 1
            if self.dropped == 1 or self.dropped % 100 == 0:
                self.get_logger().warning(
                    f'LIDAR_SCAN_ORDER_DROP count={self.dropped} stamp={stamp}')
            return
        self.last_stamp=stamp
        self.publisher.publish(msg)


def main():
    rclpy.init()
    node=OrderedScan()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
