#!/usr/bin/env python3
"""Record the operational Phase L ROS graph for offline comparison."""
import json
import math
import sys

import rclpy
import tf2_ros
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from phase_l_accuracy import quaternion_yaw


def pose(position, rotation):
    return [position.x, position.y,
            quaternion_yaw(rotation.x, rotation.y, rotation.z, rotation.w)]


class Recorder(Node):
    def __init__(self, output):
        super().__init__("lunabot_qualification_recorder")
        self.output = output
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer, self)
        self.last_slam_stamp = None
        self.create_subscription(Odometry, "/lunabot/odom", self.odom, qos_profile_sensor_data)
        self.create_subscription(LaserScan, "/lunabot/lidar/scan", self.scan, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, "/goal_pose", self.goal, 10)
        retained = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, "/lunabot/autonomy/status", self.status, retained)

    def write(self, kind, stamp, **values):
        print(json.dumps(dict(kind=kind, sim_time=stamp, **values), allow_nan=False),
              file=self.output, flush=True)

    def odom(self, message):
        stamp = Time.from_msg(message.header.stamp).nanoseconds / 1e9
        data = message.pose.pose
        self.write("odom", stamp, pose=pose(data.position, data.orientation))
        try:
            transform = self.buffer.lookup_transform("map", "chassis", Time())
        except tf2_ros.TransformException:
            return
        tf_stamp = Time.from_msg(transform.header.stamp).nanoseconds
        if tf_stamp > 0 and (self.last_slam_stamp is None or tf_stamp > self.last_slam_stamp):
            self.last_slam_stamp = tf_stamp
            data = transform.transform
            self.write("slam", tf_stamp / 1e9, pose=pose(data.translation, data.rotation))

    def scan(self, message):
        valid = sorted(r for r in message.ranges if math.isfinite(r)
                       and message.range_min <= r <= message.range_max)
        self.write("scan", Time.from_msg(message.header.stamp).nanoseconds / 1e9,
                   total_beams=len(message.ranges), valid_beams=len(valid),
                   near_minimum=sum(r <= message.range_min + 0.01 for r in valid),
                   median_range_m=valid[len(valid) // 2] if valid else None)

    def goal(self, message):
        self.write("goal", self.get_clock().now().nanoseconds / 1e9,
                   frame=message.header.frame_id,
                   position=[message.pose.position.x, message.pose.position.y])

    def status(self, message):
        if message.data.startswith("INTEGRATION_GOAL_REACHED "):
            self.write("reached", self.get_clock().now().nanoseconds / 1e9)


def main():
    rclpy.init(args=sys.argv[2:])
    with open(sys.argv[1], "w", encoding="utf-8") as output:
        node = Recorder(output)
        try:
            rclpy.spin(node)
        except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
            pass
        finally:
            node.destroy_node()
            rclpy.try_shutdown()


if __name__ == "__main__":
    main()
