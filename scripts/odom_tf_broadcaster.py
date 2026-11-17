#!/usr/bin/env python3
"""Publish odom->chassis from the ordered DiffDrive odometry stream."""
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import TransformBroadcaster


class OdometryTF(Node):
    def __init__(self):
        super().__init__('lunabot_odometry_tf')
        self.broadcaster = TransformBroadcaster(self)
        self.last_stamp = None
        self.create_subscription(Odometry, '/lunabot/odom', self.on_odom,
                                 qos_profile_sensor_data)

    def on_odom(self, msg):
        stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        if self.last_stamp is not None and stamp <= self.last_stamp:
            return
        if not msg.header.frame_id or not msg.child_frame_id:
            return
        transform = TransformStamped()
        transform.header = msg.header
        transform.child_frame_id = msg.child_frame_id
        transform.transform.translation.x = msg.pose.pose.position.x
        transform.transform.translation.y = msg.pose.pose.position.y
        transform.transform.translation.z = msg.pose.pose.position.z
        transform.transform.rotation = msg.pose.pose.orientation
        self.broadcaster.sendTransform(transform)
        self.last_stamp = stamp


def main():
    rclpy.init()
    node = OdometryTF()
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
