#!/usr/bin/env python3

from diagnostic_msgs.msg import DiagnosticArray
from rclpy.node import Node
from nav_msgs.msg import Odometry
import rclpy

LOOP_HZ = 10

class DummyOdomNode(Node):
    """
    This class produces a blank odom to fuse with robot_lozaliization so we can test just the IMU
    and display it.
    """

    def __init__(self):
        super().__init__("dummy_odom_node")
        self.get_logger().info("publishing blank odom")

        self.odom_publisher = self.create_publisher(Odometry, "odom", 10)
        self.create_timer(1.0 / LOOP_HZ, self.publish_odometry)

    def publish_odometry(self):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = 0.
        msg.pose.pose.position.y = 0.
        msg.pose.pose.position.z = 0.
        # msg.pose.pose.orientation.x = 0
        # msg.pose.pose.orientation.y = 0
        # msg.pose.pose.orientation.z = 0
        # msg.pose.pose.orientation.w = 0
        self.odom_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    try:
        node = DummyOdomNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
