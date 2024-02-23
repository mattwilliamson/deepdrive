#!/usr/bin/env python

import rclpy
from rclpy.node import Node

from std_msgs.msg import Int16


class WheelScaler(Node):
    """
       wheel_scaler
       scales the wheel readings (and inverts the sign)
    """

    def __init__(self):
        super().__init__("wheel_scaler")

        self.get_logger().info("wheel_scaler started")

        self.scale = self.declare_parameter('distance_scale', 1).value
        self.get_logger().info("wheel_scaler scale: %0.2f" % self.scale)

        self.create_subscription(Int16, "lwheel", self.lwheel_callback, 10)
        self.create_subscription(Int16, "rwheel", self.rwheel_callback, 10)

        self.lscaled_pub = self.create_publisher(Int16, "lwheel_scaled", 10)
        self.rscaled_pub = self.create_publisher(Int16, "rwheel_scaled", 10)

        ### testing sleep CPU usage
        r = self.create_rate(50)
        while rclpy.ok():
            r.sleep()

        rclpy.spin()

    def lwheel_callback(self, msg):
        self.lscaled_pub.publish(msg.data * -1 * self.scale)

    def rwheel_callback(self, msg):
        self.rscaled_pub.publish(msg.data * -1 * self.scale)


def main(args=None):
    rclpy.init(args=args)

    try:
        wheel_scaler = WheelScaler()
    except rclpy.exceptions.ROSInterruptException:
        pass

    wheel_scaler.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
