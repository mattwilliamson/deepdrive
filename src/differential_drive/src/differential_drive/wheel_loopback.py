#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from std_msgs.msg import Int16


class WheelLoopback(Node):
    """
    WheelLoopback simulates a wheel for testing purposes.

    This class represents a simulated wheel that can be used for testing purposes. It receives motor commands
    and publishes the corresponding wheel position based on the velocity and time elapsed.

    Parameters:
    - rate: The rate at which the wheel loopback node operates (default: 200).
    - timeout_secs: The timeout in seconds for detecting the absence of motor commands (default: 0.5).
    - ticks_meter: The number of ticks per meter for calculating the wheel position (default: 50).
    - velocity_scale: The scaling factor for converting motor commands to wheel velocity (default: 255).

    Subscriptions:
    - motor: The motor command topic.

    Publications:
    - wheel: The wheel position topic.

    Usage:
    - Create an instance of the WheelLoopback class.
    - Call the spin() method to start the wheel loopback node.

    Example:
    ```python
    wheel_loopback = WheelLoopback()
    wheel_loopback.spin()
    ```

    """

    def __init__(self):
        super().__init__("wheel_loopback")
        self.nodename = "wheel_loopback"
        self.get_logger().info("%s started" % self.nodename)

        self.rate = self.declare_parameter("rate", 200).value
        self.timeout_secs = self.declare_parameter("timeout_secs", 0.5).value
        self.ticks_meter = float(self.declare_parameter("ticks_meter", 50).value)
        self.velocity_scale = float(self.declare_parameter("velocity_scale", 255).value)
        self.latest_motor = 0
        self.wheel = 0

        self.create_subscription(Int16, "motor", self.motor_callback, 10)
        self.pub_wheel = self.create_publisher(Int16, "wheel", 10)

    def spin(self):
        r = self.create_rate(self.rate)
        self.secs_since_target = self.timeout_secs
        self.then = self.get_clock().now()
        self.latest_msg_time = self.get_clock().now()
        self.get_logger().info("-D- spinning")

        while rclpy.ok():
            while rclpy.ok() and self.secs_since_target < self.timeout_secs:
                self.spin_once()
                r.sleep()
                self.secs_since_target = self.get_clock().now() - self.latest_msg_time
                self.secs_since_target = self.secs_since_target.to_msg().sec

            self.secs_since_target = self.get_clock().now() - self.latest_msg_time
            self.secs_since_target = self.secs_since_target.to_msg().sec
            self.velocity = 0
            r.sleep()

    def spin_once(self):
        self.velocity = self.latest_motor / self.velocity_scale
        if abs(self.velocity) > 0:
            self.seconds_per_tick = abs(1 / (self.velocity * self.ticks_meter))
            elapsed = self.get_clock().now() - self.then
            elapsed = elapsed.to_msg().sec
            self.get_logger().info(
                "spin_once: vel=%0.3f sec/tick=%0.3f elapsed:%0.3f" % (self.velocity, self.seconds_per_tick, elapsed)
            )

            if elapsed > self.seconds_per_tick:
                self.get_logger().info("incrementing wheel")
                if self.velocity > 0:
                    self.wheel += 1
                else:
                    self.wheel -= 1
                self.pub_wheel.publish(self.wheel)
                self.then = self.get_clock().now()

    def motor_callback(self, msg):
        self.latest_motor = msg.data
        self.latest_msg_time = self.get_clock().now()


def main(args=None):
    rclpy.init(args=args)
    try:
        wheel_loopback = WheelLoopback()
        wheel_loopback.spin()
    except rclpy.exceptions.ROSInterruptException:
        pass

    wheel_loopback.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
