#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from std_msgs.msg import Int16
from rclpy.constants import S_TO_NS
from rclpy.duration import Duration

def d_to_s(duration: Duration) -> float:
    return float(duration.nanoseconds) / S_TO_NS

class WheelLoopback(Node):
    """
    WheelLoopback simulates a wheel for testing purposes.

    This class represents a simulated wheel that can be used for testing purposes. It receives motor commands
    and publishes the corresponding wheel position based on the velocity and time elapsed.

    Parameters:
    - rate: The rate at which the wheel loopback node operates (default: 200).
    - timeout_secs: The timeout in seconds for detecting the absence of motor commands (default: 0.5).
    - ticks_meter: The number of ticks per meter for calculating the wheel position (default: 50).
    - velocity_scale: The scaling factor for converting motor commands to wheel velocity (default: 32768).

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
        self.get_logger().info("started")

        self.rate = self.declare_parameter("rate", 200).value
        self.timeout_secs = self.declare_parameter("timeout_secs", 0.5).value
        self.ticks_meter = float(self.declare_parameter("ticks_meter", 50).value)
        # Might not need velocity scale
        self.velocity_scale = float(self.declare_parameter("velocity_scale", 1.0).value)
        self.latest_motor = 0.
        self.wheel = 0.
        self.velocity = 0.

        self.secs_since_target = self.timeout_secs
        self.then = self.get_clock().now()
        self.latest_msg_time = self.get_clock().now()

        self.create_subscription(Int16, "motor", self.motor_callback, 10)
        self.pub_wheel = self.create_publisher(Int16, "wheel", 10)

        self.create_timer(1.0 / self.rate, self.update)

        # Initial publish just to start things off
        self.pub_wheel.publish(Int16(data=0))
        

    def update(self):
        # self.get_logger().info(f"update self.velocity: {self.velocity} secs_since_target: {self.secs_since_target} timeout_secs: {self.timeout_secs}")

        if self.secs_since_target < self.timeout_secs:
            self.velocity = self.latest_motor * self.velocity_scale
            
            if abs(self.velocity) > 0:
                ticks_per_second = self.velocity * self.ticks_meter
                elapsed = d_to_s(self.get_clock().now() - self.then)
                self.wheel += ticks_per_second * elapsed
                self.pub_wheel.publish(Int16(data=int(self.wheel)))
                self.then = self.get_clock().now()
        else:
            # Timeout
            self.secs_since_target = d_to_s(self.get_clock().now() - self.latest_msg_time)
            self.velocity = 0.
            self.pub_wheel.publish(Int16(data=0))

    def motor_callback(self, msg):
        self.latest_motor = float(msg.data)
        self.latest_msg_time = self.get_clock().now()


def main(args=None):
    rclpy.init(args=args)
    try:
        wheel_loopback = WheelLoopback()
        rclpy.spin(wheel_loopback)
    except rclpy.exceptions.ROSInterruptException:
        pass

    wheel_loopback.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
