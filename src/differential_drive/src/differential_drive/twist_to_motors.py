#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist


class TwistToMotors(Node):
    """
    Converts Twist messages to motor commands for a differential drive robot.

    This class subscribes to the 'twist' topic and publishes the left and right wheel velocity targets
    to the 'lwheel_vtarget' and 'rwheel_vtarget' topics respectively. It calculates the left and right
    wheel velocities based on the linear and angular components of the Twist message.

    Attributes:
        w (float): The base width of the robot.
        dx (float): The linear velocity component of the Twist message.
        dr (float): The angular velocity component of the Twist message.
        ticks_since_target (int): The number of ticks since the last target was received.


    Topics:
        Subscribes to:
            - 'twist': The topic to receive Twist messages.

        Publishes to:
            - 'lwheel_vtarget': The topic to publish the left wheel velocity target.
            - 'rwheel_vtarget': The topic to publish the right wheel velocity target.
    """

    def __init__(self):
        super(TwistToMotors, self).__init__("twist_to_motors")

        self.w = self.declare_parameter("base_width", 0.2).value
        self.dx = 0
        self.dr = 0
        self.ticks_since_target = 0

        self.pub_lmotor = self.create_publisher(Float32, "lwheel_vtarget", 10)
        self.pub_rmotor = self.create_publisher(Float32, "rwheel_vtarget", 10)
        self.create_subscription(Twist, "twist", self.twist_callback, 10)

        self.rate_hz = self.declare_parameter("rate_hz", 50).value
        self.create_timer(1.0 / self.rate_hz, self.calculate_left_and_right_target)

    def calculate_left_and_right_target(self):
        right = Float32()
        left = Float32()

        right.data = 1.0 * self.dx + self.dr * self.w / 2.0
        left.data = 1.0 * self.dx - self.dr * self.w / 2.0

        self.pub_lmotor.publish(left)
        self.pub_rmotor.publish(right)

        self.ticks_since_target += 1

    def twist_callback(self, msg):
        self.ticks_since_target = 0
        self.dx = msg.linear.x
        self.dr = msg.angular.z


def main(args=None):
    rclpy.init(args=args)
    try:
        twist_to_motors = TwistToMotors()
        rclpy.spin(twist_to_motors)
    except rclpy.exceptions.ROSInterruptException:
        pass

    twist_to_motors.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
