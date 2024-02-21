#!/usr/bin/env python
#   Copyright 2012 Jon Stephan
#   jfstepha@gmail.com
#
#   This program is free software: you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation, either version 3 of the License, or
#   (at your option) any later version.
#
#   This program is distributed in the hope that it will be useful,
#   but WITHOUT ANY WARRANTY; without even the implied warranty of
#   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#   GNU General Public License for more details.
#
#   You should have received a copy of the GNU General Public License
#   along with this program.  If not, see <http://www.gnu.org/licenses/>.

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from std_msgs.msg import Int16


class WheelLoopback(Node):
    """
    wheel_loopback - simulates a wheel - just for testing

    """

    def __init__(self):
        super().__init__("wheel_loopback")
        self.nodename = "wheel_loopback"
        self.get_logger().info("%s started" % self.nodename)

        self.rate = self.declare_parameter("rate", 200).value
        self.timeout_secs = self.declare_parameter("timeout_secs", 0.5).value
        self.ticks_meter = float(self.declare_parameter('ticks_meter', 50).value)
        self.velocity_scale = float(self.declare_parameter('velocity_scale', 255).value)
        self.latest_motor = 0

        self.wheel = 0

        self.create_subscription(Int16, 'motor', self.motor_callback, 10)

        self.pub_wheel = self.create_publisher(Int16, 'wheel', 10)

    def spin(self):
        r = self.create_rate(self.rate)
        self.secs_since_target = self.timeout_secs
        self.then = self.get_clock().now()
        self.latest_msg_time = self.get_clock().now()
        self.get_logger().info("-D- spinning")

        ###### main loop #########
        while rclpy.ok():
            while rclpy.ok() and self.secs_since_target < self.timeout_secs:
                self.spin_once()
                r.sleep
                self.secs_since_target = self.get_clock().now() - self.latest_msg_time
                self.secs_since_target = self.secs_since_target.to_msg().sec
                # rospy.loginfo("  inside: secs_since_target: %0.3f" % self.secs_since_target)

            # it's been more than timeout_ticks since we recieved a message
            self.secs_since_target = self.get_clock().now() - self.latest_msg_time
            self.secs_since_target = self.secs_since_target.to_msg().sec
            # rospy.loginfo("  outside: secs_since_target: %0.3f" % self.secs_since_target)
            self.velocity = 0
            r.sleep

    def spin_once(self):
        self.velocity = self.latest_motor / self.velocity_scale
        if abs(self.velocity) > 0:
            self.seconds_per_tick = abs(1 / (self.velocity * self.ticks_meter))
            elapsed = self.get_clock().now() - self.then
            elapsed = elapsed.to_msg().sec
            self.get_logger().info(
                "spin_once: vel=%0.3f sec/tick=%0.3f elapsed:%0.3f" % (self.velocity, self.seconds_per_tick, elapsed))

            if elapsed > self.seconds_per_tick:
                self.get_logger().info("incrementing wheel")
                if self.velocity > 0:
                    self.wheel += 1
                else:
                    self.wheel -= 1
                self.pub_wheel.publish(self.wheel)
                self.then = self.get_clock().now()

    def motor_callback(self, msg):
        # rospy.loginfo("%s recieved %d" % (self.nodename, msg.data))
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


if __name__ == '__main__':
    main()
