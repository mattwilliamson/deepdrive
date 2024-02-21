#!/usr/bin/env python

# Copyright (C) 2012 Jon Stephan.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

import rclpy
from rclpy.node import Node

from std_msgs.msg import Int16
from std_msgs.msg import Float32
from numpy import array


class PidVelocity(Node):
    """
       pid_velocity - takes messages on wheel_vtarget
      target velocities for the wheels and monitors wheel for feedback
    """

    def __init__(self):
        super().__init__("pid_velocity")
        self.nodename = "pid_velocity"
        self.get_logger().info(f"{self.nodename} started")

        ### initialize variables
        self.target = 0
        self.motor = 0
        self.vel = 0
        self.integral = 0
        self.error = 0
        self.derivative = 0
        self.previous_error = 0
        self.wheel_prev = 0
        self.wheel_latest = 0
        self.then = self.get_clock().now()
        self.wheel_mult = 0
        self.prev_encoder = 0

        ### get parameters #### 
        self.Kp = self.declare_parameter('Kp', 10).value
        self.Ki = self.declare_parameter('Ki', 10).value
        self.Kd = self.declare_parameter('Kd', 0.001).value
        self.out_min = self.declare_parameter('out_min', -255).value
        self.out_max = self.declare_parameter('out_max', 255).value
        self.rate = self.declare_parameter('rate', 30).value
        self.rolling_pts = self.declare_parameter('rolling_pts', 2).value
        self.timeout_ticks = self.declare_parameter('timeout_ticks', 4).value
        self.ticks_per_meter = self.declare_parameter('ticks_meter', 20).value
        self.vel_threshold = self.declare_parameter('vel_threshold', 0.001).value
        self.encoder_min = self.declare_parameter('encoder_min', -32768).value
        self.encoder_max = self.declare_parameter('encoder_max', 32768).value
        self.encoder_low_wrap = self.declare_parameter('wheel_low_wrap', (
                    self.encoder_max - self.encoder_min) * 0.3 + self.encoder_min).value
        self.encoder_high_wrap = self.declare_parameter('wheel_high_wrap', (
                    self.encoder_max - self.encoder_min) * 0.7 + self.encoder_min).value
        self.prev_vel = [0.0] * self.rolling_pts
        self.wheel_latest = 0.0
        self.prev_pid_time = self.get_clock().now()
        self.get_logger().debug("%s got Kp:%0.3f Ki:%0.3f Kd:%0.3f tpm:%0.3f" % (
            self.nodename, self.Kp, self.Ki, self.Kd, self.ticks_per_meter))

        #### subscribers/publishers 
        self.create_subscription(Int16, 'wheel', self.wheel_callback, 10)
        self.create_subscription(Float32, 'wheel_vtarget', self.target_callback, 10)
        self.pub_motor = self.create_publisher(Float32, 'motor_cmd', 10)
        self.pub_vel = self.create_publisher(Float32, 'wheel_vel', 10)

    def spin(self):
        self.r = self.create_rate(self.rate)
        self.then = self.get_clock().now()
        self.ticks_since_target = self.timeout_ticks
        self.wheel_prev = self.wheel_latest
        self.then = self.get_clock().now()
        while rclpy.ok():
            self.spin_once()
            self.r.sleep()

    def spin_once(self):
        self.previous_error = 0.0
        self.prev_vel = [0.0] * self.rolling_pts
        self.integral = 0.0
        self.error = 0.0
        self.derivative = 0.0
        self.vel = 0.0

        # only do the loop if we've recently recieved a target velocity message
        while rclpy.ok() and self.ticks_since_target < self.timeout_ticks:
            self.calc_velocity()
            self.do_pid()
            self.pub_motor.publish(self.motor)
            self.r.sleep()
            self.ticks_since_target += 1
            if self.ticks_since_target == self.timeout_ticks:
                self.pub_motor.publish(0)

    def calc_velocity(self):
        self.dt_duration = self.get_clock().now() - self.then
        self.dt = self.dt_duration.to_sec()
        self.get_logger().debug("-D- %s caclVelocity dt=%0.3f wheel_latest=%0.3f wheel_prev=%0.3f" % (
            self.nodename, self.dt, self.wheel_latest, self.wheel_prev))

        if self.wheel_latest == self.wheel_prev:
            # we haven't received an updated wheel lately
            cur_vel = (1 / self.ticks_per_meter) / self.dt  # if we got a tick right now, this would be the velocity
            if abs(cur_vel) < self.vel_threshold:
                # if the velocity is < threshold, consider our velocity 0
                self.get_logger().debug("-D- %s below threshold cur_vel=%0.3f vel=0" % (self.nodename, cur_vel))
                self.appendVel(0)
                self.calcRollingVel()
            else:
                self.get_logger().debug("-D- %s above threshold cur_vel=%0.3f" % (self.nodename, cur_vel))
                if abs(cur_vel) < self.vel:
                    self.get_logger().debug("-D- %s cur_vel < self.vel" % self.nodename)
                    # we know we're slower than what we're currently publishing as a velocity
                    self.append_vel(cur_vel)
                    self.calc_rolling_vel()

        else:
            # we received a new wheel value
            cur_vel = (self.wheel_latest - self.wheel_prev) / self.dt
            self.append_vel(cur_vel)
            self.calc_rolling_vel()
            self.get_logger().debug("-D- %s **** wheel updated vel=%0.3f **** " % (self.nodename, self.vel))
            self.wheel_prev = self.wheel_latest
            self.then = self.get_clock().now()

        self.pub_vel.publish(self.vel)

    def append_vel(self, val):
        self.prev_vel.append(val)
        del self.prev_vel[0]

    def calc_rolling_vel(self):
        p = array(self.prev_vel)
        self.vel = p.mean()

    def do_pid(self):
        pid_dt_duration = self.get_clock().now() - self.prev_pid_time
        pid_dt = pid_dt_duration.to_sec()
        self.prev_pid_time = self.get_clock().now()

        self.error = self.target - self.vel
        self.integral = self.integral + (self.error * pid_dt)
        # rospy.loginfo("i = i + (e * dt):  %0.3f = %0.3f + (%0.3f * %0.3f)" % (self.integral, self.integral, self.error, pid_dt))
        self.derivative = (self.error - self.previous_error) / pid_dt
        self.previous_error = self.error

        self.motor = (self.Kp * self.error) + (self.Ki * self.integral) + (self.Kd * self.derivative)

        if self.motor > self.out_max:
            self.motor = self.out_max
            self.integral = self.integral - (self.error * pid_dt)
        if self.motor < self.out_min:
            self.motor = self.out_min
            self.integral = self.integral - (self.error * pid_dt)

        if self.target == 0:
            self.motor = 0

        self.get_logger().debug("vel:%0.2f tar:%0.2f err:%0.2f int:%0.2f der:%0.2f ## motor:%d " %
                       (self.vel, self.target, self.error, self.integral, self.derivative, self.motor))

    def wheel_callback(self, msg):
        enc = msg.data
        if enc < self.encoder_low_wrap and self.prev_encoder > self.encoder_high_wrap:
            self.wheel_mult = self.wheel_mult + 1

        if enc > self.encoder_high_wrap and self.prev_encoder < self.encoder_low_wrap:
            self.wheel_mult = self.wheel_mult - 1

        self.wheel_latest = 1.0 * (
                enc + self.wheel_mult * (self.encoder_max - self.encoder_min)) / self.ticks_per_meter
        self.prev_encoder = enc

    #        rospy.logdebug("-D- %s wheelCallback msg.data= %0.3f wheel_latest = %0.3f mult=%0.3f" % (self.nodename, enc, self.wheel_latest, self.wheel_mult))

    def target_callback(self, msg):
        self.target = msg.data
        self.ticks_since_target = 0
        # rospy.logdebug("-D- %s targetCallback " % (self.nodename))


def main(args=None):
    rclpy.init(args=args)
    try:
        pid_velocity = PidVelocity()
        pid_velocity.spin()
    except rclpy.exceptions.ROSInterruptException:
        pass

    pid_velocity.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
