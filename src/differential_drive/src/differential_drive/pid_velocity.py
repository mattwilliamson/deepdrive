#!/usr/bin/env python

import rclpy
from rclpy.node import Node

from std_msgs.msg import Int16
from std_msgs.msg import Float32
from numpy import array
from rclpy.constants import S_TO_NS

class PidVelocity(Node):
    """
    A class used to control the velocity of a motor using a PID controller.

    This class subscribes to the 'wheel_vtarget' topic which provides the target velocities for the wheels,
    and the 'wheel' topic for rotary encoder feedback. It uses a PID controller to adjust the motor speed
    to reach the target velocity. The PID controller uses the parameters Kp, Ki, and Kd which can be set
    as node parameters.

    Attributes
    ----------
    target : int
        The target velocity for the motor.
    motor : int
        The current velocity of the motor.
    vel : int
        The calculated velocity to be set on the motor.
    integral : float
        The integral term of the PID controller.
    error : float
        The current error between the target and actual velocity.
    derivative : float
        The derivative term of the PID controller.
    previous_error : float
        The previous error between the target and actual velocity.
    wheel_prev : int
        The previous wheel encoder count.
    wheel_latest : int
        The latest wheel encoder count.
    then : rclpy.time.Time
        The previous time the PID controller was updated.
    wheel_mult : int
        The multiplier for the wheel encoder count.
    prev_encoder : int
        The previous encoder count.
    Kp : float
        The proportional gain of the PID controller.
    Ki : float
        The integral gain of the PID controller.
    Kd : float
        The derivative gain of the PID controller.
    out_min : int
        The minimum output value for the motor.
    """

    def __init__(self):
        super().__init__("pid_velocity")

        self.target = 0.
        self.motor = 0.
        self.vel = 0.
        self.integral = 0.
        self.error = 0.
        self.derivative = 0.
        self.previous_error = 0.
        self.wheel_prev = 0.
        self.wheel_latest = 0.
        self.then = self.get_clock().now()
        self.wheel_mult = 0.
        self.prev_encoder = 0.

        self.Kp = self.declare_parameter("Kp", 10.0).value
        self.Ki = self.declare_parameter("Ki", 10.0).value
        self.Kd = self.declare_parameter("Kd", 0.001).value
        self.out_min = self.declare_parameter("out_min", -32768).value
        self.out_max = self.declare_parameter("out_max", 32768).value
        self.rate = self.declare_parameter("rate", 30).value
        self.rolling_pts = self.declare_parameter("rolling_pts", 2).value
        self.timeout_ticks = self.declare_parameter("timeout_ticks", 4).value
        self.ticks_per_meter = self.declare_parameter("ticks_meter", 20).value
        self.vel_threshold = self.declare_parameter("vel_threshold", 0.001).value
        self.encoder_min = self.declare_parameter("encoder_min", -32768).value
        self.encoder_max = self.declare_parameter("encoder_max", 32768).value
        self.encoder_low_wrap = self.declare_parameter(
            "wheel_low_wrap", (self.encoder_max - self.encoder_min) * 0.3 + self.encoder_min
        ).value
        self.encoder_high_wrap = self.declare_parameter(
            "wheel_high_wrap", (self.encoder_max - self.encoder_min) * 0.7 + self.encoder_min
        ).value
        self.prev_vel = [0.0] * self.rolling_pts
        self.wheel_latest = 0.0
        self.prev_pid_time = self.get_clock().now()

        self.create_subscription(Int16, "wheel", self.wheel_callback, 10)
        self.create_subscription(Float32, "wheel_vtarget", self.target_callback, 10)
        self.pub_motor = self.create_publisher(Int16, "motor_cmd", 10)
        self.pub_vel = self.create_publisher(Float32, "wheel_vel", 10)
        
        self.then = self.get_clock().now()
        self.ticks_since_target = self.timeout_ticks
        self.wheel_prev = self.wheel_latest
        self.then = self.get_clock().now()
        self.create_timer(1.0 / self.rate, self.update)

    def update(self):
        self.previous_error = 0.0
        self.prev_vel = [0.0] * self.rolling_pts
        self.integral = 0.0
        self.error = 0.0
        self.derivative = 0.0
        self.vel = 0.0

        self.get_logger().debug(f"update ticks_since_target: {self.ticks_since_target} timeout_ticks: {self.timeout_ticks}")

        # Perform the loop only if we have received a target velocity message recently
        if rclpy.ok() and self.ticks_since_target < self.timeout_ticks:
            self.calc_velocity()
            self.do_pid()
            self.pub_motor.publish(Int16(data=int(self.motor)))
            self.ticks_since_target += 1
            if self.ticks_since_target >= self.timeout_ticks:
                self.pub_motor.publish(Int16(data=0))

    def calc_velocity(self):
        self.dt_duration = self.get_clock().now() - self.then
        self.dt = self.dt_duration.nanoseconds / float(S_TO_NS)

        if self.wheel_latest == self.wheel_prev:
            # If we haven't received an updated wheel count recently, we estimate the current velocity by assuming that we just received a tick at this moment.
            cur_vel = (1.0 / self.ticks_per_meter) / self.dt
            if abs(cur_vel) < self.vel_threshold:
                # If the velocity is below the threshold, consider our velocity 0
                self.append_vel(0.0)
                self.calc_rolling_vel()
            else:
                if abs(cur_vel) < self.vel:
                    # We know we're slower than what we're currently publishing as a velocity
                    self.append_vel(cur_vel)
                    self.calc_rolling_vel()

        else:
            # We received a new wheel value
            cur_vel = (self.wheel_latest - self.wheel_prev) / self.dt
            self.append_vel(cur_vel)
            self.calc_rolling_vel()
            self.wheel_prev = self.wheel_latest
            self.then = self.get_clock().now()

        self.pub_vel.publish(Float32(data=float(self.vel)))

    def append_vel(self, val):
        self.prev_vel.append(val)
        del self.prev_vel[0]

    def calc_rolling_vel(self):
        p = array(self.prev_vel)
        self.vel = p.mean()

    def do_pid(self):
        """
        Performs PID control to calculate the motor output based on the target velocity and current velocity.

        Returns:
            None
        """

        pid_dt_duration = self.get_clock().now() - self.prev_pid_time
        pid_dt = pid_dt_duration.nanoseconds / float(S_TO_NS)
        self.prev_pid_time = self.get_clock().now()

        self.error = self.target - self.vel
        self.integral = self.integral + (self.error * pid_dt)
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

    def wheel_callback(self, msg):
        """
        Callback function for wheel encoder data.

        Args:
            msg: The wheel encoder data message.

        Returns:
            None
        """
        enc = msg.data
        if enc < self.encoder_low_wrap and self.prev_encoder > self.encoder_high_wrap:
            self.wheel_mult = self.wheel_mult + 1

        if enc > self.encoder_high_wrap and self.prev_encoder < self.encoder_low_wrap:
            self.wheel_mult = self.wheel_mult - 1

        self.wheel_latest = (
            1.0 * (enc + self.wheel_mult * (self.encoder_max - self.encoder_min)) / self.ticks_per_meter
        )
        self.prev_encoder = enc

    def target_callback(self, msg):
        """
        Callback function for handling target messages.

        Args:
            msg: The target message containing the desired target value.

        Returns:
            None
        """
        self.target = msg.data
        self.ticks_since_target = 0


def main(args=None):
    rclpy.init(args=args)
    try:
        pid_velocity = PidVelocity()
        rclpy.spin(pid_velocity)
    except rclpy.exceptions.ROSInterruptException:
        pass

    pid_velocity.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
