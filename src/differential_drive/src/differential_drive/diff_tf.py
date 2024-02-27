#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from math import sin, cos, pi

from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from std_msgs.msg import Int16
from rclpy.constants import S_TO_NS



class DiffTf(Node):
    """
    A class that represents a differential drive transformation node.

    This node calculates the odometry of a differential drive robot based on wheel encoder readings.
    It publishes the odometry information and transforms between the base frame and the odometry frame.

    Parameters:
        rate_hz (float): The rate at which to publish the transform.
        ticks_meter (float): The number of wheel encoder ticks per meter of travel.
        wheel_radius (float): The radius of the wheel in meters.
        base_width (float): The wheel base width in meters. Make this bigger to account for slippage.
        base_frame_id (str): The name of the base frame of the robot.
        odom_frame_id (str): The name of the odometry reference frame.
        encoder_min (int): The minimum value of the wheel encoder.
        encoder_max (int): The maximum value of the wheel encoder.
        encoder_low_wrap (float): The lower wrap value of the wheel encoder.
        encoder_high_wrap (float): The higher wrap value of the wheel encoder.
        left_wheel_frames (List[str]): The names of the left wheel frames. Parent assumed to be base_frame_id.
        right_wheel_frames (List[str]): The names of the right wheel frames. Parent assumed to be base_frame_id.


    Attributes:
        enc_left (float): Wheel encoder reading for the left wheel.
        enc_right (float): Wheel encoder reading for the right wheel.
        left (float): Actual value coming back from the left wheel.
        right (float): Actual value coming back from the right wheel.
        lmult (float): Left wheel multiplier.
        rmult (float): Right wheel multiplier.
        prev_lencoder (int): Previous left wheel encoder reading.
        prev_rencoder (int): Previous right wheel encoder reading.
        x (float): Position in the xy plane.
        y (float): Position in the xy plane.
        th (float): Orientation angle.
        dx (float): Speed in x direction.
        dr (float): Speed in rotation.
        then (Time): Time of the previous update.

    Subscriptions:
        lwheel (Int16): Left wheel encoder reading.
        rwheel (Int16): Right wheel encoder reading.

    Publishers:
        odom (Odometry): Odometry information.

    """

    def __init__(self):
        super().__init__("diff_tf")

        self.rate_hz = self.declare_parameter("rate_hz", 10.0).value  # the rate at which to publish the transform
        self.create_timer(1.0 / self.rate_hz, self.update)

        # The number of wheel encoder ticks per meter of travel
        self.ticks_meter = float(self.declare_parameter("ticks_meter", 50).value)

        # The radius of the wheel in meters
        self.wheel_radius = float(self.declare_parameter("wheel_radius", 0.1).value)

        # The wheel base width in meters
        self.base_width = float(self.declare_parameter("base_width", 0.245).value)

        # the name of the base frame of the robot
        self.base_frame_id = self.declare_parameter("base_frame_id", "base_footprint").value
        # the name of the odometry reference frame
        self.odom_frame_id = self.declare_parameter("odom_frame_id", "odom").value

        self.encoder_min = self.declare_parameter("encoder_min", -32768).value
        self.encoder_max = self.declare_parameter("encoder_max", 32768).value
        self.encoder_low_wrap = self.declare_parameter(
            "wheel_low_wrap",
            (self.encoder_max - self.encoder_min) * 0.3 + self.encoder_min,
        ).value
        self.encoder_high_wrap = self.declare_parameter(
            "wheel_high_wrap",
            (self.encoder_max - self.encoder_min) * 0.7 + self.encoder_min,
        ).value
        # the names of the wheel frames
        self.left_wheel_frames = self.declare_parameter("left_wheel_frames", ["left_wheel"]).value
        self.right_wheel_frames = self.declare_parameter("right_wheel_frames", ["right_wheel"]).value

        # internal data
        self.enc_left = None  # Wheel encoder readings
        self.enc_right = None
        self.left = 0.0  # Actual values coming back from robot
        self.right = 0.0
        self.lmult = 0.0  # Left wheel multiplier
        self.rmult = 0.0  # Right wheel multiplier
        self.prev_lencoder = 0  # Previous left wheel encoder reading
        self.prev_rencoder = 0  # Previous right wheel encoder reading
        self.x = 0.0  # Position in xy plane
        self.y = 0.0
        self.th = 0.0  # Orientation angle
        self.dx = 0.0  # Speed in x direction
        self.dr = 0.0  # Speed in rotation
        self.then = self.get_clock().now()  # Time of the previous update

        # Subscriptions
        self.create_subscription(Int16, "lwheel", self.lwheel_callback, 10)
        self.create_subscription(Int16, "rwheel", self.rwheel_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, "odom", 10)
        self.odom_broadcaster = TransformBroadcaster(self)

        self.get_logger().info(
            f"left_wheel_frames: {self.left_wheel_frames}, right_wheel_frames: {self.right_wheel_frames}"
        )

    def update(self):
        # TODO: need to publish joint_states for robot_state_publisher?
        now = self.get_clock().now()
        elapsed = now - self.then
        self.then = now
        elapsed = elapsed.nanoseconds / float(S_TO_NS)

        # Calculate odometry
        if self.enc_left == None:
            d_left = 0
            d_right = 0
        else:
            d_left = (self.left - self.enc_left) / self.ticks_meter
            d_right = (self.right - self.enc_right) / self.ticks_meter
        self.enc_left = self.left
        self.enc_right = self.right

        # Distance traveled is the average of the two wheels
        d = (d_left + d_right) / 2
        # This approximation works (in radians) for small angles
        th = (d_right - d_left) / self.base_width
        # Calculate velocities
        self.dx = d / elapsed
        self.dr = th / elapsed

        if d != 0:
            # Calculate distance traveled in x and y
            x = cos(th) * d
            y = -sin(th) * d
            # Calculate the final position of the robot
            self.x = self.x + (cos(self.th) * x - sin(self.th) * y)
            self.y = self.y + (sin(self.th) * x + cos(self.th) * y)
        if th != 0:
            self.th = self.th + th

        # Publish the odom information
        quaternion = Quaternion()
        quaternion.x = 0.0
        quaternion.y = 0.0
        quaternion.z = sin(self.th / 2)
        quaternion.w = cos(self.th / 2)

        transform_stamped_msg = TransformStamped()
        transform_stamped_msg.header.stamp = self.get_clock().now().to_msg()
        transform_stamped_msg.header.frame_id = self.odom_frame_id
        transform_stamped_msg.child_frame_id = self.base_frame_id
        transform_stamped_msg.transform.translation.x = self.x
        transform_stamped_msg.transform.translation.y = self.y
        transform_stamped_msg.transform.translation.z = 0.0
        transform_stamped_msg.transform.rotation.x = quaternion.x
        transform_stamped_msg.transform.rotation.y = quaternion.y
        transform_stamped_msg.transform.rotation.z = quaternion.z
        transform_stamped_msg.transform.rotation.w = quaternion.w

        self.odom_broadcaster.sendTransform(transform_stamped_msg)

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame_id
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = quaternion
        odom.child_frame_id = self.base_frame_id
        odom.twist.twist.linear.x = self.dx
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = self.dr
        self.odom_pub.publish(odom)

        # Publish wheels spinning
        # if self.left_wheel_frames and self.right_wheel_frames:
        #     self.calculate_wheel_rotation(d_left, d_right)

    def calculate_wheel_rotation(self, d_left, d_right):
        """
        Calculates and broadcasts the wheel rotation transforms.

        This method iterates over the left and right wheel frames, creates a `TransformStamped` message,
        and broadcasts the transform using the `odom_broadcaster`.

        The transform contains the translation and rotation information, with all values set to zero
        except for the `w` component of the rotation quaternion, which is set to 1.0.

        Args:
            d_left (float): The distance traveled by the left wheel.
            d_right (float): The distance traveled by the right wheel.

        Returns:
            None
        """

        # TODO: Get list of joints instead of frames and calculate relative rotation instead of absolute
        # https://docs.ros.org/en/foxy/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Broadcaster-Py.html

        circumference = 2 * pi * self.wheel_radius
        left_rotation = d_left / circumference
        right_rotation = d_right / circumference
        left_rotation_rad = left_rotation * 2 * pi
        right_rotation_rad = right_rotation * 2 * pi

        for left_wheel_frame in self.left_wheel_frames:
            transform_stamped_msg = TransformStamped()
            transform_stamped_msg.header.stamp = self.get_clock().now().to_msg()
            transform_stamped_msg.header.frame_id = self.base_frame_id
            transform_stamped_msg.child_frame_id = left_wheel_frame
            transform_stamped_msg.transform.translation.x = 0.0
            transform_stamped_msg.transform.translation.y = 0.0
            transform_stamped_msg.transform.translation.z = 0.0
            transform_stamped_msg.transform.rotation.x = sin(left_rotation_rad / 2)
            transform_stamped_msg.transform.rotation.y = 0.0
            transform_stamped_msg.transform.rotation.z = 0.0
            transform_stamped_msg.transform.rotation.w = cos(left_rotation_rad / 2)

            self.odom_broadcaster.sendTransform(transform_stamped_msg)

        for right_wheel_frame in self.right_wheel_frames:
            transform_stamped_msg = TransformStamped()
            transform_stamped_msg.header.stamp = self.get_clock().now().to_msg()
            transform_stamped_msg.header.frame_id = self.base_frame_id
            transform_stamped_msg.child_frame_id = right_wheel_frame
            transform_stamped_msg.transform.translation.x = 0.0
            transform_stamped_msg.transform.translation.y = 0.0
            transform_stamped_msg.transform.translation.z = 0.0
            transform_stamped_msg.transform.rotation.x = sin(right_rotation_rad / 2)
            transform_stamped_msg.transform.rotation.y = 0.0
            transform_stamped_msg.transform.rotation.z = 0.0
            transform_stamped_msg.transform.rotation.w = cos(right_rotation_rad / 2)

            self.odom_broadcaster.sendTransform(transform_stamped_msg)

    def lwheel_callback(self, msg):
        enc = msg.data
        if enc < self.encoder_low_wrap and self.prev_lencoder > self.encoder_high_wrap:
            self.lmult = self.lmult + 1

        if enc > self.encoder_high_wrap and self.prev_lencoder < self.encoder_low_wrap:
            self.lmult = self.lmult - 1

        self.left = 1.0 * (enc + self.lmult * (self.encoder_max - self.encoder_min))
        self.prev_lencoder = enc

    def rwheel_callback(self, msg):
        enc = msg.data
        if enc < self.encoder_low_wrap and self.prev_rencoder > self.encoder_high_wrap:
            self.rmult = self.rmult + 1

        if enc > self.encoder_high_wrap and self.prev_rencoder < self.encoder_low_wrap:
            self.rmult = self.rmult - 1

        self.right = 1.0 * (enc + self.rmult * (self.encoder_max - self.encoder_min))
        self.prev_rencoder = enc


def main(args=None):
    rclpy.init(args=args)
    try:
        diff_tf = DiffTf()
        rclpy.spin(diff_tf)
    except rclpy.exceptions.ROSInterruptException:
        pass

    diff_tf.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
