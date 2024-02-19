#!/usr/bin/env python3

from sensor_msgs.msg import MagneticField, Imu
from std_msgs.msg import Float64, Bool
from diagnostic_msgs.msg import DiagnosticStatus
from rclpy.node import Node
import rclpy
import busio
import time
import board
from adafruit_extended_bus import ExtendedI2C as I2C
from adafruit_bno08x import (
    BNO_REPORT_LINEAR_ACCELERATION,
    BNO_REPORT_ROTATION_VECTOR,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_MAGNETOMETER,
    REPORT_ACCURACY_STATUS,
    BNO_REPORT_SHAKE_DETECTOR,
    BNO_REPORT_ACTIVITY_CLASSIFIER,
    BNO_REPORT_STABILITY_CLASSIFIER,
)

from adafruit_bno08x.i2c import BNO08X_I2C

# TODO: Params for I2C bus and address and frame id and loop hz
LOOP_HZ = 20
DIAGNOSTIC_HZ = 1

# BNO080 (0x4b) BNO085 (0x4a)
I2C_ADDRESS = 0x4B

IMU_TOPIC = "imu_bno08x/data"
# MAG_TOPIC = 'imu_bno08x/mag'
DIAGNOSTIC_TOPIC = "imu_bno08x/status"
# SHAKE_TOPIC = "imu_bno08x/shake"
# ACTIVITY_TOPIC = "imu_bno08x/activity"
# STABILITY_TOPIC = "imu_bno08x/stability"
IMU_FRAME = "imu_link"

SAVE_CALIBRATION = False


class ImuNode(Node):
    def __init__(self):
        super().__init__("imu_bno08x_node")
        self.get_logger().info("node is starting up...")

        self.calibration_status = 0
        self.saved_calibration = False
        # self.shake_detected = False

        # self.declare_parameter('my_parameter', 'world')
        # my_param = self.get_parameter('my_parameter').get_parameter_value().string_value

        # i2c = I2C(1)
        # self.bno = BNO08X_I2C(i2c, address=0x4a)
        # This works for the Jetson Orin Nano
        i2c = busio.I2C(board.SCL, board.SDA)
        self.bno = BNO08X_I2C(i2c, address=I2C_ADDRESS)

        self.bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
        self.bno.enable_feature(BNO_REPORT_LINEAR_ACCELERATION)
        self.bno.enable_feature(BNO_REPORT_GYROSCOPE)
        self.bno.enable_feature(BNO_REPORT_MAGNETOMETER)
        self.bno.enable_feature(BNO_REPORT_ACCELEROMETER)
        # self.bno.enable_feature(BNO_REPORT_SHAKE_DETECTOR)
        # self.bno.enable_feature(BNO_REPORT_STABILITY_CLASSIFIER)

        self.imu_publisher = self.create_publisher(Imu, IMU_TOPIC, 10)
        self.create_timer(1.0 / LOOP_HZ, self.publish_imu)

        self.diagnostics = self.create_publisher(DiagnosticStatus, DIAGNOSTIC_TOPIC, 10)
        self.create_timer(1.0 / DIAGNOSTIC_HZ, self.publish_diagnostics)

        # self.shake_publisher = self.create_publisher(Bool, SHAKE_TOPIC, 10)

        # self.publisher = self.create_publisher(MagneticField, 'imu_bno08x/mag', 10)

        time.sleep(0.5)  # ensure IMU is initialized

        self.get_logger().info("node is started")

    def publish_imu(self):
        self.calibration_status = self.bno.calibration_status
        self.calibration_status = 1

        msg = Imu()
        msg.header.frame_id = IMU_FRAME
        msg.header.stamp = self.get_clock().now().to_msg()

        # Increase the covariance for the orientation if the calibration status is low
        # Status: 0=Unreliable 1=Accuracy Low 2=Accuracy Medium 3=Accuracy High 
        cov = int(5 - self.calibration_status)
        cov = (cov * cov * .02) - .079 # Weigh lower accuracy even less
        # 0 unreliable -> 0.421
        # 1 low        -> 0.241
        # 2 medium     -> 0.100
        # 3 high       -> 0.001

        # Orientation
        quat_i, quat_j, quat_k, quat_real = self.bno.quaternion
        msg.orientation.w = quat_i
        msg.orientation.x = quat_j
        msg.orientation.y = quat_k
        msg.orientation.z = quat_real
        msg.orientation_covariance[0] = cov
        msg.orientation_covariance[4] = cov
        msg.orientation_covariance[8] = cov

        # Linear Acceleration with gravity removed
        accel_x, accel_y, accel_z = self.bno.linear_acceleration
        msg.linear_acceleration.x = accel_x
        msg.linear_acceleration.y = accel_y
        msg.linear_acceleration.z = accel_z
        msg.linear_acceleration_covariance[0] = cov
        msg.linear_acceleration_covariance[4] = cov
        msg.linear_acceleration_covariance[8] = cov

        # Gyro
        gyro_x, gyro_y, gyro_z = self.bno.gyro
        msg.angular_velocity.x = gyro_x
        msg.angular_velocity.y = gyro_y
        msg.angular_velocity.z = gyro_z
        msg.angular_velocity_covariance[0] = cov
        msg.angular_velocity_covariance[4] = cov
        msg.angular_velocity_covariance[8] = cov

        # Accelerometer
        # accel_x, accel_y, accel_z = self.bno.acceleration
        # msg.linear_acceleration.x = accel_x
        # msg.linear_acceleration.y = accel_y
        # msg.linear_acceleration.z = accel_z
        # msg.linear_acceleration_covariance[0] = cov
        # msg.linear_acceleration_covariance[4] = cov
        # msg.linear_acceleration_covariance[8] = cov

        self.imu_publisher.publish(msg)

        # Magnetometer
        # mag_x, mag_y, mag_z = self.bno.magnetic
        # mag_msg.header.stamp = rospy.Time.now()
        # mag_msg.magnetic_field.x = mag_x
        # mag_msg.magnetic_field.y = mag_y
        # mag_msg.magnetic_field.z = mag_z
        # mag_msg.magnetic_field_covariance[0] = cov
        # mag_msg.magnetic_field_covariance[4] = cov
        # mag_msg.magnetic_field_covariance[8] = cov

        # shake_detected = self.bno.shake
        # if shake_detected:
        #     self.get_logger().info("Shake detected")
        # if self.shake_detected != shake_detected and type(shake_detected) is bool:
        #     self.shake_detected = shake_detected
        #     self.shake_publisher.publish(Bool(data=shake_detected))




    def publish_diagnostics(self):
        status_msg = DiagnosticStatus()
        # OK=0, WARN=1, ERROR=2, STALE=3
        status_msg.level = b"\x00"
        status_msg.name = "bno08x IMU"
        status_msg.message = REPORT_ACCURACY_STATUS[self.calibration_status]

        # Status: 0=Unreliable 1=Accuracy Low 2=Accuracy Medium 3=Accuracy High 
        if self.calibration_status == 3:
            status_msg.level = b"\x00"
            if not self.saved_calibration:
                # Calibration status is good. Let's persist it.
                self.get_logger().info("accuracy calibration is good. Saving.")
                if SAVE_CALIBRATION:
                    self.bno.save_calibration_data()
                self.saved_calibration = True
        if self.calibration_status == 2:
            status_msg.level = b"\x00"
        else:
            # Warning
            status_msg.level = b"\x01"

        self.diagnostics.publish(status_msg)



def main(args=None):
    rclpy.init(args=args)
    try:
        node = ImuNode()
        rclpy.spin(node)
        node.destroy_node()
    except KeyboardInterrupt:
        pass

    # rclpy.shutdown()
