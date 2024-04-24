#!/usr/bin/env python3

from sensor_msgs.msg import MagneticField, Imu
from std_msgs.msg import Float64, Bool
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue, DiagnosticArray
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

# TODO: Params for I2C bus

DIAGNOSTIC_HZ = 4
IMU_TOPIC = "imu_bno08x/data"
MAG_TOPIC = 'imu_bno08x/mag'
DIAGNOSTIC_TOPIC = "imu_bno08x/status"
SHAKE_TOPIC = "imu_bno08x/shake"
ACTIVITY_TOPIC = "imu_bno08x/activity"
STABILITY_TOPIC = "imu_bno08x/stability"


class ImuNode(Node):
    """
    This class represents a ROS node for publishing IMU data from the BNO08X sensor.

    I2C Address: BNO080 (0x4b / 75) BNO085 (0x4a / 74)
    
    Parameters:
    - save_calibration (bool): Flag indicating whether to save the calibration data. Default is False.
    - publish_mag (bool): Flag indicating whether to publish magnetic field data. Default is False.
    - imu_frame (str): The frame ID for the IMU data. Default is "imu_link".
    - rate (int): The publishing rate in Hz. Default is 30.
    - address (int): The I2C address of the BNO08X sensor. Default is 0x4B.
    - frequency (int): The frequency of the I2C bus. Default is 800000.
    
    """

    def __init__(self):
        super().__init__("imu_bno08x_node")
        self.get_logger().info("node is starting up...")

        self.save_calibration = self.declare_parameter('save_calibration', False).value
        self.publish_mag = self.declare_parameter('publish_mag', False).value
        self.imu_frame = self.declare_parameter('imu_frame', "imu_link").value
        self.rate = self.declare_parameter('rate', 30).value
        self.address = self.declare_parameter('address', 0x4B).value
        self.frequency = self.declare_parameter('frequency', 800000).value

        self.calibration_status = 0
        self.saved_calibration = False

        # i2c = I2C(1)
        # self.bno = BNO08X_I2C(i2c, address=0x4a)
        # This works for the Jetson Orin Nano
        i2c = busio.I2C(board.SCL, board.SDA)
        # RuntimeWarning: I2C frequency is not settable in python, ignoring!
        # i2c = busio.I2C(board.SCL, board.SDA, frequency=self.frequency)
        self.bno = BNO08X_I2C(i2c, address=self.address)

        self.bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
        self.bno.enable_feature(BNO_REPORT_LINEAR_ACCELERATION)
        self.bno.enable_feature(BNO_REPORT_GYROSCOPE)
        self.bno.enable_feature(BNO_REPORT_MAGNETOMETER)
        self.bno.enable_feature(BNO_REPORT_ACCELEROMETER)
        self.bno.enable_feature(BNO_REPORT_SHAKE_DETECTOR)
        self.bno.enable_feature(BNO_REPORT_STABILITY_CLASSIFIER)
        self.bno.enable_feature(BNO_REPORT_ACTIVITY_CLASSIFIER)

        self.imu_publisher = self.create_publisher(Imu, IMU_TOPIC, 10)
        self.create_timer(1.0 / self.rate, self.publish_imu)

        self.diagnostics = self.create_publisher(DiagnosticArray, DIAGNOSTIC_TOPIC, 10)
        self.create_timer(1.0 / DIAGNOSTIC_HZ, self.publish_diagnostics)

        if self.publish_mag:
            self.mag_publisher = self.create_publisher(MagneticField, MAG_TOPIC, 10)

        time.sleep(0.5)  # ensure IMU is initialized

        self.get_logger().info("node is started")

    def publish_imu(self):
            """
            Publishes the IMU data as ROS messages.

            This function retrieves the IMU data from the BNO08x sensor and publishes it as ROS messages.
            The IMU data includes orientation, linear acceleration, and angular velocity.
            """
            
            self.calibration_status = self.bno.calibration_status
            self.calibration_status = 1

            msg = Imu()
            msg.header.frame_id = self.imu_frame
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

            self.imu_publisher.publish(msg)

            # Magnetometer
            if self.publish_mag:
                mag_msg = MagneticField()
                mag_msg.header.frame_id = self.imu_frame
                mag_msg.header.stamp = self.get_clock().now().to_msg()
                mag_x, mag_y, mag_z = self.bno.magnetic
                mag_msg.magnetic_field.x = mag_x
                mag_msg.magnetic_field.y = mag_y
                mag_msg.magnetic_field.z = mag_z
                mag_msg.magnetic_field_covariance[0] = cov
                mag_msg.magnetic_field_covariance[4] = cov
                mag_msg.magnetic_field_covariance[8] = cov
                self.mag_publisher(mag_msg)

    def get_diagnostic_status(self):
        status_msg = DiagnosticStatus()
        status_msg.name = "IMU Calibration"
        status_msg.hardware_id = "bno08x"
        status_msg.message = REPORT_ACCURACY_STATUS[self.calibration_status]
        status_msg.values = [
            KeyValue(key="Calibration Code", value=str(self.calibration_status)),
            KeyValue(key="Calibration Status", value=REPORT_ACCURACY_STATUS[self.calibration_status]),
        ]
        
        # Status: 0=Unreliable 1=Accuracy Low 2=Accuracy Medium 3=Accuracy High 
        if self.calibration_status == 3:
            # Accuracy High 
            status_msg.level = DiagnosticStatus.OK
            if not self.saved_calibration:
                # Calibration status is good. Let's persist it.
                self.get_logger().info("accuracy calibration is good. Saving.")
                if self.save_calibration:
                    self.bno.save_calibration_data()
                self.saved_calibration = True
        elif self.calibration_status == 2:
            # Accuracy Medium
            status_msg.level = DiagnosticStatus.OK
        elif self.calibration_status == 1:
            # Accuracy LOW
            status_msg.level = DiagnosticStatus.WARN
        else:
            # Unreliable
            status_msg.level = DiagnosticStatus.ERROR

        return status_msg
    
    def get_shake_status(self):
        shake_detected = self.bno.shake

        status_msg = DiagnosticStatus()
        status_msg.level = DiagnosticStatus.OK
        status_msg.name = "Shake Detector"
        status_msg.hardware_id = "bno08x"
        status_msg.level = DiagnosticStatus.WARN if shake_detected else DiagnosticStatus.OK
        status_msg.message = "Shake Detected" if shake_detected else "No Shake Detected"
        status_msg.values = [
            KeyValue(key="detected", value=str(shake_detected)),
        ]

        return status_msg
    
    def get_stability_status(self):
            """
            Get the stability status of the BNO08x IMU. One of:
                * "Unknown" - The sensor is unable to classify the current stability
                * "On Table" - The sensor is at rest on a stable surface with very little vibration
                * "Stationary" -  The sensor’s motion is below the stable threshold but\
                the stable duration requirement has not been met. This output is only available when\
                gyro calibration is enabled
                * "Stable" - The sensor’s motion has met the stable threshold and duration requirements.
                * "In motion" - The sensor is moving.

            Returns:
                DiagnosticStatus: The stability status message.
            """
            
            stability = self.bno.stability_classification

            status_msg = DiagnosticStatus()
            status_msg.level = DiagnosticStatus.OK
            status_msg.name = "Stability Classifier"
            status_msg.hardware_id = "bno08x"
            status_msg.level = DiagnosticStatus.OK if stability in ("Stable", "On Table") else DiagnosticStatus.WARN
            status_msg.message = stability
            status_msg.values = [
                KeyValue(key="stability", value=str(stability)),
            ]

            return status_msg
    
    def get_activity_status(self):
        """
        Get the activity status of the BNO08x IMU. One of:
            * "Unknown"
            * "In-Vehicle"
            * "On-Bicycle"
            * "On-Foot"
            * "Still"
            * "Tilting"
            * "Walking"
            * "Running"
            * "On Stairs"

        {'most_likely': 'Unknown', 'Unknown': 91, 'In-Vehicle': 2, 'On-Bicycle': 0, 'On-Foot': 3, 'Still': 4, 'Tilting': 0, 'Walking': 1, 'Running': 2, 'OnStairs': 0}

        Returns:
            DiagnosticStatus: The activity status message.
        """
        activity = self.bno.activity_classification
        most_likely = activity['most_likely']
        status_msg = DiagnosticStatus()
        status_msg.level = DiagnosticStatus.OK
        status_msg.name = "Activity Classifier"
        status_msg.hardware_id = "bno08x"
        status_msg.message = most_likely
        status_msg.level = DiagnosticStatus.OK 
        if most_likely in ("Unknown", "Tilting"):
            status_msg.level = DiagnosticStatus.WARN
        elif most_likely in ("On Stairs", "Running", "On-Foot"):
            status_msg.level = DiagnosticStatus.ERROR

        status_msg.values = [KeyValue(key=k, value=str(v)) for k, v in activity.items()]
        return status_msg


    def publish_diagnostics(self):
            """
            Publishes diagnostic information about the bno08x IMU.

            This method creates a DiagnosticStatus message and populates it with relevant information
            such as the calibration status of the IMU. The message is then published using the
            diagnostics publisher.

            :return: None
            """
            
            status_array = DiagnosticArray()
            status_array.status = [
                self.get_diagnostic_status(),
                self.get_shake_status(),
                self.get_stability_status(),
                self.get_activity_status(),
            ]
            status_array.header.stamp = self.get_clock().now().to_msg()
            self.diagnostics.publish(status_array)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = ImuNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
