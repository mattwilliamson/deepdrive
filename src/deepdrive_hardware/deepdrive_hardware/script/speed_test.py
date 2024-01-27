#!/usr/bin/env python

"""
This file is for testing the motor controller. 
It can be used to find the velocity of the robot for a given PWM value.

>>> ros2 launch deepdrive_bringup speed_test.launch.xml

[speed_test-1] [INFO] [1706371514.159859499] [speed_test]: delta_distance: 0.1029999852180481, delta_time: 0.19887804985046387 ,velocity: 0.517905245428008

"""

# TODO 1 meter max range?

# Which range should we use? For a regular orientation, 0 should be pointing forward.
scan_index = 180

from deepdrive_hardware.pwm_motor import MotorDriverL293
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
# from rclpy.qos import QoSProfile
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import LaserScan

def m_to_cm(m):
    # if m == float('inf'):
    #     return "0"
    return "{:.4f}".format(m)

class SpeedTest(Node):
    def __init__(self):
        super().__init__('deepdrive_speed_test')
        self.get_logger().info("deepdrive_speed_test node has been initialised.")
        qos = QoSPresetProfiles.SENSOR_DATA.value
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, qos)
        self.robot = MotorDriverL293()
        self.last_scan = 0
        self.last_distance = 0.0

    def scan_callback(self, msg):
        if len(msg.ranges) == 0:
            self.get_logger().warn(f"laser scan ranges is empty")
            return
        
        self.scan_ranges = msg.ranges
        new_distance = msg.ranges[scan_index]

        if self.last_scan > 0:
            # self.get_logger().info(f"new_distance: {new_distance}, last_distance: {self.last_distance} ,last_scan: {self.last_scan}")
            delta_distance = new_distance - self.last_distance
            delta_time = time.time() - self.last_scan
            velocity = delta_distance / delta_time
            self.get_logger().info(f"delta_distance: {delta_distance}, delta_time: {delta_time} ,velocity: {velocity}")
        
        # 360
        # self.get_logger().info(f'Total messages: {len(msg.ranges)}')
        # self.get_logger().info(','.join(m_to_cm(x) for x in msg.ranges[170:190]))
        # self.get_logger().info(f'')

        self.last_scan = time.time()
        self.last_distance = new_distance


    def on_shutdown(self):
        print('motor_controller shutting down')
        self.robot.set_motors(0.0, 0.0)
        self.robot.cleanup()
        

def main(args=None):
    print('Hi from speed_test.')
    rclpy.init(args=args)
    speed_test = SpeedTest()
    rclpy.spin(speed_test)

    speed_test.destroy_node()
    rclpy.shutdown()
    speed_test.on_shutdown()

if __name__ == '__main__':
    main()