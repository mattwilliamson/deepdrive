import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import LaserScan, Imu, PointCloud2
from nav_msgs.msg import Odometry
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2
import json
import atexit
from datetime import datetime
import os

class DynamicOdometryCalibrationNode(Node):

    def __init__(self):
        super().__init__('dynamic_odometry_calibration_node')

        # Initialize data holders
        self.data = {}
        self.initial_data = {}
        self.message_counts = {}
        self.json_lines_file = f'/tmp/odometry_data_{datetime.now().strftime("%Y%m%d_%H%M%S")}.jsonl'
        self.jsonl_file_handle = open(self.json_lines_file, 'a')

        # Retrieve topic names and types
        topic_list = self.get_topic_names_and_types()
        self.topics = {name: types for name, types in topic_list}

        # Subscriptions
        self.create_subscriptions()

        # Timer
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        # Register exit handler
        atexit.register(self.print_detailed_report)
        atexit.register(self.close_file_handle)

    def create_subscriptions(self):
        for topic_name, topic_types in self.topics.items():
            supported = True
            qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=1)

            self.get_logger().info(f'Checking topic "{topic_name}" type: {topic_types}')

            for topic_type in topic_types:
                if topic_type == 'sensor_msgs/msg/LaserScan':
                    self.create_subscription(LaserScan, topic_name, lambda msg, tn=topic_name: self.generic_callback(msg, tn), 1)
                elif topic_type == 'nav_msgs/msg/Odometry':
                    self.create_subscription(Odometry, topic_name, lambda msg, tn=topic_name: self.generic_callback(msg, tn), qos)
                elif topic_type == 'sensor_msgs/msg/Imu':
                    self.create_subscription(Imu, topic_name, lambda msg, tn=topic_name: self.generic_callback(msg, tn), qos)
                elif topic_type == 'sensor_msgs/msg/PointCloud2':
                    self.create_subscription(PointCloud2, topic_name, lambda msg, tn=topic_name: self.generic_callback(msg, tn), qos)
                else:
                    supported = False
            if supported:
                self.get_logger().info(f'* Subscribing to topic "{topic_name}" type: {topic_types}')
                self.message_counts[topic_name] = 0

    def generic_callback(self, msg, topic_name):
        self.message_counts[topic_name] += 1

        self.get_logger().info(f'Got message  on "{topic_name}" type: {type(msg)}')

        if isinstance(msg, LaserScan):
            total = len(msg.ranges)
            front_ranges = msg.ranges[4:6]  # Assuming front beams are the first 3 measurements
            self.get_logger().info(f'Got LaserScan  on "{topic_name}" front_ranges: {front_ranges}')
            if front_ranges:
                self.data[topic_name] = {
                    'x': float(np.mean(front_ranges)),
                    'yaw': 0.,
                }
        
        elif isinstance(msg, Odometry):
            # self.get_logger().info(f'Got Odometry  on "{topic_name}" data: {msg}')
            self.data[topic_name] = {
                'x': float(msg.pose.pose.position.x),
                'yaw': self.get_yaw_from_orientation(msg.pose.pose.orientation),
            }

        elif isinstance(msg, Imu):
            # self.get_logger().info(f'Got Imu  on "{topic_name}" data: {msg}')
            self.data[topic_name] = {
                'x': 0.,
                'yaw': self.get_yaw_from_orientation(msg.orientation),
            }

        elif isinstance(msg, PointCloud2):
            points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
            if points:
                distances = [np.sqrt(x**2 + y**2 + z**2) for x, y, z in points]
                self.data[topic_name] = {
                    'x': float(np.min(distances)),
                    'yaw': 0.,
                }
        else:
            self.get_logger().info(f'Unsupported message type: {type(msg)}')

    def timer_callback(self):
        self.set_initial_positions()

        # Log data as JSON
        self.get_logger().info(json.dumps(self.data))

        # Write latest measurements to JSON Lines file
        self.jsonl_file_handle.write(json.dumps(self.data) + '\n')
        self.jsonl_file_handle.flush()

        # Print differences
        self.print_differences()

    def set_initial_positions(self):
        for topic, data in self.data.items():
            if topic not in self.initial_data:
                self.initial_data[topic] = data

    def print_differences(self):
        differences = {}

        for topic, initial_value in self.initial_data.items():
            current_value = self.data[topic]

            self.get_logger().info(f'topic "{topic}" initial_value: {initial_value}, current_value: {current_value}')

            try:
                diff_x = current_value['x'] - initial_value['x']
                diff_yaw = current_value['yaw'] - initial_value['yaw']
                differences[topic] = {
                    'x': diff_x,
                    'yaw': diff_yaw
                }
            except:
                self.get_logger().info(f"initial_value['x']: {initial_value['x']} type={type(initial_value['x'])}")
                self.get_logger().info(f"initial_value['yaw']: {initial_value['yaw']} type={type(initial_value['yaw'])}")
                self.get_logger().info(f"current_value['x']: {current_value['x']} type={type(current_value['x'])}")
                self.get_logger().info(f"current_value['yaw']: {current_value['yaw']} type={type(current_value['yaw'])}")
                self.get_logger().info("print_differences initial_value: " + json.dumps(initial_value, indent=2))
                self.get_logger().info("print_differences current_value: " + json.dumps(current_value, indent=2))
                raise

        self.get_logger().info("Differences: " + json.dumps(differences, indent=2))

    def get_yaw_from_orientation(self, orientation):
        # Convert quaternion to yaw
        t3 = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
        t4 = 1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
        return float(np.arctan2(t3, t4))

    def print_detailed_report(self):
        print('Printing detailed report:')
        if self.initial_data:
            self.print_differences()
        print(f'Message counts: {json.dumps(self.message_counts, indent=2)}')

    def close_file_handle(self):
        self.jsonl_file_handle.close()
        print(f'Closed file handle for {self.json_lines_file}')

def main(args=None):
    rclpy.init(args=args)
    node = DynamicOdometryCalibrationNode()
    executor = MultiThreadedExecutor()
    try:
        rclpy.spin(node, executor)
    except KeyboardInterrupt:
        print('KeyboardInterrupt caught, shutting down...')
    except Exception as e:
        print(f'Exception caught: {e}')
    finally:
        try:
            node.destroy_node()
            rclpy.shutdown()
        except Exception as e:
            print(f'Exception during shutdown: {e}')
        print(f'Logged data to: {node.json_lines_file}')

if __name__ == '__main__':
    main()
