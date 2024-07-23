import os
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription(
        [
            Node(
                package="ros_optical_flow_matek",
                executable="ld19_node",
                name="ld19_lidar_node",
                output="screen",
                parameters=[
                    {"port": "/dev/matek"},
                    {"frame_id": "ldlidar_link"},
                    {"topic_name": "scan"},
                ],
            )
        ]
    )