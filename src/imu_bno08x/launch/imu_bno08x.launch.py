from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
import os
import yaml
from launch.substitutions import EnvironmentVariable
import pathlib
import launch.actions
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory("imu_bno08x"),
        "config",
        "imu_bno08x.yaml",
    )

    return LaunchDescription(
        [
            # odometry publisher
            launch_ros.actions.Node(
                package="imu_bno08x",
                executable="imu_bno08x_publisher",
                name="imu_bno08x_publisher",
                output="screen",
                parameters=[params],
                remappings=[
                    ("/imu_bno08x/data", "/imu"),
                    ("/imu_bno08x/mag", "/mag"),
                    ("/imu_bno08x/status", "/diagnostics"),
                    # ("shake", "/imu_bno08x/shake"),         # NOT IMPLEMENTED - Probably bool when shake is detected
                    # ("activity", "/imu_bno08x/activity"),   # NOT IMPLEMENTED - "In-Vehicle", "On-Foot", "Still", "Tilting", "Unknown"
                    # ("stability", "/imu_bno08x/stability"), # NOT IMPLEMENTED - "On table", "Stable", or "Motion"
                ],
            ),
        ]
    )
