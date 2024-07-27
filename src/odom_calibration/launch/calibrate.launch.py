from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
import os
from launch.substitutions import EnvironmentVariable
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    # params = os.path.join(
    #     get_package_share_directory("imu_bno08x"),
    #     "config",
    #     "imu_bno08x.yaml",
    # )

    # control_node = Node(
    #     package="controller_manager",
    #     executable="ros2_control_node",
    #     parameters=[robot_controllers],
    #     output="both",
    #     remappings=[
    #         ("~/robot_description", "/robot_description"),
    #         ("/diff_drive_controller/cmd_vel_unstamped", "/cmd_vel"),
    #     ],
    #     # arguments=['--ros-args', '--log-level', 'debug']
    # )

    return LaunchDescription(
        [
            # odometry publisher
            launch_ros.actions.Node(
                package="odom_calibration",
                executable="calibrate",
                name="calibrate",
                output="screen",
                parameters=[],
                respawn=False,
                remappings=[
                    # ("/imu_bno08x/data", "/imu/data"),
                    # ("/imu_bno08x/mag", "/mag"),
                    # ("/imu_bno08x/status", "/diagnostics"),
                    # ("shake", "/imu_bno08x/shake"),         # NOT IMPLEMENTED - Probably bool when shake is detected
                    # ("activity", "/imu_bno08x/activity"),   # NOT IMPLEMENTED - "In-Vehicle", "On-Foot", "Still", "Tilting", "Unknown"
                    # ("stability", "/imu_bno08x/stability"), # NOT IMPLEMENTED - "On table", "Stable", or "Motion"
                ],
            ),
        ]
    )
