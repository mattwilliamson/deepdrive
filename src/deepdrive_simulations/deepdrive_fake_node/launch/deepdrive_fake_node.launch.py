import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    param_dir = LaunchConfiguration(
        "param_dir",
        default=os.path.join(
            get_package_share_directory("deepdrive_fake_node"),
            "param",
            "deepdrive.yaml",
        ),
    )

    rviz_dir = LaunchConfiguration(
        "rviz_dir",
        default=os.path.join(
            get_package_share_directory("deepdrive_fake_node"), "launch"
        ),
    )

    description_dir = os.path.join(get_package_share_directory('deepdrive_description'), 'launch')

    use_sim_time = LaunchConfiguration("use_sim_time", default="false")

    return LaunchDescription(
        [
            LogInfo(msg=["Execute Deepdrive Fake Node!!"]),
            DeclareLaunchArgument(
                "param_dir",
                default_value=param_dir,
                description="Specifying parameter direction",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([rviz_dir, "/rviz2.launch.py"])
            ),
            Node(
                package="deepdrive_fake_node",
                executable="deepdrive_fake_node",
                parameters=[param_dir],
                output="screen",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [description_dir, "/robot_state_publisher.launch.py"]
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                }.items(),
            ),
        ]
    )
