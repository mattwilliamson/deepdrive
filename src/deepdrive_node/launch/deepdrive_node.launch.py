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
            get_package_share_directory("deepdrive_node"),
            "param",
            "deepdrive_node.yaml",
        ),
    )
    

    rviz_dir = LaunchConfiguration(
        "rviz_dir",
        default=os.path.join(
            get_package_share_directory("deepdrive_node"), "launch"
        ),
    )

    description_dir = os.path.join(get_package_share_directory('deepdrive_description'), 'launch')

    use_sim_time = LaunchConfiguration("use_sim_time", default="false")

    uros_agent_node = Node(
        package="micro_ros_agent",
        executable="micro_ros_agent",
        name="micro_ros_agent",
        # Launch on a bunch of ports, because we don't know which number it will get
        arguments=["multiserial", "--devs", "/dev/ttyMotor1 /dev/ttyMotor2 /dev/ttyMotor3 /dev/ttyMotor4 /dev/ttyMotor5 /dev/ttyMotor6 /dev/ttyMotor7 /dev/ttyMotor8"],
        # parameters=[{"target_frame": "imu_link"}],
        remappings=[
            ("/odom", "/deepdrive_node/odom"),
        ],
    )

    return LaunchDescription(
        [
            LogInfo(msg=["Execute Deepdrive Node!!"]),
            DeclareLaunchArgument(
                "param_dir",
                default_value=param_dir,
                description="Specifying parameter direction",
            ),
            # IncludeLaunchDescription(
            #     PythonLaunchDescriptionSource([rviz_dir, "/rviz2.launch.py"])
            # ),
            Node(
                package="deepdrive_node",
                executable="deepdrive_node",
                parameters=[param_dir],
                output="both",
                arguments=['--ros-args', '--log-level', 'INFO'],

            ),
            # IncludeLaunchDescription(
            #     PythonLaunchDescriptionSource(
            #         [description_dir, "/robot_state_publisher.launch.py"]
            #     ),
            #     launch_arguments={
            #         "use_sim_time": use_sim_time,
            #     }.items(),
            # ),
            uros_agent_node,
        ]
    )
