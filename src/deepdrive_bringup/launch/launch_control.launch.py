# Copyright 2020 ros2_control Development Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os 
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="false",
            description="Start RViz2 automatically with this launch file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="false",
            description="Start robot with mock hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation (Gazebo) clock if true",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "imu_target_frame",
            default_value="imu_link",
            # default_value="camera_depth_imu_frame",
            description="Which frame to translate the IMU data to",
        )
    )
    use_sim_time = LaunchConfiguration("use_sim_time")
    imu_target_frame = LaunchConfiguration("imu_target_frame")
    description_dir = os.path.join(get_package_share_directory('deepdrive_description'), 'launch')

    # Initialize Arguments
    gui = LaunchConfiguration("gui")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")


    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("deepdrive_description"), "rviz", "model.rviz"]
    )

    imu_node = Node(
        package="imu_bno08x",
        executable="imu_bno08x_publisher",
        # parameters=[],
        output="both",
        # remappings=[("/imu_bno08x/data", "/imu/")],
    )

    
    robot_state_pub_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(description_dir, 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(gui),
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller", "--controller-manager", "/controller_manager"],
    )

    # Delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    # Fuse IMU to odometry
    robot_localization_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[
            PathJoinSubstitution([
                FindPackageShare("deepdrive_bringup"),
                "config",
                "ekf.yaml",
            ])
        ],
        remappings=[
            ("/odometry/filtered", "/odom"),
        ],
    )

    joy_params = PathJoinSubstitution([
        FindPackageShare("deepdrive_bringup"),
        "config",
        "joystick.yaml",
    ])

    joy_node = Node(
        package="joy",
        executable="joy_node",
        parameters=[joy_params, {"use_sim_time": use_sim_time}],
        # remappings=[("/cmd_vel", "/diff_drive_controller/cmd_vel_unstamped")],
    )

    teleop_node = Node(
        package="teleop_twist_joy",
        executable="teleop_node",
        name="teleop_twist_joy_node",
        parameters=[joy_params, {"use_sim_time": use_sim_time}],
        # remappings=[("/cmd_vel", "/diff_drive_controller/cmd_vel_unstamped")],
    )

    # twist_mux_params = os.path.join(
    #     get_package_share_directory(package_name), "config", "twist_mux.yaml"
    # )
    # twist_mux = Node(
    #     package="twist_mux",
    #     executable="twist_mux",
    #     parameters=[twist_mux_params, {"use_sim_time": use_sim_time}],
    #     remappings=[("/cmd_vel_out", "/diff_drive_controller/cmd_vel_unstamped")],
    # )

    # <node pkg="imu_transformer" type="imu_transformer_node" name="imu_data_transformer" output="screen">
    #     <remap from="imu_in" to="imu_raw"/>
    #     <remap from="imu_out" to="imu"/>
    #     <param name="target_frame" value="base_link"/>
    # </node>

    # https://github.com/ros-perception/imu_pipeline/blob/ros2/imu_transformer/launch/ned_to_enu.launch.xml
    #   <node pkg="tf2_ros" exec="static_transform_publisher" name="tf_imu_ned_enu"
    # args="0 0 0 1.5708 0 3.1416 imu_link_ned imu_link" output="screen"/>
    # <remap from="imu_in" to="imu_ned"/>
    # <remap from="imu_out" to="imu_enu"/>
    
    imu_publisher_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="imu_publisher_node",
        arguments=["0", "0", "0", "-1.57", "0", "1.57", "imu_link", "camera_depth_imu_frame"],
    )

    imu_transformer_node = Node(
        package="imu_transformer",
        executable="imu_transformer_node",
        name="imu_transformer_node",
        parameters=[{"target_frame": "imu_link"}],
        remappings=[
            ("imu_in", "/camera_depth/imu/data"),
            ("imu_out", "imu"),
        ],
    )

    nodes = [
        # control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        delay_rviz_after_joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
        robot_localization_node,
        imu_publisher_node,
        imu_transformer_node,
        # teleop_node,
        # joy_node,
        imu_node,
    ]

    return LaunchDescription(declared_arguments + nodes)
