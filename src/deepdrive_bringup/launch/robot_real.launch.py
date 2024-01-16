#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
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
#
# Authors: Darby Lim

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration

def generate_launch_description():
    # DEEPDRIVE_MODEL = os.environ['DEEPDRIVE_MODEL'] or 'deepdrive'
    # LDS_MODEL = os.environ['LDS_MODEL'] or 'LDS-01'
    # LDS_LAUNCH_FILE = '/hlds_laser.launch.py'

    namespace = LaunchConfiguration('namespace', default="deepdrive")
    lidar_port = LaunchConfiguration("lidar_port", default="/dev/ttyTHS0")
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    # default_model_path = os.path.join(pkg_share, 'src/description/deepdrive_description.urdf')
    description_dir = get_package_share_directory('deepdrive_description')
    urdf = os.path.join(description_dir, 'urdf', 'deepdrive_deepdrive.urdf')
    # pkg_share = launch_ros.substitutions.FindPackageShare(package='deepdrive_description').find('deepdrive_description')

    remappings = [('/tf', 'tf'),
                ('/tf_static', 'tf_static')]

    # deepdrive_param_dir = LaunchConfiguration(
    #     'deepdrive_param_dir',
    #     default=os.path.join(
    #         get_package_share_directory('deepdrive_bringup'),
    #         'param',
    #         DEEPDRIVE_MODEL + '.yaml'))

    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        'use_robot_state_pub',
        default_value='True',
        description='Whether to start the robot state publisher')


    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    
    
    with open(urdf, 'r') as infp:
        robot_description = infp.read()



    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value=use_sim_time,
                description="Use simulation (Gazebo) clock if true",
            ),
            DeclareLaunchArgument(
                "lidar_port",
                default_value=lidar_port,
                description="TTY port for LDS-01 lidar",
            ),

            declare_use_robot_state_pub_cmd,
            # DeclareLaunchArgument(
            #     'deepdrive_param_dir',
            #     default_value=deepdrive_param_dir,
            #     description='Full path to deepdrive parameter file to load'),
            # IncludeLaunchDescription(
            #     PythonLaunchDescriptionSource(
            #         [ThisLaunchFileDir(), '/deepdrive_state_publisher.launch.py']),
            #     launch_arguments={'use_sim_time': use_sim_time}.items(),
            # ),

            Node(
                condition=IfCondition(use_robot_state_pub),
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                namespace=namespace,
                output='screen',
                parameters=[{'use_sim_time': use_sim_time,
                            'robot_description': robot_description}],
                remappings=remappings
            ),

            Node(
                condition=IfCondition(use_robot_state_pub),
                package='joint_state_publisher',
                executable='joint_state_publisher',
                name='joint_state_publisher',
                namespace=namespace,
                output='screen',
                parameters=[{'use_sim_time': use_sim_time,
                            'robot_description': robot_description}],
                remappings=remappings
            ),

                    

            # Oak-d-lite Stereo Inertial Camera
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    # [ThisLaunchFileDir(), '/rtabmap.launch.py']),
                    # [ThisLaunchFileDir(), '/camera_rgbd_pcl.launch.py']),
                    [ThisLaunchFileDir(), '/camera_stereo_inertial_node.launch.py']),
                launch_arguments={"name": "deepdrive",
                            #   "params_file": params_file,
                            #   "parent_frame": LaunchConfiguration("parent_frame"),
                            #    "cam_pos_x": LaunchConfiguration("cam_pos_x"),
                            #    "cam_pos_y": LaunchConfiguration("cam_pos_y"),
                            #    "cam_pos_z": LaunchConfiguration("cam_pos_z"),
                            #    "cam_roll": LaunchConfiguration("cam_roll"),
                            #    "cam_pitch": LaunchConfiguration("cam_pitch"),
                            #    "cam_yaw": LaunchConfiguration("cam_yaw"),
                            #    "use_rviz": LaunchConfiguration("use_rviz")
                               }.items(),
            ),

            # LDS-01 LIDAR
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        os.path.join(
                            get_package_share_directory("hls_lfcd_lds_driver"), "launch"
                        ),
                        "/hlds_laser.launch.py",
                    ]
                ),
                launch_arguments={"port": lidar_port, "frame_id": "base_scan"}.items(),
            ),

            # Transform for lidar
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=[
                    "0",
                    "0",
                    "0",
                    "0",
                    "3.14159",
                    "3.14159",
                    "oak-d-base-frame",  # odom
                    "laser",
                ],
            ),

            # Foxglove Studio Bridge
            Node(
                package="foxglove_bridge",
                executable="foxglove_bridge",
                arguments=[],
            ),

            # Motor Controller
            Node(
                package="deepdrive_hardware",
                executable="motor_controller",
                parameters=[],
                # parameters=[deepdrive_param_dir],
                arguments=[],
                output="screen",
            ),
        ]
    )
