import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    name = LaunchConfiguration('name').perform(context)
    # depthai_prefix = get_package_share_directory("depthai_ros_driver")
    depthai_prefix = get_package_share_directory("deepdrive_camera")

    params_file= LaunchConfiguration("params_file")

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(depthai_prefix, 'launch', 'camera.launch.py')),
            launch_arguments={"name": name,
                              "params_file": params_file}.items()),
        Node(
            package="depthai_ros_driver",
            executable="obj_pub.py",
            remappings=[
                ('/oak/nn/detections', name+'/nn/spatial_detections'),
                ('/oak/nn/detection_markers', name+'/nn/detection_markers'),
                ('/oak/nn/text_markers', name+'/nn/text_markers'),
            ]
        )
    ]


def generate_launch_description():
    # depthai_prefix = get_package_share_directory("depthai_ros_driver")
    depthai_prefix = get_package_share_directory("deepdrive_camera")
    declared_arguments = [
        DeclareLaunchArgument("name", default_value="camera_depth"),
        DeclareLaunchArgument("params_file", default_value=os.path.join(depthai_prefix, 'param', 'detection.yaml')),
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
