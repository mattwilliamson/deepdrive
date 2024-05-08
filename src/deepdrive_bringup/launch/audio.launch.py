from typing import List
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():

    params = {
        # "seed": LaunchConfiguration("seed", default=-1),
    }

    return LaunchDescription([
        Node(
            package="audio_common",
            executable="audio_capturer_node",
            name="audio_capturer_node",
            parameters=[params],
            # condition=UnlessCondition(PythonExpression(
            #     [LaunchConfiguration("use_audio")]))
        ),

        Node(
            package="audio_common",
            executable="audio_player_node",
            name="audio_player_node",
            parameters=[params],
            # condition=IfCondition(PythonExpression(
            #     [LaunchConfiguration("use_audio")]))
        ),
    ])
