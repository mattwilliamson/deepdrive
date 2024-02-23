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
    params = os.path.join(get_package_share_directory("differential_drive"), 'config', 'differential_drive.yaml')

    return LaunchDescription([

        # odometry publisher
        launch_ros.actions.Node(
            package='differential_drive',
            executable='diff_tf',
            name='diff_tf',
            output='screen',
            parameters=[params],
            remappings=[
                ("odom", "/diff_drive/odom"),           # out: current pose and twist
                ("lwheel", "/diff_drive/left/pulses"),  # in:  wheel encoder pulses
                ("rwheel", "/diff_drive/right/pulses"), # in:  wheel encoder pulses
            ],
        ),

        # left wheel
        launch_ros.actions.Node(
            package='differential_drive',
            executable='pid_velocity',
            name='pid_velocity_left',
            output='screen',
            parameters=[params],
            remappings=[
                ("wheel", "/diff_drive/left/pulses"),            # in:  wheel encoder pulses
                ("wheel_vtarget", "/diff_drive/left/vel/cmd"),   # in:  desired velocity in meters/second
                ("motor_cmd", "/diff_drive/left/pwm"),           # out: motor_cmd arbitraty units (PWM for me)
                ("wheel_vel", "/diff_drive/left/vel/current"),   # out: current velocity of the wheel in meters/second
            ],
        ),

        # right wheel
        launch_ros.actions.Node(
            package='differential_drive',
            executable='pid_velocity',
            name='pid_velocity_right',
            output='screen',
            parameters=[params],
            remappings=[
                ("wheel", "/diff_drive/right/pulses"),            # in:  wheel encoder pulses
                ("wheel_vtarget", "/diff_drive/right/vel/cmd"),   # in:  desired velocity in meters/second
                ("motor_cmd", "/diff_drive/right/pwm"),           # out: motor_cmd arbitraty units (PWM for me)
                ("wheel_vel", "/diff_drive/right/vel/current"),   # out: current velocity of the wheel in meters/second
            ],
        ),

        # twist to motors
        launch_ros.actions.Node(
            package='differential_drive',
            executable='twist_to_motors',
            name='twist_to_motors',
            output='screen',
            parameters=[params],
            remappings=[
                ("twist", "/cmd_vel"),                                 # in:  twist commands
                ("lwheel_vtarget", "/diff_drive/right/pwm"),           # out: target velocity for the left wheel (in M/s)
                ("rwheel_vtarget", "/diff_drive/right/vel/current"),   # out: target velocity for the right wheel (in M/s)
            ],
        ),


    ])
