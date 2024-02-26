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

    ld = LaunchDescription([

        # odometry publisher
        launch_ros.actions.Node(
            package='differential_drive',
            executable='diff_tf',
            name='diff_tf',
            output='screen',
            parameters=[params],
            remappings=[
                # Base odometry just off the back tire pulses
                ("odom", "/diff_drive/odom"),           # out: current pose and twist
                ("lwheel", "/diff_drive/back/left/pulses"),  # in:  wheel encoder pulses
                ("rwheel", "/diff_drive/back/right/pulses"), # in:  wheel encoder pulses
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
                ("twist", "/cmd_vel"),                             # in:  twist commands
                ("lwheel_vtarget", "/diff_drive/left/vel/cmd"),    # out: target velocity for the left wheel (in M/s)
                ("rwheel_vtarget", "/diff_drive/right/vel/cmd"),   # out: target velocity for the right wheel (in M/s)
            ],
        ),


    ])

    # 4 wheel drive differential drive
    for x in ('front', 'back'):
        for y in ('left', 'right'):
            ld.add_action(
                launch_ros.actions.Node(
                    package='differential_drive',
                    executable='pid_velocity',
                    name=f'pid_velocity_{x}_{y}',
                    output='screen',
                    parameters=[{
                        "Kp": 10.0,
                        "Ki": 10.0,
                        "Kd": 0.001,
                        "out_min": -255,
                        "out_max": 255,
                        "rate": 30,
                        "rolling_pts": 2,
                        "timeout_ticks": 2,
                        "ticks_meter": 20,
                        "vel_threshold": 0.001,
                        "encoder_min": -32768,
                        "encoder_max": 32768,
                    }],
                    remappings=[
                        # Base odometry just off the back tire pulses
                        ("wheel", f"/diff_drive/{x}/{y}/pulses"),            # in:  wheel encoder pulses
                        ("wheel_vtarget", f"/diff_drive/{y}/vel/cmd"),  # in:  desired velocity in meters/second
                        ("motor_cmd", f"/diff_drive/{x}/{y}/pwm"),           # out: motor_cmd arbitraty units (PWM for me)
                        ("wheel_vel", f"/diff_drive/{x}/{y}/pulses"),   # out: current velocity of the wheel in meters/second
                    ],
                )
            )

    return ld