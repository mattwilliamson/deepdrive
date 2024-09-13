import os 
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    description_dir = os.path.join(get_package_share_directory('deepdrive_description'), 'launch')
    use_sim_time = LaunchConfiguration("use_sim_time", default=False)

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("deepdrive_control"),
            "config",
            "controllers.yaml",
        ]
    )

    robot_state_pub_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(description_dir, 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["deepdrive_control", "--param-file", robot_controllers],
    )

    # params = os.path.join(
    #     get_package_share_directory("imu_bno08x"),
    #     "config",
    #     "imu_bno08x.yaml",
    # )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="both",
        remappings=[
            ("~/robot_description", "/robot_description"),
            ("/diff_drive_controller/cmd_vel_unstamped", "/cmd_vel"),
        ],
        # arguments=['--ros-args', '--log-level', 'debug']
    )


    # robot_controllers = PathJoinSubstitution(
    #     [
    #         FindPackageShare("deepdrive_description"),
    #         "config",
    #         "controllers.yaml",
    #     ]
    # )

    # control_node = Node(
    #     package="controller_manager",
    #     executable="ros2_control_node",
    #     # parameters=[robot_description, robot_controllers],
    #     parameters=[robot_controllers],
    #     output="both",
    #     # remappings=[(/robot_description)],
    # )

    return LaunchDescription(
        [
            # robot_state_pub_node,
            joint_state_broadcaster_spawner,
            robot_controller_spawner,
            control_node,

        ]
    )
