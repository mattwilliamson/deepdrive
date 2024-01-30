import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    RegisterEventHandler,
)


def generate_launch_description():
    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name = "deepdrive_description"
    pkg_share = FindPackageShare(package="deepdrive_description").find(
        "deepdrive_description"
    )
    use_sim_time = LaunchConfiguration("use_sim_time")
    default_rviz_config_path = os.path.join(pkg_share, "rviz/model.rviz")
    default_model_path = os.path.join(pkg_share, "urdf/deepdrive_deepdrive.xacro")
    robot_description = {
        "robot_description": Command(["xacro ", LaunchConfiguration("model")])
    }

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rvizconfig")],
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
        # parameters=[],
        # remappings=[
        #     ("/diff_drive_controller/cmd_vel_unstamped", "/cmd_vel"),
        # ],
    )

    joy_params = os.path.join(
        get_package_share_directory("articubot_one"), "config", "joystick.yaml"
    )

    joy_node = Node(
        package="joy",
        executable="joy_node",
        parameters=[joy_params, {"use_sim_time": use_sim_time}],
        remappings=[("/cmd_vel", "/diff_drive_controller/cmd_vel_unstamped")],
    )

    teleop_node = Node(
        package="teleop_twist_joy",
        executable="teleop_node",
        name="teleop_node",
        parameters=[joy_params, {"use_sim_time": use_sim_time}],
        remappings=[("/cmd_vel", "/diff_drive_controller/cmd_vel_unstamped")],
    )

    twist_mux_params = os.path.join(
        get_package_share_directory(package_name), "config", "twist_mux.yaml"
    )
    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[twist_mux_params, {"use_sim_time": use_sim_time}],
        remappings=[("/cmd_vel_out", "/diff_drive_controller/cmd_vel_unstamped")],
    )

    # ros2 run deepdrive_teleop teleop_keyboard
    # ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_drive_controller/cmd_vel_unstamped

    # teleop_node = Node(
    #     package="deepdrive_teleop",
    #     executable="teleop_keyboard",
    #     parameters=[{"use_sim_time": use_sim_time}],
    #     remappings=[("/cmd_vel_out", "/diff_drive_controller/cmd_vel_unstamped")],
    # )

    gazebo_params_file = os.path.join(
        get_package_share_directory(package_name), "config", "gazebo_params.yaml"
    )

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("gazebo_ros"),
                    "launch",
                    "gazebo.launch.py",
                )
            ]
        ),
        launch_arguments={
            "extra_gazebo_args": "--ros-args --params-file " + gazebo_params_file
        }.items(),
    )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "deepdrive"],
        output="screen",
    )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller"],
        parameters=[{"use_sim_time": use_sim_time}],
        # remappings=[
        #     ("/diff_drive_controller/cmd_vel_unstamped", "/cmd_vel"),
        # ],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        parameters=[{"use_sim_time": use_sim_time}],
        # remappings=[
        #     ("/diff_drive_controller/cmd_vel_unstamped", "/cmd_vel"),
        # ],
    )

    # Fuse IMU to odometry
    robot_localization_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[
            os.path.join(
                get_package_share_directory("deepdrive_bringup"), "config", "ekf.yaml"
            ),
            {"use_sim_time": use_sim_time},
        ],
        remappings=[
            ("/odometry/filtered", "/odom"),
        ],
    )

    # Code for delaying a node (I haven't tested how effective it is)
    #
    # First add the below lines to imports
    # from launch.actions import RegisterEventHandler
    # from launch.event_handlers import OnProcessExit
    #
    # Then add the following below the current diff_drive_spawner
    # delayed_diff_drive_spawner = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=spawn_entity,
    #         on_exit=[diff_drive_spawner],
    #     )
    # )
    #
    # Replace the diff_drive_spawner in the final return with delayed_diff_drive_spawner

    # Launch them all!
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use sim time if true",
            ),
            DeclareLaunchArgument(
                name="model",
                default_value=default_model_path,
                description="Absolute path to robot urdf file",
            ),
            DeclareLaunchArgument(
                name="rvizconfig",
                default_value=default_rviz_config_path,
                description="Absolute path to rviz config file",
            ),
            robot_state_pub_node,
            # joy_node,
            # twist_mux,
            gazebo,
            spawn_entity,
            diff_drive_spawner,
            joint_broad_spawner,
            robot_localization_node,
            teleop_node,
            rviz_node,
        ]
    )
