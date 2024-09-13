import launch
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    RegisterEventHandler,
)
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit


from ament_index_python import get_package_share_directory
import launch_ros
from launch_ros.actions import Node
import os
from launch.conditions import IfCondition


def generate_launch_description():
    pkg_share = FindPackageShare(package="deepdrive_description").find(
        "deepdrive_description"
    )
    default_model_path = os.path.join(pkg_share, "urdf/deepdrive_deepdrive.xacro")
    default_rviz_config_path = os.path.join(pkg_share, "rviz/model.rviz")

    pkg_gazebo_ros = get_package_share_directory("gazebo_ros")
    world_path=os.path.join(pkg_share, 'worlds/deepdrive_world.world')
    # world_path = os.path.join(
    #     get_package_share_directory("deepdrive_simulations"),
    #     "worlds",
    #     "deepdrive_house.world",
    # )
    robot_description = {
        "robot_description": ParameterValue(
            Command(["xacro ", LaunchConfiguration("model")]),
            value_type=str
        )
    }

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # joint_state_publisher_node = Node(
    #     package="joint_state_publisher",
    #     executable="joint_state_publisher",
    #     name="joint_state_publisher",
    #     condition=launch.conditions.UnlessCondition(LaunchConfiguration("gui")),
    # )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rvizconfig")],
    )

    # rviz_node = Node(
    #     package="rviz2",
    #     executable="rviz2",
    #     name="rviz2",
    #     output="log",
    #     arguments=["-d", rviz_config_file],
    #     condition=IfCondition(gui),
    # )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-entity", "deepdrive", "-topic", "robot_description"],
        output="screen",
    )

    # robot_localization_node = Node(
    #     package="robot_localization",
    #     executable="ekf_node",
    #     name="ekf_filter_node",
    #     output="screen",
    #     parameters=[
    #         os.path.join(
    #             get_package_share_directory("deepdrive_bringup"), "config", "ekf.yaml"
    #         ),
    #         {"use_sim_time": LaunchConfiguration("use_sim_time")},
    #     ],
    # )

    foxglove_bridge = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("foxglove_bridge"),
                "launch",
                "foxglove_bridge_launch.xml",
            ),
        )
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
    #  [controller_manager]: [Deprecated] Passing the robot description parameter directly to the control_manager node is deprecated. Use '~/robot_description' topic from 'robot_state_publisher' instead.
    # robot_state_pub_node = Node(
    #     package="robot_state_publisher",
    #     executable="robot_state_publisher",
    #     output="both",
    #     parameters=[robot_description],
    #     # parameters=[],
    #     remappings=[
    #         ("/diff_drive_controller/cmd_vel_unstamped", "/cmd_vel"),
    #     ],
    # )

    # joint_state_broadcaster_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=[
    #         "joint_state_broadcaster",
    #         "--controller-manager",
    #         "/controller_manager",
    #     ],
    # )

    # robot_controller_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=[
    #         "diff_drive_controller",
    #         "--controller-manager",
    #         "/controller_manager",
    #     ],
    # )

    # # Delay rviz start after `joint_state_broadcaster`
    # delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=joint_state_broadcaster_spawner,
    #         on_exit=[rviz_node],
    #     )
    # )

    # # Delay start of robot_controller after `joint_state_broadcaster`
    # delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = (
    #     RegisterEventHandler(
    #         event_handler=OnProcessExit(
    #             target_action=joint_state_broadcaster_spawner,
    #             on_exit=[robot_controller_spawner],
    #         )
    #     )
    # )

    return launch.LaunchDescription(
        [
            DeclareLaunchArgument(
                name="gui",
                default_value="True",
                description="Flag to enable joint_state_publisher_gui",
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
            DeclareLaunchArgument(
                name="use_sim_time",
                default_value="True",
                description="Flag to enable use_sim_time",
            ),
            # launch.actions.ExecuteProcess(
            #     cmd=[
            #         "gazebo",
            #         "--verbose",
            #         "-s",
            #         "libgazebo_ros_init.so",
            #         "-s",
            #         "libgazebo_ros_factory.so",
            #         world_path,
            #     ],
            #     output="screen",
            # ),
            # gzserver_cmd,
            # gzclient_cmd,
            # joint_state_publisher_node,
            # spawn_entity,
            # robot_localization_node,
            # rviz_node,
            foxglove_bridge,
            # control_node,
            robot_state_publisher_node,
        # robot_state_pub_node,
            # joint_state_broadcaster_spawner,
            # robot_controller_spawner
            # delay_rviz_after_joint_state_broadcaster_spawner,
            # delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,

        ]
    )
    # nodes = [
    #     control_node,
    #     robot_state_pub_node,
    #     joint_state_broadcaster_spawner,
    #     delay_rviz_after_joint_state_broadcaster_spawner,
    #     delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
    # ]