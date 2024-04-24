import launch
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
import launch_ros
import os
import launch_ros.descriptions



def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package="deepdrive_description").find(
        "deepdrive_description"
    )
    default_model_path = os.path.join(pkg_share, "urdf/deepdrive_deepdrive.xacro")
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        # parameters=[robot_description],
        parameters=[
            {'robot_description': launch_ros.descriptions.ParameterValue(
                launch.substitutions.Command(['xacro ', LaunchConfiguration("model")]), value_type=str) 
            }
        ]

    )

    return launch.LaunchDescription(
        [
            # launch.actions.DeclareLaunchArgument(name='gui', default_value='False',
            # description='Flag to enable joint_state_publisher_gui'),
            launch.actions.DeclareLaunchArgument(
                name="model", default_value=default_model_path, description="Absolute path to robot urdf file"
            ),
            # launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
            # description='Absolute path to rviz config file'),
            launch.actions.DeclareLaunchArgument(
                name="use_sim_time", default_value="False", description="Flag to enable use_sim_time"
            ),
            # launch.actions.ExecuteProcess(cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world_path], output='screen'),
            # gzserver_cmd,
            # gzclient_cmd,
            # joint_state_publisher_node,
            robot_state_publisher_node,
            # spawn_entity,
            # robot_localization_node,
            # rviz_node,
            # foxglove_bridge
        ]
    )
