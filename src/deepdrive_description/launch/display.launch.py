import launch
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python import get_package_share_directory
import launch_ros
import os
from launch.conditions import IfCondition

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='deepdrive_description').find('deepdrive_description')
    default_model_path = os.path.join(pkg_share, 'urdf/deepdrive_deepdrive.urdf')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/model.rviz')
    world_path=os.path.join(pkg_share, 'worlds/deepdrive_world.world')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
    )
    
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=launch.conditions.UnlessCondition(LaunchConfiguration('gui'))
    )

    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    spawn_entity = launch_ros.actions.Node(
    	package='gazebo_ros', 
    	executable='spawn_entity.py',
        arguments=['-entity', 'deepdrive', '-topic', 'robot_description'],
        output='screen'
    )

    robot_localization_node = launch_ros.actions.Node(
         package='robot_localization',
         executable='ekf_node',
         name='ekf_filter_node',
         output='screen',
         parameters=[os.path.join(get_package_share_directory("deepdrive_bringup"), 'config', 'ekf.yaml'), {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    foxglove_bridge = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("foxglove_bridge"),
                "launch",
                "foxglove_bridge_launch.xml",
            ),
        )
    )


    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='gui', default_value='False',
                                            description='Flag to enable joint_state_publisher_gui'),
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to robot urdf file'),
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='False',
                                            description='Flag to enable use_sim_time'),
        # launch.actions.ExecuteProcess(cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world_path], output='screen'),
        # gzserver_cmd,
        # gzclient_cmd,
        joint_state_publisher_node,
        robot_state_publisher_node,
        spawn_entity,
        # robot_localization_node,
        rviz_node,
        foxglove_bridge
    ])