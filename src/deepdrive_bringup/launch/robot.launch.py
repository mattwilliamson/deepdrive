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
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="false",
            description="Start RViz2 automatically with this launch file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="false",
            description="Start robot with mock hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation (Gazebo) clock if true",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "imu_target_frame",
            default_value="imu_link",
            # default_value="camera_depth_imu_frame",
            description="Which frame to translate the IMU data to",
        )
    )
    use_sim_time = LaunchConfiguration("use_sim_time")
    imu_target_frame = LaunchConfiguration("imu_target_frame")
    description_dir = os.path.join(get_package_share_directory('deepdrive_description'), 'launch')

    # Initialize Arguments
    gui = LaunchConfiguration("gui")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")


    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("deepdrive_description"), "rviz", "model.rviz"]
    )

    foxglove_bridge = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("deepdrive_bringup"),
                "launch",
                "foxglove_bridge_launch.xml",
            ),
        )
    )

    depth_camera_node = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("deepdrive_camera"),
                "launch",
                "camera.launch.xml",
            ),
        )
    )

    wide_camera_node = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("deepdrive_camera"),
                "launch",
                "wide_angle_camera.launch.xml",
            ),
        )
    )


    # imu_node = Node(
    #     package="imu_bno08x",
    #     executable="imu_bno08x_publisher",
    #     # parameters=[],
    #     output="both",
    #     # remappings=[("/imu_bno08x/data", "/imu/")],
    # )
    
    robot_state_pub_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(description_dir, 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(gui),
    )

    # # Delay rviz start after `joint_state_broadcaster`
    # delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=join,
    #         on_exit=[rviz_node],
    #     )
    # )

    # Fuse IMU to odometry
    robot_localization_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[
            PathJoinSubstitution([
                FindPackageShare("deepdrive_bringup"),
                "config",
                "ekf.yaml",
            ])
        ],
        remappings=[
            ("/odometry/filtered", "/odom"),
            ("/cmd_vel", "/deepdrive/cmd_vel"),
        ],
    )

    joy_params = PathJoinSubstitution([
        FindPackageShare("deepdrive_bringup"),
        "config",
        "joystick.yaml",
    ])

    joy_node = Node(
        package="joy",
        executable="joy_node",
        parameters=[joy_params, {"use_sim_time": use_sim_time}],
        # remappings=[("/cmd_vel", "/diff_drive_controller/cmd_vel_unstamped")],
    )

    joy_params = PathJoinSubstitution([
        FindPackageShare("deepdrive_bringup"),
        "config",
        "joystick.yaml",
    ])

    # joy_node = Node(
    #     package="joy",
    #     executable="joy_node",
    #     parameters=[joy_params, {"use_sim_time": use_sim_time}],
    #     # remappings=[("/cmd_vel", "/diff_drive_controller/cmd_vel_unstamped")],
    # )

    teleop_node = Node(
        package="teleop_twist_joy",
        executable="teleop_node",
        name="teleop_twist_joy_node",
        parameters=[joy_params, {"use_sim_time": use_sim_time}],
        remappings=[("/cmd_vel", "/cmd_vel_joy")],
    )

    twist_mux_params = PathJoinSubstitution([
        FindPackageShare("deepdrive_bringup"),
        "param",
        "twist_mux.yaml",
    ])
    twist_mux_node = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[twist_mux_params, {"use_sim_time": use_sim_time}],
        remappings=[("/cmd_vel_out", "/deepdrive_micro/cmd_vel")],
    )

    # <node pkg="imu_transformer" type="imu_transformer_node" name="imu_data_transformer" output="screen">
    #     <remap from="imu_in" to="imu_raw"/>
    #     <remap from="imu_out" to="imu"/>
    #     <param name="target_frame" value="base_link"/>
    # </node>

    # https://github.com/ros-perception/imu_pipeline/blob/ros2/imu_transformer/launch/ned_to_enu.launch.xml
    #   <node pkg="tf2_ros" exec="static_transform_publisher" name="tf_imu_ned_enu"
    # args="0 0 0 1.5708 0 3.1416 imu_link_ned imu_link" output="screen"/>
    # <remap from="imu_in" to="imu_ned"/>
    # <remap from="imu_out" to="imu_enu"/>
    
    # TODO: This might be old, check if it's still needed
    # imu_publisher_node = Node(
    #     package="tf2_ros",
    #     executable="static_transform_publisher",
    #     name="imu_publisher_node",
    #     arguments=["0", "0", "0", "-1.57", "0", "1.57", "camera_depth", "camera_depth_imu_frame"],
    # )

    imu_transformer_node = Node(
        package="imu_transformer",
        executable="imu_transformer_node",
        name="imu_transformer_node",
        parameters=[{"target_frame": "camera_depth"}],
        remappings=[
            ("imu_in", "/camera_depth/imu/data"),
            ("imu_out", "/camera_depth/imu/transformed"),
        ],
    )

    # TODO: Params
    uros_agent_node = Node(
        package="micro_ros_agent",
        executable="micro_ros_agent",
        name="micro_ros_agent",
        arguments=["serial", "--dev", "/dev/ttyACM0"],
        # parameters=[{"target_frame": "imu_link"}],
        # remappings=[
            # ("imu_in", "/camera_depth/imu/data"),
            # ("imu_out", "/camera_depth/imu/transformed"),
        # ],
    )

    lidar_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("deepdrive_lidar"), 'launch', 'ldlidar_with_mgr.launch.py')
        ),
        # launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # For use with kiss-icp
    lidar_to_pointcloud_node = Node(
        package='pointcloud_to_laserscan',
        executable='laserscan_to_pointcloud_node',
        name='laserscan_to_pointcloud',
        remappings=[('scan_in', '/ldlidar_node/scan'),
                    ('cloud', '/lidar/cloud')],
        parameters=[{'target_frame': 'ldlidar_link', 'transform_tolerance': 0.01}]
    )

    # TODO: Upgrade kiss-icp and launch file
    # KISS-ICP node
    kiss_icp_node = Node(
        package="kiss_icp",
        executable="odometry_node",
        name="kiss_icp_node",
        output="screen",
        remappings=[
            ("pointcloud_topic", "/lidar/cloud"),
            # ("pointcloud_topic", "/camera_depth/points"),
        ],
        parameters=[
            {
                # ROS node configuration
                "base_frame": "base_link",
                "child_frame": "base_link",
                "odom_frame": "odom",
                # "odom_frame": "ldlidar_link",
                # "odom_frame": "camera_depth_right_camera_optical_frame",
                "publish_odom_tf": False,
                # KISS-ICP configuration
                "max_range": 12.0,
                "min_range": 1.0,
                "deskew": False,
                "max_points_per_voxel": 20,
                # "voxel_size": 1.0,
                # Adaptive threshold
                "initial_threshold": 2.0,
                "min_motion_th": 0.1,
                # Registration
                "max_num_iterations": 500,
                "convergence_criterion": 0.0001,
                "max_num_threads": 0,
                # Fixed covariances
                "position_covariance": 0.5,
                "orientation_covariance": 0.5,
                # ROS CLI arguments
                "publish_debug_clouds": True,
                "use_sim_time": use_sim_time,
            },
        ],
    )
    

    nodes = [
        robot_state_pub_node,
        uros_agent_node,
        # foxglove_bridge,
        depth_camera_node,
        # wide_camera_node,
        # delay_rviz_after_joint_state_broadcaster_spawner,
        robot_localization_node,
        # imu_publisher_node,
        imu_transformer_node,
        lidar_node,

        # Use Lidar for odom
        # lidar_to_pointcloud_node,
        # kiss_icp_node,

        teleop_node,
        # joy_node,
        twist_mux_node,
    ]

    return LaunchDescription(declared_arguments + nodes)
