from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ld = LaunchDescription()

    package_dir = FindPackageShare("imu_bno08x")
    urdf_path = PathJoinSubstitution([package_dir, "urdf", "imu.urdf"])

    robot_description_content = ParameterValue(
        Command(["xacro ", urdf_path]), value_type=str
        # Command(["cat ", urdf_path]), value_type=str
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {
                "robot_description": robot_description_content,
            }
        ],
    )

    ld.add_action(robot_state_publisher_node)



    robot_localization_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[
            PathJoinSubstitution([package_dir, "config", "ekf.yaml"])
        ],
        remappings=[
            ("/odometry/filtered", "/odom"),
        ],
    )
    ld.add_action(robot_localization_node)

    odom_node = Node(
        package="imu_bno08x",
        executable="dummy_odom",
        name="dummy_odom_publisher_node",
        remappings=[
            ("odom", "/odom/dummy"),
        ],
    )
    ld.add_action(odom_node)

    imu_publisher_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="imu_publisher_node",
        arguments=["0", "0", "0", "0", "0", "0", "base_link", "imu_link"],
    )
    ld.add_action(imu_publisher_node)

    
    return ld
