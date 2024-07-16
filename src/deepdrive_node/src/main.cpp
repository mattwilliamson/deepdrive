#include "rclcpp/rclcpp.hpp"
#include "deepdrive_node/motor_control.hpp"
#include "deepdrive_node/odometry_publisher.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    // Create a MultiThreadedExecutor
    rclcpp::executors::MultiThreadedExecutor executor;

    auto motorControlNode = std::make_shared<MotorControl>();
    auto odometryPublisherNode = std::make_shared<OdometryPublisher>();

    // Add nodes to the executor
    executor.add_node(motorControlNode);
    executor.add_node(odometryPublisherNode);

    // Spin the executor
    executor.spin();

    rclcpp::shutdown();
    
    return 0;
}
