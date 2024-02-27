#include <chrono>
#include <memory>
#include <string>

#include <rcutils/cmdline_parser.h>
#include <rclcpp/rclcpp.hpp>

#include "deepdrive_node/diff_drive_controller.hpp"
#include "deepdrive_node/deepdrive.hpp"


int main(int argc, char * argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor executor;

  auto deepdrive = std::make_shared<deepdrive::Deepdrive>();
  auto diff_drive_controller =
    std::make_shared<deepdrive::DiffDriveController>(
    deepdrive->get_wheels()->separation,
    deepdrive->get_wheels()->radius);

  executor.add_node(deepdrive);
  executor.add_node(diff_drive_controller);
  // executor.add_node(odometry);
  // executor.add_node(joint_state_publisher);
  executor.spin();

  rclcpp::shutdown();

  return 0;
}
