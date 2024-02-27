#ifndef DEEPDRIVE_NODE__DIFF_DRIVE_CONTROLLER_HPP_
#define DEEPDRIVE_NODE__DIFF_DRIVE_CONTROLLER_HPP_

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "deepdrive_node/odometry.hpp"

namespace deepdrive
{
class DiffDriveController : public rclcpp::Node
{
public:
  explicit DiffDriveController(const float wheel_seperation, const float wheel_radius);
  virtual ~DiffDriveController() {}

private:
  std::shared_ptr<rclcpp::Node> nh_;
  std::unique_ptr<Odometry> odometry_;
};
}  // namespace deepdrive
#endif  // DEEPDRIVE_NODE__DIFF_DRIVE_CONTROLLER_HPP_
