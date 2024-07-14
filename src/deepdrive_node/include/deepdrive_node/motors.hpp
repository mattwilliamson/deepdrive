#ifndef DEEPDRIVE_NODE__MOTORS_HPP_
#define DEEPDRIVE_NODE__MOTORS_HPP_

#include <array>
#include <chrono>
#include <memory>
#include <string>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/int64_multi_array.hpp>
#include "deepdrive_node/joint_state.hpp"


// This will subscribe to /motors/pulses
// This will publish to /motors/cmd
// Both will have messages of type Int64MultiArray[4] for now

namespace deepdrive
{
class Motors
{
public:
  explicit Motors(
    std::shared_ptr<rclcpp::Node> & nh,
    const double wheels_separation,
    const double wheels_radius);
  virtual ~Motors() {}

private:
  bool calculate_motors(const rclcpp::Duration & duration);

  void update_joint_state(const std::shared_ptr<sensor_msgs::msg::JointState const> & joint_state);

  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr joint_state_msg);

  void publish(const rclcpp::Time & now);

  std::shared_ptr<rclcpp::Node> nh_;
  std::unique_ptr<std_msgs::msg::Int64MultiArray> cmd_broadcaster_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<std_msgs::msg::Int64MultiArray>::SharedPtr wheel_encoders_sub_;

  double wheels_separation_;
  double wheels_radius_;

  std::string frame_id_of_motors_;
  std::string child_frame_id_of_motors_;

  bool publish_tf_;

  std::array<double, JOINT_NUM> diff_joint_positions_;
};
}  // namespace deepdrive
#endif  // DEEPDRIVE_NODE__MOTORS_HPP_
