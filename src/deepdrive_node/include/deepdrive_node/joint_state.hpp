#ifndef DEEPDRIVE_NODE__JOINT_STATE_HPP_
#define DEEPDRIVE_NODE__JOINT_STATE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <memory>
#include <string>

namespace deepdrive
{
constexpr uint8_t JOINT_NUM = 4;

constexpr double RPM_TO_MS = 0.229 * 0.0034557519189487725;

// 0.087890625[deg] * 3.14159265359 / 180 = 0.001533981f
constexpr double TICK_TO_RAD = 0.001533981;

class JointState
{
public:
  explicit JointState(
    std::shared_ptr<rclcpp::Node> & nh,
    const std::string & joint_state_topic = "joint_states",
    const std::string & pulse_topic = "/pulses/reading",
    const std::string & frame_id = "base_link");

  void publish(
    const rclcpp::Time & now);

private:
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_;
    std::shared_ptr<rclcpp::Node> nh_;
  std::string frame_id_;
  rclcpp::QoS qos_ = rclcpp::QoS(rclcpp::KeepLast(10));
};
}  // namespace deepdrive
#endif  // DEEPDRIVE_NODE__JOINT_STATE_HPP_
