#ifndef CONTROLLER_PLUGIN_HPP
#define CONTROLLER_PLUGIN_HPP

#include <memory>
#include <string>
#include <vector>
#include <thread>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "motor.hpp"

namespace deepdrive_control
{
class ControllerPlugin : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ControllerPlugin);

  ControllerPlugin();

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
    std::vector<std::shared_ptr<Motor>> motors_;
    rclcpp::Duration timeout_;
    rclcpp::Node::SharedPtr node_;  // Shared node for all motors
    std::thread spin_thread_;  // Separate thread for spinning the node

    void load_motors_from_param(const hardware_interface::HardwareInfo & info);
    bool check_motor_timeout();
    void spin_node();  // Function to spin the node in a separate thread
};

}  // namespace deepdrive_control

#endif // CONTROLLER_PLUGIN_HPP
