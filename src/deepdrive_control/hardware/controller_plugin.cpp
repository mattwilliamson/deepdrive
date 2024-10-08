#include "controller_plugin.hpp"

namespace deepdrive_control {

ControllerPlugin::ControllerPlugin()
    : node_(nullptr), spin_thread_(), timeout_(rclcpp::Duration::from_seconds(1.0)) {
}

hardware_interface::CallbackReturn ControllerPlugin::on_init(const hardware_interface::HardwareInfo &info) {
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Create a single shared node for all motors
  node_ = std::make_shared<rclcpp::Node>("deepdrive_controller_plugin_node");

  // Start spinning the node in a separate thread
  spin_thread_ = std::thread(&ControllerPlugin::spin_node, this);

  // Set timeout to default to 1000ms
  timeout_ = rclcpp::Duration::from_seconds(1.0);

  load_motors_from_param(info);  // Load motors from hardware info

  // Fetch the minimum velocity parameter
  if (info.hardware_parameters.find("min_velocity") != info.hardware_parameters.end()) {
    min_velocity_ = std::stod(info.hardware_parameters.at("min_velocity"));
    RCLCPP_INFO(node_->get_logger(), "min_velocity parameter: %f", min_velocity_);
  } else {
    min_velocity_ = 0.05;
    RCLCPP_INFO(node_->get_logger(), "min_velocity parameter defaulting to: %f", min_velocity_);
  }

  RCLCPP_INFO(node_->get_logger(), "ControllerPlugin initialized successfully");

  return hardware_interface::CallbackReturn::SUCCESS;
}

void ControllerPlugin::load_motors_from_param(const hardware_interface::HardwareInfo &info) {
  // Log all the hardware parameters
  for (const auto &param : info.hardware_parameters) {
    RCLCPP_INFO(node_->get_logger(), "Hardware parameter: %s = %s", param.first.c_str(), param.second.c_str());
  }

  // Iterate over the joint names from the HardwareInfo
  for (const auto &joint : info.joints) {
    std::string motor_name = joint.name;

    // Fetch the multipliers from the hardware parameters
    double input_multiplier = 0.0;
    double output_multiplier = 0.0;

    std::string input_multiplier_param = motor_name + "_input_multiplier";
    std::string output_multiplier_param = motor_name + "_output_multiplier";

    if (info.hardware_parameters.find(input_multiplier_param) != info.hardware_parameters.end()) {
      input_multiplier = std::stod(info.hardware_parameters.at(input_multiplier_param));
    } else {
      RCLCPP_ERROR(node_->get_logger(), "Missing input multiplier parameter for motor: %s", motor_name.c_str());
      input_multiplier - 1.0;
    }

    if (info.hardware_parameters.find(output_multiplier_param) != info.hardware_parameters.end()) {
      output_multiplier = std::stod(info.hardware_parameters.at(output_multiplier_param));
    } else {
      RCLCPP_ERROR(node_->get_logger(), "Missing output multiplier parameter for motor: %s", motor_name.c_str());
      output_multiplier = 1.0;
    }

    // double input_multiplier = std::stod(info.hardware_parameters.at(motor_name + "_input_multiplier"));
    // double output_multiplier = std::stod(info.hardware_parameters.at(motor_name + "_output_multiplier"));

    // Create the motor with the multipliers and add it to the motors_ vector
    motors_.emplace_back(std::make_shared<Motor>(motor_name, input_multiplier, output_multiplier, min_velocity_, node_));
  }
}

std::vector<hardware_interface::StateInterface> ControllerPlugin::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (const auto &motor : motors_) {
    state_interfaces.emplace_back(motor->get_name(), "velocity", &motor->get_velocity_state());
    state_interfaces.emplace_back(motor->get_name(), "position", &motor->get_position_state());
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> ControllerPlugin::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (const auto &motor : motors_) {
    command_interfaces.emplace_back(motor->get_name(), "velocity", &motor->get_velocity_command());
  }
  return command_interfaces;
}

hardware_interface::CallbackReturn ControllerPlugin::on_activate(const rclcpp_lifecycle::State &previous_state) {
  // rclcpp::Time start_time = node_->now();
  // bool all_received = false;

  for (auto &motor : motors_) {
    motor->set_commanded_velocity(0.0);  // Stop all motors
  }

  // Wait for all motors to send angle readings
  // RCLCPP_INFO(rclcpp::get_logger("ControllerPlugin"), "Waiting to receive messages from all motors...");

  // while (node_->now() - start_time < timeout_ && !all_received) {
  //   all_received = std::all_of(motors_.begin(), motors_.end(),
  //                              [](const std::shared_ptr<Motor> &motor) { return motor->has_received_message(); });

  //   if (!all_received) {
  //     rclcpp::sleep_for(std::chrono::milliseconds(100));
  //   }
  // }

  // if (!all_received) {
  //   RCLCPP_ERROR(rclcpp::get_logger("ControllerPlugin"), "Failed to receive messages from all motors within the timeout");
  //   return hardware_interface::CallbackReturn::ERROR;
  // }

  // RCLCPP_INFO(rclcpp::get_logger("ControllerPlugin"), "All motors are now active");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ControllerPlugin::on_deactivate(const rclcpp_lifecycle::State &previous_state) {
  for (auto &motor : motors_) {
    motor->set_commanded_velocity(0.0);  // Stop all motors
  }

  // Ensure the node is no longer spinning
  if (spin_thread_.joinable()) {
    spin_thread_.join();
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type ControllerPlugin::read(const rclcpp::Time &time, const rclcpp::Duration &period) {
  // TODO: Stop until all motors have sent a message again
  if (check_motor_timeout()) {
    RCLCPP_ERROR(rclcpp::get_logger("ControllerPlugin"), "Timeout occurred for one or more motors. Stopping all motors.");
    for (auto &motor : motors_) {
      motor->set_commanded_velocity(0.0);  // Stop all motors
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ControllerPlugin::write(const rclcpp::Time &time, const rclcpp::Duration &period) {
  for (auto &motor : motors_) {
    motor->write_command();
  }
  return hardware_interface::return_type::OK;
}

bool ControllerPlugin::check_motor_timeout() {
  rclcpp::Time current_time = node_->now();
  return std::any_of(motors_.begin(), motors_.end(),
                     [current_time, this](const std::shared_ptr<Motor> &motor) {
                       return motor->check_timeout(current_time, timeout_);
                     });
}

void ControllerPlugin::spin_node() {
  rclcpp::spin(node_);
}

}  // namespace deepdrive_control

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    deepdrive_control::ControllerPlugin, hardware_interface::SystemInterface)