// Copyright 2021 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "deepdrive/deepdrive_system.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

#include <JetsonGPIO.h>

namespace deepdrive_control {

bool parse_bool(const std::string &bool_string) {
  return bool_string == "true" || bool_string == "True";
}

hardware_interface::CallbackReturn
DeepdriveSystemHardware::on_init(const hardware_interface::HardwareInfo &info) {

  RCLCPP_INFO(rclcpp::get_logger("DeepdriveSystemHardware"), "on_init");

  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("DeepdriveSystemHardware"),
              "parent init success");

  hw_mock_ = parse_bool(info_.hardware_parameters["mock_hardware"]);

  hw_pin_left_enable_ = std::stoi(info_.hardware_parameters["pin_left_enable"]);
  hw_pin_left_forward_ =
      std::stoi(info_.hardware_parameters["pin_left_forward"]);
  hw_pin_left_backward_ =
      std::stoi(info_.hardware_parameters["pin_left_backward"]);
  hw_pin_right_enable_ =
      std::stoi(info_.hardware_parameters["pin_right_enable"]);
  hw_pin_right_forward_ =
      std::stoi(info_.hardware_parameters["pin_right_forward"]);
  hw_pin_right_backward_ =
      std::stoi(info_.hardware_parameters["pin_right_backward"]);
  hw_pwm_hz_ = std::stoi(info_.hardware_parameters["pwm_hz"]);

  hw_positions_.resize(info_.joints.size(),
                       std::numeric_limits<double>::quiet_NaN());
  hw_velocities_.resize(info_.joints.size(),
                        std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(),
                      std::numeric_limits<double>::quiet_NaN());

  for (const hardware_interface::ComponentInfo &joint : info_.joints) {
    // DeepdriveSystem has exactly two states and one command interface on each
    // joint
    if (joint.command_interfaces.size() != 1) {
      RCLCPP_FATAL(rclcpp::get_logger("DeepdriveSystemHardware"),
                   "Joint '%s' has %zu command interfaces found. 1 expected.",
                   joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name !=
        hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_FATAL(
          rclcpp::get_logger("DeepdriveSystemHardware"),
          "Joint '%s' have %s command interfaces found. '%s' expected.",
          joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
          hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2) {
      RCLCPP_FATAL(rclcpp::get_logger("DeepdriveSystemHardware"),
                   "Joint '%s' has %zu state interface. 2 expected.",
                   joint.name.c_str(), joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(
          rclcpp::get_logger("DeepdriveSystemHardware"),
          "Joint '%s' have '%s' as first state interface. '%s' expected.",
          joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
          hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_FATAL(
          rclcpp::get_logger("DeepdriveSystemHardware"),
          "Joint '%s' have '%s' as second state interface. '%s' expected.",
          joint.name.c_str(), joint.state_interfaces[1].name.c_str(),
          hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
DeepdriveSystemHardware::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION,
        &hw_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
        &hw_velocities_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
DeepdriveSystemHardware::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
        &hw_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn DeepdriveSystemHardware::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/) {

  // RCLCPP_INFO(rclcpp::get_logger("DeepdriveSystemHardware"), "Activated.
  // Setting pin modes. Model: %s", GPIO::model().c_str());
  RCLCPP_INFO(rclcpp::get_logger("DeepdriveSystemHardware"),
              "Activated. Setting pin modes.");

  // Pin Setup
  if (!hw_mock_) {
    // Board pin-numbering scheme
    GPIO::setmode(GPIO::BOARD);

    GPIO::setup(hw_pin_left_forward_, GPIO::OUT, GPIO::LOW);
    GPIO::setup(hw_pin_left_backward_, GPIO::OUT, GPIO::LOW);
    GPIO::setup(hw_pin_right_forward_, GPIO::OUT, GPIO::LOW);
    GPIO::setup(hw_pin_right_backward_, GPIO::OUT, GPIO::LOW);

    GPIO::setup(hw_pin_left_enable_, GPIO::OUT, GPIO::LOW);
    left_pwm_ = new GPIO::PWM(hw_pin_left_enable_, hw_pwm_hz_);
    left_pwm_->start(0);

    GPIO::setup(hw_pin_right_enable_, GPIO::OUT, GPIO::LOW);
    right_pwm_ = new GPIO::PWM(hw_pin_right_enable_, hw_pwm_hz_);
    right_pwm_->start(0);
  }

  // set some default values
  for (auto i = 0u; i < hw_positions_.size(); i++) {
    if (std::isnan(hw_positions_[i])) {
      hw_positions_[i] = 0;
      hw_velocities_[i] = 0;
      hw_commands_[i] = 0;
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("DeepdriveSystemHardware"),
              "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DeepdriveSystemHardware::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("DeepdriveSystemHardware"), "Deactivating");

  if (!hw_mock_) {
    left_pwm_->stop();
    right_pwm_->stop();
    delete left_pwm_;
    delete right_pwm_;
    GPIO::cleanup();
  }

  RCLCPP_INFO(rclcpp::get_logger("DeepdriveSystemHardware"),
              "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type
DeepdriveSystemHardware::read(const rclcpp::Time & /*time*/,
                              const rclcpp::Duration &period) {
  // BEGIN: This part here is for exemplary purposes - Please do not copy to
  // your production code
  for (std::size_t i = 0; i < hw_velocities_.size(); i++) {
    // Simulate Deepdrive wheels's movement as a first-order system
    // Update the joint status: this is a revolute joint without any limit.
    // Simply integrates
    hw_positions_[i] = hw_positions_[i] + period.seconds() * hw_velocities_[i];

    // RCLCPP_INFO(
    //   rclcpp::get_logger("DeepdriveSystemHardware"),
    //   "Got position state %.5f and velocity state %.5f for '%s'!",
    //   hw_positions_[i], hw_velocities_[i], info_.joints[i].name.c_str());
  }
  // END: This part here is for exemplary purposes - Please do not copy to your
  // production code

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type
deepdrive_control::DeepdriveSystemHardware::write(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
  // BEGIN: This part here is for exemplary purposes - Please do not copy to
  // your production code
  // RCLCPP_INFO(rclcpp::get_logger("DeepdriveSystemHardware"), "Writing...");

  for (auto i = 0u; i < hw_commands_.size(); i++) {
    // Simulate sending commands to the hardware
    RCLCPP_INFO(
      rclcpp::get_logger("DeepdriveSystemHardware"), "Got command %.5f for'%s'!", 
      hw_commands_[i], info_.joints[i].name.c_str());

    // I'm not sure what units these are, but max signal I see when sending is 1
    // so that will be max pwm
    hw_velocities_[i] = hw_commands_[i];
    if (hw_velocities_[i] > MAX_SPEED) {
      hw_velocities_[i] = MAX_SPEED;
    } else if (hw_velocities_[i] < MIN_SPEED) {
      hw_velocities_[i] = MIN_SPEED;
    }
  }

  // Currently one signal is used for left and one for right wheel
  // TODO: Enum for wheel indices
  // wheel_back_right_joint, wheel_front_left_joint, wheel_back_left_joint,
  // wheel_front_right_joint
  auto right_duty_cycle = std::abs(hw_velocities_[0] / MAX_SPEED * 100);
  right_pwm_->ChangeDutyCycle(right_duty_cycle);

  auto left_duty_cycle = std::abs(hw_velocities_[1] / MAX_SPEED * 100);
  left_pwm_->ChangeDutyCycle(left_duty_cycle);

  if (hw_velocities_[0] > 0) {
    GPIO::output(hw_pin_right_forward_, GPIO::HIGH);
    GPIO::output(hw_pin_right_backward_, GPIO::LOW);
  } else if (hw_velocities_[0] < 0) {
    GPIO::output(hw_pin_right_forward_, GPIO::LOW);
    GPIO::output(hw_pin_right_backward_, GPIO::HIGH);
  } else {
    GPIO::output(hw_pin_right_forward_, GPIO::LOW);
    GPIO::output(hw_pin_right_backward_, GPIO::LOW);
  }

  RCLCPP_INFO(rclcpp::get_logger("DeepdriveSystemHardware"),
              "Set PWM for %.5f for '%s' hw_vel: %.5f!", right_duty_cycle,
              info_.joints[0].name.c_str(), hw_velocities_[0]);
  RCLCPP_INFO(rclcpp::get_logger("DeepdriveSystemHardware"),
              "Set PWM for %.5f for '%s' hw_vel: %.5f!", right_duty_cycle,
              info_.joints[1].name.c_str(), hw_velocities_[1]);

  if (hw_velocities_[1] > 0) {
    GPIO::output(hw_pin_left_forward_, GPIO::HIGH);
    GPIO::output(hw_pin_left_backward_, GPIO::LOW);
  } else if (hw_velocities_[1] < 0) {
    GPIO::output(hw_pin_left_forward_, GPIO::LOW);
    GPIO::output(hw_pin_left_backward_, GPIO::HIGH);
  } else {
    GPIO::output(hw_pin_left_forward_, GPIO::LOW);
    GPIO::output(hw_pin_left_backward_, GPIO::LOW);
  }

  return hardware_interface::return_type::OK;
}

} // namespace deepdrive_control

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(deepdrive_control::DeepdriveSystemHardware,
                       hardware_interface::SystemInterface)
