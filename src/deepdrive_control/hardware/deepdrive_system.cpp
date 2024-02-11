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
  return bool_string == "true" || bool_string == "True" ||
         bool_string == "TRUE";
}

// TODO: Is this supposed to be object oriented?
void on_left_tick() {
  for (std::size_t i = 0; i < wheel_joints_sides_.size(); i++) {
    if (wheel_joints_sides_[i] == LEFT) {
      hw_wheel_ticks_[i]++;
    }
  }
}

void on_right_tick() {
  for (std::size_t i = 0; i < wheel_joints_sides_.size(); i++) {
    if (wheel_joints_sides_[i] == RIGHT) {
      hw_wheel_ticks_[i]++;
    }
  }
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
  use_wheel_encoder_ =
      parse_bool(info_.hardware_parameters["use_wheel_encoder"]);

  command_rate_hz_ = std::stoi(info_.hardware_parameters["command_rate"]);

  // encoder is a photointerrupt sensor for wheel rotation
  hw_encoder_ticks_per_rev_ =
      std::stoi(info_.hardware_parameters["encoder_ticks_per_rev"]);
  hw_pin_left_enable_ = std::stoi(info_.hardware_parameters["pin_left_enable"]);
  hw_pin_left_forward_ =
      std::stoi(info_.hardware_parameters["pin_left_forward"]);
  hw_pin_left_backward_ =
      std::stoi(info_.hardware_parameters["pin_left_backward"]);
  hw_pin_left_encoder_ =
      std::stoi(info_.hardware_parameters["pin_left_encoder"]);
  hw_pin_right_enable_ =
      std::stoi(info_.hardware_parameters["pin_right_enable"]);
  hw_pin_right_forward_ =
      std::stoi(info_.hardware_parameters["pin_right_forward"]);
  hw_pin_right_backward_ =
      std::stoi(info_.hardware_parameters["pin_right_backward"]);
  hw_pin_right_encoder_ =
      std::stoi(info_.hardware_parameters["pin_right_encoder"]);
  hw_pwm_hz_ = std::stoi(info_.hardware_parameters["pwm_hz"]);

  hw_positions_.resize(info_.joints.size(),
                       std::numeric_limits<double>::quiet_NaN());
  hw_velocities_.resize(info_.joints.size(),
                        std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(),
                      std::numeric_limits<double>::quiet_NaN());
  hw_wheel_ticks_.resize(info_.joints.size(),
                         std::numeric_limits<uint16_t>::quiet_NaN());
  hw_wheel_ticks_prev_.resize(info_.joints.size(),
                              std::numeric_limits<uint16_t>::quiet_NaN());

  wheel_joints_sides_.resize(info_.joints.size(),
                             std::numeric_limits<Side>::quiet_NaN());

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

    // L293 Motor Driver
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

    // Wheel encoders
    GPIO::setup(hw_pin_left_encoder_, GPIO::IN);
    GPIO::add_event_detect(hw_pin_left_encoder_, GPIO::Edge::RISING,
                           on_left_tick);

    GPIO::setup(hw_pin_right_encoder_, GPIO::IN);
    GPIO::add_event_detect(hw_pin_right_encoder_, GPIO::Edge::RISING,
                           on_right_tick);
  }

  // set some default values
  for (auto i = 0u; i < hw_positions_.size(); i++) {
    if (std::isnan(hw_positions_[i])) {
      hw_positions_[i] = 0;
      hw_velocities_[i] = 0;
      hw_commands_[i] = 0;
      hw_wheel_ticks_[i] = 0;
      hw_wheel_ticks_prev_[i] = 0;

      // If the joint name has a "left" in it, it's the left wheel
      if (info_.joints[i].name.find("left") != std::string::npos) {
        wheel_joints_sides_[i] = LEFT;
      } else {
        wheel_joints_sides_[i] = RIGHT;
      }
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
  // TODO: Do I need to do a PID controller or will ros2_control do that?

  if (use_wheel_encoder_) {
    // See how many ticks we got for this side since the last update
    for (std::size_t i = 0; i < hw_velocities_.size(); i++) {
      // Convert ticks to radians per second based on revs per tick
      // TODO: Average these out? It's showing between 50 and 100 ticks per
      // second based on the encoder with no in-between
      //  Got 1 ticks for 'wheel_back_left_joint' since last update
      //  (0.0202374130s). ticks_per_second: 49.4134304617 diff_radians:
      //  0.3141592654. Velocity: 15.5236870127 rad/s hw_encoder_ticks_per_rev_:
      //  20 Got 1 ticks for 'wheel_front_left_joint' since last update
      //  (0.0197599320s). ticks_per_second: 50.6074616046 diff_radians:
      //  0.3141592654. Velocity: 15.8988029594 rad/s hw_encoder_ticks_per_rev_:
      //  20 Got 1 ticks for 'wheel_back_left_joint' since last update
      //  (0.0197599320s). ticks_per_second: 50.6074616046 diff_radians:
      //  0.3141592654. Velocity: 15.8988029594 rad/s hw_encoder_ticks_per_rev_:
      //  20

      if (hw_wheel_ticks_[i] < hw_wheel_ticks_prev_[i]) {
        // Overflow
        hw_wheel_ticks_[i] = hw_wheel_ticks_prev_[i] - hw_wheel_ticks_prev_[i];
        hw_wheel_ticks_prev_[i] = 0;
      }

      uint16_t tick_diff = hw_wheel_ticks_[i] - hw_wheel_ticks_prev_[i];

      if (tick_diff > 0) {

        double ticks_per_second = tick_diff / period.seconds();
        // Scale up for precision
        double diff_radians =
            (1000 * tick_diff / hw_encoder_ticks_per_rev_) * 2.0 * M_PI;
        diff_radians /= 1000;

        hw_positions_[i] = hw_positions_[i] + diff_radians;
        hw_velocities_[i] = diff_radians / period.seconds();

        RCLCPP_INFO(
            rclcpp::get_logger("DeepdriveSystemHardware"),
            "Got %d ticks (total: %d) for '%s' since last update (%.10fs). "
            "ticks_per_second: %.10f diff_radians: "
            "%.10f. Velocity: %.10f rad/s hw_encoder_ticks_per_rev_: %d",
            tick_diff, hw_wheel_ticks_[i], info_.joints[i].name.c_str(), period.seconds(),
            ticks_per_second, diff_radians, hw_velocities_[i],
            hw_encoder_ticks_per_rev_);

        // Reset all ticks
        hw_wheel_ticks_prev_[i] = hw_wheel_ticks_[i];
      } else {
        hw_velocities_[i] = 0;
      }
    }
  } else {
    for (std::size_t i = 0; i < hw_velocities_.size(); i++) {
      // Simulate Deepdrive wheels's movement as a first-order system
      // Update the joint status: this is a revolute joint without any limit.
      // Simply integrates
      hw_positions_[i] =
          hw_positions_[i] + period.seconds() * hw_velocities_[i];

      // RCLCPP_INFO(
      //   rclcpp::get_logger("DeepdriveSystemHardware"),
      //   "Got position state %.5f and velocity state %.5f for '%s'!",
      //   hw_positions_[i], hw_velocities_[i], info_.joints[i].name.c_str());
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type
deepdrive_control::DeepdriveSystemHardware::write(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {

  // if (use_wheel_encoder_) {
  for (auto i = 0u; i < hw_commands_.size(); i++) {
    // Simulate sending commands to the hardware
    // RCLCPP_DEBUG(
    //   rclcpp::get_logger("DeepdriveSystemHardware"), "Got command %.5f
    //   for'%s'!", hw_commands_[i], info_.joints[i].name.c_str());

    // I'm not sure what units these are, but max signal I see when sending is
    // 16 so that will be max pwm
    if (hw_commands_[i] > MAX_SPEED) {
      hw_commands_[i] = MAX_SPEED;
    } else if (hw_commands_[i] < MIN_SPEED) {
      hw_commands_[i] = MIN_SPEED;
    }
  }

  // Currently one signal is used for left and one for right wheel
  // wheel_back_right_joint, wheel_front_left_joint, wheel_back_left_joint,
  // wheel_front_right_joint
  auto right_duty_cycle = std::abs(hw_commands_[0] / MAX_SPEED * 100);
  right_pwm_->ChangeDutyCycle(right_duty_cycle);

  auto left_duty_cycle = std::abs(hw_commands_[1] / MAX_SPEED * 100);
  left_pwm_->ChangeDutyCycle(left_duty_cycle);

  // RCLCPP_INFO(rclcpp::get_logger("DeepdriveSystemHardware"),
  //             "Set PWM for %.5f for '%s' hw_vel: %.5f!", right_duty_cycle,
  //             info_.joints[0].name.c_str(), hw_velocities_[0]);
  // RCLCPP_INFO(rclcpp::get_logger("DeepdriveSystemHardware"),
  //             "Set PWM for %.5f for '%s' hw_vel: %.5f!", left_duty_cycle,
  //             info_.joints[1].name.c_str(), hw_velocities_[1]);

  if (hw_commands_[RIGHT] > 0) {
    GPIO::output(hw_pin_right_forward_, GPIO::HIGH);
    GPIO::output(hw_pin_right_backward_, GPIO::LOW);
  } else if (hw_commands_[RIGHT] < 0) {
    GPIO::output(hw_pin_right_forward_, GPIO::LOW);
    GPIO::output(hw_pin_right_backward_, GPIO::HIGH);
  } else {
    GPIO::output(hw_pin_right_forward_, GPIO::LOW);
    GPIO::output(hw_pin_right_backward_, GPIO::LOW);
  }

  if (hw_commands_[LEFT] > 0) {
    GPIO::output(hw_pin_left_forward_, GPIO::HIGH);
    GPIO::output(hw_pin_left_backward_, GPIO::LOW);
  } else if (hw_commands_[LEFT] < 0) {
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
