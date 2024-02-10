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

#ifndef DEEPDRIVE_SYSTEM_HPP_
#define DEEPDRIVE_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include <JetsonGPIO.h>

#include "deepdrive/visibility_control.h"

namespace deepdrive_control
{
class DeepdriveSystemHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(DeepdriveSystemHardware);

  DEEPDRIVE_PUBLIC
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  DEEPDRIVE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  DEEPDRIVE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  DEEPDRIVE_PUBLIC
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  DEEPDRIVE_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  DEEPDRIVE_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  DEEPDRIVE_PUBLIC
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:

  // TODO: Figure out max speed based on the following:
  // is the command velocity in m/s or rad/s or some abstract amount?
  // l_wheel_.cmd / l_wheel_.rads_per_count / cfg_.loop_rate
  const int MAX_SPEED = 32.0;
  const int MIN_SPEED = -32.0;
  
  // Parameters for the Deepdrive PWM motor controller
  bool hw_mock_;
  bool use_wheel_encoder_;

  uint8_t hw_pin_left_enable_;
  uint8_t hw_pin_left_forward_;
  uint8_t hw_pin_left_backward_;
  uint8_t hw_pin_left_encoder_;
  uint8_t hw_pin_right_enable_;
  uint8_t hw_pin_right_forward_;
  uint8_t hw_pin_right_backward_;
  uint8_t hw_pin_right_encoder_;
  uint8_t hw_pwm_hz_;
  uint8_t hw_encoder_ticks_per_rev_;
  uint8_t command_rate_hz_;

  GPIO::PWM *left_pwm_;
  GPIO::PWM *right_pwm_;

  // Store the command for the simulated robot
  std::vector<double> hw_commands_;
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
};


// This should really be done in a separate callback class per 
// https://github.com/pjueon/JetsonGPIO/blob/master/docs/library_api.md
void on_left_tick();
void on_right_tick();

// Define type for left and right sides
typedef uint8_t Side;
const Side RIGHT = 0;
const Side LEFT = 1;

std::vector<uint16_t> hw_wheel_ticks_;

// Track which joints are left vs right
std::vector<Side> wheel_joints_sides_;

}  // namespace deepdrive_control

#endif  // DEEPDRIVE_SYSTEM_HPP_
