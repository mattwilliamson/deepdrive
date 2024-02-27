#include "deepdrive_node/deepdrive.hpp"

#include <memory>
#include <string>

using deepdrive::Deepdrive;
using namespace std::chrono_literals;

Deepdrive::Deepdrive() : Node("deepdrive_node", rclcpp::NodeOptions().use_intra_process_comms(true))
{
  RCLCPP_INFO(get_logger(), "Init Deepdrive Node Main");
  node_handle_ = std::shared_ptr<::rclcpp::Node>(this, [](::rclcpp::Node *) {});

  add_motors();
  add_wheels();

  run();
}

Deepdrive::Wheels *Deepdrive::get_wheels()
{
  return &wheels_;
}

Deepdrive::Motors *Deepdrive::get_motors()
{
  return &motors_;
}

void Deepdrive::add_motors()
{
  RCLCPP_INFO(this->get_logger(), "Add Motors");

  this->declare_parameter<float>("motors.profile_acceleration_constant");
  this->declare_parameter<float>("motors.profile_acceleration");

  this->get_parameter_or<float>(
      "motors.profile_acceleration_constant",
      motors_.profile_acceleration_constant,
      214.577);

  this->get_parameter_or<float>(
      "motors.profile_acceleration",
      motors_.profile_acceleration,
      0.0);
}

void Deepdrive::add_wheels()
{
  RCLCPP_INFO(this->get_logger(), "Add Wheels");

  this->declare_parameter<float>("wheels.separation");
  this->declare_parameter<float>("wheels.radius");

  this->get_parameter_or<float>("wheels.separation", wheels_.separation, 0.160);
  this->get_parameter_or<float>("wheels.radius", wheels_.radius, 0.033);
}

// void Deepdrive::add_sensors()
// {
//   RCLCPP_INFO(this->get_logger(), "Add Sensors");
//   sensors_.push_back(new sensors::JointState(node_handle_, "joint_states", "base_link"));
// }

void Deepdrive::run()
{
  RCLCPP_INFO(this->get_logger(), "Run!");

  heartbeat_timer(std::chrono::milliseconds(100));

  parameter_event_callback();
  cmd_vel_callback();
}

void Deepdrive::heartbeat_timer(const std::chrono::milliseconds timeout)
{
  heartbeat_timer_ = this->create_wall_timer(
      timeout,
      [this]() -> void
      {
        static uint8_t count = 0;
        std::string msg;

        // TODO: Send diagnostic array
        RCLCPP_DEBUG(this->get_logger(), "hearbeat count : %d, msg : %s", count, msg.c_str());

        count++;
      });
}

void Deepdrive::parameter_event_callback()
{
  priv_parameters_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this);
  while (!priv_parameters_client_->wait_for_service(std::chrono::seconds(1)))
  {
    if (!rclcpp::ok())
    {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return;
    }

    RCLCPP_WARN(this->get_logger(), "service not available, waiting again...");
  }

  auto param_event_callback =
      [this](const rcl_interfaces::msg::ParameterEvent::SharedPtr event) -> void
  {
    for (const auto &changed_parameter : event->changed_parameters)
    {
      RCLCPP_DEBUG(
          this->get_logger(),
          "changed parameter name : %s",
          changed_parameter.name.c_str());

      if (changed_parameter.name == "motors.profile_acceleration")
      {
        std::string sdk_msg;

        motors_.profile_acceleration =
            rclcpp::Parameter::from_parameter_msg(changed_parameter).as_double();

        motors_.profile_acceleration =
            motors_.profile_acceleration / motors_.profile_acceleration_constant;

        union Data
        {
          int32_t dword[2];
          uint8_t byte[4 * 2];
        } data;

        data.dword[0] = static_cast<int32_t>(motors_.profile_acceleration);
        data.dword[1] = static_cast<int32_t>(motors_.profile_acceleration);

        uint8_t *p_data = &data.byte[0];

        RCLCPP_INFO(
            this->get_logger(),
            "changed parameter value : %f [rev/min2] sdk_msg : %s",
            motors_.profile_acceleration,
            sdk_msg.c_str());
      }
    }
  };

  parameter_event_sub_ = priv_parameters_client_->on_parameter_event(param_event_callback);
}

void Deepdrive::cmd_vel_callback()
{
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel",
      qos,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) -> void
      {
        std::string sdk_msg;

        // TODO: PID Controller for each wheel

        union Data
        {
          int32_t dword[6];
          uint8_t byte[4 * 6];
        } data;

        data.dword[0] = static_cast<int32_t>(msg->linear.x * 100);
        data.dword[1] = 0;
        data.dword[2] = 0;
        data.dword[3] = 0;
        data.dword[4] = 0;
        data.dword[5] = static_cast<int32_t>(msg->angular.z * 100);

        uint8_t *p_data = &data.byte[0];

        RCLCPP_DEBUG(
            this->get_logger(),
            "lin_vel: %f ang_vel: %f msg : %s", msg->linear.x, msg->angular.z, sdk_msg.c_str());
      });
}
