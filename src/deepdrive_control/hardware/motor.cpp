#include "motor.hpp"

namespace deepdrive_control
{

Motor::Motor(const std::string& motor_name, double input_multiplier, double output_multiplier, rclcpp::Node::SharedPtr node)
    : name_(motor_name), velocity_state_(0.0), position_state_(0.0), velocity_command_(0.0), 
      input_multiplier_(input_multiplier), output_multiplier_(output_multiplier), node_(node)
{
    vel_pub_ = node_->create_publisher<std_msgs::msg::Float64>("/deepdrive_motor_" + motor_name + "/cmd/vel", 10);
    angle_sub_ = node_->create_subscription<std_msgs::msg::Float64>("/deepdrive_motor_" + motor_name + "/angle", 10, std::bind(&Motor::angle_callback, this, std::placeholders::_1));
}

void Motor::set_commanded_velocity(double velocity)
{
    velocity_command_ = velocity * output_multiplier_;
}

void Motor::write_command()
{
    publish_velocity();
}

void Motor::publish_velocity()
{
    std_msgs::msg::Float64 vel_msg;
    vel_msg.data = velocity_command_;
    vel_pub_->publish(vel_msg);
}

double &Motor::get_velocity_state()
{
    return velocity_state_;
}

double &Motor::get_position_state()
{
    return position_state_;
}

double &Motor::get_velocity_command()
{
    return velocity_command_;
}

std::string Motor::get_name() const
{
    return name_;
}

void Motor::angle_callback(const std_msgs::msg::Float64::SharedPtr msg)
{
    rclcpp::Time current_time = node_->now();

    // If we have a previous angle reading, calculate the velocity in radians per second
    if (message_received_)
    {
        rclcpp::Duration time_diff = current_time - last_message_time_;
        if (time_diff.seconds() > 0)
        {
            double position_diff = (msg->data - position_state_) * input_multiplier_;
            velocity_state_ = position_diff / time_diff.seconds();
        }
    }

    // Update position and time
    position_state_ = msg->data * input_multiplier_;
    last_message_time_ = current_time;
    message_received_ = true;
}

bool Motor::check_timeout(const rclcpp::Time &current_time, const rclcpp::Duration &timeout) const
{
    return message_received_ && (current_time - last_message_time_) > timeout;
}

bool Motor::has_received_message() const
{
    return message_received_;
}

}  // namespace deepdrive_control
