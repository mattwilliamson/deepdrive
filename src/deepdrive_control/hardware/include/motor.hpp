#ifndef MOTOR_HPP
#define MOTOR_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

namespace deepdrive_control
{

class Motor
{
public:
    Motor(const std::string& motor_name, double input_multiplier, double output_multiplier, double min_velocity_, rclcpp::Node::SharedPtr node);

    void set_commanded_velocity(double velocity);
    void write_command();
    void publish_velocity();

    double &get_velocity_state();
    double &get_position_state();
    double &get_velocity_command();
    std::string get_name() const;

    bool check_timeout(const rclcpp::Time &current_time, const rclcpp::Duration &timeout) const;
    bool has_received_message() const;

private:
    std::string name_;
    double velocity_state_;
    double position_state_;
    double velocity_command_;
    double min_velocity_;

    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr vel_pub_;

    double input_multiplier_;
    double output_multiplier_;

    rclcpp::Time last_message_time_;
    bool message_received_ = false;

    void angle_callback(const std_msgs::msg::Float64::SharedPtr msg);
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr angle_sub_;

    void set_min_velocity(double min_velocity)
    {
        min_velocity_ = min_velocity;
    }
};

}  // namespace deepdrive_control

#endif // MOTOR_HPP
