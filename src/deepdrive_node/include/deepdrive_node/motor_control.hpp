#ifndef MOTOR_CONTROL_HPP
#define MOTOR_CONTROL_HPP

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float32.hpp"

class MotorControl : public rclcpp::Node
{
public:
    MotorControl();

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void publishMotorCommands();
    
    double wheelDiameterLeftFront;
    double wheelDiameterLeftRear;
    double wheelDiameterRightFront;
    double wheelDiameterRightRear;
    double wheelSeparation;
    double slipMultiplier;
    int publishRate;

    double vx; // Linear velocity in the x direction
    double vz; // Angular velocity around the z-axis

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr leftFrontPublisher;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr leftRearPublisher;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr rightFrontPublisher;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr rightRearPublisher;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdVelSubscription;

    rclcpp::TimerBase::SharedPtr timer;
};

#endif // MOTOR_CONTROL_HPP
