#ifndef ODOMETRY_PUBLISHER_HPP
#define ODOMETRY_PUBLISHER_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float64.hpp"
#include <tf2/LinearMath/Quaternion.h>

class OdometryPublisher : public rclcpp::Node
{
public:
    OdometryPublisher();

private:
    void leftFrontAngleCallback(const std_msgs::msg::Float64::SharedPtr msg);
    void leftRearAngleCallback(const std_msgs::msg::Float64::SharedPtr msg);
    void rightFrontAngleCallback(const std_msgs::msg::Float64::SharedPtr msg);
    void rightRearAngleCallback(const std_msgs::msg::Float64::SharedPtr msg);
    
    void publishOdometry();
    void calculateOdometry();
    void publishJointState();

    double wheelDiameterLeftFront;
    double wheelDiameterLeftRear;
    double wheelDiameterRightFront;
    double wheelDiameterRightRear;
    double wheelSeparation;
    int publishRate;
    double slipMultiplier;

    bool invertLeftFront;
    bool invertLeftRear;
    bool invertRightFront;
    bool invertRightRear;

    double leftFrontAngle;
    double leftRearAngle;
    double rightFrontAngle;
    double rightRearAngle;

    double lastLeftFrontAngle;
    double lastLeftRearAngle;
    double lastRightFrontAngle;
    double lastRightRearAngle;

    double x;
    double y;
    double theta;

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr leftFrontAngleSubscription;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr leftRearAngleSubscription;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr rightFrontAngleSubscription;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr rightRearAngleSubscription;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr jointStatePublisher;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometryPublisher;

    rclcpp::TimerBase::SharedPtr timer;
};

#endif // ODOMETRY_PUBLISHER_HPP
