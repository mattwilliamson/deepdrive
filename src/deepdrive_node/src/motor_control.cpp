#include "deepdrive_node/motor_control.hpp"

MotorControl::MotorControl() : Node("motor_control")
{
    // Load parameters
    this->declare_parameter<double>("wheel_diameter_left_front", 0.1);
    this->declare_parameter<double>("wheel_diameter_left_rear", 0.1);
    this->declare_parameter<double>("wheel_diameter_right_front", 0.1);
    this->declare_parameter<double>("wheel_diameter_right_rear", 0.1);
    this->declare_parameter<double>("tire_width", 0.5);
    this->declare_parameter<double>("slip_multiplier", 1.0);

    this->get_parameter("wheel_diameter_left_front", wheelDiameterLeftFront);
    this->get_parameter("wheel_diameter_left_rear", wheelDiameterLeftRear);
    this->get_parameter("wheel_diameter_right_front", wheelDiameterRightFront);
    this->get_parameter("wheel_diameter_right_rear", wheelDiameterRightRear);
    this->get_parameter("tire_width", tireWidth);
    this->get_parameter("slip_multiplier", slipMultiplier);
    this->declare_parameter<double>("publish_rate", 10.0);

    double publishRate;
    this->get_parameter("publish_rate", publishRate);

    leftFrontPublisher = this->create_publisher<std_msgs::msg::Float32>("/motor/left/front/vel/cmd", 10);
    leftRearPublisher = this->create_publisher<std_msgs::msg::Float32>("/motor/left/back/vel/cmd", 10);
    rightFrontPublisher = this->create_publisher<std_msgs::msg::Float32>("/motor/right/front/vel/cmd", 10);
    rightRearPublisher = this->create_publisher<std_msgs::msg::Float32>("/motor/right/back/vel/cmd", 10);

    cmdVelSubscription = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10, std::bind(&MotorControl::cmdVelCallback, this, std::placeholders::_1));

    timer = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000 / publishRate)),
        std::bind(&MotorControl::publishMotorCommands, this));
}

void MotorControl::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    vx = msg->linear.x;
    vz = msg->angular.z;
}

void MotorControl::publishMotorCommands()
{
    double leftFrontVelocity = (vx - vz * tireWidth * slipMultiplier / 2) / (wheelDiameterLeftFront / 2);
    double leftRearVelocity = (vx - vz * tireWidth * slipMultiplier / 2) / (wheelDiameterLeftRear / 2);
    double rightFrontVelocity = (vx + vz * tireWidth * slipMultiplier / 2) / (wheelDiameterRightFront / 2);
    double rightRearVelocity = (vx + vz * tireWidth * slipMultiplier / 2) / (wheelDiameterRightRear / 2);

    std_msgs::msg::Float32 leftFrontMsg;
    std_msgs::msg::Float32 leftRearMsg;
    std_msgs::msg::Float32 rightFrontMsg;
    std_msgs::msg::Float32 rightRearMsg;

    leftFrontMsg.data = leftFrontVelocity;
    leftRearMsg.data = leftRearVelocity;
    rightFrontMsg.data = rightFrontVelocity;
    rightRearMsg.data = rightRearVelocity;

    leftFrontPublisher->publish(leftFrontMsg);
    leftRearPublisher->publish(leftRearMsg);
    rightFrontPublisher->publish(rightFrontMsg);
    rightRearPublisher->publish(rightRearMsg);
}