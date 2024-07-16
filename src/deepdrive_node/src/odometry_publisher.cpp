#include "deepdrive_node/odometry_publisher.hpp"

OdometryPublisher::OdometryPublisher() : Node("odometry_publisher")
{
    // Load parameters
    this->declare_parameter<double>("wheel_diameter_left_front", 0.1651);
    this->declare_parameter<double>("wheel_diameter_left_rear", 0.1651);
    this->declare_parameter<double>("wheel_diameter_right_front", 0.1651);
    this->declare_parameter<double>("wheel_diameter_right_rear", 0.1651);
    this->declare_parameter<double>("tire_width", 0.5);

    this->get_parameter("wheel_diameter_left_front", wheelDiameterLeftFront);
    this->get_parameter("wheel_diameter_left_rear", wheelDiameterLeftRear);
    this->get_parameter("wheel_diameter_right_front", wheelDiameterRightFront);
    this->get_parameter("wheel_diameter_right_rear", wheelDiameterRightRear);
    this->get_parameter("tire_width", tireWidth);

    this->declare_parameter<bool>("invert_left_front", false);
    this->declare_parameter<bool>("invert_left_rear", false);
    this->declare_parameter<bool>("invert_right_front", true);
    this->declare_parameter<bool>("invert_right_rear", true);

    this->get_parameter("invert_left_front", invertLeftFront);
    this->get_parameter("invert_left_rear", invertLeftRear);
    this->get_parameter("invert_right_front", invertRightFront);
    this->get_parameter("invert_right_rear", invertRightRear);
    // TODO: Separate the inversion for measuring and command velocity, one might be inverted while the other isn't
    
    leftFrontAngleSubscription = this->create_subscription<std_msgs::msg::Float64>(
        "/motor/left/front/angle", 10, std::bind(&OdometryPublisher::leftFrontAngleCallback, this, std::placeholders::_1));
    leftRearAngleSubscription = this->create_subscription<std_msgs::msg::Float64>(
        "/motor/left/back/angle", 10, std::bind(&OdometryPublisher::leftRearAngleCallback, this, std::placeholders::_1));
    rightFrontAngleSubscription = this->create_subscription<std_msgs::msg::Float64>(
        "/motor/right/front/angle", 10, std::bind(&OdometryPublisher::rightFrontAngleCallback, this, std::placeholders::_1));
    rightRearAngleSubscription = this->create_subscription<std_msgs::msg::Float64>(
        "/motor/right/back/angle", 10, std::bind(&OdometryPublisher::rightRearAngleCallback, this, std::placeholders::_1));

    jointStatePublisher = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    odometryPublisher = this->create_publisher<nav_msgs::msg::Odometry>("/deepdrive_node/odom", 10);

    timer = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&OdometryPublisher::publishOdometry, this));
}

void OdometryPublisher::leftFrontAngleCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
    leftFrontAngle = invertLeftFront ? -msg->data : msg->data;
}

void OdometryPublisher::leftRearAngleCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
    leftRearAngle = invertLeftRear ? -msg->data : msg->data;
}

void OdometryPublisher::rightFrontAngleCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
    rightFrontAngle = invertRightFront ? -msg->data : msg->data;
}

void OdometryPublisher::rightRearAngleCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
    rightRearAngle = invertRightRear ? -msg->data : msg->data;
}

void OdometryPublisher::publishOdometry()
{
    // Publish Joint State
    auto jointState = sensor_msgs::msg::JointState();
    jointState.header.stamp = this->get_clock()->now();
    jointState.name = {"left_front_wheel", "left_rear_wheel", "right_front_wheel", "right_rear_wheel"};
    jointState.position = {leftFrontAngle, leftRearAngle, rightFrontAngle, rightRearAngle};

    jointStatePublisher->publish(jointState);

    // Calculate odometry
    double deltaLeftFront = (leftFrontAngle - lastLeftFrontAngle) * (wheelDiameterLeftFront / 2.0);
    double deltaLeftRear = (leftRearAngle - lastLeftRearAngle) * (wheelDiameterLeftRear / 2.0);
    double deltaRightFront = (rightFrontAngle - lastRightFrontAngle) * (wheelDiameterRightFront / 2.0);
    double deltaRightRear = (rightRearAngle - lastRightRearAngle) * (wheelDiameterRightRear / 2.0);

    double deltaLeft = (deltaLeftFront + deltaLeftRear) / 2.0;
    double deltaRight = (deltaRightFront + deltaRightRear) / 2.0;

    double deltaDistance = (deltaLeft + deltaRight) / 2.0;
    double deltaTheta = (deltaRight - deltaLeft) / tireWidth;

    x += deltaDistance * cos(theta + (deltaTheta / 2.0));
    y += deltaDistance * sin(theta + (deltaTheta / 2.0));
    theta += deltaTheta;

    lastLeftFrontAngle = leftFrontAngle;
    lastLeftRearAngle = leftRearAngle;
    lastRightFrontAngle = rightFrontAngle;
    lastRightRearAngle = rightRearAngle;

    double vx = deltaDistance / 0.1; // Assume timer interval of 0.1 seconds
    double vtheta = deltaTheta / 0.1;

    auto odom = nav_msgs::msg::Odometry();
    odom.header.stamp = this->get_clock()->now();
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_footprint";
    
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.orientation.z = sin(theta / 2.0);
    odom.pose.pose.orientation.w = cos(theta / 2.0);

    odom.pose.covariance[0] = 0.1;
    odom.pose.covariance[7] = 0.1;
    odom.pose.covariance[14] = 0.1;
    odom.pose.covariance[21] = 0.1;
    odom.pose.covariance[28] = 0.1;
    odom.pose.covariance[35] = 0.1;

    odom.twist.twist.linear.x = vx;
    odom.twist.twist.angular.z = vtheta;

    odom.twist.covariance[0] = 0.1;
    odom.twist.covariance[7] = 0.1;
    odom.twist.covariance[14] = 0.1;
    odom.twist.covariance[21] = 0.1;
    odom.twist.covariance[28] = 0.1;
    odom.twist.covariance[35] = 0.1;

    odometryPublisher->publish(odom);
}