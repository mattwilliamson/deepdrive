#include "deepdrive_node/odometry_publisher.hpp"

#include <cmath>

OdometryPublisher::OdometryPublisher() : Node("odometry_publisher"), leftFrontAngle(0.0), leftRearAngle(0.0), rightFrontAngle(0.0), rightRearAngle(0.0), lastLeftFrontAngle(0.0), lastLeftRearAngle(0.0), lastRightFrontAngle(0.0), lastRightRearAngle(0.0), x(0.0), y(0.0), theta(0.0) {
  // Load parameters
  this->declare_parameter<double>("wheel_diameter_left_front", 0.1651);
  this->declare_parameter<double>("wheel_diameter_left_rear", 0.1651);
  this->declare_parameter<double>("wheel_diameter_right_front", 0.1651);
  this->declare_parameter<double>("wheel_diameter_right_rear", 0.1651);
  this->declare_parameter<double>("wheel_separation", 0.5);
  this->declare_parameter<int>("publish_rate", 10);
  this->declare_parameter<double>("slip_multiplier", 1.5);
  this->declare_parameter<bool>("invert_left_front", false);
  this->declare_parameter<bool>("invert_left_rear", false);
  this->declare_parameter<bool>("invert_right_front", false);
  this->declare_parameter<bool>("invert_right_rear", false);

  this->get_parameter("wheel_diameter_left_front", wheelDiameterLeftFront);
  this->get_parameter("wheel_diameter_left_rear", wheelDiameterLeftRear);
  this->get_parameter("wheel_diameter_right_front", wheelDiameterRightFront);
  this->get_parameter("wheel_diameter_right_rear", wheelDiameterRightRear);
  this->get_parameter("wheel_separation", wheelSeparation);
  this->get_parameter("publish_rate", publishRate);
  this->get_parameter("slip_multiplier", slipMultiplier);
  this->get_parameter("invert_left_front", invertLeftFront);
  this->get_parameter("invert_left_rear", invertLeftRear);
  this->get_parameter("invert_right_front", invertRightFront);
  this->get_parameter("invert_right_rear", invertRightRear);

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
      std::chrono::milliseconds(1000 / publishRate),
      std::bind(&OdometryPublisher::publishOdometry, this));

  // print out all the params to log
  RCLCPP_INFO(this->get_logger(), "wheel_diameter_left_front: %f", wheelDiameterLeftFront);
  RCLCPP_INFO(this->get_logger(), "wheel_diameter_left_rear: %f", wheelDiameterLeftRear);
  RCLCPP_INFO(this->get_logger(), "wheel_diameter_right_front: %f", wheelDiameterRightFront);
  RCLCPP_INFO(this->get_logger(), "wheel_diameter_right_rear: %f", wheelDiameterRightRear);
  RCLCPP_INFO(this->get_logger(), "wheel_separation: %f", wheelSeparation);
  RCLCPP_INFO(this->get_logger(), "invert_left_front: %d", invertLeftFront);
  RCLCPP_INFO(this->get_logger(), "invert_left_rear: %d", invertLeftRear);
  RCLCPP_INFO(this->get_logger(), "invert_right_front: %d", invertRightFront);
  RCLCPP_INFO(this->get_logger(), "invert_right_rear: %d", invertRightRear);
  RCLCPP_INFO(this->get_logger(), "publish_rate: %d", publishRate);
  RCLCPP_INFO(this->get_logger(), "slip_multiplier: %f", slipMultiplier);
}

void OdometryPublisher::leftFrontAngleCallback(const std_msgs::msg::Float64::SharedPtr msg) {
  leftFrontAngle = invertLeftFront ? -msg->data : msg->data;
}

void OdometryPublisher::leftRearAngleCallback(const std_msgs::msg::Float64::SharedPtr msg) {
  leftRearAngle = invertLeftRear ? -msg->data : msg->data;
}

void OdometryPublisher::rightFrontAngleCallback(const std_msgs::msg::Float64::SharedPtr msg) {
  rightFrontAngle = invertRightFront ? -msg->data : msg->data;
}

void OdometryPublisher::rightRearAngleCallback(const std_msgs::msg::Float64::SharedPtr msg) {
  rightRearAngle = invertRightRear ? -msg->data : msg->data;
}

void OdometryPublisher::publishOdometry() {
  publishJointState();
  calculateOdometry();
}

void OdometryPublisher::publishJointState() {
  auto jointState = sensor_msgs::msg::JointState();
  jointState.header.stamp = this->get_clock()->now();
  jointState.name = {"left_front_wheel", "left_rear_wheel", "right_front_wheel", "right_rear_wheel"};
  jointState.position = {leftFrontAngle, leftRearAngle, rightFrontAngle, rightRearAngle};

  jointStatePublisher->publish(jointState);
}

void OdometryPublisher::calculateOdometry() {
  double radius = (wheelDiameterRightRear / 2.0);
  double stepTime = 1.0 / publishRate;

  // Amount of rotation in radians since last calculation
  double deltaLeftFront = (leftFrontAngle - lastLeftFrontAngle);
  double deltaLeftRear = (leftRearAngle - lastLeftRearAngle);
  double deltaRightFront = (rightFrontAngle - lastRightFrontAngle);
  double deltaRightRear = (rightRearAngle - lastRightRearAngle);

  // Average out each side between two wheels
  double deltaLeft = (deltaLeftFront + deltaLeftRear) / 2.0;
  double deltaRight = (deltaRightFront + deltaRightRear) / 2.0;

  // Calculate change in orientation
  double deltaDistance = radius * (deltaLeft + deltaRight) / 2.0;
  double deltaTheta = radius * (deltaRight - deltaLeft) / wheelSeparation;

  // Update position and orientation
  x += deltaDistance * cos(theta + (deltaTheta / 2.0));
  y += deltaDistance * sin(theta + (deltaTheta / 2.0));
  theta += deltaTheta;

  lastLeftFrontAngle = leftFrontAngle;
  lastLeftRearAngle = leftRearAngle;
  lastRightFrontAngle = rightFrontAngle;
  lastRightRearAngle = rightRearAngle;

  double vx = deltaDistance / stepTime;
  double vtheta = deltaTheta / stepTime;

  // Apply slip multiplier
  vx /= slipMultiplier;
  vtheta /= slipMultiplier;

  auto odom = nav_msgs::msg::Odometry();
  odom.header.stamp = this->get_clock()->now();
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_footprint";

  odom.pose.pose.position.x = x;
  odom.pose.pose.position.y = y;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, theta);

  odom.pose.pose.orientation.x = q.x();
  odom.pose.pose.orientation.y = q.y();
  odom.pose.pose.orientation.z = q.z();
  odom.pose.pose.orientation.w = q.w();

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
