#include <stddef.h>

#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "gmock/gmock.h"
// #include "rclcpp/utilities.hpp"
// #include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "test_pid.hpp"
#include "rclcpp/utilities.hpp"


void PIDControllerTest::SetUpTestCase() { rclcpp::init(0, nullptr); }

void PIDControllerTest::TearDownTestCase() { rclcpp::shutdown(); }

void PIDControllerTest::SetUp()
{
    // Initialize PIDController with test parameters
    pid_controller_ = std::make_unique<deepdrive::PIDController>(0.1, 100.0, -100.0, 1.0, 0.5, 0.2);
}

void PIDControllerTest::TearDown() { pid_controller_.reset(); }


// Test just the proportional portion of the PID controller
TEST_F(PIDControllerTest, TestPOnly)
{
    double time = .1;
    double max = 100.0;
    double min = -100.0;
    double Kp = .2;
    double Ki = 0;
    double Kd = 0;

    pid_controller_ = std::make_unique<deepdrive::PIDController>(time, max, min, Kp, Kd, Ki);
    
    // The output should only change with the setpoint
    pid_controller_->setSetpoint(10.0);
    EXPECT_DOUBLE_EQ(pid_controller_->calculate(0), 2);
    EXPECT_DOUBLE_EQ(pid_controller_->calculate(0), 2);
    EXPECT_DOUBLE_EQ(pid_controller_->calculate(0), 2);
    EXPECT_DOUBLE_EQ(pid_controller_->calculate(0), 2);

    pid_controller_->setSetpoint(20.0);
    EXPECT_DOUBLE_EQ(pid_controller_->calculate(0), 4);
    EXPECT_DOUBLE_EQ(pid_controller_->calculate(0), 4);
    EXPECT_DOUBLE_EQ(pid_controller_->calculate(10), 2);
    EXPECT_DOUBLE_EQ(pid_controller_->calculate(10), 2);
    EXPECT_DOUBLE_EQ(pid_controller_->calculate(20), 0);

    // Overshoot
    EXPECT_DOUBLE_EQ(pid_controller_->calculate(22), -0.4);

    // Negative should work too
    pid_controller_->setSetpoint(-20.0);
    EXPECT_DOUBLE_EQ(pid_controller_->calculate(0), -4);
    EXPECT_DOUBLE_EQ(pid_controller_->calculate(0), -4);
    EXPECT_DOUBLE_EQ(pid_controller_->calculate(-10), -2);
    EXPECT_DOUBLE_EQ(pid_controller_->calculate(-15), -1);
    EXPECT_DOUBLE_EQ(pid_controller_->calculate(-15), -1);
}


// Test just the proportional and integrals of the PID controller
TEST_F(PIDControllerTest, TestPI)
{
    double time = .2;
    double max = 100.0;
    double min = -100.0;
    double Kp = .2;
    double Ki = .1;
    double Kd = 0;

    pid_controller_ = std::make_unique<deepdrive::PIDController>(time, max, min, Kp, Kd, Ki);
    
    // Make sure the output grows as the error grows
    pid_controller_->setSetpoint(100.0);
    EXPECT_DOUBLE_EQ(pid_controller_->calculate(0), 22);
    EXPECT_DOUBLE_EQ(pid_controller_->calculate(0), 24);
    EXPECT_DOUBLE_EQ(pid_controller_->calculate(0), 26);

    // Start closing in on the setpoint
    EXPECT_DOUBLE_EQ(pid_controller_->calculate(80), 10.4);

    // The error is still accrued, so we should overshoot
    EXPECT_DOUBLE_EQ(pid_controller_->calculate(90), 8.6);
    EXPECT_DOUBLE_EQ(pid_controller_->calculate(100), 6.6);
    EXPECT_DOUBLE_EQ(pid_controller_->calculate(110), 4.4);
    EXPECT_DOUBLE_EQ(pid_controller_->calculate(120), 2);

    // Start bouncing back
    EXPECT_DOUBLE_EQ(pid_controller_->calculate(130), -0.6);
    EXPECT_DOUBLE_EQ(pid_controller_->calculate(130), -1.2);
}


// Test all 3 params
TEST_F(PIDControllerTest, TestPID)
{
    double time = 1;
    double max = 100.0;
    double min = -100.0;
    double Kp = .1;
    double Ki = .0001;
    double Kd = .05;

    auto pc = std::make_unique<deepdrive::PIDController>(time, max, min, Kp, Kd, Ki);

    pc->setSetpoint(100.0);
    double measured = 0;
    double last_output = 100;

    for (double i = 0; i < 100; i++) {
        double output = pc->calculate(measured);
        std::cout << "Loop Number: " << i << ", Output: " << output << ", Measured: " << measured << std::endl;
        measured += output;

        // Once we get close, we'll manually inspect
        if(measured >= 95) {
            break;
        }
        EXPECT_LT(output, last_output);
        last_output = output;
    }

    // Make sure it starts to ease off when it gets close
    for (double i = 95; i < 100; i++) {
        double output = pc->calculate(i);
        EXPECT_LT(output, 1.0);
        EXPECT_GT(output, 0);
    }

    // Settle down - do we need to round to zero it out? Probably client side.
    EXPECT_LT(pc->calculate(100), .05);


    // Make sure we can't exceed the max
    pc->reset();
    pc->setSetpoint(100000.0);
    EXPECT_DOUBLE_EQ(pc->calculate(0), max);
    
}