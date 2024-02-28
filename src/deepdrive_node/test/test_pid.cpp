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
    pc_ = std::make_unique<deepdrive::PIDController>(0.0, 0.0, 0.0);
}

void PIDControllerTest::TearDown() { pc_.reset(); }

// Test just the proportional portion of the PID controller
TEST_F(PIDControllerTest, TestPOnly)
{
    pc_->setKp(.2);
    pc_->setKi(0);
    pc_->setKd(0);
    pc_->setMin(-100);
    pc_->setMax(100);

    // The output should only change with the setpoint
    pc_->setSetpoint(10.0);
    EXPECT_DOUBLE_EQ(pc_->calculate(.1, 0), 2);
    EXPECT_DOUBLE_EQ(pc_->calculate(.1, 0), 2);
    EXPECT_DOUBLE_EQ(pc_->calculate(.1, 0), 2);
    EXPECT_DOUBLE_EQ(pc_->calculate(.1, 0), 2);

    pc_->setSetpoint(20.0);
    EXPECT_DOUBLE_EQ(pc_->calculate(.1, 0), 4);
    EXPECT_DOUBLE_EQ(pc_->calculate(.1, 0), 4);
    EXPECT_DOUBLE_EQ(pc_->calculate(.1, 10), 2);
    EXPECT_DOUBLE_EQ(pc_->calculate(.1, 10), 2);
    EXPECT_DOUBLE_EQ(pc_->calculate(.1, 20), 0);

    // Overshoot
    EXPECT_DOUBLE_EQ(pc_->calculate(22), -0.4);

    // Negative should work too
    pc_->setSetpoint(-20.0);
    EXPECT_DOUBLE_EQ(pc_->calculate(0), -4);
    EXPECT_DOUBLE_EQ(pc_->calculate(0), -4);
    EXPECT_DOUBLE_EQ(pc_->calculate(-10), -2);
    EXPECT_DOUBLE_EQ(pc_->calculate(-15), -1);
    EXPECT_DOUBLE_EQ(pc_->calculate(-15), -1);
}

// Test just the proportional and integrals of the PID controller
TEST_F(PIDControllerTest, TestPI)
{
    pc_->setMin(-100);
    pc_->setMax(100);
    pc_->setKp(.2);
    pc_->setKi(.1);
    pc_->setKd(0);

    // Make sure the output grows as the error grows
    pc_->setSetpoint(100.0);
    double dt = .2;
    EXPECT_DOUBLE_EQ(pc_->calculate(dt, 0), 22);
    EXPECT_DOUBLE_EQ(pc_->calculate(dt, 0), 24);
    EXPECT_DOUBLE_EQ(pc_->calculate(dt, 0), 26);

    // Start closing in on the setpoint
    EXPECT_DOUBLE_EQ(pc_->calculate(dt, 80), 10.4);

    // The error is still accrued, so we should overshoot
    EXPECT_DOUBLE_EQ(pc_->calculate(dt, 90), 8.6);
    EXPECT_DOUBLE_EQ(pc_->calculate(dt, 100), 6.6);
    EXPECT_DOUBLE_EQ(pc_->calculate(dt, 110), 4.4);
    EXPECT_DOUBLE_EQ(pc_->calculate(dt, 120), 2);

    // Start bouncing back
    EXPECT_DOUBLE_EQ(pc_->calculate(dt, 130), -0.6);
    EXPECT_DOUBLE_EQ(pc_->calculate(dt, 130), -1.2);
}

// Test all 3 params
TEST_F(PIDControllerTest, TestPID)
{
    double max = 100;

    pc_->setMin(-max);
    pc_->setMax(max);
    pc_->setKp(.1);
    pc_->setKi(.0001);
    pc_->setKd(.05);
    pc_->setSetpoint(100.0);

    double measured = 0;
    double last_output = 100;

    for (double i = 0; i < 100; i++)
    {
        double output = pc_->calculate(measured);
        std::cout << "Loop Number: " << i << ", Output: " << output << ", Measured: " << measured << std::endl;
        measured += output;

        // Once we get close, we'll manually inspect
        if (measured >= 95)
        {
            break;
        }
        EXPECT_LT(output, last_output);
        last_output = output;
    }

    // Make sure it starts to ease off when it gets close
    for (double i = 95; i < 100; i++)
    {
        double output = pc_->calculate(i);
        EXPECT_LT(output, 1.0);
        EXPECT_GT(output, 0);
    }

    // Settle down - do we need to round to zero it out? Probably client side.
    EXPECT_LT(pc_->calculate(100), .05);

    // Make sure we can't exceed the max
    pc_->reset();
    pc_->setSetpoint(100000.0);
    EXPECT_DOUBLE_EQ(pc_->calculate(0), max);
}