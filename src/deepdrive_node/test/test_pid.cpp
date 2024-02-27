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


TEST_F(PIDControllerTest, CalculateOutput)
{
    // Input values
    double setpoint = 10.0;
    double pv = 5.0;

    // Expected output
    double expected_output = 30.1;

    // Calculate output
    double output = pid_controller_->calculate(setpoint, pv);

    // Check if output matches expected output
    EXPECT_DOUBLE_EQ(output, expected_output);
}
