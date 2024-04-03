#ifndef TEST_PID_HPP_
#define TEST_PID_HPP_

#include "deepdrive_node/pid.hpp"

#include <memory>
#include <string>
#include <vector>

#include "gmock/gmock.h"

class PIDControllerTest : public ::testing::Test
{
public:
    static void SetUpTestCase();
    static void TearDownTestCase();

    void SetUp();
    void TearDown();

protected:
    std::unique_ptr<deepdrive::PIDController> pc_;
};

#endif // TEST_PID_HPP_
