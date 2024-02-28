#include "deepdrive_node/pid.hpp"
#include <iostream>
#include <cmath>
#include <memory>
#include <string>
#include <utility>

namespace deepdrive
{
    /**
     * @brief Constructor for the PIDController class.
     *
     * @param dt The time step for the PID controller.
     * @param min The minimum output value of the PID controller.
     * @param max The maximum output value of the PID controller.
     * @param Kd The derivative gain of the PID controller.
     * @param Kp The proportional gain of the PID controller.
     * @param Ki The integral gain of the PID controller.
     */
    PIDController::PIDController(double dt, double max, double min, double Kp, double Kd, double Ki) : dt_(dt),
                                                                                                       min_(min),
                                                                                                       max_(max),
                                                                                                       Kp_(Kp),
                                                                                                       Ki_(Ki),
                                                                                                       Kd_(Kd)
    {
        pre_error_ = 0;
        integral_ = 0;
        setpoint_ = 0;
    }

    void PIDController::setSetpoint(double setpoint)
    {
        setpoint_ = setpoint;
    }

    void PIDController::setKp(double Kp)
    {
        Kp_ = Kp;
    }

    void PIDController::setKi(double Ki)
    {
        Ki_ = Ki;
    }

    void PIDController::setKd(double Kd)
    {
        Kd_ = Kd;
    }

    void PIDController::setTimeInterval(double dt)
    {
        dt_ = dt;
    }

    void PIDController::setMin(double min)
    {
        min_ = min;
    }

    void PIDController::setMax(double max)
    {
        max_ = max;
    }

    double PIDController::calculate(double pv)
    {

        // error
        double error = setpoint_ - pv;

        // Proportional portion
        double Pout = Kp_ * error;

        // Integral portion
        integral_ += error * dt_;
        double Iout = Ki_ * integral_;

        // Derivative portion
        double derivative = (error - pre_error_) / dt_;
        double Dout = Kd_ * derivative;

        // Total output
        double output = Pout + Iout + Dout;

        // Limit to max/min
        if (output > max_)
            output = max_;
        else if (output < min_)
            output = min_;

        // Save error to previous error
        pre_error_ = error;

        return output;
    }

    void PIDController::reset()
    {
        pre_error_ = 0.0;
        integral_ = 0.0;
    }

    PIDController::~PIDController()
    {
    }
}