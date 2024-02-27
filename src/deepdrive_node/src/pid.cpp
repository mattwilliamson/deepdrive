#include "deepdrive_node/pid.hpp"
#include <iostream>
#include <cmath>
#include <memory>
#include <string>
#include <utility>

namespace deepdrive {


    /**
     * Implementation
     */
    PIDController::PIDController( double dt, double max, double min, double Kp, double Kd, double Ki ) :
            dt_(dt),
            max_(max),
            min_(min),
            Kp_(Kp),
            Kd_(Kd),
            Ki_(Ki),
            pre_error_(0),
            integral_(0)
    {
    }

    double PIDController::calculate( double setpoint, double pv )
    {

        //error
        double error = setpoint - pv;

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
        if( output > max_ )
            output = max_;
        else if( output < min_ )
            output = min_;

        // Save error to previous error
        pre_error_ = error;

        return output;
    }

    void PIDController::reset()
    {
        pre_error_ = 0;
        integral_ = 0;
    }


    PIDController::~PIDController()
    {
    }
}