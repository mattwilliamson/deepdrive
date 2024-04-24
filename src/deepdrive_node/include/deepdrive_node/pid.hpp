#ifndef DEEPDRIVE_NODE__PID_HPP_
#define DEEPDRIVE_NODE__PID_HPP_

#include <limits>

namespace deepdrive
{

    /**
     * @brief A PID controller class for controlling a process variable based on a setpoint.
     */
    class PIDController
    {
    public:
        /**
         * @brief Constructor for PIDController class.
         * @param min The minimum value of the manipulated variable.
         * @param max The maximum value of the manipulated variable.
         * @param Kp The proportional gain.
         * @param Ki The integral gain.
         * @param Kd The derivative gain.
         */
        PIDController(double min, double max, double Kp, double Ki, double Kd);

        /**
         * @brief Overloaded constructor for PIDController class.
         * @param Kp The proportional gain.
         * @param Ki The integral gain.
         * @param Kd The derivative gain.
         */
        PIDController(double Kp, double Ki, double Kd)
            : min_(std::numeric_limits<double>::lowest()),
              max_(std::numeric_limits<double>::max()),
              Kp_(Kp), Ki_(Ki), Kd_(Kd)
        {
        }

        /**
         * @brief Calculates the control output based on the process variable.
         * @param dt The time difference between the current and previous time.
         * @param pv The current value of the process variable.
         * @return The control output.
         */
        double calculate(double dt, double pv);

        /**
         * @brief Calculates the control output based on the process variable.
         * @param pv The current value of the process variable.
         * @return The control output.
         */
        double calculate(double pv);

        /**
         * @brief Resets the internal state of the PID controller.
         * @details A good time to reset the PID controller is when the motors are stopped.
         */
        void reset();

        /**
         * @brief Setter for the setpoint.
         * @param setpoint The new value for the setpoint.
         */
        void setSetpoint(double setpoint);

        /**
         * @brief Setter for the proportional gain (Kp).
         * @param Kp The new value for the proportional gain.
         */
        void setKp(double Kp);

        /**
         * @brief Setter for the integral gain (Ki).
         * @param Ki The new value for the integral gain.
         */
        void setKi(double Ki);

        /**
         * @brief Setter for the derivative gain (Kd).
         * @param Kd The new value for the derivative gain.
         */
        void setKd(double Kd);

        /**
         * @brief Setter for the minimum value of output.
         * @param min The new value for the minimum value of output.
         */
        void setMin(double min);

        /**
         * @brief Setter for the maximum value of output.
         * @param max The new value for the maximum value of output.
         */
        void setMax(double max);

        /**
         * @brief Destructor for PIDController class.
         */
        ~PIDController();

    private:
        double setpoint_; // desired value for the output
        double min_;      // minimum value of output
        double max_;      // maximum value of output
        double Kp_;       // proportional gain
        double Ki_;       // integral gain
        double Kd_;       // derivative gain
        double pre_error_;
        double integral_;
    };

} // namespace deepdrive

#endif // DEEPDRIVE_NODE__PID_HPP_
