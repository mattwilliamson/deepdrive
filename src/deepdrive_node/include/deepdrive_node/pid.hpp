#ifndef DEEPDRIVE_NODE__PID_HPP_
#define DEEPDRIVE_NODE__PID_HPP_

namespace deepdrive {

/**
 * @brief A PID controller class for controlling a process variable based on a setpoint.
 */
class PIDController {
public:
    /**
     * @brief Constructor for PIDController class.
     * @param dt The loop interval time.
     * @param max The maximum value of the manipulated variable.
     * @param min The minimum value of the manipulated variable.
     * @param Kp The proportional gain.
     * @param Kd The derivative gain.
     * @param Ki The integral gain.
     */
    PIDController(double dt, double max, double min, double Kp, double Kd, double Ki);

    /**
     * @brief Calculates the control output based on the setpoint and process variable.
     * @param setpoint The desired value for the process variable.
     * @param pv The current value of the process variable.
     * @return The control output.
     */
    double calculate(double setpoint, double pv);

    /**
     * @brief Resets the internal state of the PID controller.
     * @details A good time to reset the PID controller is when the motors are stopped.
     */
    void reset();

    /**
     * @brief Destructor for PIDController class.
     */
    ~PIDController();

private:
    double dt_;   // loop interval time
    double max_;  // maximum value of manipulated variable
    double min_;  // minimum value of manipulated variable
    double Kp_;   // proportional gain
    double Kd_;   // derivative gain
    double Ki_;   // integral gain
    double pre_error_;
    double integral_;
};

} // namespace deepdrive

#endif // DEEPDRIVE_NODE__PID_HPP_
