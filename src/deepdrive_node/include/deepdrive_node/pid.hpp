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
     * @brief Setter for the derivative gain (Kd).
     * @param Kd The new value for the derivative gain.
     */
    void setKd(double Kd);

    /**
     * @brief Setter for the integral gain (Ki).
     * @param Ki The new value for the integral gain.
     */
    void setKi(double Ki);

    /**
     * @brief Destructor for PIDController class.
     */
    ~PIDController();

private:
    double setpoint_;   // desired value for the output
    double dt_;         // loop interval time
    double max_;        // maximum value of output
    double min_;        // minimum value of output
    double Kp_;         // proportional gain
    double Kd_;         // derivative gain
    double Ki_;         // integral gain
    double pre_error_;
    double integral_;
};

} // namespace deepdrive

#endif // DEEPDRIVE_NODE__PID_HPP_
