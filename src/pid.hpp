#ifndef PID_HPP
#define PID_HPP

/**
 * Generic PID (Proportional-Integral-Derivative) Controller
 * 
 * MATHEMATICAL EXPLANATION:
 * The PID controller calculates a control output based on the error between
 * a desired setpoint and the current measured value:
 * 
 *   error(t) = setpoint - measurement
 * 
 *   output(t) = Kp * error(t)                    [Proportional term]
 *             + Ki * integral(error, dt)         [Integral term]
 *             + Kd * derivative(error) / dt      [Derivative term]
 * 
 * WHERE:
 * - Kp (Proportional gain): Reacts to current error magnitude
 *   - Higher Kp = stronger immediate response
 *   - Too high = oscillation/overshoot
 * 
 * - Ki (Integral gain): Eliminates steady-state error over time
 *   - Accumulates past errors
 *   - Higher Ki = faster elimination of persistent error
 *   - Too high = overshoot and instability
 * 
 * - Kd (Derivative gain): Dampens rate of change
 *   - Predicts future error based on current rate
 *   - Higher Kd = more damping, less overshoot
 *   - Too high = noise amplification, sluggish response
 * 
 * INTEGRAL WINDUP PROTECTION:
 * The integral term can accumulate unbounded error when the system is
 * saturated (e.g., throttle at 100%). We clamp the integral to prevent this.
 */

class PIDController {
public:
    /**
     * Constructor: Initialize PID gains
     * @param Kp Proportional gain
     * @param Ki Integral gain  
     * @param Kd Derivative gain
     * @param output_min Minimum output value (e.g., 0.0 for throttle)
     * @param output_max Maximum output value (e.g., 1.0 for throttle)
     */
    PIDController(double Kp, double Ki, double Kd, 
                  double output_min = -1.0, double output_max = 1.0);

    /**
     * Update the PID controller with new measurement
     * 
     * ALGORITHM STEPS:
     * 1. Calculate error = setpoint - measurement
     * 2. Update integral: integral += error * dt (with anti-windup)
     * 3. Calculate derivative: (error - previous_error) / dt
     * 4. Compute output: Kp*error + Ki*integral + Kd*derivative
     * 5. Clamp output to [output_min, output_max]
     * 6. Store error for next derivative calculation
     * 
     * @param setpoint Desired target value
     * @param measurement Current actual value
     * @param dt Time step (seconds)
     * @return Control output (clamped to limits)
     */
    double update(double setpoint, double measurement, double dt);

    /**
     * Reset the controller state
     * Clears integral accumulation and previous error
     * Use when changing setpoint dramatically or restarting control
     */
    void reset();

    /**
     * Set new output limits (useful for different control surfaces)
     * @param min Minimum output value
     * @param max Maximum output value
     */
    void setOutputLimits(double min, double max);

    /**
     * Get individual term contributions (for tuning/debugging)
     */
    double getProportionalTerm() const { return p_term; }
    double getIntegralTerm() const { return i_term; }
    double getDerivativeTerm() const { return d_term; }

private:
    // PID gains
    double Kp;  // Proportional gain
    double Ki;  // Integral gain
    double Kd;  // Derivative gain

    // Output limits
    double output_min;
    double output_max;

    // State variables
    double integral;        // Accumulated error over time
    double previous_error;  // Error from last update (for derivative)
    bool first_update;      // Track if this is the first update

    // Individual term values (for debugging)
    double p_term;
    double i_term;
    double d_term;
};

#endif
