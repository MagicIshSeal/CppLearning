#include "pid.hpp"
#include <algorithm>  // for std::clamp

/**
 * PROGRAMMING IMPLEMENTATION EXPLANATION:
 * 
 * The PID controller maintains internal state between updates:
 * - integral: sum of all past errors * dt
 * - previous_error: error from last timestep (for derivative calculation)
 * 
 * This makes it suitable for real-time control loops where update()
 * is called repeatedly at regular intervals.
 */

PIDController::PIDController(double Kp, double Ki, double Kd,
                             double output_min, double output_max)
    : Kp(Kp), Ki(Ki), Kd(Kd),
      output_min(output_min), output_max(output_max),
      integral(0.0), previous_error(0.0), first_update(true),
      p_term(0.0), i_term(0.0), d_term(0.0)
{
}

double PIDController::update(double setpoint, double measurement, double dt)
{
    // STEP 1: Calculate current error
    // Positive error means we're below setpoint (need to increase output)
    double error = setpoint - measurement;

    // STEP 2: Update integral term (accumulated error)
    // This is the area under the error curve over time
    integral += error * dt;
    
    // STEP 3: Anti-windup - Clamp integral to reasonable bounds
    // Prevent integral from growing unbounded when output is saturated
    // Simple approach: limit integral based on output range
    double max_integral = (output_max - output_min) / (Ki + 1e-10);  // Avoid division by zero
    integral = std::clamp(integral, -max_integral, max_integral);

    // STEP 4: Calculate derivative term (rate of change of error)
    // Derivative predicts future error trend
    // Negative derivative means error is decreasing (good!)
    // On first update, derivative is 0 (no previous error to compare)
    double derivative = 0.0;
    if (!first_update && dt > 1e-10) {  // Skip derivative on first update
        derivative = (error - previous_error) / dt;
    }
    first_update = false;  // Mark that we've had at least one update

    // STEP 5: Calculate individual PID terms
    p_term = Kp * error;
    i_term = Ki * integral;
    d_term = Kd * derivative;

    // STEP 6: Sum all terms to get control output
    double output = p_term + i_term + d_term;

    // STEP 7: Clamp output to physical limits
    // (e.g., throttle can't be negative or > 100%)
    output = std::clamp(output, output_min, output_max);

    // STEP 8: Store error for next derivative calculation
    previous_error = error;

    return output;
}

void PIDController::reset()
{
    // Clear accumulated state
    // Use this when:
    // - Starting a new control task
    // - Setpoint changes dramatically
    // - Switching control modes
    integral = 0.0;
    previous_error = 0.0;
    first_update = true;
    p_term = 0.0;
    i_term = 0.0;
    d_term = 0.0;
}

void PIDController::setOutputLimits(double min, double max)
{
    output_min = min;
    output_max = max;
}
