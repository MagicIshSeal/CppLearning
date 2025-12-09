#define CATCH_CONFIG_MAIN
#include "catch_amalgamated.hpp"
#include "../src/pid.hpp"
#include <cmath>

const double tol = 1e-6;

/**
 * TEST STRATEGY:
 * 1. Test P-only control (Ki=0, Kd=0)
 * 2. Test PI control (Kd=0)
 * 3. Test PID control (all terms)
 * 4. Test output limiting/saturation
 * 5. Test reset functionality
 * 6. Test step response behavior
 */

TEST_CASE("PID - Proportional only (P controller)") {
    // P-only controller: output = Kp * error
    PIDController pid(1.0, 0.0, 0.0, 0.0, 1.0);  // Kp=1, Ki=0, Kd=0
    
    double setpoint = 50.0;
    double measurement = 40.0;
    double dt = 0.1;
    
    double output = pid.update(setpoint, measurement, dt);
    
    // Error = 50 - 40 = 10
    // Output = 1.0 * 10 = 10, but clamped to [0, 1] = 1.0
    REQUIRE(std::abs(output - 1.0) < tol);
    REQUIRE(std::abs(pid.getProportionalTerm() - 10.0) < tol);
    REQUIRE(std::abs(pid.getIntegralTerm() - 0.0) < tol);
    REQUIRE(std::abs(pid.getDerivativeTerm() - 0.0) < tol);
}

TEST_CASE("PID - Integral accumulation (I term)") {
    // PI controller to test integral buildup
    PIDController pid(0.0, 1.0, 0.0, -10.0, 10.0);  // Kp=0, Ki=1, Kd=0
    
    double setpoint = 10.0;
    double measurement = 5.0;
    double dt = 0.1;
    
    // First update: error = 5, integral = 5 * 0.1 = 0.5
    double output1 = pid.update(setpoint, measurement, dt);
    REQUIRE(std::abs(output1 - 0.5) < tol);
    
    // Second update: error still 5, integral = 0.5 + 5 * 0.1 = 1.0
    double output2 = pid.update(setpoint, measurement, dt);
    REQUIRE(std::abs(output2 - 1.0) < tol);
    
    // Third update: integral = 1.0 + 5 * 0.1 = 1.5
    double output3 = pid.update(setpoint, measurement, dt);
    REQUIRE(std::abs(output3 - 1.5) < tol);
}

TEST_CASE("PID - Derivative term (D term)") {
    // PD controller to test derivative calculation
    PIDController pid(0.0, 0.0, 1.0, -10.0, 10.0);  // Kp=0, Ki=0, Kd=1
    
    double setpoint = 100.0;
    double dt = 0.1;
    
    // First update: measurement = 50, error = 50
    // Derivative calculation requires previous error, so first call stores it
    double output1 = pid.update(setpoint, 50.0, dt);
    // Derivative term should be 0 on first call (no previous error yet)
    REQUIRE(std::abs(pid.getDerivativeTerm() - 0.0) < tol);
    
    // Second update: measurement = 60, error = 40
    // Error changed from 50 to 40, derivative = (40 - 50) / 0.1 = -100
    double output2 = pid.update(setpoint, 60.0, dt);
    // Output = 1.0 * (-100) = -100, clamped to -10
    REQUIRE(std::abs(output2 - (-10.0)) < tol);
    
    // Third update: measurement = 70, error = 30
    // Error changed from 40 to 30, derivative = (30 - 40) / 0.1 = -100
    double output3 = pid.update(setpoint, 70.0, dt);
    REQUIRE(std::abs(output3 - (-10.0)) < tol);
}

TEST_CASE("PID - Full PID controller behavior") {
    // Complete PID with all three terms
    PIDController pid(0.5, 0.1, 0.05, 0.0, 1.0);
    
    double setpoint = 60.0;
    double measurement = 30.0;
    double dt = 0.1;
    
    // First update: error = 30
    // P = 0.5 * 30 = 15
    // I = 0.1 * (30 * 0.1) = 0.3
    // D = 0 (first iteration)
    // Output = 15 + 0.3 + 0 = 15.3, clamped to 1.0
    double output = pid.update(setpoint, measurement, dt);
    REQUIRE(std::abs(output - 1.0) < tol);
    REQUIRE(pid.getProportionalTerm() > 0);
    REQUIRE(pid.getIntegralTerm() > 0);
}

TEST_CASE("PID - Output limiting and saturation") {
    PIDController pid(2.0, 0.0, 0.0, 0.0, 1.0);
    
    double setpoint = 100.0;
    double measurement = 50.0;
    double dt = 0.1;
    
    // Error = 50, P output = 2.0 * 50 = 100
    // Should be clamped to max output of 1.0
    double output = pid.update(setpoint, measurement, dt);
    REQUIRE(std::abs(output - 1.0) < tol);
    
    // Test negative saturation
    setpoint = 0.0;
    measurement = 100.0;
    // Error = -100, P output = 2.0 * (-100) = -200
    // Should be clamped to min output of 0.0
    output = pid.update(setpoint, measurement, dt);
    REQUIRE(std::abs(output - 0.0) < tol);
}

TEST_CASE("PID - Reset functionality") {
    PIDController pid(0.0, 1.0, 0.0, -10.0, 10.0);
    
    double setpoint = 10.0;
    double measurement = 5.0;
    double dt = 0.1;
    
    // Build up integral
    pid.update(setpoint, measurement, dt);
    pid.update(setpoint, measurement, dt);
    pid.update(setpoint, measurement, dt);
    
    double output_before = pid.update(setpoint, measurement, dt);
    REQUIRE(output_before > 1.0);  // Integral has accumulated
    
    // Reset should clear integral
    pid.reset();
    
    double output_after = pid.update(setpoint, measurement, dt);
    REQUIRE(std::abs(output_after - 0.5) < tol);  // Back to first step value
}

TEST_CASE("PID - Speed control simulation") {
    // Realistic scenario: control aircraft speed with throttle
    // Target: 50 m/s, starting at 30 m/s
    // PID controls throttle (0.0 to 1.0)
    
    PIDController speed_controller(0.02, 0.001, 0.01, 0.0, 1.0);
    
    double target_speed = 50.0;
    double current_speed = 30.0;
    double dt = 0.1;
    
    // Run control loop for several iterations
    for (int i = 0; i < 10; ++i) {
        double throttle = speed_controller.update(target_speed, current_speed, dt);
        
        // Throttle should be positive (need to accelerate)
        REQUIRE(throttle >= 0.0);
        REQUIRE(throttle <= 1.0);
        
        // Simulate speed increase based on throttle
        // Simple model: acceleration = throttle * 2.0 m/s²
        double acceleration = throttle * 2.0;
        current_speed += acceleration * dt;
    }
    
    // After 10 iterations (1 second), speed should have increased
    REQUIRE(current_speed > 30.0);
    REQUIRE(current_speed < 60.0);  // Shouldn't overshoot too much
}

TEST_CASE("PID - Anti-windup prevents integral buildup during saturation") {
    PIDController pid(0.1, 1.0, 0.0, 0.0, 1.0);
    
    double setpoint = 100.0;
    double measurement = 10.0;
    double dt = 0.1;
    
    // Run for many iterations with large error
    // Output will saturate at 1.0, but integral shouldn't grow unbounded
    for (int i = 0; i < 100; ++i) {
        double output = pid.update(setpoint, measurement, dt);
        REQUIRE(output <= 1.0);  // Always clamped
        
        // Integral term should be reasonable (anti-windup working)
        REQUIRE(pid.getIntegralTerm() < 50.0);  // Not growing to thousands
    }
}

TEST_CASE("PID - Setpoint tracking") {
    // Test that PID can track a changing setpoint
    PIDController pid(0.5, 0.1, 0.05, 0.0, 1.0);
    
    double measurement = 20.0;
    double dt = 0.1;
    
    // Step 1: Target 30, error = 10
    double throttle1 = pid.update(30.0, measurement, dt);
    REQUIRE(throttle1 > 0);  // Need positive throttle
    
    // Step 2: Target increased to 50, error = 30 (much larger)
    double throttle2 = pid.update(50.0, measurement, dt);
    // With larger error, response should be stronger (before saturation)
    REQUIRE(throttle2 >= throttle1);  // >= because might be saturated at 1.0
    
    // Step 3: Target reduced to 25 (closer to current), error = 5
    double throttle3 = pid.update(25.0, measurement, dt);
    // Smaller error should result in less throttle (if not saturated)
    REQUIRE(throttle3 <= throttle2);  // Should reduce or stay same
}

TEST_CASE("PID - Zero gain stability") {
    // Test that zero gains don't cause crashes
    PIDController pid(0.0, 0.0, 0.0, 0.0, 1.0);
    
    double output = pid.update(50.0, 30.0, 0.1);
    REQUIRE(std::abs(output - 0.0) < tol);
}
