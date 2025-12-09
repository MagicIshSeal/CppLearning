#define CATCH_CONFIG_MAIN
#include "catch_amalgamated.hpp"
#include "../src/integrator.hpp"
#include "../src/vec2.hpp"
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

const double tol = 1e-6;

TEST_CASE("RK4 integrator - zero acceleration") {
    Vec2 position(0.0, 0.0);
    Vec2 velocity(10.0, 5.0);
    Vec2 acceleration(0.0, 0.0);
    double dt = 1.0;

    Vec2 initial_pos = position;
    Vec2 initial_vel = velocity;

    integrateRK4(position, velocity, acceleration, dt);

    // With zero acceleration, velocity should remain constant
    REQUIRE(std::abs(velocity.x - initial_vel.x) < tol);
    REQUIRE(std::abs(velocity.y - initial_vel.y) < tol);
    
    // Position should increase by velocity * dt
    REQUIRE(std::abs(position.x - (initial_pos.x + initial_vel.x * dt)) < tol);
    REQUIRE(std::abs(position.y - (initial_pos.y + initial_vel.y * dt)) < tol);
}

TEST_CASE("RK4 integrator - constant acceleration") {
    Vec2 position(0.0, 0.0);
    Vec2 velocity(0.0, 0.0);
    Vec2 acceleration(10.0, 0.0);  // 10 m/s² in x direction
    double dt = 1.0;

    integrateRK4(position, velocity, acceleration, dt);

    // For constant acceleration: v = v0 + a*t
    // Expected velocity: 10 m/s
    REQUIRE(std::abs(velocity.x - 10.0) < 1e-3);
    REQUIRE(std::abs(velocity.y - 0.0) < tol);
    
    // For constant acceleration: x = x0 + v0*t + 0.5*a*t²
    // Expected position: 0 + 0 + 0.5*10*1 = 5.0 m
    REQUIRE(std::abs(position.x - 5.0) < 1e-3);
    REQUIRE(std::abs(position.y - 0.0) < tol);
}

TEST_CASE("RK4 integrator - gravity simulation") {
    Vec2 position(0.0, 100.0);  // Start at 100m altitude
    Vec2 velocity(20.0, 0.0);   // 20 m/s horizontal
    Vec2 acceleration(0.0, -9.81);  // Gravity
    double dt = 0.1;

    // Simulate for 1 second
    for (int i = 0; i < 10; ++i) {
        integrateRK4(position, velocity, acceleration, dt);
    }

    // After 1 second with gravity:
    // v_y = 0 - 9.81*1 ≈ -9.81 m/s
    // y = 100 + 0*1 - 0.5*9.81*1² ≈ 95.095 m
    // x = 0 + 20*1 = 20 m
    REQUIRE(std::abs(velocity.x - 20.0) < 1e-2);
    REQUIRE(std::abs(velocity.y - (-9.81)) < 1e-2);
    REQUIRE(std::abs(position.x - 20.0) < 1e-2);
    REQUIRE(std::abs(position.y - 95.095) < 1e-1);
}

TEST_CASE("RK4 integrator - circular motion approximation") {
    Vec2 position(1.0, 0.0);     // Start on unit circle
    Vec2 velocity(0.0, 1.0);     // Tangent velocity
    double dt = 0.01;
    
    // Run for quarter circle (π/2 time units with unit angular velocity)
    int steps = static_cast<int>((M_PI / 2.0) / dt);
    for (int i = 0; i < steps; ++i) {
        // Centripetal acceleration pointing toward origin
        Vec2 acceleration = position * (-1.0);
        integrateRK4(position, velocity, acceleration, dt);
    }

    // After quarter circle, should be near (0, 1)
    REQUIRE(std::abs(position.x - 0.0) < 1e-2);
    REQUIRE(std::abs(position.y - 1.0) < 1e-2);
    
    // Velocity should be tangent: (-1, 0)
    REQUIRE(std::abs(velocity.x - (-1.0)) < 1e-2);
    REQUIRE(std::abs(velocity.y - 0.0) < 1e-2);
}

TEST_CASE("RK4 integrator - energy conservation (harmonic oscillator)") {
    Vec2 position(1.0, 0.0);     // Displacement
    Vec2 velocity(0.0, 0.0);     // Initially at rest
    double dt = 0.01;
    double k = 1.0;  // Spring constant
    
    // Initial energy: E = 0.5*k*x² = 0.5*1*1 = 0.5
    double initial_energy = 0.5 * k * position.magnitudeSquared() + 0.5 * velocity.magnitudeSquared();
    
    // Simulate one period (2π for unit mass and k=1)
    int steps = static_cast<int>((2.0 * M_PI) / dt);
    for (int i = 0; i < steps; ++i) {
        Vec2 acceleration = position * (-k);  // F = -kx (Hooke's law)
        integrateRK4(position, velocity, acceleration, dt);
    }
    
    // Final energy should be close to initial (RK4 has some drift over long periods)
    double final_energy = 0.5 * k * position.magnitudeSquared() + 0.5 * velocity.magnitudeSquared();
    REQUIRE(std::abs(final_energy - initial_energy) < 0.02);  // Within 2% energy drift
    
    // Should return close to starting position
    REQUIRE(std::abs(position.x - 1.0) < 0.05);
    REQUIRE(std::abs(position.y - 0.0) < 0.05);
}

TEST_CASE("RK4 integrator - 2D projectile motion") {
    Vec2 position(0.0, 0.0);
    Vec2 velocity(50.0, 50.0);  // 45 degree launch
    Vec2 acceleration(0.0, -9.81);
    double dt = 0.05;
    
    // Simulate until landing
    while (position.y >= 0.0 && dt < 100) {
        integrateRK4(position, velocity, acceleration, dt);
    }
    
    // Range for 45° projectile: R = v²/g = 50²*√2²/9.81 ≈ 510 m
    // (accounting for both x and y components at 45°)
    double expected_range = (velocity.x * velocity.x + velocity.y * velocity.y) / 9.81;
    REQUIRE(std::abs(position.x - expected_range) < 50.0);  // Within 50m
    REQUIRE(position.y < 1.0);  // Should be near ground
}

TEST_CASE("RK4 integrator - small timestep consistency") {
    Vec2 pos1(0.0, 10.0);
    Vec2 vel1(5.0, 0.0);
    Vec2 pos2 = pos1;
    Vec2 vel2 = vel1;
    Vec2 acceleration(2.0, -9.81);
    
    // Integrate with dt = 0.1
    double dt_large = 0.1;
    integrateRK4(pos1, vel1, acceleration, dt_large);
    
    // Integrate with dt = 0.05 twice
    double dt_small = 0.05;
    integrateRK4(pos2, vel2, acceleration, dt_small);
    integrateRK4(pos2, vel2, acceleration, dt_small);
    
    // Results should be very close (RK4 is 4th order accurate)
    REQUIRE(std::abs(pos1.x - pos2.x) < 1e-6);
    REQUIRE(std::abs(pos1.y - pos2.y) < 1e-6);
    REQUIRE(std::abs(vel1.x - vel2.x) < 1e-6);
    REQUIRE(std::abs(vel1.y - vel2.y) < 1e-6);
}

TEST_CASE("Vec2 - magnitude and normalization") {
    Vec2 v(3.0, 4.0);
    REQUIRE(std::abs(v.magnitude() - 5.0) < tol);
    REQUIRE(std::abs(v.magnitudeSquared() - 25.0) < tol);
    
    Vec2 normalized = v.normalized();
    REQUIRE(std::abs(normalized.magnitude() - 1.0) < tol);
    REQUIRE(std::abs(normalized.x - 0.6) < tol);
    REQUIRE(std::abs(normalized.y - 0.8) < tol);
}

TEST_CASE("Vec2 - rotation") {
    Vec2 v(1.0, 0.0);
    
    // Rotate by 90 degrees (π/2)
    Vec2 v90 = v.rotated(M_PI / 2.0);
    REQUIRE(std::abs(v90.x - 0.0) < tol);
    REQUIRE(std::abs(v90.y - 1.0) < tol);
    
    // Rotate by 180 degrees (π)
    Vec2 v180 = v.rotated(M_PI);
    REQUIRE(std::abs(v180.x - (-1.0)) < tol);
    REQUIRE(std::abs(v180.y - 0.0) < tol);
    
    // Rotate by 270 degrees (3π/2)
    Vec2 v270 = v.rotated(3.0 * M_PI / 2.0);
    REQUIRE(std::abs(v270.x - 0.0) < tol);
    REQUIRE(std::abs(v270.y - (-1.0)) < tol);
}

TEST_CASE("Vec2 - angle") {
    Vec2 v1(1.0, 0.0);
    REQUIRE(std::abs(v1.angle() - 0.0) < tol);
    
    Vec2 v2(0.0, 1.0);
    REQUIRE(std::abs(v2.angle() - M_PI / 2.0) < tol);
    
    Vec2 v3(-1.0, 0.0);
    REQUIRE(std::abs(v3.angle() - M_PI) < tol);
    
    Vec2 v4(1.0, 1.0);
    REQUIRE(std::abs(v4.angle() - M_PI / 4.0) < tol);
}

TEST_CASE("Vec2 - dot product") {
    Vec2 v1(3.0, 4.0);
    Vec2 v2(5.0, 12.0);
    
    double dot = v1.dot(v2);
    REQUIRE(std::abs(dot - (3.0*5.0 + 4.0*12.0)) < tol);
    
    // Perpendicular vectors have zero dot product
    Vec2 v3(1.0, 0.0);
    Vec2 v4(0.0, 1.0);
    REQUIRE(std::abs(v3.dot(v4)) < tol);
}

TEST_CASE("Vec2 - arithmetic operations") {
    Vec2 v1(3.0, 4.0);
    Vec2 v2(1.0, 2.0);
    
    Vec2 sum = v1 + v2;
    REQUIRE(std::abs(sum.x - 4.0) < tol);
    REQUIRE(std::abs(sum.y - 6.0) < tol);
    
    Vec2 diff = v1 - v2;
    REQUIRE(std::abs(diff.x - 2.0) < tol);
    REQUIRE(std::abs(diff.y - 2.0) < tol);
    
    Vec2 scaled = v1 * 2.0;
    REQUIRE(std::abs(scaled.x - 6.0) < tol);
    REQUIRE(std::abs(scaled.y - 8.0) < tol);
    
    Vec2 divided = v1 / 2.0;
    REQUIRE(std::abs(divided.x - 1.5) < tol);
    REQUIRE(std::abs(divided.y - 2.0) < tol);
}
