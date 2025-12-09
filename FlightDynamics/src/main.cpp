#include <iostream>
#include <iomanip>
#include <vector>
#include <cmath>
#include "vec2.hpp"
#include "atmosphere.hpp"
#include "aero.hpp"
#include "integrator.hpp"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

int main() {
    // Aircraft state vectors
    Vec2 position(0.0, 0.0);    // (x, z) position in m
    Vec2 velocity(50.0, 10.0);  // (x, z) velocity in m/s
    double speed = velocity.magnitude();
    Vec2 velocityDir = velocity.normalized();
    
    // Aircraft parameters
    double mass = 1200.0;       // kg
    double S = 16.0;            // wing area m^2
    double CL_alpha = 5.7;      // lift curve slope 1/rad
    double CD0 = 0.025;         // parasitic drag
    double k = 0.04;            // induced drag factor
    double maxThrust = 5000.0;  // N
    
    // Control inputs
    double throttle = 0.7;      // 70% throttle
    double alpha = 5.0 * M_PI / 180.0;  // 5 degree angle of attack
    
    // Aircraft body axis direction (angle from horizontal)
    Vec2 alpha_dir(std::cos(alpha), std::sin(alpha));
    
    // Atmospheric properties
    double rho = getDensity(0.0);
    
    std::cout << "INITIAL STATE:\n";
    std::cout << "  Position: "; position.print(); std::cout << " m\n";
    std::cout << "  Velocity: "; velocity.print(); std::cout << " m/s\n";
    std::cout << "  Speed: " << speed << " m/s\n\n";
    
    // Calculate aerodynamic coefficients
    double CL = calcCL(alpha, CL_alpha);
    double CD = calcCD(CL, CD0, k);
    
    // Calculate force magnitudes
    double L_mag = calcLift(rho, speed, S, CL);
    double D_mag = calcDrag(rho, speed, S, CD);
    double W_mag = calcWeight(mass, g);
    double T_mag = calcThrust(throttle, maxThrust);
    
    // === FORCE VECTOR DECOMPOSITION ===
    
    // 1. THRUST: Acts along aircraft body axis (alpha direction)
    Vec2 F_thrust = alpha_dir * T_mag;
    
    // 2. DRAG: Acts opposite to velocity direction
    Vec2 F_drag = velocityDir * (-D_mag);
    
    // 3. LIFT: Acts perpendicular to velocity (rotated 90° CCW from velocity)
    Vec2 liftDir(-velocityDir.y, velocityDir.x);
    Vec2 F_lift = liftDir * L_mag;
    
    // 4. WEIGHT: Always acts in -z direction (downward)
    Vec2 F_weight(0.0, -W_mag);
    
    // 5. NET FORCE: Sum all force vectors
    Vec2 F_net = F_thrust + F_drag + F_lift + F_weight;
    Vec2 acceleration = F_net / mass;  // a = F/m
    
    std::cout << "FORCES:\n";
    std::cout << "  Thrust: "; F_thrust.print(); std::cout << " N\n";
    std::cout << "  Drag:   "; F_drag.print(); std::cout << " N\n";
    std::cout << "  Lift:   "; F_lift.print(); std::cout << " N\n";
    std::cout << "  Weight: "; F_weight.print(); std::cout << " N\n";
    std::cout << "  Net:    "; F_net.print(); std::cout << " N\n\n";
    
    std::cout << "ACCELERATION:\n";
    std::cout << "  "; acceleration.print(); std::cout << " m/s²\n\n";
    
    // === INTEGRATION STEP ===
    double dt = 0.1;  // 0.1 second time step
    
    std::cout << "INTEGRATING (RK4) with dt=" << dt << "s:\n";
    integrateRK4(position, velocity, acceleration, dt);
    
    for (int i = 0; i < 10; i++) {
        integrateRK4(position, velocity, acceleration, dt);
        speed = velocity.magnitude();
        std::cout << "  Step " << i+1 << ": Position "; position.print(); std::cout << " m\n";
        std::cout << "           Velocity "; velocity.print(); std::cout << " m/s\n";
        std::cout << "           Speed: " << speed << " m/s\n";
    }
    
    return 0;
}