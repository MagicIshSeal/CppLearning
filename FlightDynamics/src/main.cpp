#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <cmath>
#include "atmosphere.hpp"
#include "aero.hpp"
#include "integrator.hpp"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Aircraft parameters
struct Aircraft {
    double mass = 1200.0;      // kg
    double S = 16.0;           // wing area [m^2]
    double CL_alpha = 5.7;     // lift curve slope [1/rad]
    double CD0 = 0.025;        // parasitic drag coefficient
    double k = 0.04;           // induced drag factor
    double maxThrust = 5000.0; // maximum thrust [N]
};

void displayState(const AircraftState& state, double t, double altitude, double throttle, double alpha) {
    // Use carriage return to update same lines
    std::cout << "\r";
    std::cout << std::fixed << std::setprecision(1);
    
    std::cout << "Time: " << std::setw(5) << t << "s | "
              << "Alt: " << std::setw(6) << altitude << "m | "
              << "Speed: " << std::setw(5) << state.V << "m/s | "
              << "Dist: " << std::setw(7) << state.x << "m | "
              << "Throttle: " << std::setw(3) << (int)(throttle * 100) << "% | ";
    
    // Visual indicator
    if (state.gamma > 0.1) {
        std::cout << "CLIMB  ";
    } else if (state.gamma < -0.1) {
        std::cout << "DESCENT";
    } else if (state.V > 1.0) {
        std::cout << "CRUISE ";
    } else {
        std::cout << "GROUND ";
    }
    
    std::cout << std::flush;
}

int main() {
    Aircraft aircraft;
    
    // Initial state: on ground, ready for takeoff
    AircraftState state{0.0, 0.0, 0.0, 0.0};
    
    double t = 0.0;
    double dt = 0.1;  // 100ms time step
    
    std::cout << "\n========================================\n";
    std::cout << "      2D FLIGHT SIMULATOR - v1.0       \n";
    std::cout << "========================================\n\n";
    std::cout << "  Aircraft ready for takeoff!\n";
    std::cout << "  Mass:        " << aircraft.mass << " kg\n";
    std::cout << "  Wing Area:   " << aircraft.S << " m^2\n";
    std::cout << "  Max Thrust:  " << aircraft.maxThrust << " N\n\n";
    std::cout << "  Starting simulation in 2 seconds...\n";
    std::cout << "========================================\n";
    
    std::this_thread::sleep_for(std::chrono::seconds(2));
    
    // Simulation loop - auto-pilot for demonstration
    int phase = 0;  // 0=takeoff, 1=climb, 2=cruise, 3=descent
    double throttle = 0.0;
    double alpha = 0.0;  // angle of attack [rad]
    
    while (t < 60.0) {  // Run for 60 seconds
        double altitude = state.z;
        
        // Simple auto-pilot logic
        if (phase == 0) {  // Takeoff roll and rotation
            throttle = 1.0;
            if (state.V > 35.0) {  // Rotation speed
                alpha = 8.0 * M_PI / 180.0;  // 8 degrees
            } else {
                alpha = 0.0;
            }
            if (altitude > 5.0) phase = 1;  // Airborne!
        }
        else if (phase == 1) {  // Climb
            throttle = 0.9;
            alpha = 6.0 * M_PI / 180.0;
            if (altitude > 500.0) phase = 2;
        }
        else if (phase == 2) {  // Cruise
            throttle = 0.5;
            alpha = 3.0 * M_PI / 180.0;
            if (t > 45.0) phase = 3;
        }
        else if (phase == 3) {  // Descent
            throttle = 0.3;
            alpha = 0.0 * M_PI / 180.0;
        }
        
        // Get atmospheric properties at current altitude
        double rho = getDensity(altitude);
        
        // Calculate aerodynamic coefficients
        double CL = calcCL(alpha, aircraft.CL_alpha);
        double CD = calcCD(CL, aircraft.CD0, aircraft.k);
        
        // Calculate forces
        double L = calcLift(rho, state.V, aircraft.S, CL);
        double D = calcDrag(rho, state.V, aircraft.S, CD);
        double W = calcWeight(aircraft.mass, g);
        double T = calcThrust(throttle, aircraft.maxThrust);
        
        // Integrate equations of motion
        state = trapezoidalStep(state, L, D, W, T, aircraft.mass, dt);
        
        // Ground constraint
        if (state.z < 0.0) {
            state.z = 0.0;
            state.gamma = 0.0;
            if (state.V < 1.0) state.V = 0.0;
        }
        
        // Display current state
        displayState(state, t, altitude, throttle, alpha);
        
        // Real-time delay
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        t += dt;
    }
    
    std::cout << "\n\n========================================\n";
    std::cout << "      SIMULATION COMPLETE!             \n";
    std::cout << "========================================\n\n";
    std::cout << std::fixed << std::setprecision(1);
    std::cout << "  Final Statistics:\n";
    std::cout << "  -----------------\n";
    std::cout << "  Total Distance:  " << state.x << " m\n";
    std::cout << "  Final Altitude:  " << state.z << " m\n";
    std::cout << "  Final Speed:     " << state.V << " m/s (" << state.V * 3.6 << " km/h)\n";
    std::cout << "  Flight Time:     " << t << " seconds\n\n";
    std::cout << "========================================\n\n";
    
    return 0;
}
