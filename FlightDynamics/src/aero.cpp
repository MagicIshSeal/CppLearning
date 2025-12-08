#include "aero.hpp"
#include <cmath>

// Linear lift coefficient
double calcCL(double alpha, double CL_alpha) {
    return CL_alpha * alpha;  // alpha in radians
}

// Drag coefficient (parabolic drag polar)
double calcCD(double CL, double CD0, double k) {
    return CD0 + k * CL * CL;
}

// Lift force [N]
double calcLift(double rho, double V, double S, double CL) {
    return 0.5 * rho * V * V * S * CL;
}

// Drag force [N]
double calcDrag(double rho, double V, double S, double CD) {
    return 0.5 * rho * V * V * S * CD;
}

// Weight [N]
double calcWeight(double mass, double g) {
    return mass * g;
}

// Thrust [N] (simplified linear with throttle)
double calcThrust(double throttle, double maxThrust) {
    return throttle * maxThrust;
}
