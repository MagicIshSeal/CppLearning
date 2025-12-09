#ifndef AERO_HPP
#define AERO_HPP

// Aerodynamics module for simple flight simulator
// Calculates lift, drag, weight, and thrust

// Lift coefficient (linear approximation)
double calcCL(double alpha, double CL_alpha);

// Drag coefficient (parabolic drag polar)
double calcCD(double CL, double CD0, double k);

// Lift force
double calcLift(double rho, double V, double S, double CL);

// Drag force
double calcDrag(double rho, double V, double S, double CD);

// Weight force
double calcWeight(double mass, double g);

// Thrust force
double calcThrust(double throttle, double maxThrust);

#endif
