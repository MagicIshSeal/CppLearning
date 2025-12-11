#ifndef AERO_HPP
#define AERO_HPP

#include "aero_data.hpp"
#include <memory>

// Aerodynamics module for simple flight simulator
// Calculates lift, drag, weight, and thrust

// Lift coefficient (linear approximation - legacy)
double calcCL(double alpha, double CL_alpha);

// Drag coefficient (parabolic drag polar - legacy)
double calcCD(double CL, double CD0, double k);

// Lift coefficient from table data
double calcCL(double alpha, const AeroDataTable *table);

// Drag coefficient from table data
double calcCD(double alpha, const AeroDataTable *table);

// Lift force
double calcLift(double rho, double V, double S, double CL);

// Drag force
double calcDrag(double rho, double V, double S, double CD);

// Weight force
double calcWeight(double mass, double g);

// Thrust force
double calcThrust(double throttle, double maxThrust);

#endif
